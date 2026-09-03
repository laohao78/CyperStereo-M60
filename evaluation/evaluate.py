#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
真值评估(动捕 GT vs SLAM 轨迹):evo 引擎版。

指标与对齐全部交给 evo(行业标准 APE/ARE),本脚本只负责:
  读取(GT vicon csv / est 8 列 txt)→ 造 evo 轨迹 → 自动扫时间偏移 Δ → evo 关联/对齐/算误差 → 出 5 项交付物。

输出物(每台一个子目录):
  ① summary.json/终端汇总表   max/mean/median/min/rmse/sse/std,APE(mm)+ARE(deg)
  ② ape_timeseries.png         APE 时序曲线 + mean/median/rmse/±1σ
  ③ traj_heatmap3d.png         3D 轨迹热力图(GT 按 APE 上色 + est 对齐叠加)
  ④ xyz_compare.png            X/Y/Z 位置对比(GT vs est 对齐,查时间同步)
  ⑤ rpy_compare.png            Roll/Pitch/Yaw 对比(四元数→欧拉 ZYX 外旋)

时间约定:
  默认吃 slam_time_aligned 的会话秒 est(零点=record_start_target_unix)与 hes3 GT(unix 秒)。
  两机时钟未对时 → 自动扫 Δ:使 est_t + Δ ≈ gt_t(compara 实测 Δ≈+1.8s),随后 evo 关联。
  相机时钟(原始 CameraTrajectory.txt 的 ~1679-1712)须先经 timestamps.csv 换算成会话秒再喂
  (见 compara/数据分析.md;time_aligned 即该换算产物,位置与原始一致)。

用法:
    python3 evaluation/evaluate.py                          # compara 三对
    python3 evaluation/evaluate.py --unit 150               # 只跑一台
    python3 evaluation/evaluate.py --est x.txt --gt y.csv --name my   # 自定义一对
    python3 evaluation/evaluate.py --no-plots               # 只打表出 json,不出图
    python3 evaluation/evaluate.py --lo 0 --hi 4 --step 0.02   # 改偏移搜索范围
依赖: evo + numpy + matplotlib(在 evaluation/.venv 里: ./.venv/bin/python evaluate.py)
"""
import argparse, json, sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
REC = 1785484617.2503152          # record_start_target_unix(compara 会话轴零点)
DEFAULT_PAIRS = {
    "150": ("compara/hes3/hes3-ego.csv",        "compara/test3/slam_time_aligned/CameraTrajectory_150_time_aligned.txt"),
    "151": ("compara/hes3/hes3-left_hand.csv",  "compara/test3/slam_time_aligned/CameraTrajectory_151_time_aligned.txt"),
    "152": ("compara/hes3/hes3-right_hand.csv", "compara/test3/slam_time_aligned/CameraTrajectory_152_time_aligned.txt"),
}

# ---------------------------------------------------------------- evo 导入

_PR = None
def _evo():
    global _PR
    from evo.core import sync, trajectory, metrics
    from evo.core.metrics import PoseRelation
    _PR = PoseRelation
    return sync, trajectory, metrics, PoseRelation

# ---------------------------------------------------------------- 读取

def read_gt_csv(path, origin=None):
    """Vicon csv -> (t_session, pos_m, quat_wxyz). GT unix 秒减 origin(默认减首帧=录制起点)."""
    import csv
    t, xyz, quat = [], [], []
    for r in csv.reader(open(path)):
        if len(r) < 20 or not r[1].strip() or not r[0].strip().lstrip('-').isdigit():
            continue
        try:
            tm = float(r[1])
            if tm < 1e9:                     # 只要 unix 毫秒量级的行
                continue
            x, y, z = float(r[2]) / 1000, float(r[3]) / 1000, float(r[4]) / 1000   # mm->m
            q = (float(r[13]), float(r[14]), float(r[15]), float(r[16]))           # Qx Qy Qz Qw = x y z w
        except ValueError:
            continue
        t.append(tm / 1000.0); xyz.append((x, y, z)); quat.append(q)
    t, xyz, quat = map(np.asarray, (t, xyz, quat))
    if len(t) < 10:
        raise RuntimeError(f"GT csv 解析异常: {path}")
    # 修时间戳:<=0 或非单调的帧按帧序号在两邻有效帧间线性补
    t = _repair_time(t)
    # 剔除四元数全零(姿态无效)帧;位置一并剔除(保持配对一致)
    good = np.linalg.norm(quat, axis=1) > 1e-6
    t, xyz, quat = t[good], xyz[good], quat[good]
    if origin is None:
        origin = t[0]
    t_s = t - origin
    return t_s, xyz, quat, origin

def _repair_time(t):
    t = t.copy()
    good = np.ones(len(t), bool); good[0] = t[0] > 0
    for i in range(1, len(t)):
        good[i] = (t[i] > 0) and (t[i] > t[i - 1])
    i = 0
    while i < len(t):
        if not good[i]:
            j = i
            while j < len(t) and not good[j]:
                j += 1
            a, b = i - 1, j
            ta = t[a] if a >= 0 else (t[b] - 0.011 * (b - i + 1))
            tb = t[b] if b < len(t) else (t[a] + 0.011 * (i - a))
            n = j - i
            t[i:j] = np.linspace(ta, tb, n + 2)[1:-1]
            i = j
        else:
            i += 1
    return t

def read_est_txt(path):
    """8 列 '# t tx ty tz qx qy qz qw' -> (t, pos_m, quat_xyzw). 时间轴须与 GT 同零点."""
    t, xyz, quat = [], [], []
    for line in open(path):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        p = line.split()
        if len(p) != 8:
            continue
        v = [float(x) for x in p]
        t.append(v[0]); xyz.append(v[1:4]); quat.append(v[4:8])
    t, xyz, quat = map(np.asarray, (t, xyz, quat))
    if len(t) < 10:
        raise RuntimeError(f"est 轨迹解析异常: {path}")
    return t, xyz, quat

def to_evo(t, xyz, quat_xyzw):
    """xyz 位置 + xyzw 四元数 -> evo PoseTrajectory3D(内部要求 wxyz)."""
    _, trajectory, _, _ = _evo()
    wxyz = np.roll(quat_xyzw, 1, axis=1)          # [x,y,z,w] -> [w,x,y,z]
    return trajectory.PoseTrajectory3D(xyz, wxyz, t)

# ---------------------------------------------------------------- 对齐/误差(全走 evo)

def _assoc(gt, est, offset, max_diff=0.02):
    """关联 est 与 gt:est_t + offset ≈ gt_t. 返回 (gt子集, est子集) 或 None."""
    sync, _, _, _ = _evo()
    try:
        s_gt, s_est = sync.associate_trajectories(gt, est, max_diff=max_diff, offset_2=offset)
    except Exception:
        return None
    if len(s_est.timestamps) < 50:
        return None
    return s_gt, s_est

def _rmse_mm(gt, est, offset):
    """给定偏移,evo 关联+SE3 对齐后平移 APE rmse(mm);失败返回 inf."""
    syn = _assoc(gt, est, offset)
    if syn is None:
        return np.inf
    s_gt, s_est = syn
    s_est.align(s_gt, correct_scale=False)          # 就地改,对齐到 gt
    ape = _ape_metric(_PR.translation_part)
    ape.process_data((s_est, s_gt))
    return 1000.0 * float(ape.get_all_statistics()["rmse"])

def _ape_metric(rel):
    _, _, metrics, _ = _evo()
    return metrics.APE(rel)

def find_time_offset(gt, est, lo, hi, step):
    """扫 Δ ∈ [lo,hi]:rmse 山谷最低点即 est 相对 gt 的时间偏移(est_t+Δ≈gt_t)."""
    best, bd = np.inf, 0.0
    for d in np.arange(lo, hi + 1e-9, step):
        r = _rmse_mm(gt, est, d)
        if r < best:
            best, bd = r, d
    # 谷底细扫
    for d in np.arange(bd - step, bd + step + 1e-12, step / 40.0):
        r = _rmse_mm(gt, est, d)
        if r < best:
            best, bd = r, d
    return bd, best

# ---------------------------------------------------------------- 统计/绘图

def stats_from_ape(ape, unit_mm=True, to_deg=False):
    """evo APE 对象 -> dict(max/mean/median/min/rmse/sse/std/n). trans 单位 mm,旋转单位 deg."""
    st = ape.get_all_statistics()
    n = len(np.asarray(ape.error))
    factor = 1000.0 if unit_mm else 1.0
    sse_factor = factor * factor
    return dict(n=n,
                max=st["max"] * factor, mean=st["mean"] * factor, median=st["median"] * factor,
                min=st["min"] * factor, rmse=st["rmse"] * factor, sse=st["sse"] * sse_factor,
                std=st["std"] * factor)

def fmt(s, w=8):
    return f"{s['mean']:>{w}.2f} {s['median']:>{w}.2f} {s['rmse']:>{w}.2f} {s['max']:>{w}.2f} {s['sse']:>{w+2}.1f} {s['std']:>{w}.2f}"

def _euler_from_quat_xyzw(q):
    """(N,4) xyzw -> (N,3) roll,pitch,yaw(deg), R=Rz(yaw)Ry(pitch)Rx(roll)."""
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    sy = 2 * (w * z + x * y)
    cy = 1 - 2 * (y * y + z * z)
    yaw = np.degrees(np.arctan2(sy, cy))
    sp = 2 * (w * y - z * x)
    pitch = np.degrees(np.arcsin(np.clip(sp, -1, 1)))
    sr = 2 * (w * x + y * z)
    cr = 1 - 2 * (x * x + y * y)
    roll = np.degrees(np.arctan2(sr, cr))
    return np.stack([roll, pitch, yaw], 1)

def try_plots(name, outdir, te, ape_mm, s_gt, s_est):
    """s_gt/s_est 已 evo 对齐;te=est 会话时间轴;ape_mm=逐点 APE(mm)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"  [plot 跳过: matplotlib 不可用] {e}")
        return
    g = outdir / name; g.mkdir(parents=True, exist_ok=True)
    gt_p, est_p = s_gt.positions_xyz, s_est.positions_xyz
    mean, med, rm, sd = float(np.mean(ape_mm)), float(np.median(ape_mm)), float(np.sqrt(np.mean(ape_mm**2))), float(np.std(ape_mm))
    # ② APE 时序
    fig, ax = plt.subplots(figsize=(9.5, 3.4))
    ax.plot(te, ape_mm, lw=0.8, color="tab:red", label="APE")
    for lab, val, c in [("mean", mean, "tab:blue"), ("median", med, "tab:green"), ("rmse", rm, "tab:purple")]:
        ax.axhline(val, ls="--", lw=1, color=c, label=f"{lab} {val:.2f}")
    ax.fill_between(te, mean - sd, mean + sd, alpha=.12, color="tab:blue", label=f"±1σ {sd:.2f}")
    ax.set_title(f"{name} APE time series (max {ape_mm.max():.2f}, SSE {(ape_mm**2).sum():.1f})")
    ax.set_xlabel("est session time (s)"); ax.set_ylabel("APE (mm)"); ax.grid(alpha=.3); ax.legend(fontsize=7)
    fig.tight_layout(); fig.savefig(g / "ape_timeseries.png", dpi=130); plt.close(fig)
    # ③ 3D 热力图
    fig = plt.figure(figsize=(8.5, 6)); ax = fig.add_subplot(111, projection="3d")
    sc = ax.scatter(gt_p[:, 0], gt_p[:, 1], gt_p[:, 2], c=ape_mm, cmap="jet", s=8, alpha=0.9)
    ax.plot(est_p[:, 0], est_p[:, 1], est_p[:, 2], "k-", lw=0.8, alpha=0.5, label="est(aligned)")
    ax.plot(gt_p[0, 0], gt_p[0, 1], gt_p[0, 2], "g*", ms=15, label="gt start")
    ax.plot(est_p[0, 0], est_p[0, 1], est_p[0, 2], "c^", ms=12, label="est start")
    cb = fig.colorbar(sc, ax=ax, pad=0.1); cb.set_label("APE (mm)")
    ax.set_title(f"{name} trajectory heatmap (color=APE)"); ax.legend(fontsize=7)
    fig.tight_layout(); fig.savefig(g / "traj_heatmap3d.png", dpi=130); plt.close(fig)
    # ④ XYZ 对比
    fig, axs = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    for k, axx in enumerate(axs):
        axx.plot(te, gt_p[:, k], "b-", lw=1, label="gt")
        axx.plot(te, est_p[:, k], "r--", lw=1, label="est(aligned)")
        axx.set_ylabel(["X", "Y", "Z"][k] + " (m)"); axx.grid(alpha=.3); axx.legend(fontsize=7, loc="upper right")
    axs[0].set_title(f"{name} XYZ compare (time sync check)")
    axs[-1].set_xlabel("est session time (s)")
    fig.tight_layout(); fig.savefig(g / "xyz_compare.png", dpi=130); plt.close(fig)
    # ⑤ RPY 对比(四元数 xyzw)
    rpy_g = _euler_from_quat_xyzw(np.roll(s_gt.orientations_quat_wxyz, -1, axis=1))
    rpy_e = _euler_from_quat_xyzw(np.roll(s_est.orientations_quat_wxyz, -1, axis=1))
    fig, axs = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    for k, axx in enumerate(axs):
        axx.plot(te, rpy_g[:, k], "b-", lw=1, label="gt")
        axx.plot(te, rpy_e[:, k], "r--", lw=1, label="est(aligned)")
        axx.set_ylabel(["Roll", "Pitch", "Yaw"][k] + " (deg)"); axx.grid(alpha=.3); axx.legend(fontsize=7, loc="upper right")
    axs[0].set_title(f"{name} RPY compare (may carry static rig offset)")
    axs[-1].set_xlabel("est session time (s)")
    fig.tight_layout(); fig.savefig(g / "rpy_compare.png", dpi=130); plt.close(fig)
    print(f"  图 -> {g}/")

# ---------------------------------------------------------------- 单对评估

def evaluate_one(name, gt_path, est_path, args):
    print(f"\n===== {name}   GT={Path(gt_path).name}  est={Path(est_path).name} =====")
    # 时间轴:compara GT 会话轴零点 = REC(与 est time_aligned 同);自定义 GT 默认减其首帧
    origin = REC if args.compara_axis else None
    t_g, pos_g, quat_g, origin_used = read_gt_csv(gt_path, origin=origin)
    t_e, pos_e, quat_e = read_est_txt(est_path)
    gt_evo = to_evo(t_g, pos_g, quat_g)
    est_evo = to_evo(t_e, pos_e, quat_e)
    print(f"  GT: {len(t_g)} 帧 @ [{t_g[0]:.3f},{t_g[-1]:.3f}] s (零点 unix={origin_used:.4f})")
    print(f"  est: {len(t_e)} 帧 @ [{t_e[0]:.3f},{t_e[-1]:.3f}] s")
    # 自动时间偏移
    if args.auto_offset is None:
        delta, rmse_at = find_time_offset(gt_evo, est_evo, args.lo, args.hi, args.step)
        print(f"  时间偏移扫描: Δ={delta:+.4f}s (est+Δ≈gt)  rmse={rmse_at:.1f}mm")
    else:
        delta = args.auto_offset
    # 最终关联 + SE(3) 对齐
    syn = _assoc(gt_evo, est_evo, delta)
    if syn is None:
        raise RuntimeError(f"{name}: Δ={delta:.3f} 下无足够关联点")
    s_gt, s_est = syn
    s_est.align(s_gt, correct_scale=False)
    print(f"  公共窗: [{s_est.timestamps[0]:.3f},{s_est.timestamps[-1]:.3f}] s, {len(s_est.timestamps)} 点")
    # SE3: 平移 APE / 旋转 ARE
    ape_tr = _ape_metric(_PR.translation_part); ape_tr.process_data((s_est, s_gt))
    ape_rot = _ape_metric(_PR.rotation_angle_deg); ape_rot.process_data((s_est, s_gt))
    st_tr = stats_from_ape(ape_tr)
    st_rot = stats_from_ape(ape_rot, unit_mm=False)
    # Sim3 参考(尺度)
    s_gt2, s_est2 = _assoc(gt_evo, est_evo, delta)
    _, _, scale = s_est2.align(s_gt2, correct_scale=True)
    ape_s3 = _ape_metric(_PR.translation_part); ape_s3.process_data((s_est2, s_gt2))
    st_s3 = stats_from_ape(ape_s3)
    print("  [SE3] APE(mm):", fmt(st_tr))
    print("  [SE3] ARE(° ):", fmt(st_rot))
    print(f"  [Sim3] 尺度 s={scale:.6f}  APE(mm):", fmt(st_s3))
    res = dict(se3_ape=st_tr, se3_are=st_rot, sim3_ape=st_s3, scale=float(scale),
               delta_s=float(delta), origin_unix=float(origin_used),
               win=[float(s_est.timestamps[0]), float(s_est.timestamps[-1])])
    if not args.no_plots:
        te = np.asarray(s_est.timestamps)
        ape_mm = 1000.0 * np.asarray(ape_tr.error)
        try_plots(name, args.out, te, ape_mm, s_gt, s_est)
    return res

def main():
    ap = argparse.ArgumentParser(description="动捕 GT vs SLAM 轨迹评估(evo 引擎)")
    ap.add_argument("--unit", default=None, help="只跑一台: 150/151/152")
    ap.add_argument("--est", default=None, help="自定义 est txt(8 列会话秒)")
    ap.add_argument("--gt", default=None, help="自定义 GT vicon csv")
    ap.add_argument("--name", default=None, help="自定义该对的名字")
    ap.add_argument("--out", default=str(ROOT / "evaluation" / "results"))
    ap.add_argument("--lo", type=float, default=0.0, help="时间偏移搜索下限(秒)")
    ap.add_argument("--hi", type=float, default=4.0, help="时间偏移搜索上限(秒)")
    ap.add_argument("--step", type=float, default=0.05, help="时间偏移搜索步长(秒)")
    ap.add_argument("--auto-offset", type=float, default=None, help="跳过搜索,直接用该偏移")
    ap.add_argument("--compara-axis", action="store_true", help="GT 会话轴用 record_start_target_unix(compara 默认)")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()
    pairs = {}
    if args.gt and args.est:
        pairs[args.name or "custom"] = (args.gt, args.est)
        args.compara_axis = False
    else:
        for k, (g, e) in DEFAULT_PAIRS.items():
            if args.unit and k != args.unit:
                continue
            pairs[k] = (str(ROOT / g), str(ROOT / e))
        args.compara_axis = True
    if not pairs:
        ap.error("没有可评估的配对")
    args.out = Path(args.out)
    results = {}
    for k, (g, e) in pairs.items():
        try:
            results[k] = evaluate_one(k, g, e, args)
        except Exception as ex:
            print(f"  [!!] {k} 评估失败: {ex}", file=sys.stderr)
    # 汇总表(每台:APE(mm) 与 ARE(deg) 各一行, 列=max mean median min rmse sse std)
    print("\n================ 汇总表 (evo SE(3) 对齐) ================")
    hdr = (f"{'unit':>5} | {'量':<4} | {'max':>9} {'mean':>9} {'median':>9} {'min':>9} "
           f"{'rmse':>9} {'sse':>11} {'std':>9} | {'scale':>7} {'N':>5} {'Δ':>7}")
    print(hdr); print("-" * len(hdr))
    for k, r in results.items():
        a, ar = r["se3_ape"], r["se3_are"]
        def row(s):
            return (f"{k:>5} | " + f"{'APE':<4}" + f" | {s['max']:9.3f} {s['mean']:9.3f} {s['median']:9.3f} {s['min']:9.3f} "
                    f"{s['rmse']:9.3f} {s['sse']:11.1f} {s['std']:9.3f}")
        print(row(a) + f" | {r['scale']:7.4f} {a['n']:5d} {r['delta_s']:+7.2f}")
        print(f"{k:>5} | {'ARE':<4} | {ar['max']:9.3f} {ar['mean']:9.3f} {ar['median']:9.3f} {ar['min']:9.3f} "
              f"{ar['rmse']:9.3f} {ar['sse']:11.1f} {ar['std']:9.3f}")
    if results:
        args.out.mkdir(parents=True, exist_ok=True)
        (args.out / "summary.json").write_text(json.dumps(results, indent=1, default=float))
        print(f"\n结果 -> {args.out}/summary.json")

if __name__ == "__main__":
    main()
