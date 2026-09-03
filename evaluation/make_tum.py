#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把动捕 Vicon csv 转成 evo 用的 TUM 轨迹文件(时间轴=会话秒,单位=米)。

TUM 每行: timestamp tx ty tz qx qy qz qw
时间轴: unix 秒 - record_start_target_unix(=1785484617.2503152, 与 slam_time_aligned 的
session_time 同一零点,见 compara/test3/slam_time_aligned/alignment_metadata.json)。
四元数按文件顺序 qx,qy,qz,qw 输出(与 est/evoTUM 约定一致)。
对时间戳<=0 / 非单调 / 四元数全零的坏帧做线性时间修复或剔除(全零四元数帧剔除)。

用法:
    python3 evaluation/make_tum.py          # 写 evaluation/tum/hes3-{ego,left_hand,right_hand}.tum
    python3 evaluation/make_tum.py --gt xxx.csv -o out.tum   # 任意 vicon csv
"""
import argparse
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
START_UNIX = 1785484617.2503152          # record_start_target_unix, 与 est session_time 同零点
SRC = {
    "ego":        ROOT / "compara" / "hes3" / "hes3-ego.csv",
    "left_hand":  ROOT / "compara" / "hes3" / "hes3-left_hand.csv",
    "right_hand": ROOT / "compara" / "hes3" / "hes3-right_hand.csv",
}

def read_vicon(path):
    import csv
    t, xyz, quat = [], [], []
    for r in csv.reader(open(path)):
        if len(r) < 20 or not r[0].strip().isdigit() or not r[1].strip():
            continue
        try:
            tm = float(r[1]) / 1000.0                      # unix s
            x, y, z = (float(r[2]) / 1000, float(r[3]) / 1000, float(r[4]) / 1000)
            qx, qy, qz, qw = (float(r[13]), float(r[14]), float(r[15]), float(r[16]))
        except ValueError:
            continue
        t.append(tm); xyz.append((x, y, z)); quat.append((qx, qy, qz, qw))
    t = np.asarray(t); xyz = np.asarray(xyz); quat = np.asarray(quat)
    return t, xyz, quat

def repair_time(t):
    """线性补时间:<=0 / 非单调帧按帧序号在两邻有效帧间插值."""
    t = t.copy()
    good = np.ones(len(t), bool)
    good[0] = t[0] > 0
    for i in range(1, len(t)):
        good[i] = (t[i] > 0) and (t[i] > t[i - 1])
    i = 0
    while i < len(t):
        if not good[i]:
            j = i
            while j < len(t) and not good[j]:
                j += 1
            a, b = i - 1, j
            ta = t[a] if a >= 0 else (t[b] - (b - i + 1) * 0.011 if b + 1 < len(t) else t[b])
            tb = t[b] if b < len(t) else (t[a] + (i - a) * 0.011 if a > 0 else t[a])
            n = j - i
            t[i:j] = np.linspace(ta, tb, n + 2)[1:-1]
            i = j
        else:
            i += 1
    return t

def write_tum(path, t_s, xyz_m, quat):
    with open(path, "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw  (session s / m, origin=record_start_target_unix)\n")
        for ts, p, q in zip(t_s, xyz_m, quat):
            f.write(f"{ts:.6f} {p[0]:.9f} {p[1]:.9f} {p[2]:.9f} "
                    f"{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", default=None)
    ap.add_argument("-o", "--out", default=None)
    a = ap.parse_args()
    outdir = ROOT / "evaluation" / "tum"
    if a.gt and a.out:
        t, xyz, quat = read_vicon(a.gt)
        t = repair_time(t)
        write_tum(a.out, t - START_UNIX, xyz, quat)
        print(f"{a.out}: {len(t)} 帧")
        return
    outdir.mkdir(parents=True, exist_ok=True)
    for k, p in SRC.items():
        t, xyz, quat = read_vicon(p)
        t = repair_time(t)
        # 剔除四元数全零帧(姿态无效,如 left_hand ~13 帧)
        good = np.linalg.norm(quat, axis=1) > 1e-6
        out = outdir / f"hes3-{k}.tum"
        write_tum(out, t[good] - START_UNIX, xyz[good], quat[good])
        print(f"{out}: {good.sum()}/{len(t)} 帧 (剔除 {len(t)-good.sum()})")

if __name__ == "__main__":
    main()
