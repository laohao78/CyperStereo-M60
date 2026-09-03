# compara —— 真值评估样本数据说明

> 用途:评估 CyperStereo M60(SLAM 输出轨迹)vs 光学动捕真值的样本包,供写 APE/ARE 离线评估算法用。
> 会话:`capture_150_151_152_20260731_155652`(2026-07-31 录制,见 `test3/manifest.json`)。
> 对应关系(依据 对话.md + 目录命名):**150=头(ego)、151=左手(left_hand)、152=右手(right_hand)**,每台 M60 双目。

```
compara/
├── README.md                  # 本文档
├── hes3/                      # 动捕(度量)真值,3 个刚体 + 动捕工程文件
│   ├── hes3-ego.csv           #   → 相机 150(头/ego)的真值轨迹
│   ├── hes3-left_hand.csv     #   → 相机 151(左手)
│   ├── hes3-right_hand.csv    #   → 相机 152(右手)
│   ├── hes3-{ego,left_hand,right_hand}.trb/.xrb/.xrs   # 动捕软件原生数据(xrs 是文本工程/轨迹)
│   ├── hes3/hes3.vc0..vc5     #   动捕相机标定(vc = video camera?)
│   ├── setup.cal / Scenes.json#   动捕场设置(二进制 / 场景 json)
└── test3/                     # 采集 + SLAM 处理结果
    ├── manifest.json          #   会话清单(每台文件数/字节)
    ├── 150_right/ 151_left/ 152_right/    # 三台设备的采集
    │   └── stereo/            #   双目原始采集 + 厂商 SLAM
    │       ├── left.mp4 right.mp4         #   H.264 视频源
    │       ├── left/ right/              #   解帧 PNG,文件名=相机时间戳×1e4(如 16787097.png → 1678.7097 s)
    │       ├── timestamps.csv            #   帧→系统时钟映射
    │       ├── imu/imu.csv               #   IMU(约 200Hz)
    │       └── slam_result/              #   厂商 CyperStereo SLAM 输出
    │           ├── CameraTrajectory.txt  #   原始轨迹(相机时钟,未对齐,见下)
    │           └── SessionInfo.txt LBA_Stats.txt slam.log ...   #   日志(会话内 KF/MP 数等)
    └── slam_time_aligned/     # ★ 评估直接用:时间已对齐的 SLAM 轨迹
        ├── alignment_metadata.json        #   对齐记录(applied_shift、公共时间窗…)
        └── CameraTrajectory_{150,151,152}_time_aligned.txt
```

---

## 1. 真值(动捕)`hes3/hes3-*.csv`

Vicon 风格 **"Hierarchical Translation and Rotation"** 文本导出。三个 csv 结构相同,**刚体段都叫 `Segment1`**(段头 `[Head]`),区分靠文件名对应到各相机。

| 项 | 值 |
|---|---|
| 采样率 | 90 Hz(名义;实测 dt≈11 ms,部分 12 ms) |
| 行数 | 3534 帧 / 文件,时长 ≈39.25 s(ms 起 1785484617254) |
| 平移单位 | **mm**(`TranslationUnits`=mm) |
| 时间 | 第 2 列 `Timestamp` = **unix 毫秒** |
| 姿态 | 第 13-16 列 Qx/Qy/Qz/Qw(单位四元数);17-19 列 Ex/Ey/Ez 欧拉角(度,ZYX) |
| 量纲方向 | X/Y/Z 为刚体原点在**动捕全局系**坐标 |

数据行关键列(28 列,0 起):
```
0 Frame#  1 Timestamp(unix ms)  2 X  3 Y  4 Z(mm)
13 Qx  14 Qy  15 Qz  16 Qw      (17-19 Ex Ey Ez 度)
```
> 注意:部分行 V/A/EV/EA 列为空;真值与 SLAM 在不同坐标系,**不能直接比**,必须先空间对齐。

## 2. SLAM 轨迹

### 2a. 原始(未对齐)`stereo/slam_result/CameraTrajectory.txt`
```
相机时钟秒  tx ty tz qx qy qz qw
1679.209500 0.000000019 ...
```
时间戳是**相机开机时钟**(≈1678.7 s 起,步进 ≈0.04 s = 25 Hz),非墙上时钟。

### 2b. ★ 时间已对齐 `test3/slam_time_aligned/CameraTrajectory_{150,151,152}_time_aligned.txt`
```
# session_time tx ty tz qx qy qz qw
1.689434 0.000000019 0.000000011 0.000000002 ...
```
- 头部注释行说明列;`session_time` = 相对录制起点 0 的**秒**(墙上时钟轴)
- 由 `alignment_metadata.json` 记录换算:`applied_shift_seconds` 把相机时钟平移到系统 unix 秒,再减去 `record_start_target_unix`
- pose 数 150:821 / 151:827 / 152:821,**公共时间窗 [1.689, 33.470] s**(150:1.689–34.480,151:0.849–33.879,152:0.681–33.470)
- 平移为**米制量级**(假设 m;需在评估里把 GT 的 mm 换算后比较),姿态是 SLAM 自身坐标系

`alignment_metadata.json` 字段含义:
| 字段 | 含义 |
|---|---|
| source | 原始轨迹路径(对方机器上的绝对路径,仅供参考) |
| alignment_method | interp_system_time_s(按 timestamps.csv 插值对齐) |
| applied_shift_seconds | 减去的量 = first_camera_timestamp - first_system_time_s(≈-1677 s) |
| record_start_target_unix / _actual | 录制起点目标/实际 unix 秒 |
| common_interval_seconds | 三台公共时间窗 [1.689434, 33.470077] |

## 3. 采集附属文件

| 文件 | 格式 | 说明 |
|---|---|---|
| `stereo/timestamps.csv` | `frame_idx, camera_timestamp, system_time_ns, system_time_s` | 每帧相机时钟→系统时钟映射(时间对齐的依据),~10 Hz |
| `stereo/imu/imu.csv` | `ts, 6 个浮点` | 约 200 Hz;ts 相机时钟;6 轴为陀螺/加计(顺序未注明),单位未注明 |
| `stereo/left.mp4 right.mp4` | H.264 | 双目视频源 |
| `stereo/left/ right/` | PNG | 解帧图像,每目 842 张;文件名 = camera_timestamp×1e4 |
| `hes3/*.trb/.xrb/.vc*/.cal` | 二进制 | 动捕软件原生重建/标定文件,**评估不需要** |
| `hes3/*.xrs` | 文本(CRLF) | 动捕轨迹的另一导出(同 csv 内容,制表符分隔),评估不需要 |

## 4. 已知数据问题(评估算法要处理)

1. **真值时间戳瑕疵**:`hes3-left_hand.csv` 在 ≈30.0 s(帧 ~2701–2714)有 **约 12 帧 `Timestamp` 被写成 0**(出现 dt=-1.78e9 ms 回跳与连续 0 重复)。ego / right_hand 无此问题,仅 ±1 ms 正常抖动。
   → 对齐/取误差前需做时间戳健壮性检查(丢弃/修复非单调或归零的 GT 帧)。
2. 采样率不一致:GT 90 Hz,SLAM ≈25 Hz → 误差计算需在同一时间点上**插值对齐**。
3. 单位与坐标系不一致:GT 在动捕全局系(mm),SLAM 在自身系(米制)→ 需空间对齐(SE(3)/Sim3)+ 换算后再算误差。
