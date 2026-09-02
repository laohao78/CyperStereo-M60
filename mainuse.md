## 1.下载 SDK
```sh
# SDK ROS2 启动
cd CyperstereoSDK/ros2
colcon build --symlink-install
source install/setup.sh
ros2 run data_cap capture_image_imu

# SDK ARM 启动
cd CyperstereoSDK/samples/build
./capture_image_imu
```

## 2.构建镜像
```sh
cd /home/imcrl/data/CyperStereo-M60/kalibr_ws/src 
docker build -f ../docker/Dockerfile -t kalibr:noetic .
```

## 3.录制
> calib_data 目录结构: `bags/`(所有 bag)、`results/`(标定输出)、`target/`(标定板)、`imu/`(IMU 参数)
```sh
# 启动采集(发布 /cam0/image_raw /cam1/image_raw /imu0)
ros2 run data_cap capture_image_imu

# 录 bag(换一个终端)
mkdir -p ~/data/CyperStereo-M60/kalibr_ws/calib_data/bags
cd ~/data/CyperStereo-M60/kalibr_ws/calib_data/bags
ros2 bag record /cam0/image_raw /cam1/image_raw /imu0 -o cyperstereo_imu
# Ctrl+C 停止
```

- 目标:6×6 AprilGrid,3cm 标签 / 0.9cm 缝隙。录制时手持相机让靶板充满画面并平移/旋转、覆盖整个 99s,同时给 IMU 足够的旋转激励
- 本机序列号 = **m000023**(USB 枚举 04b4:00f9,`lsusb`/`/sys/bus/usb/devices/*/serial`),官方出厂标定在 `~/data/CyperStereo-M60/camera_yaml/cyperstereo_sn_m023.yaml`,畸变模型 KannalaBrandt8(鱼眼 = kalibr 的 pinhole-equi)

## 4.ROS2 bag → ROS1 bag(rosbags)
```sh
cd ~/data/CyperStereo-M60/kalibr_ws/calib_data/bags
rosbags-convert --src cyperstereo_imu --dst cyperstereo_imu_ros1.bag
# 验证:容器里 rosbag info 应显示 sensor_msgs/Image、sensor_msgs/Imu(不能带 /msg/)
```

## 5.抽帧(标定不需要 48Hz 全量,太慢)
```sh
cd ~/data/CyperStereo-M60/kalibr_ws/calib_data/bags
SCRIPT=~/data/CyperStereo-M60/kalibr_ws/scripts/subsample_bag.py

# 双目相机阶段:每 8 帧留 1(599 帧/目),丢 IMU
python3 $SCRIPT cyperstereo_imu_ros1.bag cyperstereo_camcalib.bag '{"default":8}' --prefixes /cam

# IMU 联合阶段:/cam* 每 3 帧留 1(1597 帧/目 @16Hz)+ /imu0 全留
python3 $SCRIPT cyperstereo_imu_ros1.bag cyperstereo_imucam.bag '{"default":3,"/imu0":1}' --prefixes /cam /imu0
# (step==1 表示全保留;旧脚本 count%1==1 恒假会把 IMU 全丢,已修)
```

## 6.kalibr 双目相机标定(必须 pinhole-equi,radtan 会挂/错)
```sh
cd ~/data/CyperStereo-M60/kalibr_ws/calib_data && \
docker run --rm -v "$PWD":/data kalibr:noetic \
  bash -lc "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && cd /data/results && \
  rosrun kalibr kalibr_calibrate_cameras \
    --bag /data/bags/cyperstereo_camcalib.bag \
    --topics /cam0/image_raw /cam1/image_raw \
    --models pinhole-equi pinhole-equi \
    --target /data/target/april_6x6_80x80cm.yaml --dont-show-report"
# 结果(写到 results/): cyperstereo_camcalib-camchain.yaml / -results-cam.txt
# 结尾若报 igraph/cairo "plotting not available" 只是 PDF 报告挂,不影响结果
```

## 7.kalibr 相机+IMU 联合标定
```sh
cd ~/data/CyperStereo-M60/kalibr_ws/calib_data && \
docker run --rm -v "$PWD":/data kalibr:noetic \
  bash -lc "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && cd /data/results && \
  rosrun kalibr kalibr_calibrate_imu_camera \
    --cam /data/results/cyperstereo_camcalib-camchain.yaml \
    --imu /data/imu/bmi088_imu_param.yaml \
    --target /data/target/april_6x6_80x80cm.yaml \
    --bag /data/bags/cyperstereo_imucam.bag --dont-show-report"
# 结果(写到 results/): cyperstereo_imucam-camchain-imucam.yaml(含 T_cam_imu + timeshift_cam_imu)
#       cyperstereo_imucam-results-imucam.txt / -report-imucam.pdf
```

## 8.结果(2026-09-02,序列号 m023)
| 项 | kalibr | 官方 m023 | 结论 |
|---|---|---|---|
| cam0 fx/fy | 237.57 / 237.58 | 236.58 / 236.64 | 一致(~0.4%) |
| cam0 cx/cy | 394.06 / 246.22 | 393.59 / 247.47 | 亚像素级 |
| cam0 k1 | 0.0944 | 0.0937 | 一致 |
| 立体基线 | 6.05 cm | 6.01 cm | 差 0.4 mm |
| 重投影误差 | 0.07–0.08 px | — | 亚像素 |
| IMU残差 | 陀螺 0.003 rad/s,加计 0.05 m/s² | — | 健康 |
| T_cam_imu | t=[+47.2,+5.9,−9.5]mm, R≈diag(-1,-1,1) | Tbc t=[+49.8,+4.2,+8.7]mm | 旋转一致;x/y 差~2mm;z 反号= yaml 存逆矩阵约定问题(按下游格式取) |
| timeshift cam-imu | −1.6 ms | — | 次帧级 |

**结论:出厂标定经 kalibr 独立验证通过**,可直接用 `cyperstereo_sn_m023.yaml`(ORB-SLAM3 风格)或 kalibr 产物。