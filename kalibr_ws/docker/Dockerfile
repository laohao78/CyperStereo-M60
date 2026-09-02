# Kalibr (ethz-asl) for arm64 (Jetson) + ROS1 Noetic, built in a container.
# Host stays on ROS2 Humble; kalibr runs here and reads a ROS1 bag.
#
# Build (context = kalibr_ws/src, so the repo is COPYable):
#   cd ~/data/CyperStereo-M60/kalibr_ws/src
#   docker build -f ../docker/Dockerfile -t kalibr:noetic .
#
# Official kalibr Dockerfile_ros1_20_04 uses the huge noetic-desktop-full base;
# this fork does not use cv_bridge, so ros:noetic-ros-base + system deps suffice.

FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Same system/python deps as kalibr's official Dockerfile_ros1_20_04, minus the
# GUI-only python3-wxgtk4.0 (not needed for headless calibration runs).
RUN apt-get update && apt-get install -y --no-install-recommends \
    git wget nano autoconf automake \
    python3-dev python3-pip python3-scipy python3-matplotlib python3-numpy \
    ipython3 python3-tk python3-igraph python3-pyx \
    libeigen3-dev libboost-all-dev libsuitesparse-dev \
    libopencv-dev \
    libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev \
    python3-catkin-tools python3-osrf-pycommon \
    && rm -rf /var/lib/apt/lists/*

# kalibr needs a manual focal length to bootstrap optimisation in some runs.
ENV KALIBR_MANUAL_FOCAL_LENGTH_INIT=1

WORKDIR /catkin_ws
RUN mkdir -p /catkin_ws/src
COPY kalibr /catkin_ws/src/kalibr

RUN cd /catkin_ws \
    && catkin init \
    && catkin config --extend /opt/ros/noetic \
    && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin build -j$(nproc)

# Runtime-only deps: the top-level kalibr python tools `import cv_bridge`
# (ROS) and `cv2`, which are not needed at compile time (build above passes
# without them). Kept as a separate layer so the 18-minute catkin build stays
# cached when this list changes.
#
# cv_bridge is compiled from source (noetic branch of vision_opencv) instead
# of using the apt ros-noetic-cv-bridge binary: the apt boost extension fails
# to initialize on this arm64 image ("initialization of cv_bridge_boost raised
# unreported exception"), while a source build against the same libopencv /
# numpy / python3.8 initializes cleanly.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-opencv \
    python3-yaml \
    python3-wxgtk4.0 \
    && rm -rf /var/lib/apt/lists/*

COPY vision_opencv/cv_bridge /catkin_ws/src/cv_bridge
RUN cd /catkin_ws && catkin build cv_bridge --no-deps -j$(nproc)

# Work around an arm64 cv_bridge_boost quirk: its C++ module init calls
# PyImport_ImportModule("cv2") from inside the extension init, which fails
# unless cv2 has already been imported by the interpreter ("initialization of
# cv_bridge_boost raised unreported exception"). Insert `import cv2` right
# before every `import cv_bridge` in kalibr's python tool sources so cv2 is
# always loaded first. Kept in the container build, not in the kalibr repo.
COPY patch_cvbridge.py /tmp/patch_cvbridge.py
RUN python3 /tmp/patch_cvbridge.py

# Convenience entry: already-sourced shell.
CMD ["/bin/bash", "-lc", "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && exec bash"]
