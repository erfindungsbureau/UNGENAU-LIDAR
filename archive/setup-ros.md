# ROS Noetic Setup via RoboStack

## Why RoboStack instead of Docker?

The Jetson TX2 runs a custom kernel (`4.4.38-jetsonbotv0.1`) with `CONFIG_CGROUP_DEVICE` disabled.
Docker requires the `devices` cgroup subsystem and refuses to start. RoboStack provides
ROS Noetic as conda packages — no kernel requirements, works natively on Ubuntu 16.04.

## Step 1: Install Miniforge (ARM64)

```bash
# On Jetson TX2
bash ~/Miniforge3-aarch64.sh -b -p ~/miniforge3
source ~/miniforge3/etc/profile.d/conda.sh

# Add to .bashrc
echo 'source $HOME/miniforge3/etc/profile.d/conda.sh' >> ~/.bashrc
```

Or use the provided installer script:
```bash
bash ~/install-robostack.sh
```

**Note on slow internet**: The Jetson has ~7KB/s internet via WiFi. Pre-download packages
from a fast machine (see "Offline Package Transfer" below).

## Step 2: Create ROS Noetic Environment

```bash
source ~/miniforge3/etc/profile.d/conda.sh

conda config --add channels conda-forge
conda config --add channels robostack-staging
conda config --set channel_priority strict

mamba create -n ros-noetic \
  ros-noetic-ros-base \
  ros-noetic-rosbag \
  ros-noetic-tf2-ros \
  ros-noetic-sensor-msgs \
  ros-noetic-nav-msgs \
  ros-noetic-geometry-msgs \
  compilers cmake pkg-config make ninja colcon-common-extensions \
  -c robostack-staging -c conda-forge

conda activate ros-noetic
```

## Offline Package Transfer (Recommended)

If the Jetson's internet is slow, download packages on a fast x86_64 machine first:

```bash
# On rosbot (fast internet, x86_64)
source ~/miniforge3/etc/profile.d/conda.sh
conda config --add channels robostack-staging

# Download linux-aarch64 packages without installing
CONDA_SUBDIR=linux-aarch64 mamba create -n ros-noetic-dl --platform linux-aarch64 \
  ros-noetic-ros-base ros-noetic-rosbag ros-noetic-tf2-ros \
  ros-noetic-sensor-msgs ros-noetic-nav-msgs ros-noetic-geometry-msgs \
  compilers cmake pkg-config make ninja colcon-common-extensions \
  -c robostack-staging -c conda-forge --yes

# Sync packages to Jetson (LAN is fast, ~100Mb/s)
rsync -av --ignore-existing \
  -e "sshpass -p nvidia ssh -o StrictHostKeyChecking=no" \
  ~/miniforge3/pkgs/*.conda \
  nvidia@192.168.31.39:~/miniforge3/pkgs/

# Then on Jetson: install from local cache
mamba create -n ros-noetic ... --offline
```

## Step 3: Build ROS Workspace

```bash
conda activate ros-noetic

mkdir -p ~/ros_ws/src && cd ~/ros_ws/src

# Ouster LiDAR driver
git clone --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git
cd ouster-ros && git checkout ros1 && cd ..

# SBG IMU driver
git clone https://github.com/SBGSystems/sbg_ros_driver.git
cd sbg_ros_driver && git checkout ros1 && cd ..

# LIO-SAM SLAM
git clone https://github.com/TixiaoShan/LIO-SAM.git

# Install dependencies
cd ~/ros_ws
rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build (use -j2 to avoid OOM on Jetson TX2)
catkin_make -DCMAKE_BUILD_TYPE=Release -j2

source ~/ros_ws/devel/setup.bash
```

## Step 4: Configure LIO-SAM

Edit `~/ros_ws/src/LIO-SAM/config/params.yaml`:

```yaml
# LiDAR (Ouster OS-1, 64-channel)
pointCloudTopic: "/os_cloud_node/points"
N_SCAN: 64
Horizon_SCAN: 1024
lidarMinRange: 0.5
lidarMaxRange: 100.0

# IMU (SBG Ellipse2)
imuTopic: "/imu/data"
imuFrequency: 200

# GPS
useImuHeadingInitialization: false
useGpsElevation: false
gpsTopic: "/gps/fix"
```

## Running the Full Stack

```bash
conda activate ros-noetic
source ~/ros_ws/devel/setup.bash

# Terminal 1: ROS Master
roscore

# Terminal 2: Ouster LiDAR
roslaunch ouster_ros driver.launch \
  sensor_hostname:=192.168.0.100 \
  lidar_mode:=1024x10

# Terminal 3: SBG Ellipse2 IMU
roslaunch sbg_driver sbg_device_uart_default.launch

# Terminal 4: LIO-SAM
roslaunch lio_sam run.launch

# Terminal 5: rosbag recording
rosbag record -O /media/nvidia/SSD1/lidar_data/bags/$(date +%Y%m%d_%H%M%S).bag \
  /imu/data /os_cloud_node/points /os_cloud_node/imu /gps/fix /tf /tf_static
```

Or use the web interface at `http://192.168.31.39:5000` to control recording.

## Add to .bashrc for Auto-Activation

```bash
echo 'source $HOME/miniforge3/etc/profile.d/conda.sh' >> ~/.bashrc
echo 'conda activate ros-noetic' >> ~/.bashrc
echo 'source $HOME/ros_ws/devel/setup.bash 2>/dev/null' >> ~/.bashrc
```
