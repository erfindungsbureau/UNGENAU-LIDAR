#!/bin/bash
# fl_replay.sh BAG OUT_BAG [PFN=3] [RATE=0.5] -- FAST-LIO2 offline neu rechnen, auf eigenem ROS-Master (stoert Live-Betrieb nicht)
set -e
BAG=$1; OUT=$2; PFN=${3:-3}; RATE=${4:-0.5}
source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11399
roscore -p 11399 >/dev/null 2>&1 & RC=$!; sleep 5
roslaunch $(dirname $0)/fl_offline.launch est:=false pfn:=$PFN >/tmp/fl_replay_node.log 2>&1 & FL=$!; sleep 6
rosbag record -O $OUT /Odometry /replay/imu >/dev/null 2>&1 & RE=$!; sleep 2
# PLAY_LOG: Fortschritt von rosbag play ("Duration: a / b") fuer den Balken im Web-UI
rosbag play -r $RATE $BAG /ouster/points:=/replay/points /imu/data:=/replay/imu --topics /ouster/points /imu/data </dev/null >${PLAY_LOG:-/dev/null} 2>&1
sleep 20   # Rest abarbeiten
kill -INT $RE; sleep 3; kill -INT $FL; sleep 3; kill $RC; wait 2>/dev/null || true
