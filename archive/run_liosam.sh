#!/bin/bash
# LIO-SAM starten für Ouster OS1-64 + Jetson TX2
# Aufruf: ./run_liosam.sh [bag_file.bag]

set -e

# Jetson auf maximale Performance
sudo nvpmodel -m 0 2>/dev/null && echo "nvpmodel: Max-Performance" || echo "nvpmodel nicht verfügbar"
sudo jetson_clocks 2>/dev/null && echo "jetson_clocks: gesetzt" || echo "jetson_clocks nicht verfügbar"

source /opt/ros/melodic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash

# Maps-Verzeichnis sicherstellen
mkdir -p /media/nvidia/SSD/lidar_data/maps

# Wenn ein Bag-File übergeben wurde: playback
if [ -n "$1" ] && [ -f "$1" ]; then
    echo "Playback-Modus: $1"
    roslaunch lio_sam run.launch &
    LAUNCH_PID=$!
    sleep 5
    rosbag play "$1" --clock -r 0.5
    wait $LAUNCH_PID
else
    echo "Live-Modus: LIO-SAM wartet auf /ouster/points + /ouster/imu"
    roslaunch lio_sam run.launch
fi
