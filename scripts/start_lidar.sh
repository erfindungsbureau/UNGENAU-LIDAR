#!/bin/bash
# LiDAR Scanner - System Start
# Läuft manuell (~/start_lidar.sh) und via systemd (lidar-system.service)

LOG_DIR=/tmp

# ROS Environment (nötig für systemd, schadet nicht im Terminal)
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
source /opt/ros/melodic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash

echo "[$(date)] === LiDAR System Start ===" | tee $LOG_DIR/lidar_start.log

# 1. WiFi / Hotspot
bash /home/nvidia/wifi_ap.sh >> $LOG_DIR/lidar_start.log 2>&1
IP=$(ip -4 addr show wlan0 | grep -oP "(?<=inet )[\d.]+" | head -1)

# 2. Alte Prozesse beenden
killall -9 rosmaster roscore python python2 python3 2>/dev/null
sleep 2

# 3. roscore
echo "[ROS] roscore..." | tee -a $LOG_DIR/lidar_start.log
nohup roscore > $LOG_DIR/roscore.log 2>&1 &
sleep 5

# 4. Ouster LiDAR
echo "[LiDAR] ouster_ros..." | tee -a $LOG_DIR/lidar_start.log
nohup roslaunch ouster_ros sensor.launch \
    sensor_hostname:=169.254.45.204 \
    udp_dest:=169.254.45.100 \
    lidar_mode:=1024x10 viz:=false \
    > $LOG_DIR/ouster.log 2>&1 &
sleep 12

# 5. SBG IMU
if [ -e /dev/ttyUSB0 ]; then
    echo "[IMU] sbg_reader..." | tee -a $LOG_DIR/lidar_start.log
    pkill -f sbg_reader 2>/dev/null
    nohup python3 /home/nvidia/sbg_reader.py > $LOG_DIR/sbg.log 2>&1 &
    sleep 2
fi

# 6. rosbridge WebSocket
echo "[Bridge] rosbridge..." | tee -a $LOG_DIR/lidar_start.log
nohup roslaunch rosbridge_server rosbridge_websocket.launch \
    port:=9090 > $LOG_DIR/rosbridge.log 2>&1 &
sleep 3

# 7. ROS→Flask Brücke (Python2)
echo "[Bridge] bridge.py..." | tee -a $LOG_DIR/lidar_start.log
nohup python2 /home/nvidia/bridge.py > $LOG_DIR/bridge.log 2>&1 &
sleep 2

# 8. Flask Web Server
echo "[Web] Flask..." | tee -a $LOG_DIR/lidar_start.log
fuser -k 5000/tcp 2>/dev/null
sleep 1
cd /home/nvidia/lidar_viewer
nohup python3 app.py > $LOG_DIR/flask.log 2>&1 &

echo "" | tee -a $LOG_DIR/lidar_start.log
echo "=== Bereit ===" | tee -a $LOG_DIR/lidar_start.log
echo "Web: http://${IP:-10.42.0.1}:5000" | tee -a $LOG_DIR/lidar_start.log
