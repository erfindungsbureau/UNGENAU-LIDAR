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
bash /home/nvidia/UNGENAU-LIDAR/scripts/wifi_ap.sh >> $LOG_DIR/lidar_start.log 2>&1
IP=$(ip -4 addr show wlan0 | grep -oP "(?<=inet )[\d.]+" | head -1)

# 2. Alte Prozesse beenden
killall -9 rosmaster roscore python python2 python3 2>/dev/null
sleep 2

# 3. roscore
echo "[ROS] roscore..." | tee -a $LOG_DIR/lidar_start.log
nohup roscore > $LOG_DIR/roscore.log 2>&1 &
sleep 5

# 4. Ouster LiDAR: wird von Flask ARM-System gestartet, hier NICHT starten
echo "[LiDAR] ouster_ros wird per ARM gestartet — übersprungen" | tee -a $LOG_DIR/lidar_start.log

# 5. SBG IMU Reader + NED->ENU Konverter
if [ -e /dev/sbg ]; then
    echo "[IMU] sbg_driver (EKF+ENU) -> /imu/data ..." | tee -a $LOG_DIR/lidar_start.log
    nohup bash -c "source /opt/ros/melodic/setup.bash && source /home/nvidia/catkin_ws/devel/setup.bash && roslaunch /home/nvidia/UNGENAU-LIDAR/ros/launch/sbg_device.launch" \
        > $LOG_DIR/sbg.log 2>&1 &
    echo "[IMU] sbg_imu_combiner -> /imu/data (EKF+ENU) ..." | tee -a $LOG_DIR/lidar_start.log
    nohup bash -c "source /opt/ros/melodic/setup.bash && source /home/nvidia/catkin_ws/devel/setup.bash && python /home/nvidia/UNGENAU-LIDAR/scripts/sbg_imu_combiner.py" \
        > $LOG_DIR/sbg_combiner.log 2>&1 &
    sleep 5
fi

# 6. LIO-SAM SLAM: wird von Flask ARM-System gestartet, hier NICHT starten
echo "[SLAM] SLAM wird per ARM gestartet — übersprungen" | tee -a $LOG_DIR/lidar_start.log

# 7. rosbridge WebSocket
echo "[Bridge] rosbridge..." | tee -a $LOG_DIR/lidar_start.log
nohup roslaunch rosbridge_server rosbridge_websocket.launch \
    port:=9090 > $LOG_DIR/rosbridge.log 2>&1 &
sleep 3

# 7. ROS→Flask Brücke
echo "[Bridge] bridge.py..." | tee -a $LOG_DIR/lidar_start.log
nohup bash -c "source /opt/ros/melodic/setup.bash && source /home/nvidia/catkin_ws/devel/setup.bash && python2 /home/nvidia/UNGENAU-LIDAR/scripts/bridge.py" > $LOG_DIR/bridge.log 2>&1 &
sleep 2

# 8. Flask Web Server
echo "[Web] Flask..." | tee -a $LOG_DIR/lidar_start.log
fuser -k 5000/tcp 2>/dev/null
sleep 1
cd /home/nvidia/UNGENAU-LIDAR/webui
nohup python3 app.py > $LOG_DIR/flask.log 2>&1 &

echo "" | tee -a $LOG_DIR/lidar_start.log
pgrep -f watchdog.sh > /dev/null || nohup /home/nvidia/UNGENAU-LIDAR/scripts/watchdog.sh > /tmp/watchdog.log 2>&1 &

echo "=== Bereit ===" | tee -a $LOG_DIR/lidar_start.log
echo "Web: http://${IP:-10.42.0.1}:5000" | tee -a $LOG_DIR/lidar_start.log
