#!/bin/bash
# LiDAR Mapper - Kompletter Start
# Aufruf: ~/start_lidar.sh

LOG_DIR=/tmp
source /opt/ros/melodic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash

echo "[$(date)] === LiDAR Mapper Start ==="

# 1. WiFi
bash /home/nvidia/wifi_ap.sh
IP=$(ip -4 addr show wlan0 | grep -oP "(?<=inet )[\d.]+" | head -1)

# 2. Alte Prozesse beenden
killall -9 rosmaster roscore python python2 python3 2>/dev/null
sleep 2

# 3. roscore
echo "[ROS] Starte roscore..."
nohup roscore > $LOG_DIR/roscore.log 2>&1 &
sleep 4

# 4. Ouster LiDAR
echo "[LiDAR] Starte ouster_ros..."
nohup roslaunch ouster_ros sensor.launch \
    sensor_hostname:=169.254.45.204 \
    udp_dest:=169.254.45.100 \
    lidar_mode:=1024x10 viz:=false \
    > $LOG_DIR/ouster.log 2>&1 &
sleep 10

# 5. SBG IMU (optional)
if [ -e /dev/ttyUSB0 ]; then
    echo "[IMU] Starte sbg_reader (direkt seriell)..."
    nohup python3 /home/nvidia/sbg_reader.py > $LOG_DIR/sbg.log 2>&1 &
    sleep 3
fi

# 6. rosbridge WebSocket
echo "[Bridge] Starte rosbridge..."
nohup roslaunch rosbridge_server rosbridge_websocket.launch \
    port:=9090 > $LOG_DIR/rosbridge.log 2>&1 &
sleep 3

# 7. ROS→Flask Brücke (Python2)
echo "[Bridge] Starte bridge.py..."
nohup python2 /home/nvidia/bridge.py > $LOG_DIR/bridge.log 2>&1 &
sleep 3

# 8. Flask Web Server
echo "[Web] Starte Flask..."
cd /home/nvidia/lidar_viewer
nohup python3 app.py > $LOG_DIR/flask.log 2>&1 &

echo ""
echo "=== LiDAR Mapper bereit ==="
echo "Web Interface: http://$IP:5000"
echo "Hotspot SSID:  LidarScanner (falls kein bekanntes Netz)"
echo "Hotspot Pass:  lidar1234"
