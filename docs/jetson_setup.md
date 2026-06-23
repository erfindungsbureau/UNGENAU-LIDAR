# LiDAR Mapper - Jetson TX2

## Hardware
- NVIDIA Jetson TX2 (Auvidea J120 Carrier Board)
- Ouster OS1-64 LiDAR (Firmware v2.4.0)
- SBG Ellipse 2 IMU (RS-422 via FTDI USB)
- Hostname: lidar / lidar.local

## Netzwerk
- WiFi:  192.168.31.39 (DHCP, StudioFiume)
- LiDAR: 169.254.45.204 (Link-Local, Ethernet eth0)
- Jetson eth0: 169.254.45.100/16 + 192.168.2.1/24
- Web Interface: http://192.168.31.39:5000

## Starten
```
~/start_lidar.sh
```
Startet: roscore → ouster_ros → rosbridge → bridge.py → Flask

## Web Interface
Browser: http://192.168.31.39:5000
- Live 3D Punktwolke (alle 200ms)
- IMU Roll/Pitch/Yaw (wenn SBG angeschlossen)
- Drag = rotieren, Scroll = zoom

## Dienste & Logs
| Dienst       | Log               | Port  |
|--------------|-------------------|-------|
| roscore      | /tmp/roscore.log  | 11311 |
| ouster_ros   | /tmp/ouster.log   | -     |
| rosbridge    | /tmp/rosbridge.log| 9090  |
| bridge.py    | /tmp/bridge.log   | -     |
| Flask        | /tmp/flask.log    | 5000  |

## SBG IMU (wenn angeschlossen)
Port: /dev/ttyUSB0, Baud: 115200
FTDI RS-422 Pinout:
  FTDI RXD+ (Gelb)  ← SBG TX+
  FTDI RXD- (Grün)  ← SBG TX-
  FTDI TXD+ (Orange)→ SBG RX+
  FTDI TXD- (Rot)   → SBG RX-

## Einzelne Dienste manuell starten
```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Nur LiDAR
roslaunch ouster_ros sensor.launch \
    sensor_hostname:=169.254.45.204 \
    udp_dest:=169.254.45.100 \
    lidar_mode:=1024x10 viz:=false

# Topics prüfen
rostopic list
rostopic hz /ouster/points   # sollte ~10 Hz zeigen

# Flask neu starten
cd ~/lidar_viewer && python3 flask_viewer.py
```

## Rosbag Recording (Daten sammeln)
```bash
source /opt/ros/melodic/setup.bash
rosbag record -o ~/bags/scan \
    /ouster/points \
    /ouster/imu \
    /sbg/ekf_euler
# Stoppen: Ctrl+C
```

## Dateien
- ~/start_lidar.sh         - Alles starten
- ~/bridge.py              - ROS→Flask Brücke (Python2)
- ~/lidar_viewer/          - Flask Web App
- ~/catkin_ws/src/         - ROS Pakete (ouster_ros, sbg_driver)

## ROS Topics
- /ouster/points           - PointCloud2, 10 Hz
- /ouster/imu              - IMU Rohdaten
- /sbg/ekf_euler           - Orientierung (Roll/Pitch/Yaw)

## Bekannte Probleme
- LiDAR verliert IP nach Neustart → start_lidar.sh löst das
- ROS Hostname muss aufgelöst werden → 127.0.0.1 MiWiFi-CB0401-srv in /etc/hosts
- GTSAM kompiliert nicht (GCC Bug ARM64) → LIO-SAM noch nicht verfügbar
