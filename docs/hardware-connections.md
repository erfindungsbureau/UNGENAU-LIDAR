# Hardware Connections

## Overview

```
┌──────────────────────────────────────────────────────────┐
│                      Jetson TX2                            │
│                                                              │
│  eth0 ───────────────────────────────► Ouster OS1-64        │
│  (192.168.2.1/24)              (192.168.2.2)                │
│                                                              │
│  USB→RS422 (/dev/ttyUSB0) ───────────► SBG Ellipse 2         │
│                                                              │
│  wlan0 ───────────────────────────────► WiFi Router          │
│  (DHCP / Hotspot-Fallback 10.42.0.1)                         │
│                                                              │
│  NVMe PCIe ────────────────────────────► 460GB SSD            │
│  (/dev/nvme0n1 → /media/nvidia/SSD)                          │
└──────────────────────────────────────────────────────────┘
```

## Ouster OS1-64 LiDAR

| | |
|---|---|
| **Interface** | Ethernet (`eth0`) |
| **LiDAR IP** | `192.168.2.2` (static) |
| **Jetson IP** | `192.168.2.1` (static, eth0) |
| **Channels** | 64 |
| **lidar_mode** | `1024x10` (10 Hz, 1024 Punkte/Umdrehung) |
| **timestamp_mode** | `TIME_FROM_ROS_TIME` |
| **ROS topic (Punkte)** | `/ouster/points` (PointCloud2, Felder: x,y,z,intensity,t,reflectivity,ring,ambient,range) |
| **ROS topic (IMU, intern)** | `/ouster/imu` (sensor_msgs/Imu — eigene IMU im Sensor, separat von SBG) |
| **TCP-Konfig-Port** | `7501` (für `set_config_param`, `reinitialize`, genutzt von Flask ARM/DISARM) |

### Verbindung prüfen
```bash
ping 192.168.2.2
rostopic hz /ouster/points   # sollte ~10 Hz zeigen
```

## SBG Ellipse 2 IMU (E-G4A3-M1)

| | |
|---|---|
| **Interface** | USB→RS-422 Adapter (`/dev/ttyUSB0`) |
| **Baud Rate** | 115200 |
| **ROS-Treiber** | `sbg_driver` (offiziell, nicht der alte direkte serielle Parser) |
| **Rohtopics** | `/sbg/imu_data`, `/sbg/ekf_quat`, `/sbg/ekf_euler` |
| **Kombiniertes Topic** | `/imu/data` (via `scripts/sbg_imu_combiner.py`, ENU + EKF-Quaternion) |

### Verbindung prüfen
```bash
ls /dev/ttyUSB*
sudo usermod -a -G dialout nvidia   # falls Permission denied (Neuanmeldung nötig)
```

Config: `ros/config/sbg_device_uart_default.yaml` (`use_enu: true`, `ros_standard: true`).

## NVMe SSD

| | |
|---|---|
| **Device** | `/dev/nvme0n1` |
| **Size** | 460GB |
| **Filesystem** | ext4 |
| **Label / Mount** | `SSD` → `/media/nvidia/SSD` |

```
/media/nvidia/SSD/lidar_data/
├── bags/        ← rosbag-Aufnahmen aus der Web-UI (SCAN-Button)
└── maps/        ← (optional) gespeicherte PCD-Karten
```

Der Mount enthält zusätzlich historische Daten aus früheren Drohnenflügen (2019-2020,
`Data_Aq_*`, `Test_Flights_*`) — nicht Teil dieses Projekts, aber auf derselben SSD.

## Stromversorgung

Jetson TX2: ~7.5W idle, ~15W unter Last. Empfohlen: USB-C PD 65W oder DC-Buchse (9–20V).
Für Feldeinsatz: 20000mAh USB-C-PD-Powerbank reicht für ca. 4–6h Scanbetrieb.
