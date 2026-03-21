# Hardware Connections

## Overview

```
┌─────────────────────────────────────────────────────┐
│                  Jetson TX2                          │
│                                                      │
│  eth0 ─────────────────────────────► Ouster OS-1    │
│  (192.168.0.1/24)         (192.168.0.100)           │
│                                                      │
│  USB/RS-232 (/dev/ttyUSB0)─────────► SBG Ellipse2   │
│                                                      │
│  wlan0 ────────────────────────────► WiFi Router     │
│  (192.168.31.39 / 10.42.0.1)        (StudioFiume)   │
│                                                      │
│  NVMe PCIe ────────────────────────► 460GB SSD       │
│  (/dev/nvme0n1 → /media/nvidia/SSD1)                │
└─────────────────────────────────────────────────────┘
```

## Ouster OS-1 LiDAR

| | |
|---|---|
| **Interface** | Ethernet (`eth0`) |
| **LiDAR IP** | `192.168.0.100` (static) |
| **Jetson IP** | `192.168.0.1` (static, set on eth0) |
| **Channels** | 64 |
| **Default lidar_mode** | 1024x10 (10Hz, 1024 horizontal points) |
| **ROS topic (points)** | `/os_cloud_node/points` |
| **ROS topic (IMU)** | `/os_cloud_node/imu` |

### Configure eth0 (one-time)
```bash
sudo nmcli con add type ethernet ifname eth0 con-name "ouster-lidar" \
  ipv4.method manual ipv4.addresses 192.168.0.1/24
sudo nmcli con up "ouster-lidar"
```

### Verify Connection
```bash
ping 192.168.0.100   # Should respond
curl http://192.168.0.100/api/v1/sensor/metadata  # Ouster HTTP API
```

## SBG Ellipse2 IMU (E-G4A3-M1)

| | |
|---|---|
| **Interface** | USB→RS-232 adapter (`/dev/ttyUSB0`) |
| **Baud Rate** | 115200 (default) |
| **Output Rate** | 200 Hz |
| **ROS topic (IMU)** | `/imu/data` |
| **ROS topic (GPS)** | `/gps/fix` |
| **ROS topic (mag)** | `/imu/mag` |

### Verify Connection
```bash
ls /dev/ttyUSB*      # Should show /dev/ttyUSB0
# Add user to dialout group if permission denied:
sudo usermod -a -G dialout nvidia
```

### SBG ROS Driver Config
Location: `~/ros_ws/src/sbg_ros_driver/config/sbg_device_uart_default.yaml`
```yaml
# Minimal config
driver:
  uart:
    port: "/dev/ttyUSB0"
    baudRate: 115200
  timeReference: "ros"
output:
  useRos: true
  # IMU output at 200Hz
```

## NVMe SSD

| | |
|---|---|
| **Device** | `/dev/nvme0n1` |
| **Size** | 460GB (Micron) |
| **Filesystem** | ext4 |
| **Label** | SSD |
| **UUID** | `7fe9b4be-c632-4680-8135-cf717a970383` |
| **Mount** | `/media/nvidia/SSD1` |
| **Free Space** | ~109GB |

### Existing Data on SSD
The SSD contains historical bag files from drone flights (2019-2020):
```
/media/nvidia/SSD1/
├── lidar_data/          ← New: handheld scanner data
│   ├── bags/
│   └── maps/
├── Compare_OS1_Velodyne/
├── Data_Aq_*/           ← Historical drone acquisitions
└── Test_Flights_*/      ← Historical test flights
```

## Power

The Jetson TX2 draws ~7.5W idle, ~15W under load.
Recommended: USB-C PD 65W or dedicated DC barrel jack (9-20V).

For field use, a 20000mAh USB-C PD powerbank provides ~4-6 hours of scanning.
