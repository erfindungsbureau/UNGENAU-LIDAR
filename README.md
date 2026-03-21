# Jetson TX2 — Handheld LiDAR Scanner

A self-contained handheld LiDAR scanner based on the NVIDIA Jetson TX2, featuring real-time SLAM (LIO-SAM), a smartphone web interface, and automatic WiFi fallback.

## Hardware

| Component | Details |
|-----------|---------|
| **Compute** | NVIDIA Jetson TX2 (Ubuntu 16.04, JetPack R28.2.1, aarch64) |
| **LiDAR** | Ouster OS-1 (64-channel) — connected via `eth0` (192.168.0.x) |
| **IMU/GPS** | SBG Ellipse2 E-G4A3-M1 — connected via USB/RS-232 |
| **Storage** | NVMe SSD 460GB mounted at `/media/nvidia/SSD1` |
| **WiFi** | `wlan0` — client mode (StudioFiume) or AP mode (LidarScanner) |

## Architecture

```
[Smartphone] ──WiFi──► [Jetson TX2 Hotspot 10.42.0.1]
                               │
                      ┌────────┴────────┐
                      │  Flask App :5000 │  ← Start/Stop/Status/Preview
                      └────────┬────────┘
                               │
                      ┌────────┴────────────────────┐
                      │  ROS Noetic (RoboStack/conda) │
                      │  ├── ouster_ros               │
                      │  ├── sbg_ros_driver           │
                      │  └── LIO-SAM                  │
                      └─┬──────────┬────────────────-─┘
                        │          │
                    eth0:       USB/RS-232:
                Ouster OS-1    SBG Ellipse2

Bags + Maps → /media/nvidia/SSD1/lidar_data/
SMB Share   → \\10.42.0.1\lidar_data
```

## Features

- **Web Interface** (`:5000`) — Start/stop recording, live status, bag file list with download
- **ROS Noetic** via RoboStack (conda) — runs on Ubuntu 16.04 without Docker
- **LIO-SAM SLAM** — tightly-coupled LiDAR-IMU odometry and mapping
- **WiFi Fallback** — connects to StudioFiume when available, creates `LidarScanner` hotspot otherwise
- **SMB Share** — scan data directly accessible from any PC/Mac at `\\10.42.0.1\lidar_data`
- **Autostart** — all services start on boot via systemd

## Quick Start

1. Power on Jetson TX2
2. Connect smartphone to WiFi `LidarScanner` (password: `lidar2024`) — or stay on StudioFiume
3. Open browser: `http://10.42.0.1:5000` (hotspot) or `http://192.168.31.39:5000` (LAN)
4. Press **"ROS starten"** → wait ~5s
5. Press **"▶ Start"** to begin recording
6. Press **"■ Stop"** when done
7. Download `.bag` file from the list, or access via SMB: `smb://10.42.0.1/lidar_data`

## Repository Structure

```
jetson-lidar-scanner/
├── README.md                    — This file
├── docs/
│   ├── setup-jetson.md          — Full Jetson TX2 setup guide
│   ├── setup-ros.md             — ROS Noetic / RoboStack build guide
│   └── hardware-connections.md  — Wiring and IP configuration
├── webui/
│   ├── app.py                   — Flask web application (Python 3.5 compatible)
│   └── templates/
│       └── index.html           — Mobile-optimized web UI
├── config/
│   ├── smb.conf                 — Samba share configuration
│   ├── wifi-fallback.sh         — WiFi client/hotspot fallback script
│   ├── lidar-hotspot.service    — systemd service for WiFi management
│   ├── lidar-hotspot.timer      — systemd timer (runs every 5 min)
│   └── lidar-webui.service      — systemd service for Flask app
└── scripts/
    ├── install-robostack.sh     — RoboStack (ROS Noetic via conda) installer
    └── deploy.sh                — Deploy all configs to Jetson via SSH
```

## Installed Services (Jetson TX2)

| Service | Status | Description |
|---------|--------|-------------|
| `lidar-webui.service` | enabled, auto-start | Flask web interface on :5000 |
| `lidar-hotspot.timer` | enabled, every 5min | WiFi client/hotspot fallback |
| `smbd` / `nmbd` | enabled, auto-start | Samba share for scan data |
| LightDM | **disabled** | Saves ~400MB RAM |

## Data Storage

All scan data is stored on the NVMe SSD:

```
/media/nvidia/SSD1/
├── lidar_data/
│   ├── bags/     ← ROS bag recordings (scan_YYYYMMDD_HHMMSS.bag)
│   └── maps/     ← LIO-SAM map output
└── docker/       ← (reserved, unused — Docker blocked by custom kernel)
```

SSD is auto-mounted via `/etc/fstab`:
```
UUID=7fe9b4be-c632-4680-8135-cf717a970383  /media/nvidia/SSD1  ext4  defaults,nofail,x-systemd.device-timeout=5  0  2
```

## Known Limitations

- **Docker not functional**: Custom kernel `4.4.38-jetsonbotv0.1` has `CONFIG_CGROUP_DEVICE=n` → Docker crashes on start. ROS Noetic is installed via RoboStack (conda) instead.
- **Slow internet on Jetson**: WiFi connection is ~7KB/s. RoboStack packages were downloaded on rosbot (fast internet) and synced via LAN rsync.
- **Python 3.5**: Ubuntu 16.04 ships Python 3.5 — Flask app avoids f-strings and Python 3.6+ features.
- **LiDAR preview**: The `/api/snapshot` endpoint shows a placeholder grid (ROS offline indicator). Full point cloud visualization requires a running ROS stack.

## Network

| Mode | IP | Access |
|------|----|--------|
| LAN (StudioFiume) | `192.168.31.39` | `http://192.168.31.39:5000` |
| Hotspot (field use) | `10.42.0.1` | `http://10.42.0.1:5000` |
| WiFi SSID | `LidarScanner` | Password: `lidar2024` |
| SMB (LAN) | `\\192.168.31.39\lidar_data` | Guest access |
| SMB (hotspot) | `\\10.42.0.1\lidar_data` | Guest access |

## rosbot (Intel NUC i7-7500U)

The `rosbot` machine at `192.168.31.238` serves as:
- Development workstation for this project
- Conda package mirror (downloaded RoboStack aarch64 packages for the Jetson)
- SSH access point for Jetson management

---

*Studio Fiume, Hardturmstrasse 132a, 8005 Zürich*
