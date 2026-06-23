# UNGENAU LiDAR Scanner

Handgehaltener 3D-Scanner auf Basis NVIDIA Jetson TX2 mit Ouster OS1-64 LiDAR.
Live-Vorschau und Scan-Aufnahme über Smartphone-Browser.

## Hardware
- NVIDIA Jetson TX2 (Auvidea J120)
- Ouster OS1-64 LiDAR (Firmware v2.4.0)
- SBG Ellipse 2 IMU (RS-422 / FTDI USB)
- 460GB NVMe SSD (Scandaten)

## System
- JetPack 4.6 / Ubuntu 18.04 / L4T R32.6.1
- ROS Melodic (apt, nicht Conda)
- Hostname: lidar / lidar.local
- Web Interface: http://192.168.31.39:5000
- SMB Share: smb://192.168.31.39/lidar_data

## Starten
```bash
~/start_lidar.sh
```

## WiFi
- Bekanntes Netz (StudioFiume): verbindet als Client → http://192.168.31.39:5000
- Kein bekanntes Netz: erstellt Hotspot **LidarScanner** → http://10.42.0.1:5000
- Passwort Hotspot: `lidar1234`

## SMB Dateitransfer
- Mac/Windows: smb://192.168.31.39/lidar_data
- User: nvidia / Pass: nvidia
- Ordner: bags/ (Rohdaten), maps/ (SLAM), exports/ (E57/ArchiCAD)

## Struktur
```
scripts/
  start_lidar.sh    Alles starten
  wifi_ap.sh        WiFi / Hotspot Logik
  bridge.py         ROS→Flask Brücke (Python2, downsampled points)
webui/
  app.py            Flask Backend (Recording, Status, Bags)
  templates/        HTML Frontend (Three.js 3D Viewer)
  static/           JS Libraries (lokal, offline-fähig)
config/
  smb.conf          Samba Konfiguration
docs/
  setup-jetson.md   Jetson Setup (JetPack 4.6)
  setup-ros.md      ROS Melodic Installation
  hardware-connections.md  Verkabelung
```

## API Endpoints
| Endpoint | Methode | Beschreibung |
|---|---|---|
| /api/status | GET | ROS, LiDAR, IMU, Disk Status |
| /api/points | GET | Live Punktwolke (downsampled JSON) |
| /api/record/start | POST | Rosbag Recording starten |
| /api/record/stop | POST | Recording stoppen |
| /api/bags | GET | Gespeicherte Bags auflisten |
| /api/download/<name> | GET | Bag herunterladen |
