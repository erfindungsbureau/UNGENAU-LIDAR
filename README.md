# UNGENAU LiDAR Scanner

Handgehaltener 3D-Scanner für den mobilen Einsatz bei Bestandsaufnahmen von Gebäuden. Der LiDAR kann mittels SLAM frei im Raum bewegt werden — die Bewegung wird durch internen (Ouster) und externen IMU (SBG Ellipse 2) kompensiert für eine zuverlässige Punktwolke, die anschliessend in ArchiCAD verwendet wird.

## Hardware

| Komponente | Modell |
|---|---|
| LiDAR | Ouster OS1-64 Gen 1 (Firmware v2.4.0) |
| IMU | SBG Systems Ellipse 2 (RS-422, Kabel CA-ELI-M-OW) |
| Recheneinheit | NVIDIA Jetson TX2, Auvidea J120 Carrier Board, 500 GB NVMe SSD |
| IMU-Adapter | FTDI TTL-232R-422 (USB→RS-422) |
| Montage | Selfiestick mit ~5 m Verlängerungskabeln für IMU und LiDAR |

## Funktionsweise

Beim Einschalten des Jetson starten alle notwendigen Programmteile automatisch.

**Steuerung über Netzwerk:**
- Bekanntes WLAN erreichbar: Jetson verbindet sich automatisch → Web Interface unter der zugewiesenen IP
- Kein bekanntes WLAN: Jetson erstellt Hotspot mit Captive Portal → [http://10.42.0.1:5000](http://10.42.0.1:5000)

**Control Panel (Webseite):**
- Scans starten und stoppen
- Scanliste mit Datum und Dateigrösse
- Konvertierung in XYZ / E57 für ArchiCAD
- Download auf Mobilgerät (Browser) oder per SMB
- Live-Vorschau der LiDAR-Punktwolke mit SLAM-Aufbau *(sekundär)*

**Prinzip:** Die gesamte SLAM-Berechnung läuft auf dem Jetson im Hintergrund — Fokus auf Effizienz und Genauigkeit. Die Webdarstellung ist bewusst sekundär.

## WiFi Konfiguration

Bekannte Netzwerke und Hotspot-Einstellungen werden in `config/wifi.conf` verwaltet:

```ini
[known_networks]
MeinWLAN|passwort
BueroWLAN|passwort
iPhoneHotspot|passwort

[hotspot]
SSID=LidarScanner
PASSWORD=lidar1234
```

Der Jetson verbindet sich automatisch mit dem **stärksten verfügbaren** Netz aus der Liste. Es können beliebig viele Netzwerke eingetragen werden (zuhause, büro, iPhone-Hotspot etc.).

## System

- JetPack 4.6 / Ubuntu 18.04 / L4T R32.6.1
- ROS Melodic
- LIO-SAM SLAM (imuPreintegration, imageProjection, featureExtraction, mapOptimization)
- imu_filter_madgwick (gain 1.0) für IMU-Fusion
- Hostname: `lidar` / `lidar.local`

## Netzwerk & Zugang

| | Adresse |
|---|---|
| WiFi Client | http://\<IP\>:5000 (IP aus Router / `lidar.local:5000`) |
| Hotspot (LidarScanner) | http://10.42.0.1:5000 |
| LiDAR (statisch, eth0) | 192.168.2.2 |
| Jetson eth0 | 192.168.2.1 |
| SMB Dateitransfer | smb://\<IP\>/lidar_data (User: nvidia / Pass: nvidia) |

## Starten

```bash
~/start_lidar.sh
```

## Dateistruktur

```
config/
  wifi.conf               WLAN-Netzwerke und Hotspot-Einstellungen  ← hier anpassen
  slam.launch             LIO-SAM + imu_filter Konfiguration
  smb.conf                Samba
scripts/
  start_lidar.sh          Alles starten (roscore, ouster, sbg, slam, flask)
  wifi_ap.sh              WiFi / Hotspot Logik (liest wifi.conf)
  bridge.py               ROS→Flask Brücke (Python 2)
  bag_to_xyz_hires.py     HD-Export: Odometrie-Poses + raw Ouster-Punkte
webui/
  app.py                  Flask Backend (Recording, Status, Export, USB)
  templates/              HTML Frontend (Steuerung, Vorschau)
docs/
  setup-jetson.md         Jetson Setup (JetPack 4.6, USB-Fix, DTB)
  hardware-connections.md Verkabelung
```

## API

| Endpoint | Methode | Beschreibung |
|---|---|---|
| /api/status | GET | LiDAR, IMU, SLAM, Disk Status |
| /api/record/start | POST | Rosbag Recording starten |
| /api/record/stop | POST | Recording stoppen |
| /api/bags | GET | Gespeicherte Scans auflisten |
| /api/bags/\<name\>/export | POST | XYZ Export (Standard) |
| /api/bags/\<name\>/export_hires | POST | HD Export (Odometrie + raw Punkte) |
| /api/export/progress | GET | Exportfortschritt (Dateigrösse) |
| /api/lidar/on | POST | LiDAR einschalten (NORMAL) |
| /api/lidar/off | POST | LiDAR ausschalten (STANDBY) |
| /api/usb/status | GET | USB-Stick erkennen |
| /api/usb/copy/\<name\> | POST | Datei auf USB-Stick kopieren |
| /api/processes | GET | Laufende Konvertierungsprozesse |
| /api/processes/\<pid\>/kill | POST | Prozess abbrechen |
