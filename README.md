# UNGENAU-LIDAR

Handheld-3D-LiDAR-Scanner für Innenraumaufnahmen — Ouster OS1-64 + SBG Ellipse 2 IMU,
verarbeitet auf einem Jetson TX2, bedient über ein Web-Interface vom Smartphone/Tablet
aus. Zielanwendung: Punktwolken-Export für ArchiCAD/BIM-Workflows.

## Ziel & Zweck

Ein tragbarer Scanner, der ohne Laptop bedient werden kann: einschalten, per Browser
ARM drücken, durch den Raum gehen, SCAN drücken, fertig. Die Punktwolke wird per SLAM
(Simultaneous Localization and Mapping) zu einer konsistenten 3D-Karte zusammengesetzt,
während man sich bewegt — das ist der schwierige Teil und Gegenstand laufender
Optimierung (siehe [docs/slam-staged-testing.md](docs/slam-staged-testing.md)).

## Technische Funktionsweise

Kurzfassung: Ouster-Punktwolke (2048x10) + SBG-IMU → FAST-LIO2 live als 3D-Vorschau im
Browser → `rosbag record` für die volle Aufnahme → nach dem Scan rechnet der Jetson die Bahn
mit allen Punkten neu und erzeugt ein 1-cm-Modell (`*_1cm.ply`).

Details zu ROS-Knoten, Topics, der ARM/DISARM-Zustandsmaschine und der Web-UI:
**[docs/architecture.md](docs/architecture.md)**.

## Schnellstart

```bash
ssh nvidia@<jetson-ip>
# Normalerweise bereits per systemd (lidar-system.service) automatisch gestartet.
# Manuell, falls nötig:
bash /home/nvidia/UNGENAU-LIDAR/scripts/start_lidar.sh
```

Browser öffnen: `http://<jetson-ip>:5000`

1. **ARM** drücken — LiDAR fährt hoch, FAST-LIO2 startet (~40–50s)
2. Wenn SLAM grün: **SCAN** drücken, durch den Raum gehen; die 3D-Karte oben zeigt, was
   schon erfasst ist (⛶ = Vollbild)
3. **SCAN** stoppen und **Speichern** — der Sensor geht aus, der Jetson rechnet das
   1-cm-Modell (Fortschrittsbalken; ein 5-min-Scan braucht etwa 20–25 min)
4. **↓ PLY laden** (CloudCompare, ArchiCAD) oder in der Bag-Liste: «1 cm» rechnet ältere
   Scans nach, dazu Download/USB-Kopie und die bisherigen XYZ-Exporte

## Repo-Struktur

```
UNGENAU-LIDAR/
├── webui/          Flask-Backend + Frontend (die Web-App, die auf :5000 läuft)
├── ros/            Launch-Files + Configs für Ouster und FAST-LIO2 (das, was tatsächlich läuft)
├── scripts/        Start/Watchdog/Bridge/Export-Skripte, WiFi-Verwaltung
├── docs/           Setup, Architektur, Hardware, SLAM-Testprotokoll
└── archive/        Abgelöste/abgebrochene Ansätze — Referenz, nicht aktiv genutzt
```

Das Repo liegt auf dem Jetson unter `/home/nvidia/UNGENAU-LIDAR` und ist der **einzige
Ort, an dem der Code liegt** — alle Skripte, Configs und systemd-Units referenzieren
absolute Pfade direkt ins Repo (keine Kopien, keine Symlinks). Ein `git status` auf dem
Jetson zeigt also ehrlich, was lokal geändert wurde.

`catkin_ws/src/{FAST_LIO, LIO-SAM, ouster_example, sbg_ros_driver, livox_ros_driver}`
sind separat, vendorisierte ROS-Pakete (C++-Quellcode der SLAM-Engines selbst) — die
werden über catkin gebaut und liegen bewusst außerhalb dieses Repos. Was tatsächlich
geladen/ausgeführt wird (Configs, Launch-Parameter), liegt in `ros/`.

## Frontend- / Code-Änderungen deployen

```bash
# 1. Lokal ändern, committen, pushen
git add ... && git commit -m "..." && git push origin main

# 2. Deploy auf den Jetson
bash scripts/deploy.sh nvidia@100.64.0.25
```

`deploy.sh` macht `git pull --ff-only` auf dem Jetson und startet `lidar-system.service`
neu (Flask cached Jinja2-Templates — ein reiner Browser-Refresh reicht bei
HTML/CSS/JS-Änderungen **nicht**, der Service-Restart ist nötig).

Alternativ direkt auf dem Jetson:
```bash
cd /home/nvidia/UNGENAU-LIDAR && git pull --ff-only
sudo systemctl restart lidar-system.service
```

## SLAM

**FAST-LIO2** läuft live nur als leichte Vorschau (jeder 8. Punkt, 30-cm-Karte), damit der
TX2 neben Treiber und Aufnahme 10 Hz hält. Das eigentliche Modell entsteht nach dem Scan
aus dem Bag: Bahn offline neu (`scripts/analysis/fl_replay.sh`), dann jede Rohmessung in
1-cm-Würfel (`scripts/analysis/dense_export.py`), zusammengefasst in
`scripts/analysis/process_scan.sh`. Details: [docs/architecture.md](docs/architecture.md).
LIO-SAM ist seit 15.11.2026 entfernt (degenerierte in symmetrischen Innenräumen, siehe
[docs/slam-staged-testing.md](docs/slam-staged-testing.md)).

## Weitere Doku

- [docs/jetson-setup.md](docs/jetson-setup.md) — Hardware, Netzwerk, Erstinstallation, IMU-Kalibrierung
- [docs/hardware-connections.md](docs/hardware-connections.md) — Pinouts, IPs, Topics im Detail
- [docs/architecture.md](docs/architecture.md) — Datenfluss, ARM/DISARM-State-Machine
- [docs/slam-staged-testing.md](docs/slam-staged-testing.md) — SLAM-Stufentest-Protokoll

## Export-Optionen

`scripts/bag_to_xyz_hires.py` unterstützt einen optionalen Reflectivity-Filter gegen
Geisterpunkte von Spiegeln/Glasflächen:
```bash
python2 scripts/bag_to_xyz_hires.py scan.bag output.xyz 1 "" 150
```
Letzter Parameter = maximale Reflectivity (0 = aus/Standard, 150-180 = guter Startwert
zum Testen). Punkte mit höherer Reflectivity werden verworfen.

## Bekannte Probleme

- **Scan-Überlagerung bei Bewegung** — gelöst (2026-06-30): falsche Extrinsik-Konvention in
  LIO-SAMs Config (Matrix war nicht transponiert). Details und Diagnoseweg:
  [docs/slam-staged-testing.md](docs/slam-staged-testing.md).
- **GCC 7.5/8 Segfault auf ARM64** beim Kompilieren von FAST-LIO2 — Workaround Clang-8,
  siehe [docs/jetson-setup.md](docs/jetson-setup.md).
- **GPS/Mag der SBG nicht in Innenräumen verlässlich** — EKF nutzt primär Beschleunigung
  + Gyro, Magnetometer nur als Yaw-Referenz (kann in Stahlbeton-Gebäuden gestört sein).
