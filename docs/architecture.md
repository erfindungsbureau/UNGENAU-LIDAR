# Architektur

## Überblick: Datenfluss

```
┌─────────────┐     /ouster/points      ┌──────────────┐
│ Ouster OS1-64│ ───────────────────────►│              │
│ (ouster_ros) │     /ouster/imu         │  FAST-LIO2   │──► /Odometry, /cloud_registered
│  2048x10     │ ───────────────────────►│ (live, leicht│
└─────────────┘                          │  = Vorschau) │
┌─────────────┐    /sbg/imu_data         │              │
│ SBG Ellipse 2│ ──┐  /sbg/ekf_quat       └──────┬───────┘
│ (sbg_driver) │   ├──────────────►┌──────────┐  │
└─────────────┘   │                │ sbg_imu_  │  │
                   └───────────────►│ combiner  │──┘ /imu/data
                                     │  .py      │
                                     └──────────┘
                                              │
                     ┌────────────────────────┴───────────────┐
                     │                                          │
              ┌──────▼──────┐                            ┌──────▼──────┐
              │  bridge.py   │  schreibt /tmp/*             │  rosbag      │
              │  (Python2)   │                              │  record      │ → bags/*.bag
              └──────┬──────┘                            └──────┬──────┘
                     │ /tmp/map_3d.bin + map_3d.json,            │ nach SCAN + Speichern
                     │ /tmp/imu.json, /tmp/bridge_status.json     ▼
                     │                                     process_scan.sh → bags/*_1cm.ply
              ┌──────▼──────┐
              │  Flask app   │  webui/app.py — ARM/DISARM, SCAN, Export, USB-Copy
              │  (Port 5000) │
              └─────────────┘
```

**Warum `bridge.py` als separate Python2-Brücke?** Die ROS-Melodic-Python-Bindings sind
Python 2; Flask läuft unter Python 3. `bridge.py` abonniert die ROS-Topics, schreibt
Status, 3D-Vorschau und IMU nach `/tmp/`, und Flask liest diese Dateien — kein direkter
ROS-Import in Flask nötig. Die rohen `/ouster/points` abonniert `bridge.py` bewusst nicht:
bei 2048x10 sind das 6 MB pro Nachricht, das Entpacken in Python kostete den TX2 zu viel.
`lidar_ok` kommt deshalb von `/ouster/imu` (gleicher Treiber).

## SLAM: live Vorschau, Modell nachher (seit 15.11.2026)

Der Ouster nimmt mit **2048x10** auf (doppelte Punktdichte). Der TX2 schafft live nicht
gleichzeitig Treiber, Aufnahme und FAST-LIO2 mit allen Punkten (Test 15.11.: Bahn bis 85 s
hinterher). Deshalb zwei Stufen:

1. **Live** (`ros/launch/slam_fast_lio_sbg.launch`): FAST-LIO2 nur als Vorschau, jeder 8. Punkt
   (`point_filter_num 8`), Karte 30 cm (`filter_size_surf/map 0.3`). Hält 10 Hz.
2. **Nach dem Scan** (`scripts/analysis/process_scan.sh BAG`, vom Web-UI nach «Speichern»
   gestartet, Sensor ist dann aus):
   - `fl_replay.sh`: FAST-LIO2 offline auf eigenem ROS-Master (Port 11399) mit jedem 3. Punkt,
     halbe Abspielgeschwindigkeit → `lidar_data/work/<scan>_poses.bag` (/Odometry, /replay/imu).
     Dauer etwa 2× Scandauer.
   - `dense_export.py`: jeder Rohpunkt (0.8–10 m), pro Spalte mit interpolierter Pose entzerrt,
     nach Schwerkraft ausgerichtet (`level.py`), in 1-cm-Würfeln gemittelt →
     `bags/<scan>_1cm.ply` (Farbe = Reflexion, Skalarfelder Reflexion + Infrarot).
     Dauer etwa 2.5× Scandauer, bis ~3.5 GB RAM.
   - Fortschritt: `PHASE …`/`FORTSCHRITT k n` in `/tmp/process.log`, rosbag-play-Zeit in
     `/tmp/process_play.log`; `app.py:process_progress()` macht daraus den Balken
     (Bahn 0–55 %, Wolke 55–98 %).

### Bündelausgleich (HBA) – getestet 18.11.2026, nicht im Ablauf

`scripts/analysis/hba_prepare.py` zerlegt einen Scan in die Eingabe von
[hku-mars/HBA](https://github.com/hku-mars/HBA) (Einzelscans lokal + `pose.json`),
`dense_export.py … HBA_DIR` exportiert mit der ausgeglichenen Bahn. HBA ist auf dem Jetson in
`~/catkin_ws/src/HBA` gebaut, mit zwei lokalen Änderungen: `write_pose` (mypcl.hpp) überschrieb das
Quaternion komponentenweise (falsche Drehungen), und `voxel_size` 1.0 statt 4.0 für Innenräume.
Ergebnis am Abendscan: Bahn bis 12 cm korrigiert, aber Wände/Böden nicht dünner
(`flaechen_dicke.py`: Wände 31.7 → 30.7 mm, Boden 16.6 → 17.2 mm, 1 Sigma). Die ~3 cm sind
Messrauschen des Ouster Gen1, nicht Drift – deshalb nicht in `process_scan.sh`.

LIO-SAM wurde am 15.11.2026 entfernt (degenerierte in symmetrischen Innenräumen, siehe
[slam-staged-testing.md](slam-staged-testing.md)); `ros/launch/slam_fast_lio_ouster_imu.launch`
bleibt für Tests mit der Ouster-internen IMU.

## ARM / DISARM — Zustandsmaschine

Das Web-UI startet/stoppt nicht einzelne ROS-Nodes manuell, sondern fasst das in einer
Zustandsmaschine zusammen (`webui/app.py`):

```
disarmed ──ARM gedrückt──► arming ──(LiDAR ok + SLAM ok)──► armed ──DISARM/Timeout (120s)──► disarming ──► disarmed
```

- **arming**: Ouster wird aus STANDBY geholt (`ouster_set_mode("NORMAL")`), `ouster_ros`-Treiber
  neu gestartet, dann gewartet bis `/tmp/bridge_status.json` `lidar_ok: true` zeigt (max. 40s),
  danach SLAM-Launch gestartet und auf `slam_ok: true` gewartet (max. 40s).
- **armed**: Scan-Aufnahme (`SCAN`-Button → `rosbag record`) ist jetzt möglich. Auto-DISARM
  nach 120s Inaktivität (Timer wird bei Scan-Start abgebrochen).
  Es ist möglich, schon vorher und auch nachträglich noch in den anderen Modus zu wechseln.
- **disarming**: Aufnahme stoppen falls aktiv, SLAM-Prozesse beenden (`kill_slam_all()` —
  SIGINT, nach 3s SIGKILL falls nötig), `ouster_ros` stoppen, Ouster auf STANDBY setzen.

Grund für STANDBY beim Disarmen: Der Ouster-Laser hat eine begrenzte Lebensdauer und sollte
nicht dauerlaufen, wenn nicht gescannt wird.

## Web-UI Komponenten (`webui/`)

- `app.py` — Flask-Backend, alle `/api/*`-Routen (Status, ARM/DISARM, SLAM-Start/Stop,
  Recording, Export bag→XYZ, USB-Kopie, `/api/map_3d` + `/api/map_3d/cubes?since=N`
  für die Vorschau, `/api/process/*` für das 1-cm-Modell). Während einer Berechnung sind
  ARM und SCAN gesperrt. Templates laden ohne Flask-Neustart neu (`TEMPLATES_AUTO_RELOAD`).
- `templates/lidar_viewer.html` — Single-Page-UI (Handy: eine Spalte, Scan-Liste als Schublade; ab 1100 px Breite drei Spalten mit fester Scan-Liste rechts): 3D-Live-Karte (Three.js, Würfel à 5 cm,
  nach Höhe eingefärbt, Bahn + Position; Knöpfe Folgen/Oben/Ganz/Vollbild), Fortschritt der
  Berechnung, IMU-Anzeige, ARM-Button, SCAN-Button, Bag-Liste mit «1 cm» / «↓ PLY».
  Die Karte holt jede Sekunde nur die neuen Würfel (Int16-Tripel; `bridge.py` hängt an
  `/tmp/map_3d.bin` an, neue Sitzungsnummer bei neuem SLAM-Lauf).
- `static/three.min.js`, `static/roslib.min.js` — Drittanbieter-Libraries, nicht im Git
  getrackt (`.gitignore`), müssen einmalig auf den Jetson kopiert werden (siehe README).

## Autostart (systemd)

| Unit | Zweck | Status |
|------|-------|--------|
| `lidar-system.service` | startet `scripts/start_lidar.sh` beim Boot (roscore, rosbridge, SBG, bridge.py, Flask) | enabled |
| `wifi-watchdog.service` | verbindet mit stärkstem bekannten WiFi, Hotspot-Fallback | **enabled** (seit 14.11.2026) |

`start_lidar.sh` startet **nicht** `ouster_ros` und **nicht** die SLAM-Engine — beide werden
ausschließlich über die ARM-Sequenz im Flask-Backend gesteuert (siehe oben).
