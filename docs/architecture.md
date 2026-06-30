# Architektur

## Überblick: Datenfluss

```
┌─────────────┐     /ouster/points      ┌──────────────┐
│ Ouster OS1-64│ ───────────────────────►│              │
│ (ouster_ros) │     /ouster/imu         │  SLAM-Engine │──► /Odometry, /cloud_registered
└─────────────┘ ───────────────────────►│ (LIO-SAM ODER│    (bzw. /lio_sam/mapping/...)
                                          │  FAST-LIO2) │
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
              │  bridge.py   │  schreibt /tmp/*.json       │  rosbag      │
              │  (Python2)   │ ◄── liest beide SLAM-Topics │  record      │ → bags/*.bag
              └──────┬──────┘                            └─────────────┘
                     │ /tmp/pts.json, /tmp/imu.json,
                     │ /tmp/bridge_status.json (lidar_ok, slam_ok, still)
              ┌──────▼──────┐
              │  Flask app   │  webui/app.py — ARM/DISARM, SCAN, Export, USB-Copy
              │  (Port 5000) │
              └─────────────┘
```

**Warum `bridge.py` als separate Python2-Brücke?** Die ROS-Melodic-Python-Bindings sind
Python 2; Flask läuft unter Python 3. `bridge.py` abonniert die ROS-Topics, schreibt
Status/Punktwolke/IMU als JSON nach `/tmp/`, und Flask liest diese Dateien — kein direkter
ROS-Import in Flask nötig.

## SLAM-Engines

Zwei Engines stehen zur Wahl, austauschbar über `webui/app.py:SLAM_LAUNCHES`:

| Engine | Launch-File | Wann |
|--------|-------------|------|
| LIO-SAM | `ros/launch/slam_lio_sam.launch` | Referenz/Vergleich — degeneriert in symmetrischen Innenräumen (siehe [slam-staged-testing.md](slam-staged-testing.md)) |
| FAST-LIO2 | `ros/launch/slam_fast_lio_sbg.launch` | Standard für Handheld-Scans mit schnellen Bewegungen |

Beide lesen IMU-Daten von `/imu/data` (SBG, via `sbg_imu_combiner.py`). Für isolierte
Tests mit der Ouster-internen IMU siehe `ros/launch/slam_fast_lio_ouster_imu.launch`
(Stufe 1 des Testprotokolls).

`bridge.py` erkennt SLAM-Aktivität anhand **beider** möglichen Output-Topics
(`/lio_sam/mapping/cloud_registered` und `/cloud_registered`), unabhängig davon welche
Engine aktiv ist.

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
  Recording, Export bag→XYZ, USB-Kopie).
- `templates/lidar_viewer.html` — Single-Page-UI: Live-3D-Punktwolke (Three.js),
  IMU-Anzeige, ARM-Button, SCAN-Button, Engine-Auswahl (LIO-SAM/FAST-LIO2), Bag-Liste/Export.
- `static/three.min.js`, `static/roslib.min.js` — Drittanbieter-Libraries, nicht im Git
  getrackt (`.gitignore`), müssen einmalig auf den Jetson kopiert werden (siehe README).

## Autostart (systemd)

| Unit | Zweck | Status |
|------|-------|--------|
| `lidar-system.service` | startet `scripts/start_lidar.sh` beim Boot (roscore, rosbridge, SBG, bridge.py, Flask) | enabled |
| `wifi-watchdog.service` | verbindet mit stärkstem bekannten WiFi, Hotspot-Fallback | disabled |

`start_lidar.sh` startet **nicht** `ouster_ros` und **nicht** die SLAM-Engine — beide werden
ausschließlich über die ARM-Sequenz im Flask-Backend gesteuert (siehe oben).
