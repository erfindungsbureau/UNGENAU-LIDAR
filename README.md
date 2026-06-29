# UNGENAU LiDAR Scanner

Handgehaltener 3D-Scanner auf Basis NVIDIA Jetson TX2 mit Ouster OS1-64 LiDAR und SBG Ellipse 2 IMU.
Live-Vorschau und Scan-Aufnahme über Smartphone-Browser.

## Hardware
- NVIDIA Jetson TX2 (Auvidea J120)
- Ouster OS1-64 LiDAR (Firmware v2.4.0)
- SBG Ellipse 2 IMU, Modell E-G4A3-M1 (RS-422 â FTDI USB, `/dev/ttyUSB0`, 921600 Bd)
- 460Â GB NVMe SSD (Scandaten)

## System
- JetPack 4.6 / Ubuntu 18.04 / L4T R32.6.1
- ROS Melodic (apt)
- Hostname: `lidar` / `lidar.local`
- Web Interface: http://192.168.31.39:5000
- SMB Share: `smb://192.168.31.39/lidar_data`

## Starten
```bash
~/start_lidar.sh
```

## WiFi
- Bekanntes Netz (StudioFiume): verbindet als Client â http://192.168.31.39:5000
- Kein bekanntes Netz: erstellt Hotspot **LidarScanner** â http://10.42.0.1:5000
- Passwort Hotspot: `lidar1234`

## SMB Dateitransfer
- Mac/Windows: `smb://192.168.31.39/lidar_data`
- User: `nvidia` / Pass: `nvidia`
- Ordner: `bags/` (Rohdaten), `maps/` (SLAM), `exports/` (E57Â /Â ArchiCAD)

---

## Architektur

### IMU-Pipeline

```
SBG Ellipse 2 (UART /dev/ttyUSB0, 921600 Bd)
  ââ sbg_driver (sbg_ros_driver v3.2.1, use_enu: true)
       ââ /sbg/imu_data   â  Beschleunigung + Gyroskop (ENU, 200 Hz)
       ââ /sbg/ekf_quat   â  Orientierung inkl. Magnetometer-Yaw (200 Hz)
  sbg_imu_combiner.py
       ââ /imu/data  â  sensor_msgs/Imu (ENU, 200 Hz)  â  LIO-SAM
```

Die SBG Ellipse 2 hat einen internen Kalman-Filter (EKF), der Gyroskop, Beschleunigungssensor und
Magnetometer fusioniert. Damit ist der Yaw-Winkel absolut referenziert (kein Gyro-Drift).
`sbg_driver use_enu:true` konvertiert NEDâENU automatisch.
`sbg_imu_combiner.py` kombiniert IMU-Rohdaten und EKF-Quaternion zum Standard-ROS-Format `/imu/data`.

### SLAM-Pipeline

```
Ouster OS1-64  â  /ouster/points (10 Hz)  â  /imu/data (200 Hz)
  ââ LIO-SAM
       ââ imageProjection    (Deskewing via IMU)
       ââ featureExtraction  (Edge + Surface Features)
       ââ imuPreintegration  (GTSAM, IMU-Vorintegration)
       ââ mapOptimization    (GTSAM iSAM2, Loop Closure ICP)
            ââ /lio_sam/mapping/cloud_registered  â  Bag / E57-Export
```

**Koordinaten-Konventionen:**
- SBG native: NED (X=Nord, Y=Ost, Z=unten) â `sbg_driver use_enu:true` â ENU
- Ouster OS1: ENU (X=vorwärts, Y=links, Z=oben)
- Extrinsics SBGâOuster: Identitätsmatrix (axiale Ausrichtung identisch)

---

## LIO-SAM Anpassungen

Die folgenden Änderungen wurden manuell in der LIO-SAM-Installation auf dem Jetson vorgenommen
(`~/catkin_ws/src/LIO-SAM/`). LIO-SAM ist ein externes Paket und nicht in diesem Repo enthalten.

### 1. `imuPreintegration.cpp` â deltaTij-Guard

Wenn zwischen zwei LiDAR-Keyframes keine IMU-Daten ankommen, ist `deltaTij = 0`. Der
GTSAM-`BetweenFactor` mit Sigma=0 erzeugt ein unbestimmtes lineares System â Crash.

```cpp
if (imuIntegratorOpt_->deltaTij() < 1e-6) {
    imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
    ++key; return;  // Keyframe überspringen
}
```

### 2. `imuPreintegration.cpp` â GTSAM try-catch

```cpp
try {
    optimizer.update(graphFactors, graphValues);
    optimizer.update();
} catch (const gtsam::IndeterminantLinearSystemException& e) {
    resetOptimization(); systemInitialized = false; return;
}
```

### 3. `mapOptmization.cpp` â Loop-Closure try-catch

Falsch-positive Loop-Closures fügen schlechte BetweenFactors ein â GTSAM-Crash + Trajektoriensprung.

```cpp
try {
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
} catch (const gtsam::IndeterminantLinearSystemException& e) {
    gtSAMgraph.resize(0); initialEstimate.clear();
    aLoopIsClosed = false; return;
}
```

### 4. `config/params.yaml` â Parameter für SBG Ellipse 2

| Parameter | Default | Angepasst | Grund |
|---|---|---|---|
| `imuAccBiasN` | 0.0002 | 0.002 | GTSAM-Stabilität |
| `imuGyrBiasN` | 0.00003 | 0.0003 | GTSAM-Stabilität |
| `historyKeyframeFitnessScore` | 0.3 | 0.05 | Striktere Loop-Closure |

---

## Repo-Struktur

```
scripts/
  start_lidar.sh          Systemstart (roscore, sbg_driver, combiner, bridge, Flask)
  watchdog.sh             Prozess-Watchdog (bridge, sbg_driver, sbg_imu_combiner)
  sbg_imu_combiner.py     /sbg/imu_data + /sbg/ekf_quat â /imu/data (ENU + EKF)
  slam.launch             LIO-SAM Launch (ohne Madgwick, ohne Relay)
  bridge.py               ROSâFlask Brücke (Python2, downsampled points)
  wifi_ap.sh              WiFi / Hotspot Logik
config/
  sbg_device_uart_default.yaml   sbg_ros_driver Konfiguration (use_enu:true, log_ekf_quat:8)
  lio_sam_params.yaml            LIO-SAM Parameter (IMU-Rauschen, Loop Closure, Voxel)
  smb.conf                       Samba Konfiguration
docs/
  setup-jetson.md         Jetson Setup (JetPack 4.6)
  setup-ros.md            ROS Melodic Installation
  hardware-connections.md Verkabelung
webui/
  app.py                  Flask Backend (Recording, Status, Bags)
  templates/              HTML Frontend (Three.js 3D Viewer)
```

## API Endpoints

| Endpoint | Methode | Beschreibung |
|---|---|---|
| `/api/status` | GET | ROS, LiDAR, IMU, Disk Status |
| `/api/points` | GET | Live Punktwolke (downsampled JSON) |
| `/api/record/start` | POST | Rosbag Recording starten |
| `/api/record/stop` | POST | Recording stoppen |
| `/api/bags` | GET | Gespeicherte Bags auflisten |
| `/api/download/<name>` | GET | Bag herunterladen |

## Bekannte Einschränkungen

- **Magnetometer indoor:** Der SBG-Magnetometer kann durch Metallstrukturen oder Elektrik gestört werden. Bei schlechter Qualität driftet der Yaw trotz EKF.
- **Loop Closure:** `historyKeyframeFitnessScore: 0.05` â bei symmetrischen Umgebungen können Falsch-Positive auftreten (try-catch fängt GTSAM-Abstürze ab).
- **LIO-SAM Indoor:** Für feature-arme Innenräume (glatte Wände) ist FAST-LIO2 besser geeignet.
- **PPS-Sync:** Keine Hardware-Zeitsynchronisation SBGâOuster. Für höhere Genauigkeit: SBG Event-Out-A â Ouster Sync-In (NMEA 9600Â Bd + 1PPS, siehe SBG Support).
