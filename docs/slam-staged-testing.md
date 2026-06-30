# SLAM-Stufentest-Protokoll

## Hintergrund

Scans zeigen wiederholt überlagernde/ghostende Punktwolken bei Bewegung. Bekannt ist:

- Ein **statischer** Scan (kein Bewegung, SBG-Pipeline) zeigt nur ~0.01°/s Yaw-Rate —
  die SBG/EKF-Pipeline selbst liefert plausible Daten im Stillstand.
- **LIO-SAM degeneriert nachweislich** in symmetrischen Innenräumen (z.B. langer
  gerader Korridor): der Kostenverlauf für reine Yaw-Rotation ist dort flach, was zu
  Drift führt — unabhängig von der IMU-Qualität.
- **FAST-LIO2** wurde installiert, weil es für schnellere Handheld-Bewegungen ausgelegt
  ist, aber bisher nicht systematisch gegen die IMU-Quelle getestet.

Offene Frage: Liegt die Überlagerung an (a) der SLAM-Engine/Degeneration, (b) der
SBG-Anbindung (Extrinsik, Zeitsynchronisation, `sbg_imu_combiner.py`), oder (c) am
Sensor/Mounting selbst? Die folgenden Stufen isolieren das systematisch — **immer
mit FAST-LIO2**, jeweils nur eine Variable verändert.

## Stufe 0 — Rohe Punktwolken-Konsistenz (keine SLAM-Engine)

Kontrollgruppe: zeigt der Sensor selbst (ohne jede Fusion) bei Stillstand konsistente,
deckungsgleiche Scans?

```bash
ssh nvidia@<jetson-ip>
source /opt/ros/melodic/setup.bash
rosbag record -O /tmp/stage0_static.bag /ouster/points --duration=20
```

LiDAR während der Aufnahme NICHT bewegen. Auswertung: zwei Frames aus dem Bag
(z.B. erstes und letztes) in einem Tool wie `pcl_viewer` oder CloudCompare überlagern.

- [x] Datum: 2026-06-30
- [x] Bag-Datei: lidar_data/staged_tests/stage0_static_20260630.bag (198 Frames, 19.7s, 594MB)
- [x] Ergebnis: Schwerpunkt-Verschiebung first->last nur 1.7mm (dx=1.1mm dy=1.2mm dz=0.3mm).
      Punktweise Median-Differenz 9mm (= Sensorrauschen). Hoehere Mean/P95/Max-Werte sind ein
      Artefakt des naiven Index-Vergleichs an Objektkanten (fehlende Returns verschieben Indizes),
      keine echte Drift. **Bestanden: Rohsensor ist bei Stillstand hochkonsistent, Hardware/Mounting
      scheidet als Fehlerquelle fuer die Scan-Ueberlagerung aus.**

## Stufe 1 — FAST-LIO2 + Ouster-interne IMU

Config: `ros/config/fast_lio_ouster_imu.yaml` (`imu_topic: /ouster/imu`)
Launch: `ros/launch/slam_fast_lio_ouster_imu.launch`

```bash
ssh nvidia@<jetson-ip>
source /opt/ros/melodic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash
roslaunch /home/nvidia/UNGENAU-LIDAR/ros/launch/slam_fast_lio_ouster_imu.launch
```

Zwei Tests:
1. **Statisch** (30s, kein Bewegung) — Drift über Zeit beobachten (`/Odometry`-Topic
   oder PCD-Export am Ende).
2. **Mit Bewegung** (gleicher Raum/Weg wie bisherige Problem-Scans) — überlagern sich
   die Punktwolken wie zuvor?

**Hypothese:** Tritt die Überlagerung hier *nicht* auf, liegt die Ursache in der
SBG-Anbindung (Stufe 2 wird das bestätigen). Tritt sie *weiterhin* auf, liegt es eher
an der Engine/Bewegungsart selbst.

- [ ] Datum:
- [ ] Bag-Datei (statisch):
- [ ] Bag-Datei (Bewegung):
- [ ] Ergebnis statisch:
- [ ] Ergebnis Bewegung:

## Stufe 2 — FAST-LIO2 + SBG Ellipse 2 (aktueller Produktivstand)

Config: `ros/config/fast_lio_sbg.yaml` (`imu_topic: /imu/data`)
Launch: `ros/launch/slam_fast_lio_sbg.launch` (= Standard-Engine im Web-UI)

Gleiche zwei Tests wie Stufe 1, möglichst gleicher Raum/Weg für direkten Vergleich.

- [ ] Datum:
- [ ] Bag-Datei (statisch):
- [ ] Bag-Datei (Bewegung):
- [ ] Ergebnis statisch:
- [ ] Ergebnis Bewegung:
- [ ] Vergleich zu Stufe 1:

## Stufe 3 — Orientierungs-Abgleich SBG vs. Ouster-IMU

Zweck: Validiert unabhängig von der SLAM-Pipeline, ob die SBG-Extrinsik-Kalibrierung
(siehe [jetson-setup.md](jetson-setup.md#imu-extrinsik-kalibrierung-sbg--ouster-os1-64))
zur tatsächlichen Bewegung passt.

Während Stufe 1 (Ouster-IMU) läuft, zusätzlich aufzeichnen:
```bash
rosbag record -O /tmp/stage3_orientation.bag /sbg/ekf_euler /Odometry
```

Nach dem Lauf: SBG-Euler-Winkel (`/sbg/ekf_euler`) gegen die von FAST-LIO2 geschätzte
Orientierung (`/Odometry`, Quaternion → Euler) für denselben Zeitraum plotten/vergleichen.
Stimmen Roll/Pitch/Yaw über die Bewegung hinweg überein (innerhalb der erwarteten
Kalibrier-Unsicherheit), ist das SBG-Mounting korrekt — unabhängig davon, ob SBG
tatsächlich in der SLAM-Pipeline verwendet wird.

- [ ] Datum:
- [ ] Bag-Datei:
- [ ] Max. Abweichung Roll / Pitch / Yaw:
- [ ] Bewertung (Kalibrierung plausibel? Nachkalibrierung nötig?):

## Ergebnis-Zusammenfassung

(Nach Abschluss aller Stufen hier eintragen: wo liegt die Ursache, welche
Engine/Config ist für den Produktivbetrieb empfohlen.)
