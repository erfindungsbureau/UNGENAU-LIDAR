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

- [x] Datum: 2026-06-30
- [ ] Bag-Datei (statisch): noch ausstehend
- [x] Bag-Datei (Bewegung, v1 ohne tatsaechliche Bewegung): stage1_ouster_imu_20260630.bag
      (598 Frames, 16 cloud_registered)
- [x] Bag-Datei (Bewegung, v2 MIT Bewegung): stage1_ouster_imu_20260630_v2.bag
      (597 Frames, 13 cloud_registered, 59.8s)
- [ ] Ergebnis statisch: noch ausstehend
- [x] Ergebnis Bewegung: **Bestaetigt in 2 unabhaengigen Laeufen: /cloud_registered publiziert nur
      13-16x in 60s (~0.2-0.27 Hz statt ~10 Hz)** - FAST-LIO2 verarbeitet mit der Ouster-internen
      IMU nur einen Bruchteil der Scans, auch bei tatsaechlicher Bewegung. Power-Mode war bereits
      MAXN (nicht die Ursache). Das ist unabhaengig von der IMU-Qualitaet ein
      Verarbeitungsgeschwindigkeits-Problem -> grosse Luecken zwischen Posen, was selbst eine
      perfekte IMU nicht ausgleichen kann. XYZ-Exports heruntergeladen, visuelle Pruefung durch
      Nutzer ausstehend.
      NEBENBEFUND: bag_to_xyz.py/bag_to_xyz_hires.py kannten nur LIO-SAM-Topics
      (/lio_sam/mapping/cloud_registered, /lio_sam/mapping/odometry), nicht FAST-LIO2s
      /cloud_registered und /Odometry -> gefixt (Commit in scripts/).

### Stufe 1 - Nachtrag (v3, mit verifizierter Bewegung)

- [x] Datum: 2026-06-30
- [x] Bag-Datei: stage1_ouster_imu_20260630_v3.bag (698 LiDAR-Frames, 69s, 17 cloud_registered)
- [x] **KRITISCHER BEFUND:** IMU-Gyro zeigt eindeutige Bewegung (62.6% der Samples > 0.1 rad/s,
      Max 1.17 rad/s = ~67 Grad/s) -- aber /Odometry-Pose bewegt sich praktisch NICHT
      (Gesamtstrecke ueber 17 Posen: 0.01m, Bounding-Box ~0x0x0m). FAST-LIO2 mit der
      Ouster-internen IMU **verliert die Bewegungsschaetzung vollstaendig**, obwohl der Sensor
      sich nachweislich bewegt hat. Die wenigen exportierten Frames sehen deshalb optisch
      deckungsgleich aus (Pose eingefroren) -- das ist aber kein funktionierendes SLAM, sondern
      ein haengen gebliebener Tracker, keine echte Kartierung.
- [x] Schlussfolgerung Stufe 1: **FAST-LIO2 + Ouster-interne IMU ist in dieser Konfiguration
      nicht nutzbar.** Wahrscheinliche Ursache: acc_cov/gyr_cov in fast_lio_ouster_imu.yaml
      sind 1:1 von der SBG-Config uebernommen -- die (vermutlich deutlich verrauschtere)
      Ouster-IMU braucht andere (hoehere) Kovarianzwerte, sonst vertraut der iEKF der IMU zu
      stark und divergiert/haengt. Vor einer erneuten Stufe-1-Verifikation muesste man diese
      Werte fuer die Ouster-IMU neu kalibrieren -- nicht weiter verfolgt, da Stufe 2 (SBG)
      bereits im Produktivbetrieb getestet werden kann.

## Stufe 2 — FAST-LIO2 + SBG Ellipse 2 (aktueller Produktivstand)

Config: `ros/config/fast_lio_sbg.yaml` (`imu_topic: /imu/data`)
Launch: `ros/launch/slam_fast_lio_sbg.launch` (= Standard-Engine im Web-UI)

Gleiche zwei Tests wie Stufe 1, möglichst gleicher Raum/Weg für direkten Vergleich.

- [x] Datum: 2026-06-30
- [x] Bag-Datei (Bewegung): stage2_sbg_20260630.bag (698 LiDAR-Frames, 69s, 17 cloud_registered)
- [x] Ergebnis: **IDENTISCHES Symptom wie Stufe 1.** SBG zeigt eindeutig massive Bewegung waehrend
      der Aufnahme (Roll-Range 93 Grad, Pitch-Range 76 Grad, volle 360 Grad Yaw-Drehung) -- aber
      FAST-LIO2 Odometry-Pose bleibt ueber alle 17 Posen quasi eingefroren (Gesamtstrecke 0.01m,
      Bounding-Box ~0x0x0m, x schwankt nur zwischen -0.006 und -0.008).
- [x] Vergleich zu Stufe 1: **Kein Unterschied zwischen Ouster-IMU und SBG.** Beide IMU-Quellen
      zeigen exakt dasselbe Symptom -> IMU-Quelle ist NICHT die Ursache der Scan-Ueberlagerung.
- [x] Zusatztest: pcd_save_en auf false gesetzt (FAST-LIO2-Quellcode warnt explizit dass PCD-Save
      die Echtzeitperformance beeintraechtigt) und mit 30s-Testaufnahme wiederholt: Rate blieb
      gleich (8 Frames / 26s = ~0.3 Hz), Pose weiterhin eingefroren (Gesamtstrecke 5mm).
      **pcd_save_en ist NICHT die Ursache.** Aenderung bleibt trotzdem bestehen (keine Nachteile,
      Produktiv-Pipeline nutzt ohnehin bag_to_xyz.py statt FAST-LIO2s internem PCD-Export).
- [x] **Schlussfolgerung Stufe 2: Das eigentliche Problem liegt tiefer in der FAST-LIO2-Konfiguration
      selbst, nicht in der IMU-Quelle.** Naechste Verdaechtige: Extrinsik-Kalibrierung
      (extrinsic_T/extrinsic_R aktuell Identitaet/Null), Zeitsynchronisation LiDAR<->IMU
      (time_sync_en: false), oder Tuning-Parameter (max_iteration, blind, point_filter_num).
      Erfordert gezieltes FAST-LIO2-Debugging, kein einfacher A/B-Test mehr.

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

## LIO-SAM Vergleichstest (Diagnose, ausserhalb der Stufen 0-3)

Nach Stufe 1+2 stand fest: FAST-LIO2 trackt die Pose ueberhaupt nicht (komplett eingefroren),
egal welche IMU-Quelle. Um zu klaeren ob das Problem an FAST-LIO2 selbst liegt oder tiefer
(Sensordaten/Zeitstempel) wurde LIO-SAM mit der gleichen Bewegung getestet.

- [x] Datum: 2026-06-30
- [x] Bag-Datei: stage_liosam_compare_20260630.bag (44.7s, mit SBG-IMU)
- [x] **Verarbeitungsrate: 102 Frames in 44.7s = ~2.3 Hz** (FAST-LIO2 zum Vergleich: ~0.27 Hz,
      8.5x langsamer).
- [x] **Pose-Tracking: LIO-SAM bewegt die Pose deutlich und plausibel.** Odometry-Werte sind in
      Millimetern (nicht Metern wie bei FAST-LIO2) -- umgerechnet: Bounding-Box ca. 0.85m x 0.49m
      x 0.15m, Gesamtstrecke ca. 2.6m ueber 45s Bewegung. Das ist eine realistische Strecke fuer
      Herumlaufen in einem Raum.
- [x] **Schlussfolgerung: Das Problem ist FAST-LIO2-spezifisch, nicht in den Rohdaten/Zeitstempeln
      begruendet.** LIO-SAM verarbeitet dieselben /ouster/points + /imu/data Stroeme erfolgreich
      mit plausibler Bewegungsschaetzung; FAST-LIO2 friert mit denselben Eingangsdaten ein.
      Naechster Schritt: FAST-LIO2-Konfiguration gezielt debuggen (Extrinsik, Zeitsynchronisation,
      max_iteration) statt IMU-Quelle zu wechseln. Die urspruengliche Scan-Ueberlagerung bei
      LIO-SAM bleibt weiterhin durch die bekannte Degeneration in symmetrischen Raeumen erklaert
      (siehe README) -- LIO-SAM trackt zwar zuverlaessig, driftet aber dort weiterhin.

## Ergebnis-Zusammenfassung

(Nach Abschluss aller Stufen hier eintragen: wo liegt die Ursache, welche
Engine/Config ist für den Produktivbetrieb empfohlen.)
