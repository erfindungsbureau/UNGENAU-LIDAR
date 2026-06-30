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
- [x] **KORREKTUR nach visueller Pruefung durch Nutzer:** Odometry-Pose bewegt sich zwar
      (anders als bei FAST-LIO2), aber NICHT plausibel -- die exportierte Punktwolke
      (stage_liosam_compare.xyz) zeigt viele kleine, verstreute Fragmente ueber eine riesige
      Flaeche verteilt, statt eines zusammenhaengenden Raums. Die anfaengliche Annahme "Werte
      sind in Millimetern, Bewegung ist plausibel" war falsch -- das Matching ist tatsaechlich
      grossflaechig falsch, nicht nur die Einheit missverstanden.
- [x] **Revidierte Schlussfolgerung: BEIDE Engines liefern mit dieser Bewegung fehlerhafte
      Ergebnisse, nur auf unterschiedliche Art** -- FAST-LIO2 friert die Pose komplett ein
      (kein Tracking), LIO-SAM trackt etwas, aber das Scan-Matching divergiert grossflaechig
      (Fragmente ueber riesige Flaeche verstreut statt kohaerenter Raum). Das spricht GEGEN ein
      FAST-LIO2-spezifisches Problem und FUER eine gemeinsame Ursache, z.B.:
      (a) die Testbewegung selbst war zu schnell/ruckartig und hat Motion-Distortion in die
      rohen Scans gebracht, die beide Algorithmen ueberfordert,
      (b) Zeitsynchronisation LiDAR<->IMU (time_sync_en: false bei FAST-LIO2; bei LIO-SAM
      separat zu pruefen),
      (c) der Testraum selbst ist fuer Scan-Matching ungeeignet (zu wenig Struktur/zu
      symmetrisch -- evtl. derselbe Raum wie bei der urspruenglichen Degenerations-Diagnose).
      Naechster Schritt: kontrollierten Test mit sehr langsamer, gleichmaessiger Bewegung in
      einem Raum mit viel Struktur (Moebel, Ecken) wiederholen, um Bewegungsgeschwindigkeit als
      Variable auszuschliessen, bevor weiter an Extrinsik/Zeitsync gedreht wird.

## Langsam-Bewegungstest (Diagnose: Geschwindigkeit als Variable ausschliessen)

- [x] Datum: 2026-06-30
- [x] Bag-Datei: stage_slow_test_20260630.bag (69s, sehr langsame/gleichmaessige Bewegung,
      Raum mit Moebeln/Ecken bestaetigt strukturreich)
- [x] **Bewegung bestaetigt:** SBG zeigt volle 360 Grad Yaw-Drehung waehrend der Aufnahme.
- [x] **Kein Unterschied zur schnellen Bewegung:** 18 cloud_registered Frames (vorher 13-17),
      Pose weiterhin eingefroren (17mm Gesamtstrecke über 70s). Geschwindigkeit/Motion-Distortion
      ist damit als Ursache ausgeschlossen.
- [x] **NEUER BEFUND:** "top" waehrend des Laufs zeigt fastlio_mapping konstant bei 100% CPU
      (Single-Core, Dauerlast). Die Frame-Zahl ist über ALLE bisherigen Tests (13, 16, 17, 18)
      auffaellig konstant bei ca. 1 Frame alle ~4 Sekunden -- unabhaengig von Aufnahmedauer,
      Bewegungsgeschwindigkeit, IMU-Quelle oder pcd_save. Das deutet auf einen
      **CPU-Performance-Engpass** hin: der Jetson TX2 kommt mit der FAST-LIO2-Rechenlast nicht
      hinterher und verarbeitet nur einen Bruchteil der eingehenden 10Hz-Scans. Die grossen
      Luecken zwischen verarbeiteten Scans (~4s) koennten dazu fuehren dass die IMU-Integration
      ueber diese Zeitspanne fehlschlaegt oder von einer internen Plausibilitaetspruefung
      verworfen wird, statt die Pose korrekt fortzuschreiben -- das wuerde das Einfrieren
      erklaeren.
- [x] **Empfohlener naechster Schritt:** Performance-Tuning statt weiterer Bewegungstests, z.B.
      point_filter_num erhoehen (Punktdichte pro Scan reduzieren), filter_size_surf/filter_size_map
      vergroessern, max_iteration reduzieren. Ziel: CPU-Last pro Scan senken bis FAST-LIO2 nahe
      Echtzeit (~10Hz) mithalten kann.

## Multi-Threading-Fix-Versuch (CMakeLists.txt Patch)

Root-Cause-Hypothese aus dem CPU-Befund: FAST_LIO/CMakeLists.txt aktiviert OpenMP-Multithreading
(omp_set_num_threads(MP_PROC_NUM)) nur fuer x86/AMD64 -- der Jetson TX2 (aarch64) faellt in den
else-Zweig und laeuft mit MP_PROC_NUM=1 (single-threaded), unabhaengig von Kernanzahl.

- [x] Datum: 2026-06-30
- [x] Fix: CMakeLists.txt Architektur-Regex um "(aarch64)|(arm)" erweitert. Patch gesichert
      unter ros/patches/FAST_LIO_CMakeLists.txt (FAST_LIO liegt ausserhalb des Git-Repos in
      catkin_ws/src, daher keine eigene Versionskontrolle -- Patch hier dokumentiert fuer
      Reproduzierbarkeit nach Neuinstallation).
- [x] Neu kompiliert mit Clang-8 (gleicher Workaround wie urspruengliche Installation). Build
      erfolgreich, CMake-Output bestaetigt: "core for MP: 2" (MP_PROC_NUM=2 aktiv,
      ProcessorCount erkannte 4 -- die volle Kernzahl 6 wird durch eine Affinitaets-Einschraenkung
      auf PID 1 nicht erkannt, siehe Hinweis unten).
- [x] **Test mit Bewegung (50s): KEINE Verbesserung.** 14 cloud_registered Frames (vorher 13-18),
      Pose weiterhin eingefroren (9.4mm Gesamtstrecke). CPU-Last blieb bei ~107% statt der
      erwarteten ~200% bei aktiver 2-Thread-Parallelisierung.
- [x] **Schlussfolgerung: Multi-Threading ist NICHT die (alleinige) Ursache.** Der mit
      #pragma omp parallel for parallelisierte Codeabschnitt (Punkt-zu-Ebene-Residuen-Berechnung)
      macht offenbar nur einen kleinen Teil der Gesamtverarbeitungszeit pro Scan aus. Die
      konstante ~13-18-Frame-Rate ueber ALLE bisherigen Tests (1 vs. 2 Threads, langsam vs.
      schnell, Ouster-IMU vs. SBG, mit/ohne pcd_save) spricht gegen reinen CPU-Durchsatz als
      Engpass und eher fuer einen anderen dominanten Kostenfaktor -- wahrscheinlichster
      Verdaechtiger: die ikd-Tree-Kartenverwaltung (Einfuegen neuer Punkte in die Baumstruktur),
      die inhaerent sequentiell ist und sich nicht trivial parallelisieren laesst. Bei vollen
      65536 Rohpunkten/Scan (nur leicht reduziert durch point_filter_num: 3) waechst diese
      Baumstruktur schnell.
- [x] **Empfohlener naechster Schritt:** Punktdichte VOR FAST-LIO2 reduzieren statt Threading
      weiter zu optimieren -- z.B. point_filter_num deutlich erhoehen (z.B. 8 statt 3), oder
      Ouster lidar_mode von 1024x10 auf 512x10 reduzieren (halbiert Rohpunktzahl). Das senkt die
      Kosten sowohl der Residuen-Berechnung als auch der ikd-Tree-Operationen proportional.
- [x] Nebenbefund: nproc zeigt nur 4 (Affinitaetsmaske von PID 1 ist "0,3-5", schliesst Kerne
      1-2 aus) obwohl 6 Kerne online sind (cat /sys/devices/system/cpu/online -> 0-5). Ursache
      nicht weiter untersucht (evtl. NVIDIA-eigene Kernreservierung), koennte bei weiterer
      Performance-Optimierung relevant werden.

## Extrinsik-Kalibrierung nachgetragen (FAST-LIO2 + LIO-SAM)

Befund: WEDER fast_lio_sbg.yaml NOCH lio_sam_params.yaml hatten je die gemessene
SBG<->Ouster-Extrinsik (Roll 5deg, Pitch 4.2deg, Translation ~30mm) angewendet -- beide nutzten
Identitaet trotz vorhandener Kalibrierungsdaten in jetson-setup.md. Zusatzbefund: die in
jetson-setup.md dokumentierte extrinsicRPY-Notation (3 Winkel) war falsch -- LIO-SAMs Code
erwartet eine 3x3-Matrix (9 Werte), keine 3 Euler-Winkel.

- [x] Datum: 2026-06-30
- [x] Fix angewendet: extrinsic_T/extrinsic_R (FAST-LIO2) bzw. extrinsicTrans/extrinsicRot/
      extrinsicRPY (LIO-SAM) auf die gemessenen Werte gesetzt.
- [x] **FAST-LIO2 Test:** Keine Verbesserung. Weiterhin eingefroren (9.7mm/45s trotz 173 Grad
      Yaw-Drehung laut SBG). Sauberer Lauf, keine Fehler/Warnungen im Log.
- [x] **LIO-SAM Test:** Weiterhin stark divergent, aber andere Groessenordnung als zuvor
      (Bounding-Box jetzt 590x553x630m statt vorher Tausende -- weiterhin physikalisch
      unplausibel fuer einen Innenraum-Scan).
- [x] **Schlussfolgerung: Extrinsik-Kalibrierung war ein echter, aber nicht der entscheidende
      Fehler.** Beide Engines scheitern weiterhin an denselben Rohdaten, nur in entgegengesetzte
      Richtungen (FAST-LIO2 zu konservativ/eingefroren, LIO-SAM zu instabil/divergent) -- dieses
      Muster deutet eher auf ein Problem in den Rohdaten selbst waehrend Bewegung hin
      (Motion-Distortion-Kompensation, Zeitsynchronisation LiDAR<->IMU) als auf
      Algorithmus-Tuning. Naechster Test: einzelne Rohscans waehrend Bewegung visuell pruefen ob
      Kanten/Waende innerhalb eines einzelnen 360-Grad-Sweeps (~100ms) bereits verzogen wirken.

## DURCHBRUCH: LIO-SAM Extrinsik-Konvention-Bug gefunden und gefixt

Root-Cause endlich identifiziert: LIO-SAM wendet extrinsicRot auf rohe IMU-Messungen an
("acc = extRot * acc" in utility.h) -- das erwartet die Rotation IMU-Frame -> LiDAR-Frame.
FAST-LIO2 hingegen erwartet LiDAR-Frame -> IMU-Frame (offset_R_L_I, passend zu unseren
Kalibrierdaten "T_lb: lidar -> imu"). Wir hatten die GLEICHE Matrix fuer beide Engines
verwendet -- korrekt fuer FAST-LIO2, aber genau verkehrt herum fuer LIO-SAM (haette
transponiert werden muessen).

- [x] Datum: 2026-06-30
- [x] Fix: extrinsicRot/extrinsicRPY in lio_sam_params.yaml auf die TRANSPONIERTE Matrix
      gesetzt (IMU->LiDAR statt LiDAR->IMU).
- [x] **Test mit Bewegung (45s, bestaetigte Bewegung am Scanner): Bounding-Box jetzt
      2.62m x 2.61m x 0.86m, Gesamtstrecke 11.5m -- absolut plausibel fuer einen
      Innenraum-Scan.** 173 Frames verarbeitet (~3.9 Hz, deutlich besser als FAST-LIO2s
      ~0.2-0.3 Hz). Trajektorie bewegt sich kohaerent, keine Sprünge/Divergenz mehr.
      XYZ-Export (579k Punkte) zur visuellen Bestaetigung heruntergeladen.
- [x] **Schlussfolgerung: LIO-SAM mit korrigierter (transponierter) Extrinsik-Matrix liefert
      ein funktionierendes Ergebnis.** FAST-LIO2 bleibt trotz umfangreicher Tests (siehe oben)
      strukturell eingefroren -- vermutete Ursache: selbstverstaerkende lokale Falle ohne
      Loop-Closure-Korrektur, nicht die Extrinsik (deren Konvention bei FAST-LIO2 von Anfang
      an korrekt war). **Empfehlung: LIO-SAM (mit korrigierter Extrinsik) als Produktiv-Engine
      verwenden, FAST-LIO2 vorerst zurueckstellen.**

## Ergebnis-Zusammenfassung

(Nach Abschluss aller Stufen hier eintragen: wo liegt die Ursache, welche
Engine/Config ist für den Produktivbetrieb empfohlen.)
