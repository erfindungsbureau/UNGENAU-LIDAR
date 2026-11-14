# Auswertung von Aufnahmen (offline)

Alle Skripte lesen Bags aus `/media/nvidia/SSD/lidar_data/bags`. Python 2 (rosbag) ausser `lc_optimize.py` (Python 3, scipy).

| Skript | Zweck |
|---|---|
| `scan_check.py BAG [odom] [cloud] [step] [n_lidar]` | Verarbeitungsquote, Bahn, Abstand Start–Ende, Wanddicke (Streuung quer zu ebenen 0.5-m-Würfeln) |
| `imu_check.py BAG` | Zeitversatz und Drehung SBG (/imu/data) ↔ Ouster-IMU per Kreuzkorrelation der Drehraten |
| `replay_cmp.sh BAG RATE OUT` + `lio_offline.launch` | Aufnahme offline durch LIO-SAM rechnen (nur bei DISARM; setzt /use_sim_time) |
| `fl_offline.launch ident:=true/false` | dasselbe für FAST-LIO2 (mit rosbag play, Topics /replay/points, /replay/imu) |
| `floor_check.py BAG [imu_topic]` | Neigung der Karte und Restgefälle des Bodens nach `scripts/level.py` (Kontrolle ohne IMU) |
| `lc_extract.py BAG OUT.npz` → `lc_optimize.py OUT.npz PREFIX [...]` | Schleifenschluss für FAST-LIO2: Stützpunkte, Punkt-Ebene-ICP, Posengraph; gibt jedes geprüfte Paar aus |

Ergebnisse 14.11.2026 (Wohnung): FAST-LIO2 schlägt LIO-SAM (100 % statt 38–49 % der Umdrehungen, schärfere Wände).
Aufrechter Testlauf: Schleifen zeigen nur 1–3 cm / < 1° Abweichung, Schleifenschluss unnötig. Stark gekippter Scan
(bis kopfüber): Meter-Abweichung, Schleifenschluss hilft kaum. SBG ↔ Ouster: 2 ms Zeitversatz, 0.4° Lage.

Montage korrigiert 14.11.2026 (abends): `extrinsic_R` = Einheit (CNC-Platte, gemessen 0.4°), `extrinsic_T` = [0, 0.030, 0.025]
(LiDAR 30 mm links, 25 mm über dem SBG), Selbstkalibrierung aus. Testlauf 1 neu gerechnet: Wände 16.9 → 16.3 mm, Neigung
der Karte 9.9° → 2.3° (Rest = Startlage). `scripts/level.py` richtet die FAST-LIO2-Karte nach der Schwerkraft aus
(mittlere spezifische Kraft des SBG, mit der FAST-LIO2-Pose in die Welt gedreht); danach Boden 0.1° = 1.5 mm/m.
Beide Exporte (`bag_to_xyz.py`, `bag_to_xyz_hires.py`) richten FAST-LIO2-Karten jetzt automatisch aus.
