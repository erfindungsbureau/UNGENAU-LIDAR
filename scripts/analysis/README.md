# Auswertung von Aufnahmen (offline)

Alle Skripte lesen Bags aus `/media/nvidia/SSD/lidar_data/bags`. Python 2 (rosbag) ausser `lc_optimize.py` (Python 3, scipy).

| Skript | Zweck |
|---|---|
| `scan_check.py BAG [odom] [cloud] [step] [n_lidar]` | Verarbeitungsquote, Bahn, Abstand Start–Ende, Wanddicke (Streuung quer zu ebenen 0.5-m-Würfeln) |
| `imu_check.py BAG` | Zeitversatz und Drehung SBG (/imu/data) ↔ Ouster-IMU per Kreuzkorrelation der Drehraten |
| `replay_cmp.sh BAG RATE OUT` + `lio_offline.launch` | Aufnahme offline durch LIO-SAM rechnen (nur bei DISARM; setzt /use_sim_time) |
| `fl_offline.launch ident:=true/false` | dasselbe für FAST-LIO2 (mit rosbag play, Topics /replay/points, /replay/imu) |
| `lc_extract.py BAG OUT.npz` → `lc_optimize.py OUT.npz PREFIX [...]` | Schleifenschluss für FAST-LIO2: Stützpunkte, Punkt-Ebene-ICP, Posengraph; gibt jedes geprüfte Paar aus |

Ergebnisse 14.11.2026 (Wohnung): FAST-LIO2 schlägt LIO-SAM (100 % statt 38–49 % der Umdrehungen, schärfere Wände).
Aufrechter Testlauf: Schleifen zeigen nur 1–3 cm / < 1° Abweichung, Schleifenschluss unnötig. Stark gekippter Scan
(bis kopfüber): Meter-Abweichung, Schleifenschluss hilft kaum. SBG ↔ Ouster: 2 ms Zeitversatz, 0.4° Lage.
