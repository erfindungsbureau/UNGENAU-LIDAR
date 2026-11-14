# Jetson TX2 — Setup

## System

| | |
|---|---|
| Board | NVIDIA Jetson TX2 (Auvidea J120 Carrier Board) |
| OS | Ubuntu 18.04.6 LTS |
| JetPack | 4.6 / L4T R32.6.1 |
| Kernel | 4.9.253-tegra |
| Hostname | lidar / lidar.local |
| ROS | ROS Melodic, nativ via apt (`/opt/ros/melodic/`) |

## Netzwerk

- WiFi: DHCP (StudioFiume) — IP per `ip -4 addr show wlan0` prüfen
- Hotspot (Feld, falls kein WiFi verfügbar): `10.42.0.1`, SSID `LidarScanner`
- WLAN: `wifi-watchdog.service` (root, beim Start aktiv) versucht alle bekannten Netze aus `/home/nvidia/config/wifi.conf` (`SSID|PASSWORT`, nicht im Repo, Rechte 600) nach Signalstärke, sonst Hotspot; im Hotspot prüft er jede Minute, ob ein bekanntes Netz erscheint. Netze hinzufügen im Interface (Bereich «WLAN») oder per `POST /api/wifi {ssid, password}`; gleicher Name ersetzt das Passwort.
- Ouster eth0: Jetson `169.254.45.100/16`, LiDAR `169.254.45.204` (Link-Local). Bewusst nicht 192.168.2.x: Heim- und Büronetze nutzen oft denselben Bereich und kollidieren mit dem WLAN. Der Ouster hat zusätzlich fest 192.168.2.2; Notfall-Zugriff ohne Link-Local: `sudo ip addr add 192.168.2.1/24 dev eth0`
- Tailscale/VPN: `100.64.0.25` (für Remote-Zugriff unabhängig vom lokalen Netz)
- Web Interface: `http://<jetson-ip>:5000`

## NVMe SSD

| | |
|---|---|
| UUID | `7fe9b4be-c632-4680-8135-cf717a970383` |
| Label | SSD |
| Mount | `/media/nvidia/SSD` |
| fstab | `UUID=7fe9b4be-...  /media/nvidia/SSD  ext4  defaults,nofail  0 2` |

Enthält `lidar_data/bags/`, `lidar_data/maps/` — siehe [architecture.md](architecture.md) für Details
zum Datenfluss.

## Deploy / Erstinstallation

```bash
git clone https://github.com/erfindungsbureau/UNGENAU-LIDAR
cd UNGENAU-LIDAR
```

Das Repo liegt auf dem Jetson unter `/home/nvidia/UNGENAU-LIDAR` und ist der **einzige Ort**,
an dem der Code liegt — keine Kopien, keine Symlinks. Alle Skripte/Launch-Files/Configs
referenzieren absolute Pfade ins Repo. Siehe [README.md](../README.md#frontend--code-änderungen-deployen)
für den laufenden Update-Workflow.

## SBG Ellipse 2 IMU — Anschluss

Port: `/dev/sbg`, Baud: 921600 (via `sbg_driver`, RS-422 über FTDI-Adapter),
Konfiguration in `ros/config/sbg_device_uart_default.yaml`.

**udev-Regel (liegt nicht im Repo, nach Neuinstallation neu anlegen).** Der FTDI-Adapter
meldet sich je nach Reihenfolge als `ttyUSB0` oder `ttyUSB1`, deshalb fester Name:

```bash
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="sbg"' \
  | sudo tee /etc/udev/rules.d/99-sbg.rules
sudo udevadm control --reload && sudo udevadm trigger
```

Verkabelung FTDI-Kabel (9 Adern) → 8-pol. Aviatikstecker → Harwin-Stecker der SBG,
Pin-Nummern an Aviatik- und Harwin-Stecker identisch:

| Pin | Signal SBG | Ader |
|---|---|---|
| 1 | GND | schwarz (+ grau, blau) |
| 2 | Rx+ | orange (FTDI TXD+) |
| 3 | Tx+ | gelb (FTDI RXD+) |
| 4 | SYNC A (PPS-Ausgang) | braun → Ouster-Box `PPS_SLAVE` |
| 6 | VIN 12 V (4–15 V) | türkis, über 0.5-A-Sicherung |
| 7 | Rx- | rot (FTDI TXD-) |
| 8 | Tx- | violett (FTDI RXD-) |

Braun, grau, türkis und blau sind am FTDI-Chip abgelötet. SYNC A verträgt höchstens 4 V.

SBG-Einstellungen im Gerät (gesetzt 11.11.2026 nach Werksreset): IMU_DATA, EKF_QUAT und
EKF_EULER mit 200 Hz, STATUS 1 Hz, SYNC A = PPS, PORT A = 921600 Baud RS-422.
Die Magnetometer-Kalibrierung ging beim Werksreset verloren.

Falls Permission-Fehler: `sudo usermod -a -G dialout nvidia` (Neuanmeldung nötig).

## Device Tree Fix (USB, einmalig nach JetPack-Flash nötig)

- DTB: `/boot/kernel_tegra186-quill-p3310-1000-c03-00-base.dtb`
- Fix: `usb2-2 vbus-supply` phandle `0x8f` → `0x8e` (vdd-usb1-5v)
- `extlinux.conf`: FDT-Zeile entsprechend ergänzt

## IMU-Extrinsik-Kalibrierung (SBG ↔ Ouster OS1-64)

Gemessen 2026-06-23/24, Scanner statisch auf flacher Fläche (aufrecht + auf dem Kopf):

### Translation (physisch gemessen)
| Achse | Wert | Beschreibung |
|-------|------|--------------|
| X | 0.000 m | SBG auf Achse des LiDARs |
| Y | ±0.030 m | 30mm rechts von Steckerseite — Vorzeichen je nach Ouster-Frame prüfen |
| Z | -0.0249 m | SBG-Referenzpunkt 24.9mm unter LiDAR-Grundplatte |

### Rotation (gemessen via Gravity-Vergleich)
| Achse | Wert |
|-------|------|
| Roll  | 5.0° = 0.0873 rad |
| Pitch | 4.2° = 0.0733 rad |
| Yaw   | unbekannt (Gravity allein nicht bestimmbar — siehe [slam-staged-testing.md](slam-staged-testing.md) Stufe 3 für Abgleich gegen Ouster-IMU) |

### Für `ros/config/lio_sam_params.yaml`
```yaml
extrinsicTrans: [0.000, 0.030, -0.0249]
extrinsicRot:   [0.9973, 0.0064, 0.0730,
                  0.0000, 0.9962, -0.0872,
                 -0.0732, 0.0869, 0.9935]
extrinsicRPY:   [0.0873, 0.0733, 0.0000]
```

**Hinweis Y-Vorzeichen:** Ouster OS1 hat Z nach oben, X vorwärts (weg vom Stecker), Y nach links
(Rechte-Hand-Regel). "Rechts von der Steckerseite" = **-Y im LiDAR-Frame** — ggf. Vorzeichen
umkehren falls Trajektorie gespiegelt wirkt.

SBG-Funktionsnachweis (Validierung der Achsenzuordnung):
- Aufrecht: roll=+0.96°, pitch=+0.09° (flache Fläche → korrekt)
- Auf dem Kopf: roll=-179.67°, pitch=-0.18° (korrekt)

## Jetson auf volle Performance schalten (vor SLAM-Lauf empfohlen)

```bash
sudo nvpmodel -m 0   # alle 6 CPU-Kerne aktivieren (Max-N Modus)
sudo jetson_clocks   # CPU/GPU-Takt fixieren
```

## Bekannte Einschränkungen

- **FAST-LIO2 muss mit `-O3` gebaut sein.** In `~/catkin_ws/src/FAST_LIO/CMakeLists.txt`
  (nicht im Repo) stand `set( CMAKE_CXX_FLAGS "-std=c++14 -O0" )`. Damit verarbeitet der
  Jetson nur 0.34 Scans/s statt 10. Mit `-O3` und Clang-8 läuft es in Echtzeit
  (gemessen 11.11.2026, stationär, ~140 % CPU). Neu bauen:
  `cd ~/catkin_ws && catkin_make --pkg fast_lio -j2` (ca. 6 Minuten).
  Zum Vergleich: LIO-SAM verarbeitet auf dem TX2 nur etwa 46 % der Scans.
- **GCC 7.5/8 Segfault auf ARM64** beim Kompilieren von `FAST_LIO/laserMapping.cpp` (Compiler-Bug
  bei tiefer Template-Instanziierung) → Workaround: Clang-8 (`apt install clang-8`,
  `-DCMAKE_CXX_COMPILER=/usr/bin/clang++-8`).
- **LIO-SAM degeneriert in symmetrischen Innenräumen** (z.B. langer gerader Korridor) — der
  Kostenverlauf für reine Yaw-Rotation ist dort flach, was zu Drift führt obwohl die IMU-Pipeline
  korrekt arbeitet (bestätigt durch statischen Scan ohne Bewegung). Deshalb FAST-LIO2 als
  Alternative installiert, siehe [slam-staged-testing.md](slam-staged-testing.md).
- **Claude Code CLI benötigt Node ≥18, Ubuntu 18.04 hat nur glibc 2.27** (Node 18+
  Binaries verlangen glibc ≥2.28). System-glibc-Upgrade ist auf diesem Embedded-System zu
  riskant. Workaround: Node 16.20.4 (letzte glibc-2.27-kompatible Version) manuell nach
  `~/.local/node16/` installiert, in PATH via .bashrc. `npm install -g @anthropic-ai/claude-code`
  läuft darauf trotz EBADENGINE-Warnung tatsächlich (getestet: `claude --version` funktioniert).
  Node-Quelle: https://nodejs.org/dist/v16.20.2/node-v16.20.2-linux-arm64.tar.xz
- **RoboStack/Conda-ROS wurde evaluiert, aber nicht verwendet** — das System läuft nativ über
  ROS Melodic (apt). Reste der RoboStack-Recherche liegen in `archive/` falls relevant.
