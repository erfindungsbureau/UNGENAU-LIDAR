# Jetson TX2 — Setup (JetPack 4.6)

## System
| | |
|---|---|
| Board | NVIDIA Jetson TX2 (Auvidea J120) |
| OS | Ubuntu 18.04.6 LTS |
| JetPack | 4.6 / L4T R32.6.1 |
| Kernel | 4.9.253-tegra |
| Hostname | lidar |

## Netzwerk
- WiFi: 192.168.31.39 (DHCP, StudioFiume)
- LiDAR Ethernet eth0: 169.254.45.100/16 + 192.168.2.1/24
- Hotspot (Feld): 10.42.0.1, SSID: LidarScanner

## NVMe SSD
UUID: 7fe9b4be-c632-4680-8135-cf717a970383, Label: SSD
fstab: UUID=7fe9b4be...  /media/nvidia/SSD  ext4  defaults,nofail  0 2
Inhalt: lidar_data/bags, lidar_data/maps, lidar_data/exports

## Deploy
```bash
git clone https://github.com/erfindungsbureau/UNGENAU-LIDAR
cd UNGENAU-LIDAR
bash scripts/deploy.sh nvidia@192.168.31.39
```

## Device Tree Fix (USB)
DTB: /boot/kernel_tegra186-quill-p3310-1000-c03-00-base.dtb
Fix: usb2-2 vbus-supply phandle 0x8f → 0x8e (vdd-usb1-5v)
extlinux.conf: FDT Zeile hinzugefügt

## Bekannte Einschränkungen
- GTSAM kompiliert nicht (GCC7/8 ARM64 Bug) → LIO-SAM ausstehend
- Fan PWM bindet nicht sauber → extern kühlen

## IMU Extrinsik-Kalibrierung (SBG ↔ Ouster)

Gemessen 2026-06-23, Scanner statisch auf flacher Fläche (aufrecht + auf dem Kopf):

| Achse | Offset |
|-------|--------|
| Roll  | ~5.0°  |
| Pitch | ~4.2°  |
| Yaw   | unbekannt (Gravity allein nicht bestimmbar) |

Für LIO-SAM `params.yaml`:
```yaml
extrinsicRot:  [1, 0, 0,  0, 1, 0,  0, 0, 1]   # TODO: exakte Matrix einsetzen
extrinsicRPY:  [0.087, 0.073, 0.0]               # roll=5deg, pitch=4.2deg, yaw=TBD
```

SBG Ellipse 2 Funktionsnachweis:
- Aufrecht: roll=+0.96°, pitch=+0.09° (flache Fläche → korrekt)
- Auf dem Kopf: roll=-179.67°, pitch=-0.18° (korrekt)

## Vollständige Extrinsik-Kalibrierung SBG ↔ Ouster OS1-64

### Translation (physisch gemessen, 2026-06-24)
| Achse | Wert | Beschreibung |
|-------|------|--------------|
| X | 0.000 m | SBG auf Achse des LiDARs |
| Y | ±0.030 m | 30mm rechts von Steckerseite — **Vorzeichen je nach Ouster-Frame prüfen** |
| Z | -0.0249 m | SBG-Referenzpunkt 24.9mm unter LiDAR-Grundplatte |

### Rotation (gemessen via Gravity-Vergleich, 2026-06-23)
| Achse | Wert |
|-------|------|
| Roll  | 5.0° = 0.0873 rad |
| Pitch | 4.2° = 0.0733 rad |
| Yaw   | unbekannt (0° angenommen) |

### LIO-SAM params.yaml
```yaml
# Extrinsics: T_lb (lidar -> imu/SBG)
extrinsicTrans: [0.000, 0.030, -0.0249]
extrinsicRot:   [0.9973, 0.0064, 0.0730,
                  0.0000, 0.9962, -0.0872,
                 -0.0732, 0.0869, 0.9935]
extrinsicRPY:   [0.0873, 0.0733, 0.0000]
```

**Hinweis Y-Vorzeichen:** Der Ouster OS1 hat Z nach oben (Rotationsachse), X vorwärts (weg vom Stecker).
Y zeigt nach links (Rechte-Hand-Regel). "Rechts von der Steckerseite" = **-Y im LiDAR-Frame** → ggf. -0.030 verwenden.
Im ersten SLAM-Test prüfen ob die Trajektorie plausibel ist; falls gespiegelt, Y-Vorzeichen umkehren.
