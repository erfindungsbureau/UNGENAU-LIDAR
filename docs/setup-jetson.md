# Jetson TX2 — Setup Guide

## System Info

| | |
|---|---|
| **Board** | NVIDIA Jetson TX2 (Quill carrier board) |
| **OS** | Ubuntu 16.04.7 LTS (Xenial) |
| **JetPack** | R28.2.1 |
| **Kernel** | 4.4.38-jetsonbotv0.1 (custom, aarch64) |
| **Python** | 3.5.2 |
| **IP (LAN)** | 192.168.31.39 (WiFi, DHCP) |
| **Hostname** | tegra-ubuntu |

## Initial Setup (already applied)

### 1. Disable LightDM
```bash
sudo systemctl disable lightdm
sudo systemctl stop lightdm
# Saves ~400MB RAM and reduces CPU load
```

### 2. Mount NVMe SSD
The SSD (`/dev/nvme0n1`, ext4, label "SSD") is **not** auto-mounted by default on this custom kernel.

**fstab entry** (already added):
```
UUID=7fe9b4be-c632-4680-8135-cf717a970383  /media/nvidia/SSD1  ext4  defaults,nofail,x-systemd.device-timeout=5  0  2
```

Mount manually: `sudo mount /dev/nvme0n1 /media/nvidia/SSD1`

### 3. Docker (non-functional)
Docker 18.09.7 is installed but **cannot start** due to the custom kernel missing `CONFIG_CGROUP_DEVICE`. Use RoboStack instead.

Error:
```
Error starting daemon: Devices cgroup isn't mounted
```

### 4. Install Python packages
```bash
sudo apt-get install -y python3-pip python3-pil samba
sudo pip3 install "flask<2.0"
```

## Deploy Project Files

From rosbot (or any machine with SSH access):

```bash
git clone git@github.com:modulator100/jetson-lidar-scanner.git
cd jetson-lidar-scanner
bash scripts/deploy.sh nvidia@192.168.31.39
```

Or manually:

```bash
TARGET=nvidia@192.168.31.39

# Web UI
scp -r webui/ $TARGET:/home/nvidia/lidar-webui/

# Config files
sshpass -p nvidia scp config/smb.conf $TARGET:/tmp/
sshpass -p nvidia scp config/wifi-fallback.sh $TARGET:/tmp/
sshpass -p nvidia scp config/*.service $TARGET:/tmp/
sshpass -p nvidia scp config/*.timer $TARGET:/tmp/

# Apply configs
ssh $TARGET 'sudo cp /tmp/smb.conf /etc/samba/smb.conf && \
  sudo cp /tmp/wifi-fallback.sh /usr/local/bin/ && \
  sudo chmod +x /usr/local/bin/wifi-fallback.sh && \
  sudo cp /tmp/*.service /tmp/*.timer /etc/systemd/system/ && \
  sudo systemctl daemon-reload && \
  sudo systemctl enable lidar-webui lidar-hotspot.timer smbd nmbd && \
  sudo systemctl start lidar-webui smbd nmbd lidar-hotspot.timer'
```

## Services

```bash
# Flask web UI
sudo systemctl status lidar-webui.service
sudo journalctl -u lidar-webui.service -f

# WiFi fallback (timer + service)
sudo systemctl status lidar-hotspot.timer
sudo systemctl status lidar-hotspot.service

# Samba
sudo systemctl status smbd nmbd

# Run WiFi fallback manually
sudo /usr/local/bin/wifi-fallback.sh
cat /var/log/wifi-fallback.log
```

## Network Details

### WiFi Fallback Logic
1. On boot (and every 5min): scan for `StudioFiume`
2. If found → connect as client → Jetson reachable at `192.168.31.39`
3. If not found → create `LidarScanner` hotspot → Jetson reachable at `10.42.0.1`

### Add StudioFiume Password (one-time)
```bash
sudo nmcli dev wifi connect "StudioFiume" password "YOUR_PASSWORD" ifname wlan0
```

## Troubleshooting

### SSD not mounted after reboot
```bash
sudo mount /dev/nvme0n1 /media/nvidia/SSD1
```
If persistent, check fstab: `cat /etc/fstab | grep nvme`

### Flask web UI not starting
```bash
sudo journalctl -u lidar-webui.service -n 20
# Common cause: SSD not mounted → lidar_data dirs missing
# Fix:
sudo mount /dev/nvme0n1 /media/nvidia/SSD1
sudo systemctl restart lidar-webui.service
```

### Conda environment missing after reboot
The conda environment persists — only the SSD mount is needed:
```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros-noetic
rosversion -d  # should print: noetic
```
