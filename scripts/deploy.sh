#!/bin/bash
# Deploy all LiDAR scanner configs to Jetson TX2
# Usage: bash scripts/deploy.sh [user@host]

set -e

TARGET="${1:-nvidia@192.168.31.39}"
PASS="nvidia"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

SSH="sshpass -p $PASS ssh -o StrictHostKeyChecking=no"
SCP="sshpass -p $PASS scp -o StrictHostKeyChecking=no"

echo "=== Deploying to $TARGET ==="

# Web UI
echo "[1/4] Deploying Flask web UI..."
$SSH "$TARGET" "mkdir -p ~/lidar-webui/templates"
$SCP "$REPO_ROOT/webui/app.py" "$TARGET:~/lidar-webui/app.py"
$SCP "$REPO_ROOT/webui/templates/index.html" "$TARGET:~/lidar-webui/templates/index.html"

# Config files
echo "[2/4] Deploying config files..."
$SCP "$REPO_ROOT/config/smb.conf" "$TARGET:/tmp/smb.conf"
$SCP "$REPO_ROOT/config/wifi-fallback.sh" "$TARGET:/tmp/wifi-fallback.sh"
$SCP "$REPO_ROOT/config/lidar-hotspot.service" "$TARGET:/tmp/"
$SCP "$REPO_ROOT/config/lidar-hotspot.timer" "$TARGET:/tmp/"
$SCP "$REPO_ROOT/config/lidar-webui.service" "$TARGET:/tmp/"

# Apply configs on target
echo "[3/4] Applying configs..."
$SSH "$TARGET" "echo $PASS | sudo -S bash -s" << 'EOF'
cp /tmp/smb.conf /etc/samba/smb.conf
cp /tmp/wifi-fallback.sh /usr/local/bin/wifi-fallback.sh
chmod +x /usr/local/bin/wifi-fallback.sh
cp /tmp/lidar-hotspot.service /etc/systemd/system/
cp /tmp/lidar-hotspot.timer /etc/systemd/system/
cp /tmp/lidar-webui.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable lidar-webui.service lidar-hotspot.timer
systemctl enable smbd nmbd
systemctl restart lidar-webui.service smbd nmbd
systemctl start lidar-hotspot.timer
EOF

# Verify
echo "[4/4] Verifying services..."
$SSH "$TARGET" "sudo systemctl is-active lidar-webui smbd lidar-hotspot.timer 2>/dev/null"

echo ""
echo "=== Deploy complete ==="
echo "Web UI: http://$(echo $TARGET | cut -d@ -f2):5000"
echo "Samba:  smb://$(echo $TARGET | cut -d@ -f2)/lidar_data"
