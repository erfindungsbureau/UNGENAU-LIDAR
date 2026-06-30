#!/bin/bash
# Deploy: lokale Änderungen auf den Jetson bringen und Dienste neu starten.
#
# Das Git-Repo UNGENAU-LIDAR liegt auf dem Jetson unter /home/nvidia/UNGENAU-LIDAR
# und ist dort der EINZIGE Ort an dem der Code liegt (keine Kopien, keine Symlinks).
# Alle Skripte/Configs/Launch-Files referenzieren absolute Pfade ins Repo.
#
# Workflow:
#   1. Lokal committen + pushen (git push origin main)
#   2. bash scripts/deploy.sh ausführen -> zieht auf dem Jetson (git pull) und
#      startet die betroffenen systemd-Dienste neu.
#
# Usage: bash scripts/deploy.sh [user@host]

set -e

TARGET="${1:-nvidia@100.64.0.25}"
PASS="nvidia"
SSH="sshpass -p $PASS ssh -o StrictHostKeyChecking=no"

echo "=== Deploy auf $TARGET ==="

echo "[1/3] git pull auf dem Jetson..."
$SSH "$TARGET" "cd /home/nvidia/UNGENAU-LIDAR && git pull --ff-only"

echo "[2/3] lidar-system.service neu starten (Flask, ROS-Pipeline)..."
$SSH "$TARGET" "echo $PASS | sudo -S systemctl restart lidar-system.service"

echo "[3/3] Status prüfen..."
sleep 3
$SSH "$TARGET" "systemctl is-active lidar-system.service"
$SSH "$TARGET" "curl -s --max-time 5 http://localhost:5000/api/status | python3 -m json.tool | head -10"

echo ""
echo "=== Deploy abgeschlossen ==="
echo "Web UI: http://$(echo $TARGET | cut -d@ -f2):5000"
