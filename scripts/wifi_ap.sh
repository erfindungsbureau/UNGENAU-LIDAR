#!/bin/bash
# Startet WiFi Hotspot "LidarScanner" wenn kein bekanntes Netz verfügbar

KNOWN_SSID="StudioFiume"
AP_SSID="LidarScanner"
AP_PASS="lidar1234"

echo "[WiFi] Prüfe Verbindung zu $KNOWN_SSID..."

# Warte max 15s auf bekanntes Netz
for i in $(seq 1 15); do
    if nmcli -t -f SSID dev wifi | grep -q "^$KNOWN_SSID$" 2>/dev/null; then
        nmcli dev wifi connect "$KNOWN_SSID" 2>/dev/null
        sleep 3
        if nmcli -t -f STATE general | grep -q "connected"; then
            echo "[WiFi] Verbunden mit $KNOWN_SSID"
            IP=$(ip -4 addr show wlan0 | grep -oP "(?<=inet )[\d.]+")
            echo "[WiFi] IP: $IP"
            exit 0
        fi
    fi
    sleep 1
done

echo "[WiFi] $KNOWN_SSID nicht gefunden - starte Hotspot $AP_SSID"

# Hotspot erstellen
nmcli dev wifi hotspot \
    ifname wlan0 \
    ssid "$AP_SSID" \
    password "$AP_PASS" 2>&1

sleep 3
IP=$(ip -4 addr show wlan0 | grep -oP "(?<=inet )[\d.]+")
echo "[WiFi] Hotspot aktiv: $AP_SSID"
echo "[WiFi] IP: $IP"
echo "[WiFi] Web Interface: http://$IP:5000"
