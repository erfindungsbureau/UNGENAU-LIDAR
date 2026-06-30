#!/bin/bash
# wifi_connect.sh — verbindet mit stärkstem bekannten Netz oder Hotspot-Fallback
# Unterbricht NIEMALS eine bestehende funktionierende Verbindung.

CONFIG="/home/nvidia/config/wifi.conf"
LOG="/tmp/wifi.log"
MODE_FILE="/tmp/wifi_mode"
AP_SSID="LidarScanner"
AP_PASS="lidar1234"

log() { echo "[$(date '+%H:%M:%S')] [WiFi] $*" | tee -a "$LOG"; }

# --- Prüfen ob bereits gut verbunden ---
CURRENT_SSID=$(nmcli -t -f active,ssid dev wifi | grep '^yes' | cut -d: -f2)
STATE=$(nmcli -t -f STATE general | head -1)
if [[ "$STATE" == "connected" && -n "$CURRENT_SSID" ]]; then
    IP=$(ip -4 addr show wlan0 | grep -oP "(?<=inet )[\d.]+" | head -1)
    log "Bereits verbunden mit '$CURRENT_SSID' ($IP) — nichts zu tun"
    echo "client" > "$MODE_FILE"
    exit 0
fi

# --- Config einlesen ---
declare -A KNOWN_PASS
SECTION=""
while IFS= read -r line; do
    [[ "$line" =~ ^[[:space:]]*# || -z "${line// }" ]] && continue
    if [[ "$line" =~ ^\[(.+)\]$ ]]; then SECTION="${BASH_REMATCH[1]}"; continue; fi
    if [[ "$SECTION" == "known_networks" && "$line" == *"|"* ]]; then
        SSID="${line%%|*}"; PASS="${line##*|}"
        [[ -n "$SSID" && -n "$PASS" ]] && KNOWN_PASS["$SSID"]="$PASS"
    fi
    if [[ "$SECTION" == "hotspot" ]]; then
        KEY="${line%%=*}"; VAL="${line##*=}"
        [[ "$KEY" == "SSID" ]] && AP_SSID="$VAL"
        [[ "$KEY" == "PASSWORD" ]] && AP_PASS="$VAL"
    fi
done < "$CONFIG"

log "Nicht verbunden — suche bekannte Netzwerke: ${!KNOWN_PASS[*]}"

# --- Band-Lock entfernen ---
for CON in $(nmcli -t -f NAME,TYPE connection show | grep wireless | cut -d: -f1); do
    nmcli connection modify "$CON" 802-11-wireless.band "" 802-11-wireless.bssid "" 2>/dev/null
done

# --- Scan ---
nmcli dev wifi rescan ifname wlan0 2>/dev/null
sleep 5

BEST_SSID=""
BEST_SIGNAL=-999
while IFS=: read -r SSID SIGNAL; do
    SSID="${SSID//\*/}"; SSID="${SSID// /}"
    SIGNAL="${SIGNAL// /}"
    if [[ -n "${KNOWN_PASS[$SSID]}" ]] && [[ "${SIGNAL:-0}" -gt "$BEST_SIGNAL" ]]; then
        BEST_SSID="$SSID"; BEST_SIGNAL="${SIGNAL:-0}"
    fi
done < <(nmcli -t -f SSID,SIGNAL dev wifi list ifname wlan0 2>/dev/null)

# --- Verbinden ---
if [[ -n "$BEST_SSID" ]]; then
    log "Verbinde mit '$BEST_SSID' (Signal: ${BEST_SIGNAL}%)..."
    PASS="${KNOWN_PASS[$BEST_SSID]}"
    if nmcli connection show "$BEST_SSID" &>/dev/null; then
        nmcli connection modify "$BEST_SSID" 802-11-wireless-security.psk "$PASS" 2>/dev/null
        nmcli connection up "$BEST_SSID" 2>/dev/null
    else
        nmcli dev wifi connect "$BEST_SSID" password "$PASS" 2>/dev/null
    fi
    sleep 6
    STATE=$(nmcli -t -f STATE general | head -1)
    IP=$(ip -4 addr show wlan0 | grep -oP "(?<=inet )[\d.]+" | head -1)
    if [[ "$STATE" == "connected" && -n "$IP" ]]; then
        log "Verbunden mit '$BEST_SSID' — IP: $IP"
        echo "client" > "$MODE_FILE"
        exit 0
    fi
    log "Verbindung fehlgeschlagen"
fi

# --- Hotspot ---
log "Erstelle Hotspot '$AP_SSID'"
nmcli connection delete "$AP_SSID" 2>/dev/null
nmcli dev wifi hotspot ifname wlan0 ssid "$AP_SSID" password "$AP_PASS" 2>/dev/null
sleep 3
IP=$(ip -4 addr show wlan0 | grep -oP "(?<=inet )[\d.]+" | head -1)
log "Hotspot aktiv — IP: $IP"
echo "hotspot" > "$MODE_FILE"
