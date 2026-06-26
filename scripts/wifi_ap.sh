#!/bin/bash
# wifi_connect.sh — verbindet mit stärkstem bekannten Netz oder Hotspot-Fallback
# Unterbricht NIEMALS eine bestehende funktionierende Verbindung.

CONFIG="/home/nvidia/config/wifi.conf"
LOG="/tmp/wifi.log"
MODE_FILE="/tmp/wifi_mode"
AP_SSID="LidarScanner"
AP_PASS="lidar1234"

log() { echo "[$(date '+%H:%M:%S')] [WiFi] $*" | tee -a "$LOG"; }

# --- Prüfen ob bereits verbunden (Ping-Test ist zuverlässiger als nmcli) ---
if ping -c 1 -W 3 1.1.1.1 &>/dev/null; then
    CURRENT=$(nmcli -t -f NAME connection show --active 2>/dev/null | grep -v "^lo\|docker\|tailscale\|eth" | head -1)
    IP=$(ip -4 addr show wlan0 | grep -oP "(?<=inet )[\d.]+" | head -1)
    log "Bereits verbunden${CURRENT:+ mit '$CURRENT'} ($IP) — nichts zu tun"
    echo "client" > "$MODE_FILE"
    exit 0
fi

# --- Config einlesen ---
# Parallele Arrays statt assoc array (funktioniert auch mit Leerzeichen in SSIDs)
NET_SSIDS=()
NET_PASSES=()
SECTION=""
while IFS= read -r line; do
    [[ "$line" =~ ^[[:space:]]*# || -z "${line// }" ]] && continue
    if [[ "$line" =~ ^\[(.+)\]$ ]]; then SECTION="${BASH_REMATCH[1]}"; continue; fi
    if [[ "$SECTION" == "known_networks" && "$line" == *"|"* ]]; then
        SSID="${line%%|*}"; PASS="${line##*|}"
        [[ -n "$SSID" && -n "$PASS" ]] && NET_SSIDS+=("$SSID") && NET_PASSES+=("$PASS")
    fi
    if [[ "$SECTION" == "hotspot" ]]; then
        KEY="${line%%=*}"; VAL="${line##*=}"
        [[ "$KEY" == "SSID" ]] && AP_SSID="$VAL"
        [[ "$KEY" == "PASSWORD" ]] && AP_PASS="$VAL"
    fi
done < "$CONFIG"

log "Nicht verbunden — suche Netzwerke: ${NET_SSIDS[*]}"

# --- Band-Lock entfernen (verhindert 5GHz/2.4GHz Probleme) ---
for CON in $(nmcli -t -f NAME,TYPE connection show | grep wireless | cut -d: -f1); do
    nmcli connection modify "$CON" 802-11-wireless.band "" 802-11-wireless.bssid "" 2>/dev/null
done

# --- Scan ---
nmcli dev wifi rescan ifname wlan0 2>/dev/null
sleep 5

# Stärkstes bekanntes Netz finden
BEST_SSID=""
BEST_SIGNAL=-999
BEST_PASS=""
while IFS=: read -r SCAN_SSID SIGNAL; do
    SCAN_SSID="${SCAN_SSID//\*/}"; SCAN_SSID="${SCAN_SSID# }"; SCAN_SSID="${SCAN_SSID% }"
    SIGNAL="${SIGNAL// /}"
    for i in "${!NET_SSIDS[@]}"; do
        if [[ "${NET_SSIDS[$i]}" == "$SCAN_SSID" ]] && [[ "${SIGNAL:-0}" -gt "$BEST_SIGNAL" ]]; then
            BEST_SSID="$SCAN_SSID"
            BEST_SIGNAL="${SIGNAL:-0}"
            BEST_PASS="${NET_PASSES[$i]}"
        fi
    done
done < <(nmcli -t -f SSID,SIGNAL dev wifi list ifname wlan0 2>/dev/null)

# --- Verbinden ---
if [[ -n "$BEST_SSID" ]]; then
    log "Verbinde mit '$BEST_SSID' (Signal: ${BEST_SIGNAL}%)..."
    if nmcli connection show "$BEST_SSID" &>/dev/null; then
        nmcli connection modify "$BEST_SSID" 802-11-wireless-security.psk "$BEST_PASS" 2>/dev/null
        nmcli connection up "$BEST_SSID" 2>/dev/null
    else
        nmcli dev wifi connect "$BEST_SSID" password "$BEST_PASS" 2>/dev/null
    fi
    sleep 6
    if ping -c 1 -W 3 1.1.1.1 &>/dev/null; then
        IP=$(ip -4 addr show wlan0 | grep -oP "(?<=inet )[\d.]+" | head -1)
        log "Verbunden mit '$BEST_SSID' — IP: $IP"
        echo "client" > "$MODE_FILE"
        exit 0
    fi
    log "Verbindung mit '$BEST_SSID' fehlgeschlagen"
fi

# --- Hotspot ---
log "Erstelle Hotspot '$AP_SSID'"
nmcli connection delete "$AP_SSID" 2>/dev/null
nmcli dev wifi hotspot ifname wlan0 ssid "$AP_SSID" password "$AP_PASS" 2>/dev/null
sleep 3
IP=$(ip -4 addr show wlan0 | grep -oP "(?<=inet )[\d.]+" | head -1)
log "Hotspot aktiv — IP: $IP"
echo "hotspot" > "$MODE_FILE"
