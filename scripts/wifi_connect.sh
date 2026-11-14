#!/bin/bash
# wifi_connect.sh — verbindet mit dem stärksten bekannten Netz, sonst Hotspot.
#   wifi_connect.sh          verbinden (tut nichts, wenn schon mit einem Netz verbunden)
#   wifi_connect.sh --list   nur die sichtbaren bekannten Netze ausgeben, stärkstes zuerst
# Läuft als root (wifi-watchdog.service). Netze: /home/nvidia/config/wifi.conf
# Format: Zeile "SSID|PASSWORT" unter [known_networks]; SSID darf Leerzeichen enthalten.

CONFIG="${WIFI_CONF:-/home/nvidia/config/wifi.conf}"  # WIFI_CONF nur für Tests
LOG="/tmp/wifi.log"
MODE_FILE="/tmp/wifi_mode"
AP_SSID="LidarScanner"
AP_PASS="lidar1234"

log() { echo "[$(date '+%H:%M:%S')] [WiFi] $*" | tee -a "$LOG"; }

# --- Config einlesen ---
declare -A KNOWN_PASS
SECTION=""
while IFS= read -r line || [[ -n "$line" ]]; do
    [[ "$line" =~ ^[[:space:]]*# || -z "${line// }" ]] && continue
    if [[ "$line" =~ ^\[(.+)\]$ ]]; then SECTION="${BASH_REMATCH[1]}"; continue; fi
    if [[ "$SECTION" == "known_networks" && "$line" == *"|"* ]]; then
        SSID="${line%%|*}"; PASS="${line##*|}"
        [[ -n "$SSID" && -n "$PASS" ]] && KNOWN_PASS["$SSID"]="$PASS"
    fi
    if [[ "$SECTION" == "hotspot" ]]; then
        KEY="${line%%=*}"; VAL="${line#*=}"
        [[ "$KEY" == "SSID" ]] && AP_SSID="$VAL"
        [[ "$KEY" == "PASSWORD" ]] && AP_PASS="$VAL"
    fi
done < "$CONFIG"

active_ssid() { nmcli -t -f active,ssid dev wifi 2>/dev/null | sed -n 's/^yes://p' | head -1 | sed 's/\\:/:/g'; }
wlan_ip()     { ip -4 addr show wlan0 | grep -oP "(?<=inet )[\d.]+" | head -1; }

# Sichtbare bekannte Netze, stärkstes zuerst, jedes nur einmal
visible_known() {
    nmcli -t -f SIGNAL,SSID dev wifi list ifname wlan0 2>/dev/null | while IFS=: read -r SIG SSID; do
        SSID="${SSID//\\:/:}"
        [[ -n "$SSID" && -n "${KNOWN_PASS[$SSID]+x}" ]] && echo "$SIG $SSID"
    done | sort -rn | awk '{ s = substr($0, index($0, " ") + 1) } !seen[s]++ { print s }'
}

if [[ "$1" == "--list" ]]; then
    nmcli dev wifi rescan ifname wlan0 2>/dev/null; sleep 4
    visible_known
    exit 0
fi

# --- Schon verbunden? Der eigene Hotspot zählt nicht. Bestehende Verbindung nie unterbrechen. ---
CUR=$(active_ssid)
if [[ -n "$CUR" && "$CUR" != "$AP_SSID" && -n "$(wlan_ip)" ]]; then
    log "Bereits verbunden mit '$CUR' ($(wlan_ip)) — nichts zu tun"
    echo "client" > "$MODE_FILE"
    exit 0
fi

log "Nicht verbunden — bekannte Netzwerke: ${!KNOWN_PASS[*]}"

# --- Band-Lock entfernen ---
for CON in $(nmcli -t -f NAME,TYPE connection show | grep wireless | cut -d: -f1); do
    nmcli connection modify "$CON" 802-11-wireless.band "" 802-11-wireless.bssid "" 2>/dev/null
done

# --- Scan, dann alle sichtbaren bekannten Netze der Reihe nach versuchen ---
nmcli dev wifi rescan ifname wlan0 2>/dev/null
sleep 5

while IFS= read -r S; do
    [[ -z "$S" ]] && continue
    log "Verbinde mit '$S'..."
    P="${KNOWN_PASS[$S]}"
    if nmcli connection show "$S" &>/dev/null; then
        nmcli connection modify "$S" 802-11-wireless-security.psk "$P" 2>/dev/null
        nmcli connection up "$S" ifname wlan0 2>/dev/null
    else
        nmcli dev wifi connect "$S" password "$P" ifname wlan0 2>/dev/null
    fi
    sleep 6
    if [[ "$(active_ssid)" == "$S" && -n "$(wlan_ip)" ]]; then
        log "Verbunden mit '$S' — IP: $(wlan_ip)"
        echo "client" > "$MODE_FILE"
        exit 0
    fi
    log "Verbindung mit '$S' fehlgeschlagen"
done < <(visible_known)

# --- Kein Netz gefunden oder verbunden: Hotspot ---
log "Erstelle Hotspot '$AP_SSID'"
nmcli connection delete "$AP_SSID" 2>/dev/null
nmcli dev wifi hotspot ifname wlan0 ssid "$AP_SSID" password "$AP_PASS" 2>/dev/null
sleep 3
log "Hotspot aktiv — IP: $(wlan_ip)"
echo "hotspot" > "$MODE_FILE"
