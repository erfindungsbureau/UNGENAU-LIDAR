#!/bin/bash
# Verbindet mit dem stärksten bekannten WLAN aus config/wifi.conf
# Erstellt Hotspot wenn kein bekanntes Netz erreichbar ist

CONFIG="/home/nvidia/config/wifi.conf"
LOG_PREFIX="[WiFi]"

# Hotspot-Defaults (überschrieben durch wifi.conf)
AP_SSID="LidarScanner"
AP_PASS="lidar1234"

# --- Config einlesen ---
declare -A KNOWN_NETWORKS
SECTION=""
while IFS= read -r line; do
    # Kommentare und Leerzeilen überspringen
    [[ "$line" =~ ^[[:space:]]*# ]] && continue
    [[ -z "${line// }" ]] && continue
    # Sektion erkennen
    if [[ "$line" =~ ^\[(.+)\]$ ]]; then
        SECTION="${BASH_REMATCH[1]}"
        continue
    fi
    if [[ "$SECTION" == "known_networks" ]]; then
        SSID="${line%%|*}"
        PASS="${line##*|}"
        [[ -n "$SSID" && -n "$PASS" ]] && KNOWN_NETWORKS["$SSID"]="$PASS"
    fi
    if [[ "$SECTION" == "hotspot" ]]; then
        KEY="${line%%=*}"
        VAL="${line##*=}"
        [[ "$KEY" == "SSID" ]] && AP_SSID="$VAL"
        [[ "$KEY" == "PASSWORD" ]] && AP_PASS="$VAL"
    fi
done < "$CONFIG"

if [[ ${#KNOWN_NETWORKS[@]} -eq 0 ]]; then
    echo "$LOG_PREFIX Keine bekannten Netzwerke in $CONFIG konfiguriert"
fi

# --- Stärkstes verfügbares bekanntes Netz finden ---
echo "$LOG_PREFIX Suche bekannte Netzwerke (warte 10s auf Scan)..."
sleep 3
nmcli dev wifi rescan 2>/dev/null
sleep 5

BEST_SSID=""
BEST_SIGNAL=-999
while IFS= read -r line; do
    SSID=$(echo "$line" | awk -F'  +' '{print $2}' | xargs)
    SIGNAL=$(echo "$line" | grep -oP '\d+(?= +\S+$)' | tail -1)
    if [[ -n "${KNOWN_NETWORKS[$SSID]}" ]] && [[ "${SIGNAL:-0}" -gt "$BEST_SIGNAL" ]]; then
        BEST_SSID="$SSID"
        BEST_SIGNAL="${SIGNAL:-0}"
    fi
done < <(nmcli -t -f SSID,SIGNAL dev wifi list 2>/dev/null)

# --- Verbinden ---
if [[ -n "$BEST_SSID" ]]; then
    echo "$LOG_PREFIX Verbinde mit '$BEST_SSID' (Signal: ${BEST_SIGNAL}%)..."
    PASS="${KNOWN_NETWORKS[$BEST_SSID]}"

    # Existierende Verbindung prüfen, sonst neu anlegen
    if nmcli connection show "$BEST_SSID" &>/dev/null; then
        nmcli connection up "$BEST_SSID" 2>/dev/null
    else
        nmcli dev wifi connect "$BEST_SSID" password "$PASS" 2>/dev/null
    fi

    sleep 5
    if nmcli -t -f STATE general | grep -q "connected"; then
        IP=$(ip -4 addr show wlan0 | grep -oP "(?<=inet )[\d.]+" | head -1)
        echo "$LOG_PREFIX Verbunden mit '$BEST_SSID'"
        echo "$LOG_PREFIX Web Interface: http://$IP:5000"
        exit 0
    else
        echo "$LOG_PREFIX Verbindung mit '$BEST_SSID' fehlgeschlagen"
    fi
fi

# --- Hotspot erstellen ---
echo "$LOG_PREFIX Kein bekanntes Netz erreichbar — starte Hotspot '$AP_SSID'"
nmcli dev wifi hotspot \
    ifname wlan0 \
    ssid "$AP_SSID" \
    password "$AP_PASS" 2>&1

sleep 3
IP=$(ip -4 addr show wlan0 | grep -oP "(?<=inet )[\d.]+" | head -1)
echo "$LOG_PREFIX Hotspot aktiv: $AP_SSID"
echo "$LOG_PREFIX Web Interface: http://$IP:5000"
