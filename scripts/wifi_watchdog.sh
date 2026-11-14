#!/bin/bash
# wifi_watchdog.sh — überwacht WiFi, reconnect bei Verlust, Hotspot-Fallback

LOG="/tmp/wifi.log"
MODE_FILE="/tmp/wifi_mode"
CONNECT="/home/nvidia/UNGENAU-LIDAR/scripts/wifi_connect.sh"
FAIL_COUNT=0
MAX_FAILS=3
HOTSPOT_TICK=0

log() { echo "[$(date '+%H:%M:%S')] [Watchdog] $*" | tee -a "$LOG"; }

log "Watchdog gestartet"

while true; do
    sleep 30
    MODE=$(cat "$MODE_FILE" 2>/dev/null || echo "unknown")

    if [[ "$MODE" == "client" ]]; then
        # Verbindung prüfen via Ping
        if ping -c 1 -W 3 1.1.1.1 &>/dev/null; then
            FAIL_COUNT=0
        else
            FAIL_COUNT=$((FAIL_COUNT + 1))
            log "Ping fehlgeschlagen ($FAIL_COUNT/$MAX_FAILS)"
            if [[ $FAIL_COUNT -ge $MAX_FAILS ]]; then
                log "Verbindung verloren — reconnect..."
                FAIL_COUNT=0
                bash "$CONNECT"
            fi
        fi

    elif [[ "$MODE" == "hotspot" ]]; then
        # Alle 60 s prüfen, ob ein bekanntes Netz sichtbar ist (--list scannt selbst)
        HOTSPOT_TICK=$((HOTSPOT_TICK + 1))
        if [[ $HOTSPOT_TICK -ge 2 ]]; then
            HOTSPOT_TICK=0
            FOUND=$(bash "$CONNECT" --list | head -1)
            if [[ -n "$FOUND" ]]; then
                log "Bekanntes Netz '$FOUND' gefunden — wechsle zu Client"
                bash "$CONNECT"
            fi
        fi

    else
        bash "$CONNECT"
    fi
done
