#!/bin/bash
# Watchdog: überwacht bridge.py, sbg_driver und sbg_imu_combiner

LOG=/tmp/watchdog.log
ROS_ENV="source /opt/ros/melodic/setup.bash && source /home/nvidia/catkin_ws/devel/setup.bash"

log() { echo "[$(date '+%H:%M:%S')] $*" | tee -a $LOG; }

start_bridge() {
    log "BRIDGE: Starte bridge.py..."
    nohup bash -c "$ROS_ENV && python2 /home/nvidia/bridge.py" > /tmp/bridge.log 2>&1 &
    sleep 5
    pgrep -f "python2.*bridge.py" > /dev/null && log "BRIDGE: OK" || log "BRIDGE: WARNUNG"
}

start_sbg_driver() {
    log "SBG_DRIVER: Starte sbg_driver + combiner..."
    nohup bash -c "$ROS_ENV && roslaunch sbg_driver sbg_device.launch" > /tmp/sbg_driver.log 2>&1 &
    sleep 5
    nohup bash -c "$ROS_ENV && python /home/nvidia/sbg_imu_combiner.py" > /tmp/sbg_combiner.log 2>&1 &
    sleep 2
    pgrep -f "sbg_device" > /dev/null && log "SBG_DRIVER: OK" || log "SBG_DRIVER: WARNUNG"
    pgrep -f "sbg_imu_combiner" > /dev/null && log "SBG_COMBINER: OK" || log "SBG_COMBINER: WARNUNG"
}

log "=== Watchdog gestartet ==="

pgrep -f "python2.*bridge.py" > /dev/null || start_bridge
pgrep -f "sbg_device" > /dev/null || ([ -e /dev/ttyUSB0 ] && start_sbg_driver)

while true; do
    pgrep -f "python2.*bridge.py" > /dev/null || { log "BRIDGE: Prozess weg!"; start_bridge; }
    if [ -e /dev/ttyUSB0 ]; then
        pgrep -f "sbg_device" > /dev/null || { log "SBG_DRIVER: Prozess weg!"; start_sbg_driver; }
        pgrep -f "sbg_imu_combiner" > /dev/null || {
            pgrep -f "sbg_device" > /dev/null && {
                log "SBG_COMBINER: Prozess weg! Neustart..."
                nohup bash -c "$ROS_ENV && python /home/nvidia/sbg_imu_combiner.py" > /tmp/sbg_combiner.log 2>&1 &
            }
        }
    fi
    sleep 30
done
