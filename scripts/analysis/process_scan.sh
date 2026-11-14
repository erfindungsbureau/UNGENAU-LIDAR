#!/bin/bash
# process_scan.sh BAG -- nach dem Scan: Bahn mit FAST-LIO2 neu rechnen (ohne Zeitdruck, alle Punkte),
# dann jede Rohmessung in 1-cm-Wuerfel -> BAG ohne .bag + _1cm.ply. Fortschritt in /tmp/process.log
# (Zeilen "PHASE ..." und "FORTSCHRITT k n"), rosbag play in /tmp/process_play.log. Gestartet von app.py.
set -e
BAG=$1; STEM=${BAG%.bag}; WORK=/media/nvidia/SSD/lidar_data/work; mkdir -p $WORK
POSES=$WORK/$(basename $STEM)_poses.bag
D=$(dirname $0)
echo "PHASE bahn"
PLAY_LOG=/tmp/process_play.log $D/fl_replay.sh $BAG $POSES 3 0.5
echo "PHASE wolke"
source /opt/ros/melodic/setup.bash
python2 -u $D/dense_export.py $BAG ${STEM}_1cm.ply 0.01 0.8 10 - $POSES </dev/null
echo "PHASE fertig"
