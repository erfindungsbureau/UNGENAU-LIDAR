#!/bin/bash
# process_scan.sh BAG -- nach dem Scan: Bahn mit FAST-LIO2 neu rechnen (ohne Zeitdruck, alle Punkte),
# dann jede Rohmessung in 1-cm-Wuerfel -> BAG ohne .bag + _1cm.ply, dann ebene Flaechen glaetten
# -> _1cm_glatt.ply. Fortschritt in /tmp/process.log
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
echo "PHASE glatt"
# nicht fatal: scheitert das Glaetten (z.B. Speicher), bleibt das 1-cm-Modell trotzdem gueltig
if python3 -u $D/flaechen_glaetten.py ${STEM}_1cm.ply ${STEM}_1cm_glatt.ply.part </dev/null; then
  mv ${STEM}_1cm_glatt.ply.part ${STEM}_1cm_glatt.ply
else
  rm -f ${STEM}_1cm_glatt.ply.part; echo "WARNUNG Glaetten fehlgeschlagen, nur _1cm.ply"
fi
# 3D-Vorschau fuers Web-UI (Klick auf den Scan); nicht fatal, sonst entsteht sie beim ersten Klick
M=${STEM}_1cm.ply; [ -f ${STEM}_1cm_glatt.ply ] && M=${STEM}_1cm_glatt.ply
python3 $D/ply_vorschau.py $M $WORK/$(basename $STEM)_vorschau </dev/null || echo "WARNUNG Vorschau fehlgeschlagen"
echo "PHASE fertig"
