#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
bag_to_xyz.py  --  rosbag -> XYZ fuer ArchiCAD
Verwendet /lio_sam/mapping/cloud_registered (SLAM) wenn vorhanden,
sonst /ouster/points (Rohdaten, nur fuer statische Scans).

Aufruf: python bag_to_xyz.py input.bag output.xyz [frame_step] [max_sec]
"""
import sys
import numpy as np
import rosbag
import level
import sensor_msgs.point_cloud2 as pc2

bag_file   = sys.argv[1]
xyz_file   = sys.argv[2]
frame_step = int(sys.argv[3])   if len(sys.argv) > 3 else 1
max_sec    = float(sys.argv[4]) if len(sys.argv) > 4 else None

SLAM_TOPICS = ['/lio_sam/mapping/cloud_registered', '/cloud_registered']
RAW_TOPIC   = '/ouster/points'

with rosbag.Bag(bag_file, 'r') as bag:
    available = bag.get_type_and_topic_info().topics.keys()

topic = None
for t in SLAM_TOPICS:
    if t in available:
        topic = t
        print("Modus: SLAM-korrigiert (%s)" % topic)
        break
if topic is None:
    topic = RAW_TOPIC
    print("WARNUNG: Kein SLAM-Topic im Bag -- Rohdaten werden exportiert")
    print("  (Bewegung des Scanners fuehrt zu Verzerrungen)")

# FAST-LIO2 legt die Karte in die Startlage des Scanners -> nach der Schwerkraft ausrichten
L = np.eye(3)
if topic == '/cloud_registered':
    L_, tilt, n = level.level_rotation(bag_file)
    if L_ is not None:
        L = L_
        print("Ausrichtung nach Schwerkraft: Karte um %.2f Grad gedreht (%d IMU-Werte)" % (tilt, n))
    else:
        print("WARNUNG: keine /Odometry- und /imu/data-Daten -- Karte nicht ausgerichtet")

print("Exportiere: %s -> %s  (step=%d, max_sec=%s)" % (
    bag_file, xyz_file, frame_step, str(max_sec)))

count = 0
frame = 0
t0    = None
ts    = 0

with open(xyz_file, 'w') as f_out:
    with rosbag.Bag(bag_file, 'r') as bag:
        for _topic, msg, t in bag.read_messages(topics=[topic]):
            ts = t.to_sec()
            if t0 is None:
                t0 = ts
            if max_sec is not None and (ts - t0) > max_sec:
                break
            frame += 1
            if frame % frame_step != 0:
                continue
            pts = np.array(list(pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True)), dtype=np.float64)
            if len(pts) == 0:
                continue
            pts = pts[(pts ** 2).sum(axis=1) >= 0.25].dot(L.T)
            for x, y, z in pts:
                f_out.write('%.4f %.4f %.4f\n' % (x, y, z))
            count += len(pts)

elapsed = (ts - t0) if t0 else 0
print("Fertig: %d Punkte, %d Frames, %.1fs" % (count, frame, elapsed))
