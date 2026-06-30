#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
bag_to_xyz.py  --  rosbag -> XYZ fuer ArchiCAD
Verwendet /lio_sam/mapping/cloud_registered (SLAM) wenn vorhanden,
sonst /ouster/points (Rohdaten, nur fuer statische Scans).

Aufruf: python bag_to_xyz.py input.bag output.xyz [frame_step] [max_sec]
"""
import sys
import rosbag
import sensor_msgs.point_cloud2 as pc2

bag_file   = sys.argv[1]
xyz_file   = sys.argv[2]
frame_step = int(sys.argv[3])   if len(sys.argv) > 3 else 1
max_sec    = float(sys.argv[4]) if len(sys.argv) > 4 else None

SLAM_TOPIC = '/lio_sam/mapping/cloud_registered'
RAW_TOPIC  = '/ouster/points'

with rosbag.Bag(bag_file, 'r') as bag:
    available = bag.get_type_and_topic_info().topics.keys()

if SLAM_TOPIC in available:
    topic = SLAM_TOPIC
    print("Modus: SLAM-korrigiert (%s)" % SLAM_TOPIC)
else:
    topic = RAW_TOPIC
    print("WARNUNG: Kein SLAM-Topic im Bag -- Rohdaten werden exportiert")
    print("  (Bewegung des Scanners fuehrt zu Verzerrungen)")

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
            for p in pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True):
                x, y, z = float(p[0]), float(p[1]), float(p[2])
                if x*x + y*y + z*z < 0.25:
                    continue
                f_out.write('%.4f %.4f %.4f\n' % (x, y, z))
                count += 1

elapsed = (ts - t0) if t0 else 0
print("Fertig: %d Punkte, %d Frames, %.1fs" % (count, frame, elapsed))
