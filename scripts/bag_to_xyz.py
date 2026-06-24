#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
bag_to_xyz.py - Konvertiert rosbag /ouster/points -> XYZ Punktwolke
Aufruf: python2 bag_to_xyz.py input.bag output.xyz [frame_step]
"""
import sys, os, rosbag
import sensor_msgs.point_cloud2 as pc2

bag_file  = sys.argv[1]
xyz_file  = sys.argv[2]
frame_step = int(sys.argv[3]) if len(sys.argv) > 3 else 5  # jeder 5. Frame

print("Exportiere: %s -> %s (jeder %d. Frame)" % (bag_file, xyz_file, frame_step))

count = 0
frame = 0
with open(xyz_file, 'w') as f_out:
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/ouster/points']):
            frame += 1
            if frame % frame_step != 0:
                continue
            for p in pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True):
                x, y, z = float(p[0]), float(p[1]), float(p[2])
                if x*x + y*y + z*z < 0.25:
                    continue
                f_out.write('%.4f %.4f %.4f\n' % (x, y, z))
                count += 1

print("Fertig: %d Punkte aus %d Frames" % (count, frame))
