#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
floor_check.py BAG [imu_topic] -- Neigung des Bodens in /cloud_registered, vor und nach level.py.
Unabhaengige Kontrolle der Schwerkraft-Ausrichtung (nutzt nur die Punktwolke).
Boden = groesste waagrechte Ebene im unteren Teil der Karte.
"""
import os, sys, math
import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import level

bag_file = sys.argv[1]
imu_topic = sys.argv[2] if len(sys.argv) > 2 else level.IMU_TOPIC

pts = []
with rosbag.Bag(bag_file, 'r') as bag:
    for i, (_, msg, _) in enumerate(bag.read_messages(topics=['/cloud_registered'])):
        if i % 5:
            continue
        p = np.array(list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)))
        if len(p):
            pts.append(p[::4])
P = np.vstack(pts)


def floor_plane(P):
    """Normale (nach oben) und Punktzahl der Bodenebene."""
    z = P[:, 2]
    lo = np.percentile(z, 1)
    hist, edges = np.histogram(z, bins=np.arange(lo - 0.05, lo + 0.8, 0.03))
    zf = edges[np.argmax(hist)] + 0.015
    sel = P[np.abs(z - zf) < 0.15]
    for _ in range(6):  # Ebene einpassen, Ausreisser verwerfen
        c = sel.mean(axis=0)
        n = np.linalg.svd(sel - c, full_matrices=False)[2][2]
        if n[2] < 0:
            n = -n
        d = (P - c).dot(n)
        sel = P[np.abs(d) < 0.03]
    return n, len(sel)


def report(name, n, k):
    tilt = math.degrees(math.acos(min(1.0, n[2])))
    print("%-16s Neigung Boden %.2f Grad = %.1f mm/m  (Gefaelle x %.1f, y %.1f mm/m; %d Punkte)" % (
        name, tilt, 1000 * math.tan(math.radians(tilt)), -1000 * n[0] / n[2], -1000 * n[1] / n[2], k))


L, tilt, n = level.level_rotation(bag_file, imu_topic=imu_topic)
if L is None:
    sys.exit("Keine /Odometry- bzw. IMU-Daten im Bag")
# Boden in der ausgerichteten Karte suchen (in einer schiefen Karte verschmiert er ueber die Hoehe).
# Der Rest ist echtes Bodengefaelle plus Beschleunigungsfehler des SBG (~0.1 Grad).
nb, k = floor_plane(P.dot(L.T))
print("Karte vor Ausrichtung: Boden %.2f Grad geneigt" % math.degrees(math.acos(min(1.0, L.T.dot(nb)[2]))))
report("nach Schwerkraft", nb, k)
