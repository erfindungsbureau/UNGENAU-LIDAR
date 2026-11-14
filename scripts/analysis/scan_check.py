#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
scan_check.py -- Auswertung einer bewegten SLAM-Aufnahme.

1. Verarbeitungsquote: SLAM-Posen pro LiDAR-Umdrehung
2. Bahn: Laenge, Tempo, Neigung, Spruenge, Abstand Start-Ende
3. Wanddicke: Wolke in Wuerfel von 0.5 m teilen; in ebenen Wuerfeln ist die Streuung
   quer zur Flaeche die "Dicke" der Wand. Vergleich Stillstand (erste Sekunden) / ganze Aufnahme.

Aufruf: python scan_check.py input.bag [odom_topic] [cloud_topic] [step] [n_lidar]
  step:    jeder n-te Punkt der Wolke (gleiche Dichte fuer beide Verfahren: FAST-LIO2 gibt schon jeden 3. aus)
  n_lidar: Anzahl LiDAR-Umdrehungen, falls die Bag keine /ouster/points enthaelt (Offline-Durchlauf)
"""
import sys
import numpy as np
import rosbag

bag_path = sys.argv[1]
odom_t = sys.argv[2] if len(sys.argv) > 2 else "/Odometry"
cloud_t = sys.argv[3] if len(sys.argv) > 3 else "/cloud_registered"
STEP = int(sys.argv[4]) if len(sys.argv) > 4 else 4
N_LIDAR = int(sys.argv[5]) if len(sys.argv) > 5 else None
CELL, MAX_R, STILL = 0.5, 8.0, 30   # Wuerfel m, max. Distanz m, Scans Stillstand


def plane_stats(p):
    """Streuung quer zur Flaeche (m) je ebenem Wuerfel -> dict Wuerfel-Schluessel: Dicke."""
    c = np.floor(p / CELL).astype(np.int64)
    key = (c[:, 0] + 1000) * 4000000 + (c[:, 1] + 1000) * 2000 + (c[:, 2] + 1000)
    o = np.argsort(key)
    key, p = key[o], p[o]
    uk, st, n = np.unique(key, return_index=True, return_counts=True)
    out = {}
    for k, s, m in zip(uk, st, n):
        if m < 100:
            continue
        ev = np.linalg.eigvalsh(np.cov(p[s:s + m].T))
        if ev[0] < 0.06 * ev[1] and ev[1] > 0.005:      # eben (Dicke bis ca. 35 mm zugelassen) und flaechig ausgedehnt
            out[k] = np.sqrt(ev[0])
    return out


def selftest():
    r = np.random.RandomState(0)
    p = np.c_[r.uniform(0.01, 0.49, 5000), r.uniform(0.01, 0.49, 5000), 0.25 + r.normal(0, 0.01, 5000)]
    t = list(plane_stats(p).values())
    assert len(t) == 1 and abs(t[0] - 0.01) < 0.001, t


selftest()

odo, clouds, n_lidar = [], [], 0
bag = rosbag.Bag(bag_path)
for topic, msg, _ in bag.read_messages(topics=["/ouster/points", odom_t, cloud_t]):
    if topic == "/ouster/points":
        n_lidar += 1
    elif topic == odom_t:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        odo.append([msg.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    else:
        off = dict((f.name, f.offset) for f in msg.fields)
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)[::STEP]
        xyz = np.stack([raw[:, off[k]:off[k] + 4].copy().view(np.float32)[:, 0] for k in "xyz"], axis=1)
        clouds.append((msg.header.stamp.to_sec(), xyz))
bag.close()
n_lidar = N_LIDAR or n_lidar
odo = np.array(odo)
t, pos, q = odo[:, 0], odo[:, 1:4], odo[:, 4:8]

print("== Verarbeitung ==")
print("LiDAR-Umdrehungen %d, SLAM-Posen %d (%.0f %%), Dauer %.0f s" %
      (n_lidar, len(odo), 100.0 * len(odo) / max(n_lidar, 1), t[-1] - t[0]))

d = np.linalg.norm(np.diff(pos, axis=0), axis=1)
dt = np.maximum(np.diff(t), 1e-3)
v = d / dt
zx = 2 * (q[:, 0] * q[:, 2] + q[:, 1] * q[:, 3])        # z-Achse des Sensors in Weltkoordinaten
zy = 2 * (q[:, 1] * q[:, 2] - q[:, 0] * q[:, 3])
zz = 1 - 2 * (q[:, 0] ** 2 + q[:, 1] ** 2)
tilt = np.degrees(np.arccos(np.clip(zz, -1, 1)))
dq = np.abs(np.sum(q[1:] * q[:-1], axis=1))
rot = np.degrees(2 * np.arccos(np.clip(dq, -1, 1))) / dt
print("== Bahn ==")
print("Weglaenge %.1f m, Tempo Median %.2f m/s, max %.2f m/s" % (d.sum(), np.median(v), v.max()))
print("Drehrate Median %.0f Grad/s, 95%% %.0f, max %.0f" % (np.median(rot), np.percentile(rot, 95), rot.max()))
print("Neigung gegen Start: Median %.0f Grad, max %.0f Grad" % (np.median(tilt), tilt.max()))
print("Spruenge (> 0.3 m zwischen zwei Posen): %d" % int((d > 0.3).sum()))
print("Abstand Start-Ende: %.3f m (dx %.3f, dy %.3f, dz %.3f), Hoehe min/max %.2f/%.2f m" %
      (np.linalg.norm(pos[-1] - pos[0]), pos[-1, 0] - pos[0, 0], pos[-1, 1] - pos[0, 1],
       pos[-1, 2] - pos[0, 2], pos[:, 2].min(), pos[:, 2].max()))
dqe = abs(np.dot(q[0], q[-1]))
print("Drehung Start-Ende: %.1f Grad" % np.degrees(2 * np.arccos(min(dqe, 1.0))))

print("== Wanddicke (Streuung quer zur Flaeche, Wuerfel %.1f m, bis %.0f m) ==" % (CELL, MAX_R))
parts = []
for i, (ct, xyz) in enumerate(clouds):
    j = np.argmin(np.abs(t - ct))
    r = np.linalg.norm(xyz - pos[j], axis=1)
    parts.append(xyz[(r > 0.5) & (r < MAX_R)].astype(np.float64))
print("Punkte pro Scan in der Wolke (jeder %d.): Median %d" % (STEP, np.median([len(x) for x in parts])))
still = plane_stats(np.concatenate(parts[:STILL]))
full = plane_stats(np.concatenate(parts))
both = [k for k in still if k in full]
f = np.array(list(full.values())) * 1000
print("Stillstand (erste %d Scans): %d ebene Wuerfel, Median %.1f mm" %
      (STILL, len(still), np.median(list(still.values())) * 1000))
print("Ganze Aufnahme: %d ebene Wuerfel, Median %.1f mm, 90%%-Wert %.1f mm" %
      (len(full), np.median(f), np.percentile(f, 90)))
if both:
    print("Dieselben %d Wuerfel: Stillstand %.1f mm -> ganze Aufnahme %.1f mm" %
          (len(both), np.median([still[k] for k in both]) * 1000, np.median([full[k] for k in both]) * 1000))
print("Ausdehnung der Wolke: %s m" % np.round(np.ptp(np.concatenate(parts[::10]), axis=0), 1))
