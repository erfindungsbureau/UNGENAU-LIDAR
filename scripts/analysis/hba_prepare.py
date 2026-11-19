#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""hba_prepare.py -- Eingabe fuer HBA (hku-mars/HBA, Buendelausgleich) aus einem Scan.

Pro Ouster-Bild: Rohpunkte (MIN_R..MAX_R) mit der zeitlich interpolierten FAST-LIO2-Pose entzerrt
(wie dense_export.py), dann ins Scanner-Koordinatensystem zum Bild-Ende zurueckgerechnet und auf
VOXEL ausgeduennt. HBA erwartet genau das: Punkte lokal, Pose = lokal -> Welt.

Aufruf: python2 hba_prepare.py BAG OUT_DIR [POSE_BAG|-] [VOXEL=0.05] [MIN_R=0.8] [MAX_R=20]
Ausgabe: OUT_DIR/pcd/<i>.pcd, OUT_DIR/pose.json ("tx ty tz qw qx qy qz" je Zeile, ohne Leerzeile
am Ende: HBAs Leser liest sonst eine Pose zu viel), OUT_DIR/pose_orig.json (Kopie, HBA ueberschreibt
pose.json), OUT_DIR/frames.txt (Bildnummer im Bag und Zeit des Bild-Endes je Pose).
"""
import sys, os, math
import numpy as np
import rosbag, yaml


bag_file, out = sys.argv[1], sys.argv[2]
POSE_BAG = sys.argv[3] if len(sys.argv) > 3 and sys.argv[3] != '-' else None
VOX = float(sys.argv[4]) if len(sys.argv) > 4 else 0.05
MIN_R = float(sys.argv[5]) if len(sys.argv) > 5 else 0.8
MAX_R = float(sys.argv[6]) if len(sys.argv) > 6 else 20.0

cfg = yaml.safe_load(open('/home/nvidia/UNGENAU-LIDAR/ros/config/fast_lio_sbg.yaml'))['mapping']
EXT_R = np.array(cfg['extrinsic_R'], float).reshape(3, 3); EXT_T = np.array(cfg['extrinsic_T'], float)
# FAST-LIO2-Posen (Stempel = Scan-Ende); Interpolation wie dense_export.py
ot, op, oq = [], [], []
with rosbag.Bag(POSE_BAG or bag_file) as b:
    for _, m, _ in b.read_messages(topics=['/Odometry']):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        ot.append(m.header.stamp.to_sec()); op.append([p.x, p.y, p.z]); oq.append([q.w, q.x, q.y, q.z])
ot, op, oq = np.array(ot), np.array(op), np.array(oq)
for i in range(1, len(oq)):
    if np.dot(oq[i], oq[i - 1]) < 0: oq[i] *= -1


def poses_at(t):
    i = np.clip(np.searchsorted(ot, t) - 1, 0, len(ot) - 2)
    a = np.clip((t - ot[i]) / (ot[i + 1] - ot[i]), 0, 1)[:, None]
    P = op[i] * (1 - a) + op[i + 1] * a
    q = oq[i] * (1 - a) + oq[i + 1] * a; q /= np.linalg.norm(q, axis=1)[:, None]
    w, x, y, z = q.T
    R = np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
                  2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
                  2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], 1).reshape(-1, 3, 3)
    return R, P


def quat_of(R):
    """Rotationsmatrix -> (w, x, y, z), stabil auch bei 180 Grad (Shepperd: groesste Diagonale waehlen)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2 * math.sqrt(1 + tr)
        q = (s / 4, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2 * math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2])
        q = ((R[2, 1] - R[1, 2]) / s, s / 4, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s)
    elif R[1, 1] > R[2, 2]:
        s = 2 * math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2])
        q = ((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, s / 4, (R[1, 2] + R[2, 1]) / s)
    else:
        s = 2 * math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1])
        q = ((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, s / 4)
    return q


# Selbsttest quat_of: 90 Grad und 180 Grad um z
_q = quat_of(np.array([[0., -1, 0], [1, 0, 0], [0, 0, 1]]))
assert abs(_q[0] - math.sqrt(.5)) < 1e-9 and abs(_q[3] - math.sqrt(.5)) < 1e-9 and abs(_q[1]) + abs(_q[2]) < 1e-9
_q = quat_of(np.diag([-1., -1, 1]))
assert abs(abs(_q[3]) - 1) < 1e-9 and abs(_q[0]) < 1e-9

PT = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('_p', '<f4'), ('i', '<f4'), ('t', '<u4'),
               ('refl', '<u2'), ('ring', '<u2'), ('amb', '<u2'), ('_q', '<u2'), ('range', '<u4'), ('_r', 'V12')])


def write_pcd(path, X):
    X = np.ascontiguousarray(X, '<f4')
    with open(path, 'wb') as f:
        f.write(('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\n'
                 'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n' % (len(X), len(X))).encode())
        f.write(X.tobytes())


if not os.path.isdir(os.path.join(out, 'pcd')):
    os.makedirs(os.path.join(out, 'pcd'))
lines, frames = [], []
with rosbag.Bag(bag_file) as b:
    n_frames = b.get_message_count('/ouster/points')
    for k, (_, m, _) in enumerate(b.read_messages(topics=['/ouster/points'])):
        if k % 50 == 0:
            print('FORTSCHRITT %d %d' % (k, n_frames)); sys.stdout.flush()
        A = np.frombuffer(m.data, PT)
        r = A['range'] / 1000.0
        A = A[(r >= MIN_R) & (r <= MAX_R)]
        if len(A) < 1000:
            continue
        t0 = m.header.stamp.to_sec()
        t_end = t0 + A['t'].max() * 1e-9
        if t_end < ot[0] or t0 > ot[-1]:
            continue                                   # ausserhalb der Bahn
        ut, ui = np.unique(A['t'], return_inverse=True)
        R, P = poses_at(t0 + ut * 1e-9)
        X = np.stack([A['x'], A['y'], A['z']], 1).astype(np.float64).dot(EXT_R.T) + EXT_T
        Xw = np.einsum('nij,nj->ni', R[ui], X) + P[ui]                 # entzerrt, Welt
        Re, Pe = poses_at(np.array([t_end]))
        Re, Pe = Re[0], Pe[0]
        Xl = (Xw - Pe).dot(Re)                                         # zurueck ins lokale System am Bild-Ende
        q = np.floor(Xl / VOX).astype(np.int64)
        q += 2 ** 20
        _, first = np.unique((q[:, 0] << 42) | (q[:, 1] << 21) | q[:, 2], return_index=True)
        i = len(lines)
        write_pcd(os.path.join(out, 'pcd', '%d.pcd' % i), Xl[first])
        w, x, y, z = quat_of(Re)
        lines.append('%.6f %.6f %.6f %.9f %.9f %.9f %.9f' % (Pe[0], Pe[1], Pe[2], w, x, y, z))
        frames.append('%d %.6f' % (k, t_end))
for name in ('pose.json', 'pose_orig.json'):
    with open(os.path.join(out, name), 'w') as f:
        f.write('\n'.join(lines))                      # keine Leerzeile am Ende (HBA-Leser)
with open(os.path.join(out, 'frames.txt'), 'w') as f:
    f.write('\n'.join(frames) + '\n')
print('FERTIG %d Posen -> %s' % (len(lines), out))
