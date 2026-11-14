#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""dense_export.py -- dichte Punktwolke aus einem Live-Bag: JEDER Ouster-Rohpunkt (nicht nur jeder
point_filter_num-te wie in /cloud_registered), pro Spalte mit der zeitlich interpolierten FAST-LIO2-Pose
entzerrt, nach Schwerkraft ausgerichtet (level.py), in Wuerfeln gemittelt mit Reflexion und Infrarot.

Aufruf: python2 dense_export.py BAG OUT_PREFIX [VOXEL=0.01] [MIN_R=0.8] [MAX_R=10] [MASK_DIR|-] [POSE_BAG]
  POSE_BAG: Bag einer Neuberechnung (fl_replay.sh) mit /Odometry und /replay/imu statt der Live-Posen
  MASK_DIR: Ordner mit region.npy + shot_transform.json (von Hand gezeichneter Grundriss, match_shot.py)
Ausgabe: OUT_PREFIX_xyz.f32 (n*3), _refl.f32, _amb.f32, _cnt.u16, _path.f32 (Bahn), Zeilen-Info auf stdout;
  endet OUT_PREFIX auf .ply, stattdessen eine PLY-Datei (Web-UI, process_scan.sh). stdout: FORTSCHRITT k n
"""
import sys, os, json, math
import numpy as np
import rosbag, yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import level

bag_file, out = sys.argv[1], sys.argv[2]
VOX = float(sys.argv[3]) if len(sys.argv) > 3 else 0.01
MIN_R = float(sys.argv[4]) if len(sys.argv) > 4 else 0.8   # Ouster Gen1 Datenblatt: ab 0.8 m zuverlaessig
MAX_R = float(sys.argv[5]) if len(sys.argv) > 5 else 10.0
MASK = sys.argv[6] if len(sys.argv) > 6 and sys.argv[6] != "-" else None
POSE_BAG = sys.argv[7] if len(sys.argv) > 7 else None

cfg = yaml.safe_load(open('/home/nvidia/UNGENAU-LIDAR/ros/config/fast_lio_sbg.yaml'))['mapping']
EXT_R = np.array(cfg['extrinsic_R'], float).reshape(3, 3); EXT_T = np.array(cfg['extrinsic_T'], float)
L, tilt, _ = level.level_rotation(POSE_BAG, imu_topic="/replay/imu") if POSE_BAG else level.level_rotation(bag_file)
print('Ausrichtung: %.2f Grad' % tilt)

# FAST-LIO2-Posen (Stempel = Scan-Ende, wie die Punktzeiten header.stamp + t)
ot, op, oq = [], [], []
with rosbag.Bag(POSE_BAG or bag_file) as b:
    for _, m, _ in b.read_messages(topics=['/Odometry']):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        ot.append(m.header.stamp.to_sec()); op.append([p.x, p.y, p.z]); oq.append([q.w, q.x, q.y, q.z])
ot, op, oq = np.array(ot), np.array(op), np.array(oq)
for i in range(1, len(oq)):                       # gleiches Vorzeichen fuer nlerp
    if np.dot(oq[i], oq[i - 1]) < 0: oq[i] *= -1


def poses(t):
    """Pose zur Zeit t (Vektor): Position linear, Drehung normiertes lerp (Schritte 0.1 s, Fehler vernachlaessigbar)."""
    i = np.clip(np.searchsorted(ot, t) - 1, 0, len(ot) - 2)
    a = np.clip((t - ot[i]) / (ot[i + 1] - ot[i]), 0, 1)[:, None]
    P = op[i] * (1 - a) + op[i + 1] * a
    q = oq[i] * (1 - a) + oq[i + 1] * a; q /= np.linalg.norm(q, axis=1)[:, None]
    w, x, y, z = q.T
    R = np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
                  2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
                  2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], 1).reshape(-1, 3, 3)
    return R, P


inside = None
if MASK:
    region = np.load(os.path.join(MASK, 'region.npy')); T = json.load(open(os.path.join(MASK, 'shot_transform.json')))
    th = math.radians(T['deg']); c, s_ = math.cos(th), math.sin(th)

    def inside(X):
        x, y = X[:, 0], X[:, 1]
        if T['flip']: x = -x
        u = ((c * x - s_ * y) * T['s'] + T['tx']).astype(int); v = (-(s_ * x + c * y) * T['s'] + T['ty']).astype(int)
        ok = (u >= 0) & (u < region.shape[1]) & (v >= 0) & (v < region.shape[0])
        r = np.zeros(len(X), bool); r[ok] = region[v[ok], u[ok]]; return r

PT = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('_p', '<f4'), ('i', '<f4'), ('t', '<u4'),
               ('refl', '<u2'), ('ring', '<u2'), ('amb', '<u2'), ('_q', '<u2'), ('range', '<u4'),
               ('_r', 'V12')])  # point_step 48
acc = None; buf = []; n_raw = n_used = 0


def reduce(parts):
    K = np.concatenate([p[0] for p in parts]); V = np.concatenate([p[1] for p in parts])
    u, inv = np.unique(K, return_inverse=True)
    S = np.stack([np.bincount(inv, V[:, j], len(u)).astype(np.float32) for j in range(V.shape[1])], 1)  # float32: TX2 hat nur 8 GB
    return u, S


with rosbag.Bag(bag_file) as b:
    n_frames = b.get_message_count('/ouster/points')
    for k, (_, m, _) in enumerate(b.read_messages(topics=['/ouster/points'])):
        if k % 20 == 0:
            print('FORTSCHRITT %d %d' % (k, n_frames)); sys.stdout.flush()   # fuer den Balken im Web-UI
        assert m.point_step == 48 and not m.is_bigendian
        A = np.frombuffer(m.data, PT); n_raw += len(A)
        r = A['range'] / 1000.0
        A = A[(r >= MIN_R) & (r <= MAX_R)]
        if not len(A): continue
        t = m.header.stamp.to_sec() + A['t'] * 1e-9
        ut, ui = np.unique(A['t'], return_inverse=True)        # eine Pose pro Spalte
        R, P = poses(m.header.stamp.to_sec() + ut * 1e-9)
        X = np.stack([A['x'], A['y'], A['z']], 1).astype(np.float64).dot(EXT_R.T) + EXT_T
        X = (np.einsum('nij,nj->ni', R[ui], X) + P[ui]).dot(L.T)
        sel = inside(X) if inside else np.ones(len(X), bool)
        X = X[sel]; n_used += len(X)
        q = np.floor(X / VOX).astype(np.int64) + 2 ** 20
        key = (q[:, 0] << 42) | (q[:, 1] << 21) | q[:, 2]
        V = np.column_stack([X, A['refl'][sel], A['amb'][sel], np.ones(len(X))]).astype(np.float32)
        buf.append((key, V))
        if sum(len(x[0]) for x in buf) > 6000000:
            acc = reduce(buf + ([acc] if acc else [])); buf = []
            print('  Frame %d: %d Rohpunkte, %d verwendet, %d Wuerfel' % (k + 1, n_raw, n_used, len(acc[0])))
            sys.stdout.flush()
    if buf: acc = reduce(buf + ([acc] if acc else []))

u, S = acc; cnt = S[:, 5]
if out.endswith('.ply'):
    # Ein File fuer CloudCompare/ArchiCAD: Farbe = Reflexion (grau), zwei Skalarfelder.
    # Nur zwei: CloudCompare-Importdialog ordnet bei drei Feldern eines doppelt zu.
    del u
    r1, r99 = np.percentile(S[::50, 3] / cnt[::50], [1, 99])
    PLY = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', 'u1'), ('g', 'u1'), ('b', 'u1'),
                    ('refl', '<f4'), ('amb', '<f4')])
    with open(out + '.part', 'wb') as f:
        f.write(('ply\nformat binary_little_endian 1.0\ncomment %s, alle Rohpunkte, %.0f-mm-Wuerfel gemittelt, '
                 'nach Schwerkraft ausgerichtet\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\n'
                 'property uchar red\nproperty uchar green\nproperty uchar blue\nproperty float scalar_Reflexion\n'
                 'property float scalar_Infrarot\nend_header\n' % (os.path.basename(bag_file), VOX * 1000, len(S))).encode())
        for a in range(0, len(S), 4000000):
            s, c = S[a:a + 4000000], cnt[a:a + 4000000]
            rec = np.empty(len(s), PLY)
            for j, n in enumerate('xyz'): rec[n] = s[:, j] / c
            rec['refl'] = s[:, 3] / c; rec['amb'] = s[:, 4] / c
            g = (255 * np.clip((rec['refl'] - r1) / (r99 - r1), 0, 1) ** 0.8).astype(np.uint8)
            rec['r'] = rec['g'] = rec['b'] = g
            f.write(rec.tobytes())
    os.rename(out + '.part', out)
    print('Rohpunkte %d, verwendet %d, Wuerfel %.0f mm: %d -> %s' % (n_raw, n_used, VOX * 1000, len(S), out))
    sys.exit(0)
(S[:, :3] / cnt[:, None]).astype('<f4').tofile(out + '_xyz.f32')
(S[:, 3] / cnt).astype('<f4').tofile(out + '_refl.f32')
(S[:, 4] / cnt).astype('<f4').tofile(out + '_amb.f32')
np.minimum(cnt, 65535).astype('<u2').tofile(out + '_cnt.u16')
op.dot(L.T).astype('<f4').tofile(out + '_path.f32')
print('Rohpunkte %d, verwendet %d, Wuerfel %.0f mm: %d (Median %d Messungen/Wuerfel)'
      % (n_raw, n_used, VOX * 1000, len(u), np.median(cnt)))
