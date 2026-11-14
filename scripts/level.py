#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
level.py -- Karte nach der Schwerkraft ausrichten (FAST-LIO2).

FAST-LIO2 legt die Welt in die Lage des Scanners beim Start, nicht lotrecht.
Ueber den ganzen Scan gemittelt zeigt die vom SBG gemessene spezifische Kraft,
mit der FAST-LIO-Pose (IMU-Rahmen) in die Welt gedreht, genau nach oben
(Bewegungsbeschleunigungen mitteln sich bei einem Rundgang weg).
level_rotation() gibt die kleinste Drehung zurueck, die dieses "oben" auf +Z legt.

Aufruf als Skript: python level.py BAG  -> gibt Neigung aus (Selbsttest ohne BAG).
"""
import sys, math
import numpy as np

ODOM_TOPIC = '/Odometry'
IMU_TOPIC = '/imu/data'


def quats_to_rots(q):
    """q: (N,4) x,y,z,w -> (N,3,3)"""
    q = q / np.linalg.norm(q, axis=1)[:, None]
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2*(y*y + z*z); R[:, 0, 1] = 2*(x*y - z*w);     R[:, 0, 2] = 2*(x*z + y*w)
    R[:, 1, 0] = 2*(x*y + z*w);     R[:, 1, 1] = 1 - 2*(x*x + z*z); R[:, 1, 2] = 2*(y*z - x*w)
    R[:, 2, 0] = 2*(x*z - y*w);     R[:, 2, 1] = 2*(y*z + x*w);     R[:, 2, 2] = 1 - 2*(x*x + y*y)
    return R


def up_vector(odom_t, odom_q, imu_t, imu_acc, max_dt=0.06):
    """Mittleres "oben" in Weltkoordinaten aus Posen (IMU-Rahmen) und Beschleunigungen."""
    odom_t = np.asarray(odom_t); imu_t = np.asarray(imu_t)
    idx = np.clip(np.searchsorted(odom_t, imu_t), 1, len(odom_t) - 1)
    prev_closer = (imu_t - odom_t[idx - 1]) < (odom_t[idx] - imu_t)
    idx = idx - prev_closer.astype(int)
    ok = np.abs(odom_t[idx] - imu_t) <= max_dt
    R = quats_to_rots(np.asarray(odom_q)[idx[ok]])
    f_world = np.einsum('nij,nj->ni', R, np.asarray(imu_acc)[ok])
    return f_world.mean(axis=0), int(ok.sum())


def rotation_to_z(u):
    """Kleinste Drehung, die Vektor u auf +Z legt."""
    u = u / np.linalg.norm(u)
    v = np.cross(u, [0.0, 0.0, 1.0]); s = np.linalg.norm(v); c = u[2]
    if s < 1e-12:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K.dot(K) * ((1 - c) / (s * s))


def level_rotation(bag_file, odom_topic=ODOM_TOPIC, imu_topic=IMU_TOPIC):
    """(R_level, neigung_grad, n_samples) oder (None, None, 0), wenn Topics fehlen."""
    import rosbag
    ot, oq, it, ia = [], [], [], []
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, _ in bag.read_messages(topics=[odom_topic, imu_topic]):
            if topic == odom_topic:
                q = msg.pose.pose.orientation
                ot.append(msg.header.stamp.to_sec()); oq.append([q.x, q.y, q.z, q.w])
            else:
                a = msg.linear_acceleration
                it.append(msg.header.stamp.to_sec()); ia.append([a.x, a.y, a.z])
    if len(ot) < 10 or len(it) < 100:
        return None, None, 0
    o = np.argsort(ot); ot = np.array(ot)[o]; oq = np.array(oq)[o]
    up, n = up_vector(ot, oq, it, ia)
    if n < 100:
        return None, None, 0
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up[2] / np.linalg.norm(up)))))
    return rotation_to_z(up), tilt, n


def _selftest():
    rng = np.random.RandomState(0)
    up_true = np.array([0.05, -0.03, 1.0]); up_true /= np.linalg.norm(up_true)
    n = 2000
    q = rng.randn(n, 4)  # beliebige Orientierungen des Scanners
    R = quats_to_rots(q)
    f_world = up_true * 9.81 + rng.randn(n, 3) * 0.5  # Schwerkraft + Bewegung/Rauschen
    acc = np.einsum('nji,nj->ni', R, f_world)  # in den IMU-Rahmen
    t = np.arange(n) * 0.005
    up, k = up_vector(t, q, t, acc)
    L = rotation_to_z(up)
    assert k == n
    assert np.allclose(L.dot(up_true), [0, 0, 1], atol=2e-3), L.dot(up_true)
    assert np.allclose(L.T.dot(L), np.eye(3), atol=1e-9)
    assert np.allclose(rotation_to_z(np.array([0, 0, 2.0])), np.eye(3))
    print("Selbsttest ok")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        _selftest()
    else:
        L, tilt, n = level_rotation(sys.argv[1])
        if L is None:
            print("Keine /Odometry- bzw. /imu/data-Daten im Bag")
        else:
            print("Neigung der Karte: %.2f Grad (%d IMU-Werte)" % (tilt, n))
            print(np.array2string(L, precision=5))
