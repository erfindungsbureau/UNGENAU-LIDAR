#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, json, time, math, struct, os, threading
import numpy as np
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry

# ── 2D-Karte (Grundriss) aus der SLAM-Ausgabe, fuer die Vorschau im Web-UI ──
MAP_RES   = 0.10          # Rasterweite (m)
MAP_BAND  = (-0.8, 0.8)   # Hoehenband relativ zur Starthoehe des Scanners: Waende, nicht Boden/Decke
map_lock  = threading.Lock()
map_cells = set()         # {(i, j)}: Zelle i*MAP_RES..(i+1)*MAP_RES in Welt-X, j in Welt-Y
map_path  = []            # [[x, y], ...] Weg des Scanners, Punkt alle 0.2 m
map_pose  = [0.0, 0.0, 0.0]
map_z0    = [None]        # Starthoehe; None = noch keine Pose
map_n     = [0]
last_odom_ts = [0.0]


def map_cells_from(xyz, z0):
    """Zellen (i, j) aller Punkte im Hoehenband um z0."""
    z = xyz[:, 2] - z0
    sel = xyz[(z > MAP_BAND[0]) & (z < MAP_BAND[1])]
    if not len(sel):
        return set()
    ij = np.floor(sel[:, :2] / MAP_RES).astype(np.int32)
    return set(map(tuple, np.unique(ij, axis=0).tolist()))


_t = np.array([[0.05, 0.05, 1.0], [0.15, -0.05, 1.5], [0.05, 0.07, 1.2], [3.0, 3.0, -0.5]], np.float32)
assert map_cells_from(_t, 1.2) == set([(0, 0), (1, -1)]), map_cells_from(_t, 1.2)

imu_data      = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "gyro_mag": 0.0,
                 "ax": 0.0, "ay": 0.0, "az": 0.0,
                 "wx": 0.0, "wy": 0.0, "wz": 0.0}
last_lidar_ts = [0.0]
last_slam_ts  = [0.0]
scan_pts      = []
last_scan_t   = [0.0]

def imu_cb(msg):
    ax = msg.linear_acceleration.x
    ay = msg.linear_acceleration.y
    az = msg.linear_acceleration.z
    wx = msg.angular_velocity.x
    wy = msg.angular_velocity.y
    wz = msg.angular_velocity.z
    imu_data["roll"]     = round(math.degrees(math.atan2(ay, math.sqrt(ax*ax + az*az))), 1)
    imu_data["pitch"]    = round(math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az))), 1)
    imu_data["gyro_mag"] = round(math.sqrt(wx*wx + wy*wy + wz*wz), 4)
    imu_data["ax"]       = round(ax, 3)
    imu_data["ay"]       = round(ay, 3)
    imu_data["az"]       = round(az, 3)
    imu_data["wx"]       = round(wx, 4)
    imu_data["wy"]       = round(wy, 4)
    imu_data["wz"]       = round(wz, 4)

def lidar_cb(msg):
    global scan_pts
    last_lidar_ts[0] = time.time()

    now = time.time()
    if now - last_scan_t[0] < 0.15:   # max ~6 Hz
        return
    last_scan_t[0] = now

    fields = {f.name: f.offset for f in msg.fields}
    x_off = fields.get('x', 0)
    y_off = fields.get('y', 4)
    z_off = fields.get('z', 8)
    step  = msg.point_step
    data  = bytes(msg.data)
    n     = len(data) // step

    stride = max(1, n // 1000)
    pts = []
    for i in xrange(0, n, stride):
        base = i * step
        try:
            x = struct.unpack_from('<f', data, base + x_off)[0]
            y = struct.unpack_from('<f', data, base + y_off)[0]
            z = struct.unpack_from('<f', data, base + z_off)[0]
        except Exception:
            continue
        if math.isnan(x) or math.isnan(y) or math.isnan(z):
            continue
        r = math.sqrt(x*x + y*y)
        if r < 0.3 or r > 30.0:
            continue
        if z < -0.5 or z > 2.0:
            continue
        pts.append([round(x, 2), round(y, 2)])

    scan_pts = pts

def slam_cb(msg):
    last_slam_ts[0] = time.time()
    map_n[0] += 1
    if map_n[0] % 2 or map_z0[0] is None:   # jede 2. Wolke reicht fuer die Vorschau, spart CPU
        return
    off = dict((f.name, f.offset) for f in msg.fields)
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    xyz = np.stack([raw[:, off[k]:off[k] + 4].copy().view(np.float32)[:, 0] for k in "xyz"], axis=1)
    new = map_cells_from(xyz[np.isfinite(xyz).all(axis=1)], map_z0[0])
    with map_lock:
        map_cells.update(new)

def odom_cb(msg):
    now = time.time()
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    with map_lock:
        if now - last_odom_ts[0] > 10.0:   # neuer SLAM-Lauf: Karte leeren
            map_cells.clear()
            del map_path[:]
            map_z0[0] = p.z
        last_odom_ts[0] = now
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        map_pose[:] = [round(p.x, 2), round(p.y, 2), round(yaw, 3)]
        if not map_path or math.hypot(p.x - map_path[-1][0], p.y - map_path[-1][1]) > 0.2:
            map_path.append([round(p.x, 2), round(p.y, 2)])

def write_map():
    with map_lock:
        d = {"t": last_odom_ts[0], "res": MAP_RES, "cells": list(map_cells),
             "path": list(map_path), "pose": list(map_pose) if map_path else None}
    # shortcut: schickt jede Sekunde die ganze Karte (~10 Byte/Zelle); bei sehr grossen Scans auf Deltas umstellen
    with open("/tmp/map_2d.json.tmp", "w") as f:
        json.dump(d, f, separators=(",", ":"))
    os.rename("/tmp/map_2d.json.tmp", "/tmp/map_2d.json")

rospy.init_node("bridge", anonymous=False)
rospy.Subscriber("/ouster/imu",                       Imu,         imu_cb,   queue_size=5)
rospy.Subscriber("/ouster/points",                    PointCloud2, lidar_cb, queue_size=1, buff_size=2**20)
rospy.Subscriber("/lio_sam/mapping/cloud_registered", PointCloud2, slam_cb, queue_size=1, buff_size=2**20)
rospy.Subscriber("/cloud_registered",                 PointCloud2, slam_cb, queue_size=1, buff_size=2**20)
rospy.Subscriber("/lio_sam/mapping/odometry",         Odometry,    odom_cb, queue_size=5)
rospy.Subscriber("/Odometry",                         Odometry,    odom_cb, queue_size=5)  # FAST-LIO2

rospy.loginfo("Bridge gestartet")

tick = 0
while not rospy.is_shutdown():
    now = time.time()
    tick += 1
    if tick % 5 == 0:
        write_map()
    with open("/tmp/imu.json",           "w") as f: json.dump(imu_data, f)
    with open("/tmp/scan_2d.json",       "w") as f: json.dump({"pts": scan_pts, "t": last_lidar_ts[0]}, f)
    with open("/tmp/bridge_status.json", "w") as f:
        json.dump({
            "lidar_ok": (now - last_lidar_ts[0]) < 3.0,
            "slam_ok":  (now - last_slam_ts[0])  < 5.0,
            "still":    imu_data["gyro_mag"] < 0.05,
        }, f)
    time.sleep(0.2)
