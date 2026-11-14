#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""bridge.py -- ROS -> Dateien in /tmp fuer das Web-UI (app.py).
  /tmp/imu.json            Lage + Drehrate (Ouster-IMU), fuer Wasserwaage und «Sensor ruhig»
  /tmp/bridge_status.json  lidar_ok / slam_ok / still
  /tmp/map_3d.bin          3D-Vorschau: Int16-Tripel (Wuerfelindex x,y,z a MAP_RES), nur angehaengt
  /tmp/map_3d.json         Sitzung, Anzahl Wuerfel, Bahn, aktuelle Pose
Die rohen /ouster/points werden bewusst NICHT abonniert: bei 2048x10 sind das 6 MB/Nachricht,
das Entpacken in Python kostete den TX2 spuerbar Rechenzeit. lidar_ok kommt von der Ouster-IMU
(gleicher Treiber, gleicher Datenstrom).
"""
import rospy, json, time, math, os, threading
import numpy as np
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry

MAP_RES  = 0.05            # Wuerfel der 3D-Vorschau (m)
MAP_BIN  = "/tmp/map_3d.bin"
MAP_JSON = "/tmp/map_3d.json"
map_lock = threading.Lock()
map_keys = set()           # bereits gesendete Wuerfel (int64-Schluessel)
map_path = []              # [[x, y, z], ...] alle 0.2 m
map_pose = [None]          # [x, y, z, yaw]
map_session = [int(time.time())]
map_n = [0]
cloud_n = [0]
last_odom_ts = [0.0]


def cube_keys(xyz):
    """Wuerfelindizes (n,3) int16 und eindeutige int64-Schluessel."""
    q = np.floor(xyz / MAP_RES).astype(np.int64)
    q = q[(np.abs(q) < 30000).all(axis=1)]
    key = ((q[:, 0] + 32768) << 32) | ((q[:, 1] + 32768) << 16) | (q[:, 2] + 32768)
    key, i = np.unique(key, return_index=True)
    return q[i].astype(np.int16), key


_q, _k = cube_keys(np.array([[0.01, 0.01, 0.01], [0.02, 0.03, 0.04], [-0.01, 0.0, 0.1]], np.float32))
assert len(_k) == 2 and sorted(_q.tolist()) == [[-1, 0, 2], [0, 0, 0]], _q

imu_data = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "gyro_mag": 0.0,
            "ax": 0.0, "ay": 0.0, "az": 0.0, "wx": 0.0, "wy": 0.0, "wz": 0.0}
last_lidar_ts = [0.0]
last_slam_ts  = [0.0]


def imu_cb(msg):
    last_lidar_ts[0] = time.time()
    ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
    wx, wy, wz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
    imu_data["roll"]     = round(math.degrees(math.atan2(ay, math.sqrt(ax*ax + az*az))), 1)
    imu_data["pitch"]    = round(math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az))), 1)
    imu_data["gyro_mag"] = round(math.sqrt(wx*wx + wy*wy + wz*wz), 4)
    imu_data.update(ax=round(ax, 3), ay=round(ay, 3), az=round(az, 3),
                    wx=round(wx, 4), wy=round(wy, 4), wz=round(wz, 4))


def reset_map():
    """Neuer SLAM-Lauf: Vorschau leeren, Sitzungsnummer wechseln (Browser verwirft dann seine Wolke)."""
    map_keys.clear(); del map_path[:]; map_pose[0] = None; map_n[0] = 0
    map_session[0] = int(time.time())
    open(MAP_BIN, "wb").close()


def slam_cb(msg):
    last_slam_ts[0] = time.time()
    cloud_n[0] += 1
    if cloud_n[0] % 2:                       # jede 2. Wolke reicht fuer die Vorschau
        return
    off = dict((f.name, f.offset) for f in msg.fields)
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    xyz = np.stack([raw[:, off[k]:off[k] + 4].copy().view(np.float32)[:, 0] for k in "xyz"], axis=1)
    q, key = cube_keys(xyz[np.isfinite(xyz).all(axis=1)])
    with map_lock:
        new = np.array([k not in map_keys for k in key.tolist()], bool)
        if not new.any():
            return
        map_keys.update(key[new].tolist())
        with open(MAP_BIN, "ab") as f:
            f.write(q[new].astype("<i2").tobytes())
        map_n[0] += int(new.sum())


def odom_cb(msg):
    now = time.time()
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    with map_lock:
        if now - last_odom_ts[0] > 10.0:     # neuer SLAM-Lauf
            reset_map()
        last_odom_ts[0] = now
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        map_pose[0] = [round(p.x, 2), round(p.y, 2), round(p.z, 2), round(yaw, 3)]
        if not map_path or math.sqrt(sum((a - b) ** 2 for a, b in zip([p.x, p.y, p.z], map_path[-1]))) > 0.2:
            map_path.append([round(p.x, 2), round(p.y, 2), round(p.z, 2)])


def write_map():
    with map_lock:
        d = {"session": map_session[0], "n": map_n[0], "res": MAP_RES,
             "path": list(map_path), "pose": map_pose[0]}
    with open(MAP_JSON + ".tmp", "w") as f:
        json.dump(d, f, separators=(",", ":"))
    os.rename(MAP_JSON + ".tmp", MAP_JSON)


reset_map()
rospy.init_node("bridge", anonymous=False)
rospy.Subscriber("/ouster/imu",       Imu,         imu_cb,  queue_size=5)
rospy.Subscriber("/cloud_registered", PointCloud2, slam_cb, queue_size=1, buff_size=2**22)   # FAST-LIO2
rospy.Subscriber("/Odometry",         Odometry,    odom_cb, queue_size=5)
rospy.loginfo("Bridge gestartet")

tick = 0
while not rospy.is_shutdown():
    now = time.time()
    tick += 1
    if tick % 5 == 0:
        write_map()
    with open("/tmp/imu.json", "w") as f:
        json.dump(imu_data, f)
    with open("/tmp/bridge_status.json", "w") as f:
        json.dump({
            "lidar_ok": (now - last_lidar_ts[0]) < 3.0,
            "slam_ok":  (now - last_slam_ts[0])  < 5.0,
            "still":    imu_data["gyro_mag"] < 0.05,
        }, f)
    time.sleep(0.2)
