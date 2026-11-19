#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""bridge.py -- ROS -> Dateien in /tmp fuer das Web-UI (app.py).
  /tmp/imu.json            Lage + Drehrate (Ouster-IMU), fuer Wasserwaage und «Sensor ruhig»
  /tmp/bridge_status.json  lidar_ok / slam_ok / still
  /tmp/map_3d.bin          3D-Vorschau: Int16-Tripel (Wuerfelindex x,y,z a MAP_RES), nur angehaengt
  /tmp/map_3d.json         Sitzung, Anzahl Wuerfel, Bahn, aktuelle Pose
  /tmp/pose.json           aktuelle Pose + Scan-Qualitaet (Ampel), 5x pro Sekunde (Live-Anzeige Scanner)
Die rohen /ouster/points werden bewusst NICHT abonniert: bei 2048x10 sind das 6 MB/Nachricht,
das Entpacken in Python kostete den TX2 spuerbar Rechenzeit. lidar_ok kommt von der Ouster-IMU
(gleicher Treiber, gleicher Datenstrom).
"""
import rospy, json, time, math, os, threading
import numpy as np
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from collections import deque

MAP_RES  = 0.05            # Wuerfel der 3D-Vorschau (m)
MAP_BIN  = "/tmp/map_3d.bin"
MAP_JSON = "/tmp/map_3d.json"
map_lock = threading.Lock()
map_keys = set()           # bereits gesendete Wuerfel (int64-Schluessel)
map_path = []              # [[x, y, z], ...] alle 0.2 m
map_pose = [None]          # [x, y, z, qw, qx, qy, qz]
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
        odom_hist.append((msg.header.stamp.to_sec(), p.x, p.y, p.z))
        map_pose[0] = [round(p.x, 3), round(p.y, 3), round(p.z, 3),
                       round(q.w, 4), round(q.x, 4), round(q.y, 4), round(q.z, 4)]
        if not map_path or math.sqrt(sum((a - b) ** 2 for a, b in zip([p.x, p.y, p.z], map_path[-1]))) > 0.2:
            map_path.append([round(p.x, 2), round(p.y, 2), round(p.z, 2)])


# ── Scan-Qualitaet (Ampel + Warnton im Web-UI) ──────────────────────────────────────────
# Quelle: /fastlio/quality aus dem gepatchten FAST-LIO: [Scan-Ende, Punkte, ausgeduennt, Treffer, Restfehler, degen].
# Schwellen geeicht 19.11.2026 mit Live-Einstellung (jeder 8. Punkt) an 261118_Atelier (gut) und
# 261118_Atelier2 (Lage um 180 s verloren): vor dem Verlust Treffer ~200, Restfehler > 0.033 m und
# Bahnspruenge > 1.5 m/s; im guten Scan Treffer kurz bis 170, Restfehler nie ueber 0.030 m.
Q_TREFFER_GELB, Q_TREFFER_ROT = 300, 150
Q_REST_ROT = 0.033            # m, Median der letzten Sekunde
Q_SPRUNG_ROT = 2.0            # m/s zwischen zwei Posen: kein Gehen mehr, sondern ein Sprung der Bahn
Q_DREH_GELB = math.radians(60)
q_hist = deque(maxlen=10)     # (Treffer, Restfehler, degen, Scan-Ende) der letzten ~1 s
Q_HALTEN = 3.0                # s: eine Warnung bleibt so lange stehen (sonst flackert die Ampel)
q_gehalten = [(0, "Gut", 0.0)]  # (Stufe, Text, Zeitpunkt)
q_last_wall = [0.0]
odom_hist = deque(maxlen=10)  # (Zeit, x, y, z)


def quality_cb(msg):
    d = msg.data
    if len(d) >= 6:
        q_hist.append((d[3], d[4], d[5], d[0]))
        q_last_wall[0] = time.time()


def quality():
    """Ampel fuer das Web-UI: None (kein SLAM), sonst Stufe 0/1/2 mit Grund."""
    now = time.time()
    if now - q_last_wall[0] > 3.0 or not q_hist:
        return None
    eff = float(np.median([h[0] for h in q_hist]))
    rest = float(np.median([h[1] for h in q_hist]))
    degen = float(np.median([h[2] for h in q_hist]))
    verzug = now - q_hist[-1][3]
    if not -5 < verzug < 600:                 # Zeitquelle nicht vergleichbar (z.B. PPS-Modus)
        verzug = 0.0
    sprung = 0.0
    o = list(odom_hist)
    for a, b in zip(o, o[1:]):
        if b[0] > a[0]:
            sprung = max(sprung, math.sqrt(sum((b[k] - a[k]) ** 2 for k in (1, 2, 3))) / (b[0] - a[0]))
    dreh = imu_data.get("gyro_mag", 0.0) or 0.0
    gruende = []
    if eff < Q_TREFFER_ROT: gruende.append((2, "Kaum Treffer – Abstand nehmen, langsamer"))
    if rest > Q_REST_ROT: gruende.append((2, "Lage unsicher – zurück an eine gut erfasste Stelle"))
    if sprung > Q_SPRUNG_ROT: gruende.append((2, "Positionssprung – Lage vermutlich verloren"))
    if verzug > 2.0: gruende.append((2, "SLAM hinkt %.0f s nach – kurz stehen bleiben" % verzug))
    if eff < Q_TREFFER_GELB: gruende.append((1, "Wenig Treffer – mehr Abstand zu Wänden"))
    if dreh > Q_DREH_GELB: gruende.append((1, "Langsamer drehen"))
    if verzug > 1.0: gruende.append((1, "SLAM hinkt nach"))
    stufe, text = max(gruende) if gruende else (0, "Gut")
    h = q_gehalten[0]
    if stufe >= h[0]:
        q_gehalten[0] = (stufe, text, now)
    elif now - h[2] < Q_HALTEN:
        stufe, text = h[0], h[1]
    else:
        q_gehalten[0] = (stufe, text, now)
    return {"stufe": stufe, "text": text, "treffer": int(eff), "rest_mm": round(rest * 1000, 1),
            "degen": round(degen, 3), "dreh": int(math.degrees(dreh)), "verzug": round(verzug, 1)}


def _test_quality():
    """Selbsttest der Ampel mit kuenstlichen Werten."""
    t = time.time()
    q_hist.clear(); odom_hist.clear(); q_last_wall[0] = t
    q_gehalten[0] = (0, "Gut", 0.0)
    for _ in range(10): q_hist.append((800, 0.025, 0.2, t))
    assert quality()["stufe"] == 0
    q_hist.append((200, 0.025, 0.2, t)); [q_hist.append((200, 0.025, 0.2, t)) for _ in range(9)]
    assert quality()["stufe"] == 1
    for _ in range(10): q_hist.append((800, 0.04, 0.2, t))
    assert quality()["stufe"] == 2
    for _ in range(10): q_hist.append((800, 0.025, 0.2, t))
    assert quality()["stufe"] == 2                       # gehalten
    q_gehalten[0] = (2, "x", t - 5)
    assert quality()["stufe"] == 0                       # nach Q_HALTEN wieder gut
    q_last_wall[0] = t - 10
    assert quality() is None
    q_hist.clear(); q_last_wall[0] = 0.0; q_gehalten[0] = (0, "Gut", 0.0)


def write_pose():
    with map_lock:
        d = {"session": map_session[0], "pose": map_pose[0]}
    d["q"] = quality()
    with open("/tmp/pose.json.tmp", "w") as f:
        json.dump(d, f, separators=(",", ":"))
    os.rename("/tmp/pose.json.tmp", "/tmp/pose.json")


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
rospy.Subscriber("/fastlio/quality",  Float64MultiArray, quality_cb, queue_size=20)
_test_quality()
rospy.loginfo("Bridge gestartet")

tick = 0
while not rospy.is_shutdown():
    now = time.time()
    tick += 1
    if tick % 5 == 0:
        write_map()
    write_pose()
    with open("/tmp/imu.json", "w") as f:
        json.dump(imu_data, f)
    with open("/tmp/bridge_status.json", "w") as f:
        json.dump({
            "lidar_ok": (now - last_lidar_ts[0]) < 3.0,
            "slam_ok":  (now - last_slam_ts[0])  < 5.0,
            "still":    imu_data["gyro_mag"] < 0.05,
        }, f)
    time.sleep(0.2)
