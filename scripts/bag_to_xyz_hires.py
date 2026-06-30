#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
bag_to_xyz_hires.py - Hochaufloesender XYZ Export
Verwendet /lio_sam/mapping/odometry (Poses) + /ouster/points (rohe 65k Punkte/Frame)
statt des downgesampelten /lio_sam/mapping/cloud_registered.

Aufruf: python bag_to_xyz_hires.py input.bag output.xyz [frame_step] [max_sec] [max_reflectivity]
# max_reflectivity: 0=aus (Standard), sonst Punkte mit Reflectivity > Wert verwerfen (gegen Spiegel/Glas)
"""
import sys, math, bisect
import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2

bag_file   = sys.argv[1]
xyz_file   = sys.argv[2]
frame_step = int(sys.argv[3])   if len(sys.argv) > 3 else 1
max_sec    = float(sys.argv[4]) if len(sys.argv) > 4 else None
# Reflectivity-Filter gegen Spiegelungen/Glas (spiegelnde Flaechen erzeugen oft
# anomal hohe oder sehr inkonsistente Reflektivitaetswerte -> Geisterpunkte).
# 0 = deaktiviert (Standard). Sinnvoller Startwert zum Testen: 150-180.
max_reflectivity = int(sys.argv[5]) if len(sys.argv) > 5 else 0

# Extrinsic: LiDAR -> IMU/SBG (gemessen, siehe docs/jetson-setup.md
# "IMU-Extrinsik-Kalibrierung". Gleiche Konvention/Richtung wie FAST-LIO2s
# offset_R_L_I -- NICHT die transponierte Version, die LIO-SAM fuer
# extrinsicRot braucht, siehe docs/slam-staged-testing.md fuer Details
# zum Konventionsunterschied zwischen den beiden Engines.)
EXT_R = np.array([[0.9973, 0.0064, 0.0730],
                  [0.0000, 0.9962, -0.0872],
                  [-0.0732, 0.0869, 0.9935]], dtype=np.float64)
EXT_T = np.array([0.000, 0.030, -0.0249], dtype=np.float64)

ODOM_TOPICS = ['/lio_sam/mapping/odometry', '/Odometry']  # LIO-SAM bzw. FAST-LIO2
LIDAR_TOPIC = '/ouster/points'
SLAM_TOPICS = ['/lio_sam/mapping/cloud_registered', '/cloud_registered']

def quat_to_rot(x, y, z, w):
    n = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
        [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)]
    ], dtype=np.float64)

def slerp_quat(q1, q2, t):
    dot = np.dot(q1, q2)
    if dot < 0:
        q2 = -q2; dot = -dot
    if dot > 0.9995:
        return q1 + t*(q2-q1)
    theta0 = math.acos(dot)
    theta  = theta0 * t
    sin0   = math.sin(theta0)
    return (math.sin(theta0-theta)/sin0)*q1 + (math.sin(theta)/sin0)*q2

# Odometry einlesen
print("Lese Odometry-Poses...")
pose_times = []
pose_R     = []
pose_Q     = []   # Quaternionen [x,y,z,w] fuer korrekte Slerp-Interpolation
pose_T     = []

with rosbag.Bag(bag_file, 'r') as bag:
    avail = bag.get_type_and_topic_info().topics.keys()
    ODOM_TOPIC = next((t for t in ODOM_TOPICS if t in avail), None)
    has_lidar  = LIDAR_TOPIC in avail
    has_slam   = any(t in avail for t in SLAM_TOPICS)

if ODOM_TOPIC is None or not has_lidar:
    if has_slam:
        print("Kein Odometry/LiDAR-Topic — fallback auf cloud_registered")
        import subprocess, sys as _sys
        r = subprocess.call([_sys.executable,
            '/home/nvidia/UNGENAU-LIDAR/scripts/bag_to_xyz.py', bag_file, xyz_file] + sys.argv[3:])
        sys.exit(r)
    else:
        print("FEHLER: Weder Odometry noch SLAM-Topic gefunden.")
        sys.exit(1)
print("Odometry-Quelle: %s" % ODOM_TOPIC)

with rosbag.Bag(bag_file, 'r') as bag:
    for _, msg, t in bag.read_messages(topics=[ODOM_TOPIC]):
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        pose_times.append(t.to_sec())
        pose_R.append(quat_to_rot(q.x, q.y, q.z, q.w))
        pose_Q.append(np.array([q.x, q.y, q.z, q.w], dtype=np.float64))
        pose_T.append(np.array([p.x, p.y, p.z], dtype=np.float64))

print("  %d Poses geladen" % len(pose_times))
if not pose_times:
    print("FEHLER: Keine Odometry-Daten im Bag.")
    sys.exit(1)

def get_pose(ts):
    """Pose per linearer Interpolation zwischen naechsten 2 Eintraegen."""
    idx = bisect.bisect_left(pose_times, ts)
    if idx == 0:
        return pose_R[0], pose_T[0]
    if idx >= len(pose_times):
        return pose_R[-1], pose_T[-1]
    t0, t1 = pose_times[idx-1], pose_times[idx]
    alpha = (ts - t0) / (t1 - t0) if t1 > t0 else 0.0
    # Lineare Interpolation Translation
    t_interp = pose_T[idx-1] * (1-alpha) + pose_T[idx] * alpha
    # Echtes Slerp via Quaternion (vorher faelschlich Naechster-Nachbar -- bei
    # schneller Rotation und ~2Hz Pose-Rate fuehrte das zu sichtbarem "Zerreissen"
    # der Punktwolke zwischen aufeinanderfolgenden Rohscans)
    q_interp = slerp_quat(pose_Q[idx-1], pose_Q[idx], alpha)
    R_interp = quat_to_rot(q_interp[0], q_interp[1], q_interp[2], q_interp[3])
    return R_interp, t_interp

# Export
print("Exportiere Punkte...")
count = 0
frame = 0
t0    = None
ts    = 0

with open(xyz_file, 'w') as f_out:
    with rosbag.Bag(bag_file, 'r') as bag:
        for _, msg, t in bag.read_messages(topics=[LIDAR_TOPIC]):
            ts = t.to_sec()
            if t0 is None:
                t0 = ts
            if max_sec is not None and (ts - t0) > max_sec:
                break
            frame += 1
            if frame % frame_step != 0:
                continue

            R_odom, t_odom = get_pose(ts)

            if max_reflectivity > 0:
                raw = np.array(list(pc2.read_points(
                    msg, field_names=('x','y','z','reflectivity'), skip_nans=True)), dtype=np.float64)
                if len(raw) == 0:
                    continue
                raw = raw[raw[:,3] <= max_reflectivity]
                pts = raw[:, :3]
            else:
                pts = np.array(list(pc2.read_points(
                    msg, field_names=('x','y','z'), skip_nans=True)), dtype=np.float64)
            if len(pts) == 0:
                continue

            # Distanzfilter (< 0.5m = zu nah, > 120m = zu weit)
            dist2 = pts[:,0]**2 + pts[:,1]**2 + pts[:,2]**2
            pts = pts[(dist2 >= 0.25) & (dist2 <= 14400)]

            # LiDAR -> IMU Frame
            pts_imu = np.dot(EXT_R, pts.T).T + EXT_T

            # IMU -> Welt Frame
            pts_world = np.dot(R_odom, pts_imu.T).T + t_odom

            for pw in pts_world:
                f_out.write('%.4f %.4f %.4f\n' % (pw[0], pw[1], pw[2]))
                count += 1

            if frame % 50 == 0:
                elapsed = ts - t0
                print("  Frame %d / %.1fs — %d Punkte bisher" % (frame, elapsed, count))

elapsed = (ts - t0) if t0 else 0
print("Fertig: %d Punkte, %d Frames, %.1fs" % (count, frame, elapsed))
