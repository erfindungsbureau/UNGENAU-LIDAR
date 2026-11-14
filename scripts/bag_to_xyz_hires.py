#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
bag_to_xyz_hires.py - Hochaufloesender XYZ Export
Verwendet /lio_sam/mapping/odometry (Poses) + /ouster/points (rohe 65k Punkte/Frame)
statt des downgesampelten /lio_sam/mapping/cloud_registered.

Aufruf: python bag_to_xyz_hires.py input.bag output.xyz [frame_step] [max_sec] [max_reflectivity] [max_pose_jump_deg] [max_pose_jump_m] [per_frame_dir]
# max_reflectivity:   0=aus (Standard), sonst Punkte mit Reflectivity > Wert verwerfen (gegen Spiegel/Glas)
# max_pose_jump_deg:  0=aus (Standard), sonst Rohframes verwerfen deren Pose-Intervall einen
#                     groesseren Rotationssprung zwischen Folgeposen hat (gegen verschmierte/
#                     verdrehte Punktwolken bei instabilem SLAM-Tracking, z.B. schnelles Neigen)
# max_pose_jump_m:    0=aus (Standard), analog fuer Translations-Spruenge
# per_frame_dir:      falls gesetzt, wird JEDER Rohscan als eigene Datei
#                     frame_<Nr>_t<Sekunden>.xyz in dieses Verzeichnis geschrieben (fuer die
#                     visuelle Einzel-Frame-Pruefung in CloudCompare), statt einer kombinierten
#                     output.xyz. output.xyz wird in diesem Fall ignoriert.
"""
import sys, math, bisect
import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2
import level

bag_file   = sys.argv[1]
xyz_file   = sys.argv[2]
frame_step = int(sys.argv[3])   if len(sys.argv) > 3 else 1
max_sec    = float(sys.argv[4]) if len(sys.argv) > 4 else None
# Reflectivity-Filter gegen Spiegelungen/Glas (spiegelnde Flaechen erzeugen oft
# anomal hohe oder sehr inkonsistente Reflektivitaetswerte -> Geisterpunkte).
# 0 = deaktiviert (Standard). Sinnvoller Startwert zum Testen: 150-180.
max_reflectivity  = int(sys.argv[5])   if len(sys.argv) > 5 else 0
# Pose-Qualitaets-Gate: Rohframes verwerfen, deren zeitlich umschliessendes Pose-Intervall
# einen zu grossen Sprung zeigt (Hinweis auf instabiles/degeneriertes SLAM-Tracking statt
# echter Bewegung). 0 = deaktiviert (Standard, bisheriges Verhalten unveraendert).
max_pose_jump_deg = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0
max_pose_jump_m   = float(sys.argv[7]) if len(sys.argv) > 7 else 0.0
per_frame_dir     = sys.argv[8]        if len(sys.argv) > 8 else None
# Sammeldatei mit Frame-Nummer+Zeitstempel als Extra-Spalten (CloudCompare importiert
# diese beim ASCII-Import als Skalarfelder -- damit lassen sich verdaechtige Frames
# durch Einfaerben/Filtern nach Frame-Nummer direkt in EINER Datei eingrenzen, statt
# viele Einzeldateien durchklicken zu muessen).
with_frame_id     = (sys.argv[9] == '1')      if len(sys.argv) > 9 else False

# WICHTIG: LIO-SAM verwendet extrinsicRot/extrinsicTrans NUR um rohe IMU-Vektoren
# (acc/gyr) ins LiDAR-Koordinatensystem zu drehen (siehe utility.h: "acc = extRot*acc").
# In imageProjection.cpp und mapOptmization.cpp (Punktwolken-Verarbeitung und
# Registrierung) wird KEINE Extrinsik auf die Punkte selbst angewendet -- LIO-SAM
# behandelt den LiDAR-Frame direkt als "Body Frame". Die in /lio_sam/mapping/odometry
# veroeffentlichte Pose beschreibt also bereits die Pose des LiDAR-Frames selbst,
# keine separate IMU-Frame-Pose. EXT_R/EXT_T duerfen deshalb HIER NICHT angewendet
# werden (fruehere Version tat das faelschlich und verursachte sichtbare Verdrehung
# der Punktwolke, die mit der Gesamtrotation des Scans zunahm).
EXT_R = np.eye(3, dtype=np.float64)
EXT_T = np.array([0.0, 0.0, 0.0], dtype=np.float64)

# /odometry/imu_incremental: LIO-SAM's IMU-Preintegrations-Pose, gleiche Rate wie IMU
# (400Hz), fuers eigene Motion-Deskewing gedacht -- deutlich praeziser als nachtraegliche
# Interpolation zwischen den seltenen Keyframe-Posen (~2.8Hz), da der Zeitabstand zur
# naechsten Rohscan-Zeit meist nur wenige Millisekunden betraegt statt bis zu ~350ms.
# Wird bevorzugt verwendet, wenn im Bag vorhanden.
ODOM_TOPICS = ['/odometry/imu_incremental', '/lio_sam/mapping/odometry', '/Odometry']
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

def quat_angle_deg(q1, q2):
    """Rotationswinkel (Grad) zwischen zwei Quaternionen -- Mass fuer Pose-Sprung."""
    dot = abs(np.dot(q1, q2))
    dot = min(1.0, dot)
    return math.degrees(2*math.acos(dot))

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
        print("Kein Odometry/LiDAR-Topic -- fallback auf cloud_registered")
        import subprocess, sys as _sys
        r = subprocess.call([_sys.executable,
            '/home/nvidia/UNGENAU-LIDAR/scripts/bag_to_xyz.py', bag_file, xyz_file] + sys.argv[3:])
        sys.exit(r)
    else:
        print("FEHLER: Weder Odometry noch SLAM-Topic gefunden.")
        sys.exit(1)
print("Odometry-Quelle: %s" % ODOM_TOPIC)
# FAST-LIO2 (/Odometry) gibt die Pose des SBG (IMU-Rahmen) aus, nicht die des LiDAR:
# Rohpunkte brauchen hier die Extrinsik aus der FAST-LIO2-Konfiguration.
# shortcut: nimmt die aktuelle Konfiguration; Bags vor dem 14.11.2026 19 Uhr wurden mit
# extrinsic_R ~6.6 Grad gerechnet -> fuer diese den Export ueber /cloud_registered nehmen.
L_LEVEL = np.eye(3)
if ODOM_TOPIC == '/Odometry':
    import yaml
    cfg = yaml.safe_load(open('/home/nvidia/UNGENAU-LIDAR/ros/config/fast_lio_sbg.yaml'))['mapping']
    EXT_R = np.array(cfg['extrinsic_R'], dtype=np.float64).reshape(3, 3)
    EXT_T = np.array(cfg['extrinsic_T'], dtype=np.float64)
    print("FAST-LIO2-Extrinsik: T = %s" % EXT_T)
    _L, _tilt, _n = level.level_rotation(bag_file)
    if _L is not None:
        L_LEVEL = _L
        print("Ausrichtung nach Schwerkraft: Karte um %.2f Grad gedreht (%d IMU-Werte)" % (_tilt, _n))
# Bei der hochfrequenten IMU-Inkremental-Pose wird NICHT interpoliert (naechster
# Nachbar genuegt bei ~400Hz / max ~2.5ms Abstand) -- das eliminiert die Fehlerquelle
# "ungenaue Interpolation ueber grosse Zeitluecken", die bei den seltenen Keyframe-Posen
# (~2.8Hz, bis ~350ms Abstand) sichtbaren Frame-zu-Frame-Versatz verursacht hat.
USE_NEAREST = (ODOM_TOPIC == '/odometry/imu_incremental')
if USE_NEAREST:
    print("Hochfrequente Pose erkannt -- Naechster-Nachbar statt Interpolation (praeziser).")

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

# Pose-Qualitaets-Gate vorbereiten: interval_bad[idx] = Intervall zwischen
# pose_times[idx-1] und pose_times[idx] gilt als unzuverlaessig (Rotations- oder
# Translationssprung ueber Schwellwert -- deutet auf SLAM-Tracking-Ausfall statt
# echter Bewegung hin, siehe docs/slam-staged-testing.md).
interval_bad = [False] * len(pose_times)
n_bad_intervals = 0
if max_pose_jump_deg > 0 or max_pose_jump_m > 0:
    for i in range(1, len(pose_times)):
        bad = False
        if max_pose_jump_deg > 0:
            rot_jump = quat_angle_deg(pose_Q[i-1], pose_Q[i])
            if rot_jump > max_pose_jump_deg:
                bad = True
        if max_pose_jump_m > 0:
            trans_jump = np.linalg.norm(pose_T[i] - pose_T[i-1])
            if trans_jump > max_pose_jump_m:
                bad = True
        interval_bad[i] = bad
        if bad:
            n_bad_intervals += 1
    print("  Pose-Qualitaets-Gate aktiv: %d von %d Intervallen als unzuverlaessig markiert "
          "(Schwellwerte: %.1f Grad, %.2f m)" % (n_bad_intervals, len(pose_times)-1,
                                                   max_pose_jump_deg, max_pose_jump_m))

def get_pose(ts):
    """Pose zum Zeitpunkt ts. Bei hochfrequenter IMU-Inkremental-Pose (USE_NEAREST):
    naechster Nachbar (kein Interpolationsfehler bei ~400Hz moeglich). Sonst lineare
    Interpolation/Slerp zwischen den naechsten 2 Keyframe-Eintraegen. Gibt zusaetzlich
    zurueck, ob das genutzte Intervall als unzuverlaessig markiert ist (Pose-Qualitaets-
    Gate, siehe max_pose_jump_deg/max_pose_jump_m -- nur relevant ohne USE_NEAREST)."""
    idx = bisect.bisect_left(pose_times, ts)
    if idx == 0:
        return pose_R[0], pose_T[0], False
    if idx >= len(pose_times):
        return pose_R[-1], pose_T[-1], False

    if USE_NEAREST:
        t0, t1 = pose_times[idx-1], pose_times[idx]
        nearest = idx-1 if (ts - t0) <= (t1 - ts) else idx
        return pose_R[nearest], pose_T[nearest], False

    t0, t1 = pose_times[idx-1], pose_times[idx]
    alpha = (ts - t0) / (t1 - t0) if t1 > t0 else 0.0
    # Lineare Interpolation Translation
    t_interp = pose_T[idx-1] * (1-alpha) + pose_T[idx] * alpha
    # Echtes Slerp via Quaternion (vorher faelschlich Naechster-Nachbar -- bei
    # schneller Rotation und ~2Hz Pose-Rate fuehrte das zu sichtbarem "Zerreissen"
    # der Punktwolke zwischen aufeinanderfolgenden Rohscans)
    q_interp = slerp_quat(pose_Q[idx-1], pose_Q[idx], alpha)
    R_interp = quat_to_rot(q_interp[0], q_interp[1], q_interp[2], q_interp[3])
    return R_interp, t_interp, interval_bad[idx]

# Export
if per_frame_dir:
    import os
    if not os.path.isdir(per_frame_dir):
        os.makedirs(per_frame_dir)
    print("Einzel-Frame-Export nach: %s" % per_frame_dir)
print("Exportiere Punkte...")
count = 0
frame = 0
t0    = None
ts    = 0
n_skipped_unreliable = 0

f_out = None if per_frame_dir else open(xyz_file, 'w')
try:
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

            R_odom, t_odom, unreliable = get_pose(ts)
            if unreliable:
                n_skipped_unreliable += 1
                continue

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
            pts_world = (np.dot(R_odom, pts_imu.T).T + t_odom).dot(L_LEVEL.T)

            if per_frame_dir:
                fname = os.path.join(per_frame_dir, "frame_%04d_t%.2f.xyz" % (frame, ts - t0))
                with open(fname, 'w') as ff:
                    for pw in pts_world:
                        ff.write('%.4f %.4f %.4f\n' % (pw[0], pw[1], pw[2]))
                        count += 1
            elif with_frame_id:
                trel = ts - t0
                for pw in pts_world:
                    f_out.write('%.4f %.4f %.4f %d %.2f\n' % (pw[0], pw[1], pw[2], frame, trel))
                    count += 1
            else:
                for pw in pts_world:
                    f_out.write('%.4f %.4f %.4f\n' % (pw[0], pw[1], pw[2]))
                    count += 1

            if frame % 50 == 0:
                elapsed = ts - t0
                print("  Frame %d / %.1fs -- %d Punkte bisher" % (frame, elapsed, count))
finally:
    if f_out:
        f_out.close()

elapsed = (ts - t0) if t0 else 0
print("Fertig: %d Punkte, %d Frames, %.1fs" % (count, frame, elapsed))
if max_pose_jump_deg > 0 or max_pose_jump_m > 0:
    print("Pose-Qualitaets-Gate: %d Rohframes wegen unzuverlaessiger Pose verworfen" % n_skipped_unreliable)
