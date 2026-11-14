# -*- coding: utf-8 -*-
"""lc_extract.py BAG OUT.npz -- (Python 2, rosbag) FAST-LIO2-Posen und Wolken im Sensorrahmen fuer lc_optimize.py."""
import sys
import numpy as np
import rosbag

def qmat(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

odo, clouds = {}, []
for topic, msg, _ in rosbag.Bag(sys.argv[1]).read_messages(topics=["/Odometry", "/cloud_registered"]):
    st = msg.header.stamp.to_sec()
    if topic == "/Odometry":
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        odo[st] = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    else:
        off = dict((f.name, f.offset) for f in msg.fields)
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)[::2]
        xyz = np.stack([raw[:, off[k]:off[k]+4].copy().view(np.float32)[:, 0] for k in "xyz"], axis=1)
        clouds.append((st, xyz[np.isfinite(xyz).all(axis=1)].astype(np.float64)))
ot = np.array(sorted(odo))
times, poses, pts, cnt = [], [], [], []
for st, xyz in clouds:
    k = ot[np.argmin(np.abs(ot - st))]
    if abs(k - st) > 0.02:
        continue
    o = odo[k]
    R = qmat(o[3:]); t = np.array(o[:3])
    pb = (xyz - t).dot(R)                        # Welt -> Sensor: R^T (p - t)
    r = np.linalg.norm(pb, axis=1)
    pb = pb[(r > 0.5) & (r < 15.0)]
    times.append(st); poses.append(o); pts.append(pb.astype(np.float32)); cnt.append(len(pb))
np.savez(sys.argv[2], times=np.array(times), poses=np.array(poses), pts=np.concatenate(pts), cnt=np.array(cnt))
print("%d Umdrehungen, %d Punkte (im Sensorrahmen)" % (len(times), sum(cnt)))
