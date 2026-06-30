#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, json, time, math, struct
from sensor_msgs.msg import PointCloud2, Imu

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

rospy.init_node("bridge", anonymous=False)
rospy.Subscriber("/ouster/imu",                       Imu,         imu_cb,   queue_size=5)
rospy.Subscriber("/ouster/points",                    PointCloud2, lidar_cb, queue_size=1, buff_size=2**20)
rospy.Subscriber("/lio_sam/mapping/cloud_registered", PointCloud2, slam_cb, queue_size=1, buff_size=2**20)
rospy.Subscriber("/cloud_registered",                 PointCloud2, slam_cb, queue_size=1, buff_size=2**20)

rospy.loginfo("Bridge gestartet")

while not rospy.is_shutdown():
    now = time.time()
    with open("/tmp/imu.json",           "w") as f: json.dump(imu_data, f)
    with open("/tmp/scan_2d.json",       "w") as f: json.dump({"pts": scan_pts, "t": last_lidar_ts[0]}, f)
    with open("/tmp/bridge_status.json", "w") as f:
        json.dump({
            "lidar_ok": (now - last_lidar_ts[0]) < 3.0,
            "slam_ok":  (now - last_slam_ts[0])  < 5.0,
            "still":    imu_data["gyro_mag"] < 0.05,
        }, f)
    time.sleep(0.2)
