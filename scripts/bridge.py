#!/usr/bin/env python
import rospy, json, time
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

pts_data = []

def cb(msg):
    global pts_data
    pts = []
    total = msg.width * msg.height
    step = max(1, total // 600)
    for i, p in enumerate(pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True)):
        if i % step == 0:
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            if x*x + y*y + z*z > 0.25:
                pts.append([round(x,2), round(y,2), round(z,2)])
    pts_data = pts

rospy.init_node("bridge", anonymous=True)
rospy.Subscriber("/ouster/points", PointCloud2, cb, queue_size=1)
print("Bridge started")

while not rospy.is_shutdown():
    open("/tmp/pts.json","w").write(json.dumps(pts_data))
    time.sleep(0.15)
