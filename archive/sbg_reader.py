#!/usr/bin/env python
"""
sbg_reader.py -- SBG Ellipse 2 -> ROS sensor_msgs/Imu
Liest rohe IMU-Daten (Klasse 0x03, Msg 0x00) und publisht als /sbg/imu.
200 Hz output.
"""
import struct, math, time, threading, serial
import rospy
from sensor_msgs.msg import Imu

def read_sbg(pub):
    while not rospy.is_shutdown():
        try:
            s = serial.Serial("/dev/ttyUSB0", 921600, timeout=0.1)
            buf = bytearray()
            seq = 0
            rospy.loginfo("SBG verbunden auf /dev/ttyUSB0")
            while not rospy.is_shutdown():
                chunk = s.read(512)
                if chunk:
                    buf.extend(chunk)
                i = 0
                while i < len(buf) - 7:
                    if buf[i] != 0xFF or buf[i+1] != 0x5A:
                        i += 1; continue
                    cls, mid = buf[i+2], buf[i+3]
                    length = buf[i+4] | (buf[i+5] << 8)
                    total = 6 + length + 2
                    if i + total > len(buf): break
                    if cls == 0x03 and mid == 0x00 and length >= 30:
                        p = buf[i+6:i+6+length]
                        ax, ay, az = struct.unpack_from("<fff", p, 6)
                        gx, gy, gz = struct.unpack_from("<fff", p, 18)
                        msg = Imu()
                        msg.header.stamp = rospy.Time.now()
                        msg.header.frame_id = "sbg_imu"
                        msg.header.seq = seq; seq += 1
                        msg.linear_acceleration.x = ax
                        msg.linear_acceleration.y = ay
                        msg.linear_acceleration.z = az
                        msg.angular_velocity.x = gx
                        msg.angular_velocity.y = gy
                        msg.angular_velocity.z = gz
                        # Kein Quaternion verfuegbar (wird von imu_filter berechnet)
                        msg.orientation_covariance[0] = -1
                        msg.linear_acceleration_covariance[0]  = 0.0001
                        msg.angular_velocity_covariance[0]     = 0.0001
                        pub.publish(msg)
                    i += total
                del buf[:i]
        except Exception as e:
            rospy.logwarn("SBG Fehler: %s -- Neustart in 3s" % e)
            time.sleep(3)

rospy.init_node("sbg_reader", anonymous=False)
pub = rospy.Publisher("/sbg/imu", Imu, queue_size=10)
t = threading.Thread(target=read_sbg, args=(pub,))
t.daemon = True
t.start()
rospy.loginfo("SBG Reader gestartet -> /sbg/imu")
rospy.spin()
