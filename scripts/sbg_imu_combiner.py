#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
sbg_imu_combiner.py
Kombiniert /sbg/imu_data (acc+gyro, ENU via use_enu:true) +
           /sbg/ekf_quat (Orientierung, ENU, mit Magnetometer)
zu sensor_msgs/Imu auf /imu/data fuer LIO-SAM.
"""
import rospy
from sensor_msgs.msg import Imu
from sbg_driver.msg import SbgImuData, SbgEkfQuat

last_ekf = None

def ekf_cb(msg):
    global last_ekf
    last_ekf = msg

def imu_cb(msg):
    global last_ekf
    if last_ekf is None:
        return

    out = Imu()
    out.header.stamp    = msg.header.stamp
    out.header.frame_id = "imu_link"

    # Acceleration + Gyro aus SbgImuData (bereits ENU durch use_enu:true)
    out.linear_acceleration.x = msg.accel.x
    out.linear_acceleration.y = msg.accel.y
    out.linear_acceleration.z = msg.accel.z
    out.angular_velocity.x    = msg.gyro.x
    out.angular_velocity.y    = msg.gyro.y
    out.angular_velocity.z    = msg.gyro.z

    # Orientierung aus SBG EKF (Magnetometer-gestützt, ENU)
    out.orientation.x = last_ekf.quaternion.x
    out.orientation.y = last_ekf.quaternion.y
    out.orientation.z = last_ekf.quaternion.z
    out.orientation.w = last_ekf.quaternion.w

    # Kovarianzen
    out.linear_acceleration_covariance[0] = 0.0001
    out.angular_velocity_covariance[0]    = 0.00001

    # Orientierungs-Kovarianz aus EKF accuracy (rad)
    ax, ay, az = last_ekf.accuracy.x**2, last_ekf.accuracy.y**2, last_ekf.accuracy.z**2
    out.orientation_covariance[0] = ax
    out.orientation_covariance[4] = ay
    out.orientation_covariance[8] = az

    pub.publish(out)

rospy.init_node("sbg_imu_combiner", anonymous=False)
pub = rospy.Publisher("/imu/data", Imu, queue_size=10)
rospy.Subscriber("/sbg/imu_data", SbgImuData, imu_cb, queue_size=50)
rospy.Subscriber("/sbg/ekf_quat", SbgEkfQuat, ekf_cb, queue_size=50)
rospy.loginfo("SBG IMU Combiner: /sbg/imu_data + /sbg/ekf_quat -> /imu/data (ENU+EKF)")
rospy.spin()
