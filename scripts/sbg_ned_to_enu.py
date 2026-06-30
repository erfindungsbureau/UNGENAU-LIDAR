#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
SBG Ellipse 2 NED -> ENU Konverter

SBG Ellipse 2 liefert Daten in NED-Konvention:
  X = vorwaerts, Y = rechts, Z = unten
  az = -9.8 wenn flach (Schwerkraft in +Z_NED, Spezifische Kraft = -g in Z)

Madgwick mit world_frame:enu erwartet ENU-Konvention:
  X = vorwaerts, Y = links, Z = oben
  az = +9.8 wenn flach

Transformation NED -> ENU:
  [1  0  0]   (X bleibt: vorwaerts = vorwaerts)
  [0 -1  0]   (Y flippt: rechts -> links)
  [0  0 -1]   (Z flippt: unten -> oben)

Gilt gleichermassen fuer Beschleunigung und Winkelgeschwindigkeit.
"""
import rospy
from sensor_msgs.msg import Imu

pub = None

def imu_cb(msg):
    out = Imu()
    out.header = msg.header
    out.header.frame_id = "sbg_imu_enu"

    # Beschleunigung: NED -> ENU
    out.linear_acceleration.x =  msg.linear_acceleration.x
    out.linear_acceleration.y = -msg.linear_acceleration.y
    out.linear_acceleration.z = -msg.linear_acceleration.z

    # Winkelgeschwindigkeit: NED -> ENU
    out.angular_velocity.x =  msg.angular_velocity.x
    out.angular_velocity.y = -msg.angular_velocity.y
    out.angular_velocity.z = -msg.angular_velocity.z

    # Kovarianzmatrizen unveraendert uebernehmen
    out.linear_acceleration_covariance = msg.linear_acceleration_covariance
    out.angular_velocity_covariance    = msg.angular_velocity_covariance
    out.orientation_covariance         = [-1.0] + [0.0] * 8  # keine Orientierung

    pub.publish(out)

rospy.init_node("sbg_ned_to_enu", anonymous=False)
pub = rospy.Publisher("/sbg/imu_enu", Imu, queue_size=10)
rospy.Subscriber("/sbg/imu", Imu, imu_cb, queue_size=20)
rospy.loginfo("SBG NED->ENU Konverter gestartet")
rospy.spin()
