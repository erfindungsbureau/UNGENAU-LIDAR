#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
ouster_restamp.py -- laeuft nur im Zeitmodus PPS (Frontend-Schalter "Zeit: PPS").

Der Ouster stempelt dann nach dem SBG-PPS: Sekunden seit Pulsbeginn, keine Uhrzeit.
LIO-SAM, Aufnahme und Export brauchen aber Jetson-Zeit (wie die SBG-IMU).
Umrechnung: offset = Minimum von (Empfangszeit - Sensorzeit) der IMU-Pakete der
letzten 10 s. Das Minimum ist die kuerzeste Uebertragung, Verzoegerungen fallen weg.

  /ouster/points_pps -> /ouster/points
  /ouster/imu_pps    -> /ouster/imu

Selbsttest ohne ROS:  python ouster_restamp.py --selftest
"""
import collections
import sys

WINDOW = 10.0  # s Fenster fuer das Minimum
JUMP = 0.5     # s: groessere Aenderung = Sensorzeit gesprungen (PPS weg/neu) -> neu beginnen


class OffsetEstimator(object):
    def __init__(self):
        self.samples = collections.deque()  # (empfang, differenz)
        self.offset = None

    def update(self, now, stamp):
        d = now - stamp
        if self.offset is not None and abs(d - self.offset) > JUMP:
            self.samples.clear()
        self.samples.append((now, d))
        while self.samples[0][0] < now - WINDOW:
            self.samples.popleft()
        self.offset = min(s[1] for s in self.samples)
        return self.offset


def selftest():
    e = OffsetEstimator()
    # Sensorzeit 100 s, Jetson 1000 s, Latenz 2..9 ms
    for i in range(500):
        e.update(1000.0 + i * 0.01 + 0.002 + (i % 8) * 0.001, 100.0 + i * 0.01)
    assert abs(e.offset - 900.002) < 1e-6, e.offset
    # Sensorzeit springt (PPS neu) -> Offset folgt sofort
    e.update(1005.0 + 0.003, 3.0)
    assert abs(e.offset - 1002.003) < 1e-6, e.offset
    print("selftest ok")


def main():
    import rospy
    from sensor_msgs.msg import PointCloud2, Imu

    rospy.init_node("ouster_restamp")
    est = OffsetEstimator()
    imu_pub = rospy.Publisher("/ouster/imu", Imu, queue_size=100)
    cloud_pub = rospy.Publisher("/ouster/points", PointCloud2, queue_size=2)

    def imu_cb(msg):
        old = est.offset
        off = est.update(rospy.get_time(), msg.header.stamp.to_sec())
        if old is not None and abs(off - old) > JUMP:
            rospy.logwarn("Ouster-Zeit gesprungen (%.3f s) -- PPS unterbrochen?", off - old)
        msg.header.stamp += rospy.Duration.from_sec(off)
        imu_pub.publish(msg)

    def cloud_cb(msg):
        if est.offset is None:
            return  # noch keine IMU-Pakete -> Offset unbekannt
        msg.header.stamp += rospy.Duration.from_sec(est.offset)
        cloud_pub.publish(msg)

    rospy.Subscriber("/ouster/imu_pps", Imu, imu_cb, queue_size=100)
    rospy.Subscriber("/ouster/points_pps", PointCloud2, cloud_cb, queue_size=2, buff_size=2**24)
    rospy.loginfo("ouster_restamp laeuft (PPS-Sensorzeit -> Jetson-Zeit)")
    rospy.spin()


if __name__ == "__main__":
    selftest() if "--selftest" in sys.argv else main()
