#!/bin/bash
# replay_cmp.sh BAG RATE OUT — rechnet eine Aufnahme offline mit LIO-SAM durch (Rohdaten /ouster/points + /imu/data)
# Nur bei DISARM laufen lassen: nutzt Simulationszeit und startet eigene LIO-SAM-Knoten.
BAG=$1; RATE=$2; OUT=$3
source /opt/ros/melodic/setup.bash; source /home/nvidia/catkin_ws/devel/setup.bash
rosparam set /use_sim_time true
roslaunch /home/nvidia/UNGENAU-LIDAR/scripts/analysis/lio_offline.launch > /tmp/lio_offline_$(basename $OUT .bag).log 2>&1 &
L=$!
sleep 10
rosbag record -O "$OUT" /lio_sam/mapping/odometry /lio_sam/mapping/cloud_registered_raw /lio_sam/mapping/cloud_registered __name:=cmp_rec > /dev/null 2>&1 &
sleep 3
rosbag play --clock -q -r "$RATE" "$BAG" --topics /ouster/points /imu/data /ouster/points:=/replay/points /imu/data:=/replay/imu
sleep 8
rosnode kill /cmp_rec > /dev/null; sleep 3
kill -INT $L; wait $L
rosparam set /use_sim_time false
echo "fertig: $OUT"
