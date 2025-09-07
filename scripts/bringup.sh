#!/bin/sh

ros2 launch create_bringup create_1.launch config:=$HOME/create_ws/config/create.yaml &

ros2 launch mpu6050driver mpu6050driver_launch.py &

ros2 run imu_filter_madgwick imu_filter_madgwick_node   --ros-args   -r /imu/data_raw:=/imu/data_raw   -r /imu/data:=/imu/data   -p use_mag:=false   -p world_frame:="enu"   -p publish_tf:=false   -p stateless:=false   -p gain:=0.1 &


