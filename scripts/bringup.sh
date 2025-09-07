#!/bin/bash
set -euo pipefail

export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8

LOGDIR="$HOME/robot_logs"
mkdir -p "$LOGDIR"

# --- source ROS (temporarily disable nounset) ---
if [ -f /opt/ros/kilted/setup.bash ]; then
  set +u
  source /opt/ros/kilted/setup.bash
  set -u
else
  echo "Missing /opt/ros/kilted/setup.bash" >&2
  exit 1
fi

# --- source your workspaces if they exist ---
if [ -f "$HOME/create_ws/install/setup.bash" ]; then
  set +u; source "$HOME/create_ws/install/setup.bash"; set -u
fi

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  set +u; source "$HOME/ros2_ws/install/setup.bash"; set -u
fi

# Optional: give the USB devices a moment to enumerate
sleep 2

echo $HOME
# --- launch Create 1 ---
nohup ros2 launch create_bringup create_1.launch \
  config:="$HOME/create_ws/config/create.yaml" \
  > "$LOGDIR/create_bringup.log" 2>&1 &

# --- launch MPU6050 driver ---
nohup ros2 launch mpu6050driver mpu6050driver_launch.py \
  > "$LOGDIR/mpu6050driver.log" 2>&1 &

# --- launch Madgwick filter ---
nohup ros2 run imu_filter_madgwick imu_filter_madgwick_node \
  --ros-args \
  -r /imu/data_raw:=/imu/data_raw \
  -r /imu/data:=/imu/data \
  -p use_mag:=false \
  -p world_frame:=enu \
  -p publish_tf:=false \
  -p stateless:=false \
  -p gain:=0.1 \
  > "$LOGDIR/imu_filter_madgwick.log" 2>&1 &

nohup ros2 run robot_localization ekf_node \
  --ros-args --params-file "$HOME/create_ws/config/ekf.yaml" \
  > "$LOGDIR/ekf.log" 2>&1 &


nohup ros2 launch hls_lfcd_lds_driver hlds_laser.launch.py port:=/dev/ttyACM0 > "$LOGDIR/hls_lfcd.log" 2>&1 &

echo "Bringup launched at $(date)" >> "$LOGDIR/cron.log"
