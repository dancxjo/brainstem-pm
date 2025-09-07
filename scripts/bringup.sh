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
# --- brainstem handshake (HELLO -> wait for READY) ---
detect_dev() {
  if [[ -n "${DEV:-}" ]]; then echo "$DEV"; return 0; fi
  local cand
  for cand in /dev/ttyACM* /dev/ttyUSB*; do
    [[ -e "$cand" ]] && { echo "$cand"; return 0; }
  done
  return 1
}

cfg_port() {
  local dev=$1
  stty -F "$dev" 115200 cs8 -cstopb -parenb -ixon -ixoff -crtscts -echo -icanon raw min 1 time 0 || true
}

brainstem_hello() {
  local dev=$1
  cfg_port "$dev"
  # Open dedicated FDs for robust bidir I/O
  exec 3<"$dev"
  exec 4>"$dev"
  # Send HELLO and wait up to 8s for READY
  printf "HELLO\r\n" >&4
  local deadline=$(( $(date +%s) + 8 ))
  local line
  echo "[bringup] Waiting for READY from brainstem on $dev..." >&2
  while (( $(date +%s) < deadline )); do
    if IFS= read -r -t 0.5 line <&3; then
      echo "[brainstem] $line" >> "$LOGDIR/brainstem_handshake.log"
      if [[ "$line" == READY ]]; then
        echo "[bringup] Brainstem READY" >&2
        exec 3<&- 4>&-
        return 0
      fi
    fi
  done
  exec 3<&- 4>&-
  echo "[bringup] ERROR: No READY from brainstem on $dev" >&2
  return 1
}

dev=$(detect_dev || true)
if [[ -n "${dev:-}" ]]; then
  brainstem_hello "$dev" || exit 1
else
  echo "[bringup] WARNING: No /dev/ttyACM* or /dev/ttyUSB* found; skipping HELLO handshake" >&2
fi

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

# basic bring-up (change device/size/fps to match what you saw)
nohup ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 \
  -p image_size:=[1280,1024] \
  -p pixel_format:=MJPG \
  -p output_encoding:=bgr8 \
  -p frame_rate:=30 \
  -p use_sensor_data_qos:=true \
  > "$LOGDIR/cam1.log" 2>&1 &

echo "Bringup launched at $(date)" >> "$LOGDIR/cron.log"
