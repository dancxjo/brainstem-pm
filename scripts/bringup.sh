#!/bin/bash
set -euo pipefail

export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8

LOGDIR="$HOME/robot_logs"
mkdir -p "$LOGDIR"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"

# --- OLED helpers (status + final dashboard) ---
oled_msg() {
  # Usage: oled_msg HEADER [LINE ...]
  local header=${1:-BRINGUP}; shift || true
  local args=("$REPO_DIR/scripts/midbrain/oled_bringup_status.py" --header "$header")
  for ln in "$@"; do args+=( -l "$ln" ); done
  OLED_ORIENTATION=${OLED_ORIENTATION:-portrait} \
  {
    if command -v uv >/dev/null 2>&1; then
      uv run "${args[@]}"
    elif [[ -x "$REPO_DIR/.venv/bin/python" ]]; then
      "$REPO_DIR/.venv/bin/python" "${args[@]}"
    else
      python3 "${args[@]}"
    fi
  } >/dev/null 2>&1 || true
}

start_oled_dashboard() {
  OLED_ORIENTATION=${OLED_ORIENTATION:-portrait}
  if command -v uv >/dev/null 2>&1; then
    nohup uv run "$REPO_DIR/scripts/midbrain/oled_assemblage.py" --splash \
      > "$LOGDIR/oled_assemblage.log" 2>&1 &
  elif [[ -x "$REPO_DIR/.venv/bin/python" ]]; then
    nohup "$REPO_DIR/.venv/bin/python" "$REPO_DIR/scripts/midbrain/oled_assemblage.py" --splash \
      > "$LOGDIR/oled_assemblage.log" 2>&1 &
  else
    nohup python3 "$REPO_DIR/scripts/midbrain/oled_assemblage.py" --splash \
      > "$LOGDIR/oled_assemblage.log" 2>&1 &
  fi
}

# --- source ROS (temporarily disable nounset) ---
oled_msg BRINGUP "Starting…"

# --- source ROS (temporarily disable nounset) ---
if [ -f /opt/ros/kilted/setup.bash ]; then
  set +u
  source /opt/ros/kilted/setup.bash
  set -u
else
  echo "Missing /opt/ros/kilted/setup.bash" >&2
  oled_msg ERROR "ROS env missing" "/opt/ros/kilted"
  exit 1
fi
oled_msg BRINGUP "ROS env OK"

# --- source your workspaces if they exist ---
if [ -f "$HOME/create_ws/install/setup.bash" ]; then
  set +u; source "$HOME/create_ws/install/setup.bash"; set -u
fi

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  set +u; source "$HOME/ros2_ws/install/setup.bash"; set -u
fi

# Optional: give the USB devices a moment to enumerate
echo "Snoozing 1"
sleep 2
echo "End snooze"
oled_msg BRINGUP "USB ready"

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
  oled_msg BRINGUP "Brainstem: HELLO" "on $dev"
  brainstem_hello "$dev" || exit 1
  oled_msg BRINGUP "Brainstem READY"
else
  echo "[bringup] WARNING: No /dev/ttyACM* or /dev/ttyUSB* found; skipping HELLO handshake" >&2
  oled_msg WARN "No brainstem port" "/dev/ttyACM*|USB*"
fi

echo "Waiting for robot to be up"
sleep 3
echo "Starting to connect. Robot must be on"
oled_msg BRINGUP "Launching nodes…"

# --- launch Create 1 ---
oled_msg NODE "create_bringup"
nohup ros2 launch create_bringup create_1.launch \
  config:="$HOME/create_ws/config/create.yaml" \
  > "$LOGDIR/create_bringup.log" 2>&1 &

# --- launch MPU6050 driver ---
oled_msg NODE "mpu6050driver"
nohup ros2 launch mpu6050driver mpu6050driver_launch.py \
  > "$LOGDIR/mpu6050driver.log" 2>&1 &

# --- launch Madgwick filter ---
oled_msg NODE "imu_filter_madgwick"
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

oled_msg NODE "robot_localization ekf"
nohup ros2 run robot_localization ekf_node \
  --ros-args --params-file "$HOME/create_ws/config/ekf.yaml" \
  > "$LOGDIR/ekf.log" 2>&1 &


oled_msg NODE "hlds_laser"
nohup ros2 launch hls_lfcd_lds_driver hlds_laser.launch.py port:=/dev/ttyACM0 > "$LOGDIR/hls_lfcd.log" 2>&1 &

# basic bring-up (change device/size/fps to match what you saw)
oled_msg NODE "v4l2_camera"
nohup ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 \
  -p image_size:=[1280,1024] \
  -p pixel_format:=MJPG \
  -p output_encoding:=bgr8 \
  -p frame_rate:=30 \
  -p use_sensor_data_qos:=true \
> "$LOGDIR/cam1.log" 2>&1 &

echo "Bringup launched at $(date)" >> "$LOGDIR/cron.log"
oled_msg BRINGUP "All launched" "Dashboard starting"
start_oled_dashboard
