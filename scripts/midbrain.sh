#!/usr/bin/env bash
set -euo pipefail

# Simple midbrain helper for talking to the brainstem MCU over UART.
#
# Usage examples:
#   scripts/midbrain.sh monitor                 # show telemetry
#   scripts/midbrain.sh ping 1                  # send PING,1
#   scripts/midbrain.sh safe 1                  # SAFE,1 (resume)
#   scripts/midbrain.sh estop                   # SAFE,0 (ESTOP)
#   scripts/midbrain.sh twist 0.2 0.0 1         # TWIST,vx,wz,seq
#   scripts/midbrain.sh stop                    # TWIST,0,0,<seq>
#   scripts/midbrain.sh stats                   # STATS
#   scripts/midbrain.sh forebrain               # monitor + periodic PING
#
# Device selection:
#   - Pass device path as DEV=/dev/ttyACM0 environment variable
#   - Or first argument starting with /dev (e.g., /dev/ttyUSB0)
#   - Otherwise auto-detects the first of /dev/ttyACM* or /dev/ttyUSB*

DEV_DEFAULT=""

detect_dev() {
  if [[ -n "${DEV:-}" ]]; then
    echo "$DEV"; return 0;
  fi
  if [[ $# -ge 1 && "$1" == /dev/* ]]; then
    echo "$1"; return 0;
  fi
  local cand
  for cand in /dev/ttyACM* /dev/ttyUSB*; do
    if [[ -e "$cand" ]]; then echo "$cand"; return 0; fi
  done
  echo ""; return 1
}

cfg_port() {
  local dev=$1
  stty -F "$dev" 115200 cs8 -cstopb -parenb -ixon -ixoff -crtscts -echo -icanon raw min 1 time 0 || true
}

send_line() {
  local dev=$1
  shift
  local line="$*"
  # Append \r\n to be tolerant of host/firmware expectations
  printf "%s\r\n" "$line" > "$dev"
}

usage() {
  cat <<EOF
Usage: ${0##*/} [DEV] <command> [args]

Commands:
  monitor                 Show telemetry (Ctrl-C to exit)
  forebrain               Monitor + send PING every second
  ping <seq>              Send PING,<seq>
  safe <0|1>              Send SAFE,<v>
  estop                   SAFE,0 + brief note
  resume                  SAFE,1
  twist <vx> <wz> [seq]   Send TWIST,<vx>,<wz>,<seq> (seq defaults to epoch mod 1e6)
  stop                    Send TWIST,0,0,<seq>
  stats                   Send STATS
  led <mask>              Send LED,<mask>
  replay <since_eid>      Send REPLAY,<since_eid>

Examples:
  ${0##*/} monitor
  ${0##*/} ping 1
  DEV=/dev/ttyUSB0 ${0##*/} twist 0.2 0.0 42
EOF
}

main() {
  local dev
  dev=$(detect_dev ${1:-}) || { echo "No serial device found. Set DEV or pass /dev/..." >&2; exit 1; }
  if [[ "$1" == /dev/* ]]; then shift; fi
  cfg_port "$dev"

  local cmd=${1:-}
  shift || true
  case "$cmd" in
    monitor)
      echo "[midbrain] Monitoring $dev (Ctrl-C to exit)..." >&2
      exec cat "$dev"
      ;;
    forebrain)
      echo "[midbrain] Forebrain loop on $dev (Ctrl-C to exit)..." >&2
      # Start telemetry reader
      ( exec cat "$dev" ) &
      local rpid=$!
      trap 'kill $rpid 2>/dev/null || true' EXIT INT TERM
      local i=0
      while true; do
        send_line "$dev" "PING,$i"
        # Optional: ensure linkUp and clear ESTOP on start
        if (( i == 0 )); then send_line "$dev" "SAFE,1"; fi
        i=$(( (i+1) % 1000000 ))
        sleep 1
      done
      ;;
    ping)
      local seq=${1:?seq required}
      send_line "$dev" "PING,$seq"
      ;;
    safe)
      local v=${1:?0|1 required}
      send_line "$dev" "SAFE,$v"
      ;;
    estop)
      echo "[midbrain] ESTOP" >&2
      send_line "$dev" "SAFE,0"
      # Immediately stop all motion to prevent unintended movement
      local seq=$(( $(date +%s) % 1000000 ))
      send_line "$dev" "TWIST,0.0,0.0,$seq"
      ;;
    resume)
      echo "[midbrain] RESUME" >&2
      send_line "$dev" "SAFE,1"
      ;;
    twist)
      local vx=${1:?vx}
      local wz=${2:?wz}
      local seq=${3:-$(( $(date +%s) % 1000000 ))}
      send_line "$dev" "TWIST,$vx,$wz,$seq"
      ;;
    stop)
      local seq=$(( $(date +%s) % 1000000 ))
      send_line "$dev" "TWIST,0.0,0.0,$seq"
      ;;
    stats)
      send_line "$dev" "STATS"
      ;;
    led)
      local mask=${1:?mask}
      send_line "$dev" "LED,$mask"
      ;;
    replay)
      local since=${1:?since_eid}
      send_line "$dev" "REPLAY,$since"
      ;;
    ""|-h|--help|help)
      usage
      ;;
    *)
      echo "Unknown command: $cmd" >&2
      usage
      exit 2
      ;;
  esac
}

main "$@"

