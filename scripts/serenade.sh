#!/usr/bin/env bash
set -euo pipefail

# Serenade the brainstem over USB/serial from a Debian client.
#
# - Auto-detects the serial device unless DEV=/dev/ttyXXX is provided or
#   a /dev/... path is given as the first argument.
# - Plays the forebrain connect trill on first contact.
# - Then loops: plays the ESTOP alarm (SAFE,0), waits, resumes (SAFE,1), repeats.
#
# Usage:
#   scripts/serenade.sh                # auto-detect /dev/ttyACM* or /dev/ttyUSB*
#   scripts/serenade.sh /dev/ttyACM0   # explicit device
#   DEV=/dev/ttyUSB0 scripts/serenade.sh

DEV_DEFAULT=""

detect_dev() {
  if [[ -n "${DEV:-}" ]]; then echo "$DEV"; return 0; fi
  if [[ $# -ge 1 && "$1" == /dev/* ]]; then echo "$1"; return 0; fi
  local cand
  for cand in /dev/ttyACM* /dev/ttyUSB*; do
    if [[ -e "$cand" ]]; then echo "$cand"; return 0; fi
  done
  return 1
}

cfg_port() {
  local dev=$1
  # Match firmware defaults: 115200 8N1, raw, no flow control
  stty -F "$dev" 115200 cs8 -cstopb -parenb -ixon -ixoff -crtscts -echo -icanon raw min 1 time 0 || true
}

send_line() {
  local dev=$1; shift
  local line="$*"
  printf "%s\r\n" "$line" > "$dev"
}

usage() {
  echo "Usage: ${0##*/} [DEV]" >&2
}

main() {
  local dev
  dev=$(detect_dev ${1:-}) || { echo "No serial device found. Set DEV= or pass /dev/..." >&2; exit 1; }
  if [[ "${1:-}" == /dev/* ]]; then shift; fi

  echo "[serenade] Using device: $dev" >&2
  cfg_port "$dev"

  # Tee telemetry to stdout so user can see STATE/HELLO/etc. Ctrl-C to exit.
  ( exec cat "$dev" ) &
  local rpid=$!
  trap 'kill $rpid 2>/dev/null || true' EXIT INT TERM

  # Nudge link up and clear any prior ESTOP.
  # First activity transitions into FOREBRAIN and plays a trill.
  send_line "$dev" "PING,0"
  send_line "$dev" "SAFE,1"
  echo "[serenade] Played connect trill; starting loop..." >&2

  local i=0
  while true; do
    # Song B: ESTOP alarm (distinct and repeatable)
    send_line "$dev" "SAFE,0"
    sleep 1
    send_line "$dev" "SAFE,1"
    # Keep link alive and provide visible activity
    send_line "$dev" "PING,$i"
    i=$(( (i+1) % 1000000 ))
    sleep 4
  done
}

main "$@"

