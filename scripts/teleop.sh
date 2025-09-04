#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

PORT=${PORT:-2525}
# Allow override: BRAINSTEM_SERIAL=/dev/ttyACM0 BRAINSTEM_BAUD=115200

if ! python3 -c "import aiohttp,serial" 2>/dev/null; then
  echo "Missing deps. Please run: pip3 install aiohttp pyserial" >&2
  exit 1
fi

exec python3 tools/teleop_server.py --port "$PORT"

