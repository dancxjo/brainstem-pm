#!/usr/bin/env python3
"""
Tiny teleop webserver for brainstem-pm.

Serves http://localhost:2525 with a simple page that listens for arrow keys
and sends velocity commands over WebSocket. The server converts those into
brainstem serial lines (TWIST, SAFE, etc.) and writes them to the selected
serial port.

Usage:
  python3 tools/teleop_server.py [--port 2525]

Environment:
  BRAINSTEM_SERIAL   Explicit serial device (e.g. /dev/ttyACM0, COM3)
  BRAINSTEM_BAUD     Baud rate (default 115200)

Dependencies:
  pip install aiohttp pyserial
"""
import asyncio
import os
import sys
import json
import time
from pathlib import Path
from typing import Optional, Set

from aiohttp import web
import serial
import serial.tools.list_ports


ROOT = Path(__file__).resolve().parent
STATIC_DIR = ROOT / "static"


def pick_serial_port(explicit: Optional[str]) -> Optional[str]:
    if explicit:
        return explicit
    # Try to find a likely USB CDC device
    candidates = []
    for p in serial.tools.list_ports.comports():
        # Prefer ACM/USB names on Unix
        if p.device and ("ttyACM" in p.device or "ttyUSB" in p.device or "cu.usb" in p.device):
            score = 1
            desc = (p.description or "").lower()
            if any(k in desc for k in ["arduino", "micro", "leonardo", "teensy", "cdc", "usb serial"]):
                score += 1
            candidates.append((score, p.device))
        # On Windows, include COM ports
        if p.device and p.device.upper().startswith("COM"):
            candidates.append((0, p.device))
    if not candidates:
        return None
    candidates.sort(key=lambda t: (-t[0], t[1]))
    return candidates[0][1]


class BrainstemSerial:
    def __init__(self, dev: str, baud: int = 115200):
        self.dev = dev
        self.baud = baud
        self._ser: Optional[serial.Serial] = None
        self._lock = asyncio.Lock()

    async def open(self):
        # Open in thread to avoid blocking loop
        def _open():
            return serial.Serial(self.dev, self.baud, timeout=0)

        loop = asyncio.get_running_loop()
        self._ser = await loop.run_in_executor(None, _open)

    async def write_line(self, line: str):
        if not self._ser:
            raise RuntimeError("serial not open")
        data = (line.rstrip("\n") + "\n").encode("ascii", errors="ignore")
        async with self._lock:
            # Use executor to avoid blocking
            loop = asyncio.get_running_loop()
            await loop.run_in_executor(None, self._ser.write, data)

    async def read_lines(self):
        if not self._ser:
            raise RuntimeError("serial not open")
        buf = bytearray()
        loop = asyncio.get_running_loop()
        read = self._ser.read
        while True:
            # Read small chunks without blocking the event loop too long
            chunk = await loop.run_in_executor(None, read, 128)
            if chunk:
                buf.extend(chunk)
                while True:
                    try:
                        nl = buf.index(0x0A)
                    except ValueError:
                        break
                    line = bytes(buf[:nl]).decode("ascii", errors="ignore").rstrip("\r")
                    del buf[: nl + 1]
                    yield line
            else:
                await asyncio.sleep(0.01)

    async def close(self):
        if self._ser and self._ser.is_open:
            ser = self._ser
            self._ser = None
            loop = asyncio.get_running_loop()
            await loop.run_in_executor(None, ser.close)


class TeleopServer:
    def __init__(self, serial_dev: str, baud: int):
        self.serial = BrainstemSerial(serial_dev, baud)
        self.ws_clients: Set[web.WebSocketResponse] = set()
        self.seq = 1
        # Debug/health
        self.tx_count = 0
        self.rx_count = 0
        self.last_tx = None  # type: Optional[str]
        self.last_rx = None  # type: Optional[str]
        self.started_ts = time.time()

    async def start(self):
        await self.serial.open()

    def routes(self):
        r = web.RouteTableDef()

        @r.get("/")
        async def index(request):
            return web.FileResponse(STATIC_DIR / "index.html")

        @r.get("/ws")
        async def ws_handler(request):
            ws = web.WebSocketResponse(heartbeat=25.0)
            await ws.prepare(request)
            self.ws_clients.add(ws)
            print("[teleop] ws connected; total=", len(self.ws_clients))
            # Send hello with port info
            await ws.send_json({"type": "hello", "serial": self.serial.dev, "baud": self.serial.baud})
            try:
                async for msg in ws:
                    if msg.type == web.WSMsgType.TEXT:
                        await self.handle_msg(ws, msg.data)
                    elif msg.type == web.WSMsgType.ERROR:
                        break
            finally:
                self.ws_clients.discard(ws)
                print("[teleop] ws disconnected; total=", len(self.ws_clients))
            return ws

        @r.get("/healthz")
        async def healthz(request):
            return web.json_response({"ok": True, "serial": self.serial.dev})

        @r.get("/static/{name}")
        async def static_file(request):
            name = request.match_info["name"]
            p = STATIC_DIR / name
            if not p.is_file():
                raise web.HTTPNotFound()
            return web.FileResponse(p)

        @r.get("/status")
        async def status(request):
            return web.json_response({
                "ok": True,
                "serial": self.serial.dev,
                "baud": self.serial.baud,
                "ws_clients": len(self.ws_clients),
                "tx_count": self.tx_count,
                "rx_count": self.rx_count,
                "last_tx": self.last_tx,
                "last_rx": self.last_rx,
                "uptime_s": int(time.time() - self.started_ts),
            })

        return r

    async def handle_msg(self, ws: web.WebSocketResponse, data: str):
        try:
            msg = json.loads(data)
        except Exception:
            return
        t = msg.get("type")
        if t == "twist":
            vx = float(msg.get("vx", 0.0))
            wz = float(msg.get("wz", 0.0))
            self.seq = (self.seq + 1) & 0xFFFFFFFF
            line = f"TWIST,{vx:.3f},{wz:.3f},{self.seq}"
            print(f"[teleop] ws→serial: {line}")
            try:
                await self.serial.write_line(line)
                self.tx_count += 1
                self.last_tx = line
            finally:
                # Fan out TX echo for UI logs
                payload = json.dumps({"type": "tx", "line": line})
                for w in list(self.ws_clients):
                    try:
                        await w.send_str(payload)
                    except Exception:
                        pass
        elif t == "safe":
            v = 1 if bool(msg.get("enable", True)) else 0
            line = f"SAFE,{v}"
            print(f"[teleop] ws→serial: {line}")
            try:
                await self.serial.write_line(line)
                self.tx_count += 1
                self.last_tx = line
            finally:
                payload = json.dumps({"type": "tx", "line": line})
                for w in list(self.ws_clients):
                    try:
                        await w.send_str(payload)
                    except Exception:
                        pass
        elif t == "ping":
            self.seq = (self.seq + 1) & 0xFFFFFFFF
            line = f"PING,{self.seq}"
            print(f"[teleop] ws→serial: {line}")
            try:
                await self.serial.write_line(line)
                self.tx_count += 1
                self.last_tx = line
            finally:
                payload = json.dumps({"type": "tx", "line": line})
                for w in list(self.ws_clients):
                    try:
                        await w.send_str(payload)
                    except Exception:
                        pass
        elif t == "led":
            try:
                mask = int(msg.get("mask", 0))
            except Exception:
                mask = 0
            line = f"LED,{mask}"
            print(f"[teleop] ws→serial: {line}")
            try:
                await self.serial.write_line(line)
                self.tx_count += 1
                self.last_tx = line
            finally:
                payload = json.dumps({"type": "tx", "line": line})
                for w in list(self.ws_clients):
                    try:
                        await w.send_str(payload)
                    except Exception:
                        pass

    async def telemetry_task(self):
        # Read MCU lines and fan out to any clients
        async for line in self.serial.read_lines():
            print(f"[teleop] serial→ws: {line}")
            self.rx_count += 1
            self.last_rx = line
            if not self.ws_clients:
                continue
            dead = []
            payload = json.dumps({"type": "rx", "line": line})
            for ws in list(self.ws_clients):
                try:
                    await ws.send_str(payload)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                self.ws_clients.discard(ws)


async def main_async(argv):
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=2525, help="HTTP port (default 2525)")
    ap.add_argument("--serial", type=str, default=os.environ.get("BRAINSTEM_SERIAL"), help="Serial device path")
    ap.add_argument("--baud", type=int, default=int(os.environ.get("BRAINSTEM_BAUD", "115200")))
    args = ap.parse_args(argv)

    dev = pick_serial_port(args.serial)
    if not dev:
        print("ERROR: No serial device found. Set BRAINSTEM_SERIAL.", file=sys.stderr)
        return 2

    server = TeleopServer(dev, args.baud)
    await server.start()

    app = web.Application()
    app.add_routes(server.routes())

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "127.0.0.1", args.port)
    await site.start()

    print(f"teleop: http://127.0.0.1:{args.port}  serial={dev} baud={args.baud}")
    # Background telemetry fanout
    tel_task = asyncio.create_task(server.telemetry_task())
    try:
        while True:
            await asyncio.sleep(3600)
    except (asyncio.CancelledError, KeyboardInterrupt):
        pass
    finally:
        tel_task.cancel()
        await server.serial.close()
        await runner.cleanup()


def main():
    try:
        asyncio.run(main_async(sys.argv[1:]))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
