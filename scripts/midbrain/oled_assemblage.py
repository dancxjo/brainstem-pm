#!/usr/bin/env python3
import argparse
import os
import signal
import socket
import subprocess
import sys
import time
from datetime import datetime

import psutil
from PIL import ImageFont
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import sh1106, ssd1306

# ---------- Config defaults ----------
DEFAULT_ADDRS = [0x3C, 0x3D]
DEFAULT_WIDTH, DEFAULT_HEIGHT = 128, 64
DEFAULT_ROTATE = 3  # portrait default; use 2 if panel is inverted in case
DEFAULT_ORIENTATION = "landscape"  # portrait | landscape | portrait180 | landscape180 | auto
DEFAULT_CONTROLLER = "sh1106"  # your panel
DEFAULT_INTERVAL = 3.0  # seconds per page
DEFAULT_H_OFFSET = 2    # SH1106 often needs this (landscape). Portrait usually 0.

# ---------- Small helpers ----------
def ip_addrs():
    # Prefer non-loopback IPv4s in a simple, dependency-free way
    addrs = []
    try:
        out = subprocess.check_output("hostname -I", shell=True, text=True).strip()
        for tok in out.split():
            if tok and not tok.startswith("127."):
                addrs.append(tok)
    except Exception:
        pass
    if not addrs:
        # Fallback to route trick
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            addrs = [s.getsockname()[0]]
            s.close()
        except Exception:
            addrs = ["0.0.0.0"]
    return addrs

def cpu_temp_c():
    # Works on Pi/Ubuntu if thermal zone is exposed
    paths = [
        "/sys/class/thermal/thermal_zone0/temp",
        "/sys/class/hwmon/hwmon0/temp1_input",
    ]
    for p in paths:
        try:
            v = open(p).read().strip()
            return int(v) / (1000.0 if len(v) > 3 else 1.0)
        except Exception:
            continue
    return None

def human_pct(n):
    return f"{n:.0f}%"

def draw_header(draw, text, w):
    draw.rectangle((0, 0, w-1, 12), outline=255, fill=0)
    draw.text((2, 1), text, fill=255)

def pick_font():
    # Use the tiny default bitmap font (no extra deps); switch to a TTF if you want
    return ImageFont.load_default()

# ---------- OLED setup / probe ----------
def make_device(controller, width, height, addr=None, rotate=DEFAULT_ROTATE, h_offset=DEFAULT_H_OFFSET):
    addrs = [addr] if addr is not None else DEFAULT_ADDRS
    last_err = None
    for a in addrs:
        try:
            serial = i2c(port=1, address=a)
            if controller == "sh1106":
                # Synchronize offset with rotation to avoid portrait distortion on SH1106
                # If caller passed explicit h_offset, honor it; else auto-pick.
                if h_offset is None:
                    eff_h_offset = DEFAULT_H_OFFSET if rotate in (0, 2) else 0
                else:
                    eff_h_offset = h_offset
                return sh1106(serial, width=width, height=height, rotate=rotate, h_offset=eff_h_offset)
            elif controller == "ssd1306":
                return ssd1306(serial, width=width, height=height, rotate=rotate)
            elif controller == "auto":
                # Try SH1106 first (common on 1.3"), then SSD1306
                try:
                    if h_offset is None:
                        eff_h_offset = DEFAULT_H_OFFSET if rotate in (0, 2) else 0
                    else:
                        eff_h_offset = h_offset
                    return sh1106(serial, width=width, height=height, rotate=rotate, h_offset=eff_h_offset)
                except Exception:
                    return ssd1306(serial, width=width, height=height, rotate=rotate)
        except Exception as e:
            last_err = e
            continue
    raise RuntimeError(f"Could not init display at {addrs} ({last_err})")

# ---------- Pages ----------
def page_splash(device, name="Midbrain", sub="SH1106 @ Geekworm"):
    w, h = device.width, device.height
    f = pick_font()
    with canvas(device) as draw:
        draw.rectangle((0, 0, w-1, h-1), outline=255, fill=0)
        draw.text((4, 8), name, fill=255, font=f)
        draw.text((4, 24), sub, fill=255, font=f)
        draw.text((4, 48), "Booting…", fill=255, font=f)

def page_network(device):
    w, h = device.width, device.height
    f = pick_font()
    addrs = ip_addrs()
    with canvas(device) as draw:
        draw_header(draw, "NET", w)
        y = 14
        for ip in addrs[:3]:  # show up to 3
            draw.text((2, y), ip, fill=255, font=f)
            y += 12

def page_cpu(device):
    w, h = device.width, device.height
    f = pick_font()
    load1, load5, load15 = psutil.getloadavg()
    freq = psutil.cpu_freq()
    temp = cpu_temp_c()
    with canvas(device) as draw:
        draw_header(draw, "CPU", w)
        draw.text((2, 14), f"Load: {load1:.2f} {load5:.2f} {load15:.2f}", fill=255, font=f)
        if freq:
            draw.text((2, 26), f"Freq: {freq.current/1000:.2f} GHz", fill=255, font=f)
        if temp is not None:
            draw.text((2, 38), f"Temp: {temp:.1f}°C", fill=255, font=f)

def page_mem_disk(device):
    w, h = device.width, device.height
    f = pick_font()
    vm = psutil.virtual_memory()
    du = psutil.disk_usage("/")
    with canvas(device) as draw:
        draw_header(draw, "MEM/DISK", w)
        draw.text((2, 14), f"RAM: {human_pct(vm.percent)}  ({vm.used//(1024**2)}M/{vm.total//(1024**2)}M)", fill=255, font=f)
        draw.text((2, 26), f"Disk:{human_pct(du.percent)}  ({du.used//(1024**3)}G/{du.total//(1024**3)}G)", fill=255, font=f)

def page_clock(device):
    w, h = device.width, device.height
    f = pick_font()
    now = datetime.now()
    with canvas(device) as draw:
        draw_header(draw, "TIME", w)
        draw.text((2, 16), now.strftime("%Y-%m-%d"), fill=255, font=f)
        draw.text((2, 30), now.strftime("%H:%M:%S"), fill=255, font=f)

# OPTIONAL: ROS 2 peek without bringing rclpy
def page_ros(device):
    w, h = device.width, device.height
    f = pick_font()
    try:
        nodes = subprocess.check_output("ros2 node list 2>/dev/null | wc -l", shell=True, text=True).strip()
        topics = subprocess.check_output("ros2 topic list 2>/dev/null | wc -l", shell=True, text=True).strip()
        with canvas(device) as draw:
            draw_header(draw, "ROS 2", w)
            draw.text((2, 16), f"Nodes:  {nodes}", fill=255, font=f)
            draw.text((2, 28), f"Topics: {topics}", fill=255, font=f)
    except Exception:
        with canvas(device) as draw:
            draw_header(draw, "ROS 2", w)
            draw.text((2, 16), "ros2 CLI not found", fill=255, font=f)

PAGES = [page_network, page_cpu, page_mem_disk, page_clock, page_ros]

# ---------- Main ----------
def main():
    ap = argparse.ArgumentParser(description="Tiny SH1106/SSD1306 status dashboard")
    # Orientation maps to rotate unless --rotate is explicitly provided.
    ap.add_argument(
        "--orientation",
        choices=["portrait", "landscape", "portrait180", "landscape180", "auto"],
        default=os.getenv("OLED_ORIENTATION", DEFAULT_ORIENTATION),
        help="Orientation preset: portrait/landscape (+180 variants) or auto"
    )
    # When set (CLI or env), --rotate takes precedence over --orientation
    env_rotate = os.getenv("OLED_ROTATE")
    ap.add_argument(
        "--rotate",
        type=int,
        choices=[0, 1, 2, 3],
        default=(int(env_rotate) if env_rotate is not None else None),
        help="Rotation override: 0=0°, 1=90° CW, 2=180°, 3=270°"
    )

    ap.add_argument("--controller", choices=["auto", "sh1106", "ssd1306"],
                    default=os.getenv("OLED_CONTROLLER", DEFAULT_CONTROLLER))
    ap.add_argument("--addr", type=lambda s: int(s, 0), default=os.getenv("OLED_ADDR"),
                    help="I2C address (e.g., 0x3C). If omitted, auto-try 0x3C/0x3D.")
    ap.add_argument("--width", type=int, default=int(os.getenv("OLED_W", DEFAULT_WIDTH)))
    ap.add_argument("--height", type=int, default=int(os.getenv("OLED_H", DEFAULT_HEIGHT)))
    ap.add_argument("--h-offset", dest="h_offset", type=int,
                    default=(int(os.getenv("OLED_H_OFFSET")) if os.getenv("OLED_H_OFFSET") is not None else None),
                    help="SH1106 horizontal offset. If omitted, auto: 2 for landscape, 0 for portrait")
    ap.add_argument("--interval", type=float, default=float(os.getenv("OLED_INTERVAL", DEFAULT_INTERVAL)))
    ap.add_argument("--name", default=os.getenv("OLED_NAME", "Midbrain"))
    ap.add_argument("--splash", action="store_true", help="Show splash for 2s at start")
    args = ap.parse_args()

    # Determine rotate from either explicit rotate or orientation preset
    def rotation_from_orientation(orient, w, h):
        o = (orient or "").lower()
        if o == "auto":
            # Choose portrait if taller than wide, else landscape
            o = "portrait" if h > w else "landscape"
        if o == "portrait":
            return 3  # matches default portrait need
        if o == "portrait180":
            return 1
        if o == "landscape":
            return 0
        if o == "landscape180":
            return 2
        # Fallback to default portrait
        return DEFAULT_ROTATE

    rotate = args.rotate if args.rotate is not None else rotation_from_orientation(args.orientation, args.width, args.height)

    device = make_device(args.controller, args.width, args.height, args.addr, rotate, args.h_offset)
    running = True

    def _shutdown(signum, frame):
        nonlocal running
        running = False
        try:
            device.hide()
        except Exception:
            pass

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    if args.splash:
        page_splash(device, args.name)
        time.sleep(2)

    i = 0
    while running:
        try:
            PAGES[i % len(PAGES)](device)
        except Exception as e:
            # Show the error briefly but don't crash the loop
            with canvas(device) as draw:
                draw.text((0, 0), "ERR:", fill=255)
                draw.text((0, 12), str(e)[:18], fill=255)
        time.sleep(args.interval)
        i += 1

    # Clear on exit
    try:
        device.hide()
    except Exception:
        pass

if __name__ == "__main__":
    sys.exit(main())
