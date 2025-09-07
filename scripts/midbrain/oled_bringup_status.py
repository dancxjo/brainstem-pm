#!/usr/bin/env python3
import argparse
import os
import sys
from contextlib import suppress

try:
    from PIL import ImageFont
    from luma.core.interface.serial import i2c
    from luma.core.render import canvas
    from luma.oled.device import sh1106, ssd1306
except Exception as e:
    print(f"[oled_bringup_status] Missing deps: {e}", file=sys.stderr)
    sys.exit(0)  # don't fail bringup on missing OLED deps


DEFAULT_ADDRS = [0x3C, 0x3D]
DEFAULT_WIDTH, DEFAULT_HEIGHT = 128, 64
DEFAULT_ROTATE = 3
DEFAULT_ORIENTATION = os.getenv("OLED_ORIENTATION", "portrait")
DEFAULT_CONTROLLER = os.getenv("OLED_CONTROLLER", "sh1106")


def pick_font():
    return ImageFont.load_default()


def rotation_from_orientation(orient, w, h):
    o = (orient or "").lower()
    if o == "auto":
        o = "portrait" if h > w else "landscape"
    if o == "portrait":
        return 3
    if o == "portrait180":
        return 1
    if o == "landscape":
        return 0
    if o == "landscape180":
        return 2
    return DEFAULT_ROTATE


def make_device(controller, width, height, addr=None, rotate=DEFAULT_ROTATE, h_offset=None):
    addrs = [addr] if addr is not None else DEFAULT_ADDRS
    last_err = None
    for a in addrs:
        try:
            serial = i2c(port=1, address=a)
            if controller == "sh1106":
                eff_h_offset = (2 if rotate in (0, 2) else 0) if h_offset is None else h_offset
                return sh1106(serial, width=width, height=height, rotate=rotate, h_offset=eff_h_offset)
            elif controller == "ssd1306":
                return ssd1306(serial, width=width, height=height, rotate=rotate)
            elif controller == "auto":
                with suppress(Exception):
                    eff_h_offset = (2 if rotate in (0, 2) else 0) if h_offset is None else h_offset
                    return sh1106(serial, width=width, height=height, rotate=rotate, h_offset=eff_h_offset)
                return ssd1306(serial, width=width, height=height, rotate=rotate)
        except Exception as e:
            last_err = e
            continue
    raise RuntimeError(f"Could not init display at {addrs} ({last_err})")


def main():
    ap = argparse.ArgumentParser(description="Render a simple one-shot bringup status message on the OLED")
    ap.add_argument("--header", default=os.getenv("OLED_STATUS_HEADER", "BRINGUP"))
    ap.add_argument("--controller", choices=["auto", "sh1106", "ssd1306"], default=DEFAULT_CONTROLLER)
    ap.add_argument("--addr", type=lambda s: int(s, 0), default=os.getenv("OLED_ADDR"))
    ap.add_argument("--width", type=int, default=int(os.getenv("OLED_W", DEFAULT_WIDTH)))
    ap.add_argument("--height", type=int, default=int(os.getenv("OLED_H", DEFAULT_HEIGHT)))
    ap.add_argument("--orientation", choices=["portrait", "landscape", "portrait180", "landscape180", "auto"],
                    default=DEFAULT_ORIENTATION)
    env_rotate = os.getenv("OLED_ROTATE")
    ap.add_argument("--rotate", type=int, choices=[0, 1, 2, 3], default=(int(env_rotate) if env_rotate is not None else None))
    ap.add_argument("--h-offset", dest="h_offset", type=int,
                    default=(int(os.getenv("OLED_H_OFFSET")) if os.getenv("OLED_H_OFFSET") is not None else None))
    ap.add_argument("--clear", action="store_true", help="Clear screen instead of drawing text")
    ap.add_argument("-l", "--line", action="append", help="Add a line of text (repeatable)")
    ap.add_argument("message", nargs="*", help="Optional freeform message; split into lines using ';'")
    args = ap.parse_args()

    rotate = args.rotate if args.rotate is not None else rotation_from_orientation(args.orientation, args.width, args.height)
    try:
        device = make_device(args.controller, args.width, args.height, args.addr, rotate, args.h_offset)
    except Exception as e:
        print(f"[oled_bringup_status] WARN: {e}", file=sys.stderr)
        return 0

    lines = []
    if args.line:
        lines.extend(args.line)
    if args.message:
        joined = " ".join(args.message)
        lines.extend([s for s in joined.split(";") if s])
    if not lines and not args.clear:
        lines = ["Starting…"]

    f = pick_font()
    w, h = device.width, device.height
    with canvas(device) as draw:
        if args.clear:
            draw.rectangle((0, 0, w-1, h-1), outline=0, fill=0)
        else:
            # Header band
            draw.rectangle((0, 0, w-1, 12), outline=255, fill=0)
            draw.text((2, 1), str(args.header)[:16], fill=255, font=f)
            y = 14
            for ln in lines[:4]:
                draw.text((2, y), ln[:20], fill=255, font=f)
                y += 12
    # Do not clear/hide on exit; leave content visible until next update
    return 0


if __name__ == "__main__":
    sys.exit(main())

