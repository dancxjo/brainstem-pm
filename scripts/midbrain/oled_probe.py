import time
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306, sh1106

candidates = [
    ("ssd1306", ssd1306, (128, 64)),
    ("ssd1306", ssd1306, (128, 32)),
    ("sh1106",  sh1106,  (128, 64)),
]

addresses = [0x3C, 0x3D]
rotations = [0, 2]  # 0°, 180°

def try_one(name, cls, wh, addr, rot):
    serial = i2c(port=1, address=addr)
    w, h = wh
    # Some SH1106 panels need a 2-pixel horizontal offset; luma defaults are fine in most cases,
    # but you can pass 'h_offset=2' via kwargs if you still see a vertical “shear”.
    kwargs = {}
    if name == "sh1106":
        kwargs.setdefault("h_offset", 2)
    dev = cls(serial, width=w, height=h, rotate=rot, **kwargs)
    with canvas(dev) as draw:
        # Border + diagonals + label = easy visual sanity check
        draw.rectangle((0, 0, w-1, h-1), outline=255, fill=0)
        draw.line((0, 0, w-1, h-1), fill=255)
        draw.line((0, h-1, w-1, 0), fill=255)
        draw.text((2, 2), f"{name} {w}x{h} addr=0x{addr:02x} rot={rot}", fill=255)
    return dev

for name, cls, wh in candidates:
    for addr in addresses:
        for rot in rotations:
            print(f"Trying {name} {wh} @ 0x{addr:02x} rot={rot} ...")
            try:
                dev = try_one(name, cls, wh, addr, rot)
                time.sleep(3)  # look at the screen
                dev.hide()     # blank between tries
                time.sleep(0.3)
            except Exception as e:
                print("  -> failed:", e)

print("Done. Use the combo that looked correct.")
