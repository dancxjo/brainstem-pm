from time import sleep
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import sh1106   # swap to sh1106 if yours is SH1106

serial = i2c(port=1, address=0x3C)     # change to 0x3D if needed
device = sh1106(serial, width=128, height=64)

with canvas(device) as draw:
    draw.text((0, 0), "Hello, midbrain!", fill=255)
    draw.text((0, 16), "Geekworm OLED OK", fill=255)

sleep(5)
