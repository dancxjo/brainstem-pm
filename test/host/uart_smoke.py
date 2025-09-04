#!/usr/bin/env python3
"""
UART smoke tester for the brainstem skeleton.

Examples:
  python3 test/host/uart_smoke.py /dev/ttyACM0 --ping 2 --range 0.18 --set soft_stop_m=0.25 --fuzz --pause --resume --replay
"""
import sys
import time
import argparse

try:
    import serial  # pyserial
except Exception as e:
    print("pyserial not available. Install with: pip install pyserial", file=sys.stderr)
    sys.exit(1)

def send_line(ser, s):
    ser.write((s + "\n").encode("ascii"))
    ser.flush()
    print("→", s)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--ping", type=int, default=0, help="send N pings")
    ap.add_argument("--range", type=float, default=None, help="send RANGE,<m>,0")
    ap.add_argument("--set", action="append", default=[], help="SET key=val pairs")
    ap.add_argument("--fuzz", action="store_true", help="send malformed lines")
    ap.add_argument("--fuzz-linelength", action="store_true", help="send 200B line to trigger overflow")
    ap.add_argument("--pause", action="store_true")
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--replay", action="store_true")
    ap.add_argument("--rate", type=int, default=None, help="simulate slow reader by sleeping us per read batch")
    args = ap.parse_args()
    port = args.port
    baud = args.baud
    ser = serial.Serial(port, baudrate=baud, timeout=0.01)
    t0 = time.time()
    last = time.time()
    buf = b""

    # Boot handshake
    time.sleep(0.2)
    send_line(ser, "SAFE,1")
    # PINGs
    for i in range(args.ping):
        send_line(ser, f"PING,{i}")
        time.sleep(0.02)
    # Params
    for kv in args.set:
        if "=" not in kv:
            continue
        k,v = kv.split("=",1)
        send_line(ser, f"SET,{k},{v}")
        time.sleep(0.05)
    # TWIST path
    time.sleep(0.2)
    send_line(ser, "TWIST,0.2,0.0,1")
    time.sleep(0.6)
    send_line(ser, "TWIST,0.0,0.0,2")
    # RANGE
    if args.range is not None:
        send_line(ser, f"RANGE,{args.range:.2f},0")
    # PAUSE/RESUME
    if args.pause:
        send_line(ser, "PAUSE")
        time.sleep(0.3)
    if args.resume:
        send_line(ser, "RESUME")

    # FUZZ
    if args.fuzz:
        send_line(ser, "TWIST,abc,0,1")  # bad number
        send_line(ser, "UNKN,1,2")       # unknown verb
        send_line(ser, "PING,1*ZZ")      # bad CRC
    if args.fuzz_linelength:
        send_line(ser, "X"*200)

    next_led = time.time() + 1.0
    last_eid = 0

    try:
        while time.time() - t0 < 8.0:
            # Read and print lines from device
            data = ser.read(1024)
            if data:
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if line:
                        s = line.decode(errors="replace")
                        print("←", s)
                        if ",eid=" in s:
                            try:
                                last_eid = int(s.rsplit(",eid=",1)[1])
                            except Exception:
                                pass
            if time.time() > next_led:
                send_line(ser, "LED,1")
                next_led += 1.0
            if args.replay and last_eid:
                # Ask for last 5 events
                send_line(ser, f"REPLAY,{max(0,last_eid-5)}")
                args.replay = False
            if args.rate:
                time.sleep(args.rate / 1_000_000.0)
            time.sleep(0.01)
    finally:
        ser.close()

if __name__ == "__main__":
    main()
