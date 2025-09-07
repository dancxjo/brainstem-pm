#!/usr/bin/env python3
import sys, time, serial, json

def main():
    if len(sys.argv) < 2:
        print("Usage: host_sanity.py /dev/ttyACM0", file=sys.stderr)
        sys.exit(2)
    dev = sys.argv[1]
    ser = serial.Serial(dev, 115200, timeout=0.1)
    # Request wake/init
    ser.write(b"HELLO\n")
    t0 = time.time()
    ready = False
    while time.time() - t0 < 8:
        ln = ser.readline().decode(errors="ignore").strip()
        if ln:
            print(ln)
            if ln == "READY":
                ready = True
                break
    if not ready:
        sys.exit("No READY from brainstem")
    # Query status via escape
    ser.write(b"\xFF\x00!status\n")
    time.sleep(0.2)
    buf = ser.read(512).decode(errors="ignore")
    for line in buf.splitlines():
        if line.startswith("STATUS:"):
            print(line)
            try:
                js = json.loads(line[len("STATUS:"):])
                print("state:", js.get("state"), "baud:", js.get("baud"))
            except Exception:
                pass
            break

if __name__ == "__main__":
    main()

