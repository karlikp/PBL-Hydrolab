#!/usr/bin/env python3
"""Poll the drone's STATUS every 2 seconds and print battery voltage
plus any low-battery / shutdown events. Useful for verifying the
safety thresholds while you sweep the bench supply voltage.

Usage:
    watch_battery.py [-p /dev/ttyUSB0]

Exit with Ctrl-C.
"""

import argparse
import sys
import time
import zlib

try:
    import serial
except ImportError:
    sys.exit("install pyserial first:  pip install pyserial")


def build_frame(payload: str) -> bytes:
    cs = zlib.adler32(payload.encode("utf-8"))
    return f"{payload}*{cs:08X}\n".encode("ascii")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("-p", "--port", default="/dev/ttyUSB0")
    p.add_argument("-b", "--baud", type=int, default=115200)
    p.add_argument("-i", "--interval", type=float, default=2.0,
                   help="seconds between STATUS polls (default: 2)")
    a = p.parse_args()

    s = serial.Serial(a.port, a.baud, timeout=0.3)
    print(f"watching {a.port} — Ctrl-C to exit")

    last_poll = 0.0
    try:
        while True:
            if time.monotonic() - last_poll > a.interval:
                s.write(build_frame("CMD,STATUS,"))
                last_poll = time.monotonic()

            line = s.readline()
            if not line:
                continue
            d = line.decode("ascii", errors="replace").rstrip()
            # Surface only the interesting frames
            if "BATTERY" in d or "ERROR" in d or "BOOT" in d:
                print(time.strftime("%H:%M:%S"), d)
    except KeyboardInterrupt:
        pass
    finally:
        s.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
