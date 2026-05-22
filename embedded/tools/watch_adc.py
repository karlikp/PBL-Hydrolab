#!/usr/bin/env python3
"""Poll the drone's CMD,ADC_SCAN every ~1 s and print the raw ADC values
for the candidate SUPADC pins. Use this to watch the ADC reading live
while sweeping bench supply voltage — helps confirm which pin tracks
battery and at what voltages the ADC saturates.

Usage:
    watch_adc.py [-p /dev/ttyUSB0] [-i 1.0]

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
    p.add_argument("-i", "--interval", type=float, default=1.0,
                   help="seconds between ADC_SCAN polls (default: 1)")
    a = p.parse_args()

    s = serial.Serial(a.port, a.baud, timeout=0.2)
    print(f"watching {a.port} — Ctrl-C to exit\n"
          f"format: HH:MM:SS  IO1 raw/mv | IO2 raw/mv | IO3 raw/mv | IO5 raw/mv")

    last_poll = 0.0
    pending = {}
    try:
        while True:
            if time.monotonic() - last_poll > a.interval:
                s.write(build_frame("CMD,ADC_SCAN,"))
                last_poll = time.monotonic()
                pending = {}

            line = s.readline()
            if not line:
                continue
            d = line.decode("ascii", errors="replace").rstrip()
            if "EVT,SYS,ADC,IO" not in d:
                continue

            # Frame: EVT,SYS,ADC,IO5/raw=3484/mv=2900*XXXXXXXX
            try:
                body = d.split(",", 3)[3]              # IO5/raw=3484/mv=2900*...
                pin_part, rest = body.split("/", 1)    # "IO5", "raw=3484/mv=2900*..."
                raw_part, mv_part = rest.split("/", 1) # "raw=3484", "mv=2900*..."
                raw = int(raw_part.split("=")[1])
                mv_clean = mv_part.split("*")[0]       # "mv=2900"
                mv = int(mv_clean.split("=")[1])
                pending[pin_part] = (raw, mv)
            except Exception:
                continue

            # Once we've collected all four pins, print one line
            if len(pending) == 4:
                ts = time.strftime("%H:%M:%S")
                parts = []
                for io in ("IO1", "IO2", "IO3", "IO5"):
                    raw, mv = pending[io]
                    sat = " *" if raw >= 4095 else "  "
                    parts.append(f"{io} {raw:4d}/{mv:4d}mv{sat}")
                print(f"{ts}  " + " | ".join(parts))
                pending = {}
    except KeyboardInterrupt:
        pass
    finally:
        s.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
