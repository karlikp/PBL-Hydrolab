#!/usr/bin/env python3
"""Send a CMD frame to the drone over serial.

The skeleton firmware listens for CMD frames on the USB serial link.
Each frame needs an Adler-32 checksum, which is annoying to compute by
hand — this script does that for you.

Usage examples:
    send_cmd.py START_C1
    send_cmd.py STATUS
    send_cmd.py --watch 15 START_C1     # send then listen 15 seconds
    send_cmd.py -p /dev/ttyUSB1 PING

Run from the host (laptop), not the embedded board.
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
    """Return the bytes for a complete '<payload>*<adler>\\n' frame."""
    payload_bytes = payload.encode("utf-8")
    cs = zlib.adler32(payload_bytes)
    return payload_bytes + f"*{cs:08X}\n".encode("ascii")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("verb", help="Command verb, e.g. START_C1, STATUS, PING")
    parser.add_argument("args", nargs="?", default="",
                        help="Optional command argument string (most verbs take none)")
    parser.add_argument("-p", "--port", default="/dev/ttyUSB0",
                        help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("-b", "--baud", type=int, default=115200,
                        help="Baud rate (default: 115200, matches skeleton firmware)")
    parser.add_argument("--watch", type=float, default=5.0,
                        help="Seconds to listen for replies after sending. "
                             "Set to 0 to skip listening. (default: 5)")
    a = parser.parse_args()

    payload = f"CMD,{a.verb},{a.args}"
    frame = build_frame(payload)

    try:
        ser = serial.Serial(a.port, a.baud, timeout=0.2)
    except serial.SerialException as e:
        sys.exit(f"open {a.port}: {e}")

    # Brief settle, then drain whatever was already in the rx buffer.
    time.sleep(0.1)
    if ser.in_waiting:
        ser.read(ser.in_waiting)

    print(f"-> {frame.decode('ascii', errors='replace').rstrip()}")
    ser.write(frame)

    if a.watch > 0:
        deadline = time.monotonic() + a.watch
        while time.monotonic() < deadline:
            line = ser.readline()
            if line:
                print(f"<- {line.decode('ascii', errors='replace').rstrip()}")

    ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
