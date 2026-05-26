#!/usr/bin/env python3
"""Send a CMD frame to the drone over serial, retrying until an ACK
or NACK for the requested verb is seen (or `--retries` runs out).

The drone always emits exactly one `EVT,SYS,{ACK,NACK},<verb>` for
every valid CMD frame it receives. If the link drops the CMD frame
mid-flight (we've observed this on the V5 radios under load), no
ACK ever comes back. This script resends the CMD up to `--retries`
times until that ACK/NACK appears. Once it does, the script switches
into a longer `--watch` window so follow-up state/step events get
captured too.

Usage:
    send_cmd.py PING
    send_cmd.py STATUS
    send_cmd.py SERVO_MOVE 1 200
    send_cmd.py -p /dev/ttyUSB1 -b 57600 LEVEL 1

Exit code is 0 if an ACK/NACK was received; 1 if we gave up.
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
    parser.add_argument("args", nargs="*",
                        help="Zero or more argument values; joined with commas "
                             "on the wire (so 'SERVO_MOVE 1 200' sends "
                             "CMD,SERVO_MOVE,1,200)")
    parser.add_argument("-p", "--port", default="/dev/ttyUSB0",
                        help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("-b", "--baud", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("--watch", type=float, default=5.0,
                        help="Seconds to listen for follow-up frames after the "
                             "ACK/NACK lands. Set to 0 to exit immediately on "
                             "ACK. (default: 5)")
    parser.add_argument("--retries", type=int, default=4,
                        help="Max number of CMD send attempts. Each attempt "
                             "waits up to --ack-timeout for an ACK/NACK before "
                             "resending. (default: 4)")
    parser.add_argument("--ack-timeout", type=float, default=2.0,
                        help="Seconds to wait per attempt for an ACK/NACK. "
                             "(default: 2)")
    a = parser.parse_args()

    if a.retries < 1:
        sys.exit("--retries must be >= 1")

    args_joined = ",".join(a.args)
    payload = f"CMD,{a.verb},{args_joined}"
    frame = build_frame(payload)

    # An ACK/NACK for our verb has a known textual prefix. The verb
    # itself is delimited from the rest of the payload by `*` (ACK) or
    # `:` (NACK), so we match those exact characters to avoid e.g.
    # PUMP_OFF being matched by a PUMP NACK.
    ack_prefix  = f"EVT,SYS,ACK,{a.verb}*".encode("ascii")
    nack_prefix = f"EVT,SYS,NACK,{a.verb}:".encode("ascii")

    try:
        ser = serial.Serial(a.port, a.baud, timeout=0.1)
    except serial.SerialException as e:
        sys.exit(f"open {a.port}: {e}")

    # Settle and drain any in-flight bytes.
    time.sleep(0.1)
    if ser.in_waiting:
        ser.read(ser.in_waiting)

    received_ack_nack = False
    for attempt in range(1, a.retries + 1):
        label = "->" if attempt == 1 else f"-> [retry {attempt}/{a.retries}]"
        print(f"{label} {frame.decode('ascii', errors='replace').rstrip()}")
        ser.write(frame)

        deadline = time.monotonic() + a.ack_timeout
        while time.monotonic() < deadline:
            line = ser.readline()
            if not line:
                continue
            print(f"<- {line.decode('ascii', errors='replace').rstrip()}")
            if line.startswith(ack_prefix) or line.startswith(nack_prefix):
                received_ack_nack = True
                break
        if received_ack_nack:
            break

    if not received_ack_nack:
        print(f"-- no ACK/NACK after {a.retries} attempts; giving up",
              file=sys.stderr)
    elif a.watch > 0:
        # Drone acknowledged; keep listening so the operator sees the
        # follow-up events (state/step changes for START_*, the EVT
        # bundle for STATUS, periodic TLM, etc.).
        deadline = time.monotonic() + a.watch
        while time.monotonic() < deadline:
            line = ser.readline()
            if line:
                print(f"<- {line.decode('ascii', errors='replace').rstrip()}")

    ser.close()
    return 0 if received_ack_nack else 1


if __name__ == "__main__":
    raise SystemExit(main())
