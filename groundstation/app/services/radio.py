"""Serial link to the drone.

Owns the one serial port — both directions. A background thread reads
inbound frames, verifies Adler-32, parses TLM/EVT, updates a live
state model, persists what we care about (measurements only while the
drone is in MEASURING; every EVT into the audit log; geo-tagged
collections into their own table), and broadcasts everything to any
SSE subscribers waiting in the FastAPI event loop.

Outbound CMD frames are built from `send_cmd()` and written through
the same serial port under a lock so they can never interleave with
the listener's reads or each other.

Threading model:

    [serial pyhw]──read──▶ _listen_loop (bg thread)
                            │ parse + checksum
                            ├──▶ DB writes (measurements/collections/events)
                            ├──▶ state mutation (under state_lock)
                            └──▶ broadcast(event)──┐
                                                   │ call_soon_threadsafe
                                                   ▼
                                            asyncio.Queue per SSE subscriber
                                                   │
                                                   ▼
                                            FastAPI event loop (main thread)

    POST /api/cmd/<verb> ──▶ send_cmd() ──serial.write under write_lock──▶ [serial hw]
"""

import asyncio
import re
import serial
import threading
import time
import zlib
from datetime import datetime
from typing import Optional

from app.database import insert_measurement, insert_collection, insert_event


# Frame size limit per protocol §2 (256 bytes including newline). Be
# slightly generous on the inbound side; refuse only the truly long.
MAX_FRAME_LEN = 280


def adler32(data: bytes) -> int:
    """Return Adler-32 of `data` (matches zlib.adler32 and the firmware)."""
    return zlib.adler32(data)


def build_frame(payload: str) -> bytes:
    """Build a complete `<payload>*<HEX>\\n` frame ready for the wire."""
    data = payload.encode("utf-8")
    return data + f"*{adler32(data):08X}\n".encode("ascii")


def parse_kv(details: str) -> dict[str, str]:
    """Parse 'k1=v1,k2=v2,...' details into a dict (values stay strings)."""
    out: dict[str, str] = {}
    for piece in details.split(","):
        if "=" in piece:
            k, v = piece.split("=", 1)
            out[k.strip()] = v.strip()
    return out


# ---------------- live state ----------------

class LiveState:
    """Snapshot of the drone state inferred from the inbound EVT/TLM stream.

    Mutated only from the radio thread (under `lock`). Read from any
    thread via `snapshot()` (also takes `lock`).
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.system_mode: str = "UNKNOWN"
        self.tanks: dict[str, dict] = {
            t: {"state": "UNKNOWN", "step": "NONE"} for t in ("C1", "C2", "C3")
        }
        # Per-tank level-sensor cache: result of the most recent
        # EVT,SYS,LEVEL,id=N,... response. `full` is the interpreted
        # state (post ACTIVE_LOW); `raw` is the unfiltered digitalRead.
        # `received_at` is server-side ISO 8601 so the UI can grey out
        # stale readings (e.g. radio dropped, sensor unreachable).
        self.tank_levels: dict[str, dict] = {
            t: {"full": None, "raw": None, "received_at": None}
            for t in ("C1", "C2", "C3")
        }
        self.elmetron: dict = {"state": "UNKNOWN", "step": "NONE"}
        self.firmware_version: Optional[str] = None
        self.last_tlm: dict = {}            # last parsed TLM (any mode)
        self.last_tlm_at: Optional[str] = None  # server-side ISO 8601
        self.last_battery_volts: Optional[float] = None
        self.last_gps: dict = {}            # last EVT,SYS,GPS,... payload

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "system_mode": self.system_mode,
                "tanks": {k: dict(v) for k, v in self.tanks.items()},
                "tank_levels": {k: dict(v) for k, v in self.tank_levels.items()},
                "elmetron": dict(self.elmetron),
                "firmware_version": self.firmware_version,
                "last_tlm": dict(self.last_tlm),
                "last_tlm_at": self.last_tlm_at,
                "last_battery_volts": self.last_battery_volts,
                "last_gps": dict(self.last_gps),
            }


# ---------------- radio service ----------------

class RadioService:
    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 115200):
        self.port = port
        self.baud = baud
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.ser: Optional[serial.Serial] = None
        self.state = LiveState()

        # Coordinates outbound writes (CMDs from any FastAPI handler).
        self.write_lock = threading.Lock()

        # SSE subscribers. Each is an asyncio.Queue living in the
        # FastAPI event loop. The radio thread posts to them via
        # call_soon_threadsafe so cross-thread access stays sound.
        self.subscribers: set[asyncio.Queue] = set()
        self.subscribers_lock = threading.Lock()
        self.main_loop: Optional[asyncio.AbstractEventLoop] = None

    # -------- lifecycle --------

    def start(self, main_loop: asyncio.AbstractEventLoop):
        """Open the port and spin up the listener.

        `main_loop` is the FastAPI event loop, captured here so the
        background thread can schedule subscriber sends on it via
        call_soon_threadsafe.
        """
        if self.running:
            return
        self.main_loop = main_loop
        self.running = True
        self.thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.thread.start()
        print(f"[Radio] Service started on {self.port} @ {self.baud}")

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        print("[Radio] Service stopped.")

    # -------- outbound --------

    def send_cmd(self, verb: str, args: Optional[list[str]] = None) -> bool:
        """Build and write a CMD frame. Thread-safe. Returns True on write."""
        if not self.ser:
            return False
        payload = "CMD," + verb
        if args:
            payload += "," + ",".join(args)
        frame = build_frame(payload)
        with self.write_lock:
            try:
                self.ser.write(frame)
                self.ser.flush()
                return True
            except Exception as e:
                print(f"[Radio] send_cmd failed: {e}")
                return False

    # -------- SSE pub/sub --------

    def subscribe(self) -> asyncio.Queue:
        """Register an SSE subscriber. Caller must call `unsubscribe`."""
        q: asyncio.Queue = asyncio.Queue()
        with self.subscribers_lock:
            self.subscribers.add(q)
        return q

    def unsubscribe(self, q: asyncio.Queue):
        with self.subscribers_lock:
            self.subscribers.discard(q)

    def _broadcast(self, event: dict):
        """Push an event to every SSE subscriber, hop into the FastAPI loop."""
        loop = self.main_loop
        if not loop:
            return
        with self.subscribers_lock:
            queues = list(self.subscribers)
        for q in queues:
            try:
                loop.call_soon_threadsafe(q.put_nowait, event)
            except RuntimeError:
                # Loop closed during shutdown — silently drop.
                pass

    # -------- inbound --------

    def _listen_loop(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)  # let the link settle on hot-attach
        except serial.SerialException as e:
            print(f"[Radio] Could not open {self.port}: {e}")
            self.running = False
            return

        print("[Radio] Listening for frames...")
        while self.running:
            try:
                line = self.ser.readline()
                if not line:
                    continue
                if len(line) > MAX_FRAME_LEN:
                    continue
                text = line.decode("utf-8", errors="ignore").strip()
                if not text or "*" not in text:
                    continue
                self._handle_line(text)
            except serial.SerialException as e:
                print(f"[Radio] Serial error: {e}")
                time.sleep(1)
            except Exception as e:
                print(f"[Radio] Loop error: {e}")
                time.sleep(0.1)

        try:
            self.ser.close()
        except Exception:
            pass

    def _handle_line(self, line: str):
        # Split payload*checksum
        try:
            payload, cs_hex = line.rsplit("*", 1)
            received_cs = int(cs_hex, 16)
        except ValueError:
            return  # malformed
        if adler32(payload.encode("utf-8")) != received_cs:
            # Silent drop per protocol §3 / §8.
            return

        if payload.startswith("TLM,"):
            self._handle_tlm(payload)
        elif payload.startswith("EVT,"):
            self._handle_evt(payload)
        # CMD frames echoed back to us aren't expected (we sent them);
        # ignore anything else silently.

    # -------- TLM --------

    # TLM format: TLM,<ts>,<lat>,<lon>,<cond>,<temp>,<ph>,<oxygen>,
    #             <water_flag>,<measurement_valid>
    # 10 comma-separated parts including the TLM prefix.

    def _handle_tlm(self, payload: str):
        parts = payload.split(",")
        if len(parts) < 10:
            # Older firmware (missing measurement_valid) — accept with
            # measurement_valid defaulted to 0 to keep things tolerant.
            parts = parts + ["0"] * (10 - len(parts))
        try:
            tlm = {
                "drone_utc": parts[1],
                "lat": int(parts[2]) / 1e7,
                "lon": int(parts[3]) / 1e7,
                "cond": float(parts[4]),
                "temp": float(parts[5]),
                "ph": float(parts[6]),
                "oxygen": float(parts[7]),
                "water_flag": bool(int(parts[8])),
                "measurement_valid": bool(int(parts[9])),
            }
        except (ValueError, IndexError):
            return

        received_at = datetime.utcnow().isoformat()

        # Update live state
        with self.state.lock:
            self.state.last_tlm = tlm
            self.state.last_tlm_at = received_at
            current_mode = self.state.system_mode

        # Persist only while MEASURING. Heartbeat TLMs outside the
        # measurement window are intentionally not stored — they only
        # serve "drone is alive," and that's tracked in live state.
        if current_mode == "MEASURING":
            try:
                insert_measurement({
                    "received_at": received_at,
                    **tlm,
                })
            except Exception as e:
                print(f"[Radio] insert_measurement failed: {e}")

        self._broadcast({
            "type": "tlm",
            "received_at": received_at,
            "system_mode": current_mode,
            "tlm": tlm,
        })

    # -------- EVT --------

    # EVT format: EVT,<source>,<kind>,<details...>
    # `details` may contain commas (e.g. EVT,C1,COLLECTED,lat=...,lon=...).
    # So we split on at most 3 commas — first 3 fields are fixed.

    def _handle_evt(self, payload: str):
        parts = payload.split(",", 3)
        if len(parts) < 3:
            return
        source = parts[1]
        kind = parts[2]
        details = parts[3] if len(parts) == 4 else ""
        received_at = datetime.utcnow().isoformat()

        # Persist the EVT in the audit log regardless of source/kind.
        try:
            insert_event({
                "received_at": received_at,
                "source": source,
                "kind": kind,
                "details": details,
            })
        except Exception as e:
            print(f"[Radio] insert_event failed: {e}")

        # Update state based on what kind of EVT this is.
        self._update_state_from_evt(source, kind, details)

        self._broadcast({
            "type": "evt",
            "received_at": received_at,
            "source": source,
            "kind": kind,
            "details": details,
        })

    def _update_state_from_evt(self, source: str, kind: str, details: str):
        # System-wide state
        if source == "SYS" and kind == "STATE":
            with self.state.lock:
                self.state.system_mode = details
            return

        # Per-tank state / step
        if source in ("C1", "C2", "C3"):
            if kind == "STATE":
                with self.state.lock:
                    self.state.tanks[source]["state"] = details
                return
            if kind == "STEP":
                with self.state.lock:
                    self.state.tanks[source]["step"] = details
                return
            if kind == "COLLECTED":
                self._handle_collected(source, details)
                return

        # Elmetron state / step
        if source == "ELMETRON":
            if kind == "STATE":
                with self.state.lock:
                    self.state.elmetron["state"] = details
                return
            if kind == "STEP":
                with self.state.lock:
                    self.state.elmetron["step"] = details
                return

        # SYS housekeeping
        if source == "SYS":
            if kind == "BOOT":
                with self.state.lock:
                    self.state.firmware_version = details
                return
            if kind == "BATTERY":
                # Format: "11.40V/raw=4095/mv=3300" — pull the leading float.
                m = re.match(r"([\d.]+)\s*V", details)
                if m:
                    try:
                        with self.state.lock:
                            self.state.last_battery_volts = float(m.group(1))
                    except ValueError:
                        pass
                return
            if kind == "GPS":
                kv = parse_kv(details)
                with self.state.lock:
                    self.state.last_gps = kv
                return
            if kind == "LEVEL":
                # EVT,SYS,LEVEL,id=N,full=0|1,raw=0|1 — periodic CMD,LEVEL
                # poll from the level-poller, used by the operator UI to
                # show a live wet/dry indicator per tank.
                kv = parse_kv(details)
                try:
                    tank_id = int(kv.get("id", "0"))
                    full = kv.get("full") == "1"
                    raw = kv.get("raw") == "1"
                except ValueError:
                    return
                tank_key = f"C{tank_id}" if 1 <= tank_id <= 3 else None
                if tank_key:
                    with self.state.lock:
                        self.state.tank_levels[tank_key] = {
                            "full": full,
                            "raw": raw,
                            "received_at": datetime.utcnow().isoformat(),
                        }
                return

    def _handle_collected(self, tank: str, details: str):
        """Parse `valid=1,fix=1,lat=...,lon=...,utc=...` and store it."""
        kv = parse_kv(details)
        try:
            lat = int(kv.get("lat", "0")) / 1e7
            lon = int(kv.get("lon", "0")) / 1e7
        except ValueError:
            lat, lon = None, None
        fix_valid = kv.get("fix", "0") == "1"
        collected_at_utc = kv.get("utc")
        received_at = datetime.utcnow().isoformat()
        try:
            insert_collection({
                "received_at": received_at,
                "tank": tank,
                "collected_at_utc": collected_at_utc,
                "lat": lat,
                "lon": lon,
                "fix_valid": fix_valid,
            })
        except Exception as e:
            print(f"[Radio] insert_collection failed: {e}")
