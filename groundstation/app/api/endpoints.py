"""HTTP API for the ground station.

Three layers:

  /api/data/all_readings        legacy alias kept for any external
                                  caller; redirects to the new
                                  measurements endpoint.
  /api/measurements/recent       historical TLM rows logged while the
                                  drone was MEASURING.
  /api/collections/recent        per-tank geo-tagged collection events.
  /api/events/recent             EVT audit log.
  /api/state/now                 one-shot snapshot of the live state
                                  (system mode, tank states, elmetron,
                                  last TLM, battery, firmware version).
  /api/stream/live               Server-Sent-Events stream pushing
                                  every EVT and TLM as they arrive.
  /api/cmd/{verb}                POST: build a CMD frame and write it
                                  to the serial port. ACK/NACK comes
                                  back over the EVT stream.
"""

import asyncio
import json
from typing import Optional

from fastapi import APIRouter, Body, HTTPException, Query, Request
from sse_starlette.sse import EventSourceResponse

from app.database import (
    get_recent_measurements,
    get_recent_collections,
    get_recent_events,
)
from app import config as cfg_mod

router = APIRouter(prefix="/api", tags=["api"])


def _radio(request: Request):
    """Pull the RadioService instance stashed by main.py at startup."""
    svc = getattr(request.app.state, "radio_service", None)
    if svc is None:
        raise HTTPException(status_code=503, detail="radio_service not running")
    return svc


# ---------------- historical data ----------------

@router.get("/measurements/recent")
def measurements_recent(limit: int = Query(5000, ge=1, le=20000),
                        valid_only: bool = False,
                        start: Optional[str] = None,
                        end: Optional[str] = None):
    rows = get_recent_measurements(
        limit=limit, valid_only=valid_only,
        start_iso=start, end_iso=end,
    )
    return {"count": len(rows), "measurements": rows}


@router.get("/collections/recent")
def collections_recent(limit: int = Query(1000, ge=1, le=5000)):
    rows = get_recent_collections(limit=limit)
    return {"count": len(rows), "collections": rows}


@router.get("/events/recent")
def events_recent(limit: int = Query(500, ge=1, le=5000)):
    rows = get_recent_events(limit=limit)
    return {"count": len(rows), "events": rows}


# ---------------- legacy alias ----------------

@router.get("/data/all_readings")
def legacy_all_readings(limit: int = Query(5000, ge=1, le=20000)):
    """Compatibility shim for the original dashboard fetch path.

    Returns measurement rows mapped to the old key names so the existing
    static/index.html keeps working until it migrates to the new
    measurements endpoint.
    """
    rows = get_recent_measurements(limit=limit)
    readings = [{
        "time": r["received_at"],
        "lat": r["lat"],
        "lon": r["lon"],
        "cond": r["cond"],
        "temp": r["temp"],
        "ph": r["ph"],
        "oxygen": r["oxygen"],
        "water_flag": bool(r["water_flag"]),
        "measurement_valid": bool(r["measurement_valid"]),
    } for r in rows]
    return {"count": len(readings), "readings": readings}


# ---------------- configuration ----------------

@router.get("/config")
def get_config(request: Request):
    """The active site config (tank↔channel/servo mapping, timeout,
    serial, poll interval)."""
    return getattr(request.app.state, "config", cfg_mod.load_config())


@router.get("/config/defaults")
def get_config_defaults():
    """Factory defaults — used by the settings page's Reset preview."""
    return cfg_mod.default_config()


@router.post("/config/validate")
def validate_config_endpoint(new_cfg: dict = Body(...)):
    """Validate a candidate config without saving — lets the settings
    page report exactly which setting is bad (e.g. before Re-provision)."""
    errs = cfg_mod.errors_for(new_cfg)
    return {"valid": not errs, "errors": errs}


@router.get("/serial/ports")
def serial_ports():
    """Enumerate serial ports the OS currently sees (cross-platform via
    pyserial). Powers the settings-page port picker. Linux shows
    /dev/ttyUSB*, Windows COMx, macOS /dev/cu.*."""
    try:
        from serial.tools import list_ports
    except Exception as e:
        return {"ports": [], "error": str(e)}
    ports = []
    for p in sorted(list_ports.comports(), key=lambda x: x.device):
        desc = (p.description or "").strip()
        # Skip phantom native serial ports (Linux lists 32× /dev/ttyS*
        # with no USB vid and an "n/a" description). USB adapters — the
        # CP210x / FTDI we actually use — carry a vid. The datalist still
        # lets the user type any port, so filtering here loses nothing.
        if p.vid is None and desc in ("", "n/a"):
            continue
        ports.append({
            "device": p.device,
            "description": "" if desc == "n/a" else desc,
        })
    return {"ports": ports}


async def _apply_config(request: Request, new_cfg: dict):
    """Persist, swap into app.state, reconnect serial if changed,
    re-provision. Returns the stored config + provisioning result."""
    try:
        stored = cfg_mod.save_config(new_cfg)
    except cfg_mod.ConfigError as e:
        raise HTTPException(status_code=422, detail={"errors": e.errors})

    old = getattr(request.app.state, "config", None)
    request.app.state.config = stored

    radio = getattr(request.app.state, "radio_service", None)
    provisioning = {"status": "pending", "detail": "no radio service"}
    if radio is not None:
        radio.current_config = stored
        # Reconnect only if the serial settings actually changed.
        if old is None or old.get("serial") != stored.get("serial"):
            radio.reconnect(stored["serial"]["port"], stored["serial"]["baud"])
        provisioning = await radio.provision(stored)

    return {"config": stored, "provisioning": provisioning}


@router.post("/config")
async def post_config(request: Request, new_cfg: dict = Body(...)):
    """Validate + save a full config, then apply it (reconnect serial if
    changed, re-provision the ESP). 422 with an error list on invalid
    input — nothing is persisted in that case."""
    return await _apply_config(request, new_cfg)


@router.post("/config/reset")
async def reset_config(request: Request):
    """Reset to factory defaults and apply."""
    return await _apply_config(request, cfg_mod.default_config())


@router.post("/config/provision")
async def reprovision(request: Request):
    """Re-push the current config to the ESP without changing it."""
    radio = _radio(request)
    cfg = getattr(request.app.state, "config", cfg_mod.load_config())
    return await radio.provision(cfg)


# ---------------- live state ----------------

@router.get("/state/now")
def state_now(request: Request):
    return _radio(request).state.snapshot()


@router.get("/stream/live")
async def stream_live(request: Request):
    radio = _radio(request)
    q = radio.subscribe()

    async def event_iterator():
        # Push the current snapshot first so a fresh client doesn't
        # have to wait for the next inbound frame to render correctly.
        await q.put({"type": "snapshot", "state": radio.state.snapshot()})
        try:
            while True:
                if await request.is_disconnected():
                    break
                try:
                    event = await asyncio.wait_for(q.get(), timeout=15.0)
                except asyncio.TimeoutError:
                    # Heartbeat comment-line keeps the connection alive
                    # through corporate proxies that drop idle SSE.
                    yield {"event": "ping", "data": "keepalive"}
                    continue
                yield {"data": json.dumps(event)}
        finally:
            radio.unsubscribe(q)

    return EventSourceResponse(event_iterator())


# ---------------- command send ----------------

@router.post("/cmd/{verb}")
async def send_cmd(verb: str, request: Request, args: Optional[str] = None):
    """Write a CMD frame to the drone and wait for its ACK/NACK.

    `verb` is the command name (case-sensitive, matches what the
    firmware expects — START_C1, E_STOP, STATUS, ...).

    Optional `args` query string is comma-joined onto the payload, so
    `POST /api/cmd/SERVO_MOVE?args=1,200` sends `CMD,SERVO_MOVE,1,200`.

    The frame is resent on timeout (radio frames get dropped), so the
    response reflects the actual outcome rather than just "written":
      {"result": "confirmed"}                 — drone ACK'd
      {"result": "rejected", "reason": "..."} — drone NACK'd
      {"result": "no_response", "attempts": N}— no ACK after retries
      {"result": "send_failed"}               — serial write failed

    The per-subsystem STATE events on /api/stream/live remain the
    source of truth for the UI; this return is advisory (see
    send_cmd_confirmed's at-least-once note).
    """
    radio = _radio(request)
    arg_list = [a for a in args.split(",")] if args else None
    outcome = await radio.send_cmd_confirmed(verb, arg_list)
    if outcome.get("result") == "send_failed":
        raise HTTPException(status_code=503, detail="serial write failed")
    return {"verb": verb, "args": arg_list or [], **outcome}
