"""Ground station entry point.

Spins up FastAPI + SQLite + the RadioService background thread.
Configuration lives in env vars so flipping between bench (CP210x at
115200 on /dev/ttyUSB0) and radio (SiK at 57600 on /dev/ttyUSB1) is a
one-line change:

    SERIAL_PORT=/dev/ttyUSB1 SERIAL_BAUD=57600 uvicorn main:app --reload

The radio service is stashed on `app.state.radio_service` so the API
handlers can reach it without a circular import.
"""

import asyncio
import os
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from app.api.endpoints import router as api_router
from app.database import init_db
from app.services.radio import RadioService


SERIAL_PORT = os.environ.get("SERIAL_PORT", "/dev/ttyUSB0")
SERIAL_BAUD = int(os.environ.get("SERIAL_BAUD", "115200"))

# How often to poll STATUS so the operator panel can keep the battery
# readout fresh without a firmware change. Cheap (~one CMD round-trip),
# but no need to hammer it.
STATUS_POLL_INTERVAL_S = 30.0

# How often to round-robin through CMD,LEVEL,1/2/3 so the UI shows a
# live wet/dry indicator per tank. One tank per tick: with 0.7 s
# between firings each tank refreshes every ~2.1 s and the radio sees
# ~1.4 frames/s of LEVEL CMD/EVT chatter on top of TLM.
LEVEL_POLL_INTERVAL_S = 0.7


@asynccontextmanager
async def lifespan(app: FastAPI):
    init_db()

    radio = RadioService(port=SERIAL_PORT, baud=SERIAL_BAUD)
    loop = asyncio.get_running_loop()
    radio.start(main_loop=loop)
    app.state.radio_service = radio

    # Background task: periodic STATUS refresh for battery / sanity.
    async def status_poller():
        # Brief initial delay so the link has time to settle and we
        # don't fire before the listener has even opened the port.
        await asyncio.sleep(5.0)
        while True:
            try:
                radio.send_cmd("STATUS")
            except Exception as e:
                print(f"[Poller] STATUS send failed: {e}")
            await asyncio.sleep(STATUS_POLL_INTERVAL_S)

    poller_task = asyncio.create_task(status_poller())

    async def level_poller():
        # Round-robin LEVEL polls so each tank's wet/dry indicator
        # stays live in the operator panel. Brief warm-up delay so
        # the radio link has settled before we start chirping.
        await asyncio.sleep(3.0)
        tank_id = 1
        while True:
            try:
                radio.send_cmd("LEVEL", [str(tank_id)])
            except Exception as e:
                print(f"[Poller] LEVEL {tank_id} send failed: {e}")
            tank_id = tank_id % 3 + 1
            await asyncio.sleep(LEVEL_POLL_INTERVAL_S)

    level_task = asyncio.create_task(level_poller())

    try:
        yield
    finally:
        poller_task.cancel()
        level_task.cancel()
        radio.stop()


app = FastAPI(title="Water Quality Drone — Ground Station", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.mount("/static", StaticFiles(directory="static", html=True), name="static")
app.include_router(api_router)


@app.get("/health")
def health():
    radio = getattr(app.state, "radio_service", None)
    return {
        "status": "ok",
        "radio_running": bool(radio and radio.running),
        "port": SERIAL_PORT,
        "baud": SERIAL_BAUD,
    }
