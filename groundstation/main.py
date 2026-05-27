"""Ground station entry point.

Spins up FastAPI + SQLite + the RadioService background thread.

Serial port/baud and the level-poll interval come from the persisted
config (settings page), so no env-var flags are needed day to day —
set them once in the UI. Env vars still override the config for
one-off runs:

    SERIAL_PORT=/dev/ttyUSB1 SERIAL_BAUD=57600 uvicorn main:app

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
from app import config as cfg_mod

# Periodic STATUS refresh (battery / sanity). Cheap; no need to hammer.
STATUS_POLL_INTERVAL_S = 30.0


@asynccontextmanager
async def lifespan(app: FastAPI):
    init_db()
    cfg = cfg_mod.load_config()
    app.state.config = cfg

    # Serial from config, with env-var override for one-off runs.
    port = os.environ.get("SERIAL_PORT", cfg["serial"]["port"])
    baud = int(os.environ.get("SERIAL_BAUD", cfg["serial"]["baud"]))
    level_interval = float(os.environ.get(
        "LEVEL_POLL_INTERVAL_S", cfg["level_poll_interval_s"]))

    radio = RadioService(port=port, baud=baud)
    radio.current_config = cfg  # so BOOT-triggered re-provision has it
    loop = asyncio.get_running_loop()
    radio.start(main_loop=loop)
    app.state.radio_service = radio

    async def status_poller():
        await asyncio.sleep(5.0)
        while True:
            try:
                radio.send_cmd("STATUS")
            except Exception as e:
                print(f"[Poller] STATUS send failed: {e}")
            await asyncio.sleep(STATUS_POLL_INTERVAL_S)

    async def level_poller():
        # Round-robin LEVEL polls over the ENABLED tanks only, each at
        # its configured physical channel, so the wet/dry pills track
        # the actual wiring. Re-reads app.state.config each pass so a
        # settings change takes effect without a restart.
        await asyncio.sleep(3.0)
        idx = 0
        while True:
            current = app.state.config
            channels = [current["tanks"][t]["channel"]
                        for t in cfg_mod.enabled_tanks(current)]
            interval = float(current.get("level_poll_interval_s", 1.5))
            if channels and interval > 0:
                ch = channels[idx % len(channels)]
                idx += 1
                try:
                    radio.send_cmd("LEVEL", [str(ch)])
                except Exception as e:
                    print(f"[Poller] LEVEL {ch} send failed: {e}")
                await asyncio.sleep(interval)
            else:
                # Polling disabled or no enabled tanks — idle-check slowly.
                await asyncio.sleep(2.0)

    async def initial_provision():
        # Give the listener a moment to open the port, then push the
        # config. If the ESP later reboots, the BOOT handler re-provisions.
        await asyncio.sleep(4.0)
        try:
            await radio.provision(cfg)
        except Exception as e:
            print(f"[Provision] initial provision failed: {e}")

    poller_task = asyncio.create_task(status_poller())
    level_task = asyncio.create_task(level_poller()) \
        if level_interval > 0 else None
    provision_task = asyncio.create_task(initial_provision())

    try:
        yield
    finally:
        poller_task.cancel()
        if level_task:
            level_task.cancel()
        provision_task.cancel()
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
        "port": getattr(radio, "port", None),
        "baud": getattr(radio, "baud", None),
    }
