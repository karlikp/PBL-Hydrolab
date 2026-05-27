# Ground station — H2O drone

FastAPI backend + browser dashboard for operating the water-quality
drone over a serial link (either the RFD868 radio for real missions or
a CP210x USB adapter for bench testing).

The dashboard is a **single-screen operator UI**: live system state on
the left (per-tank cards with state badge, level pill, step indicator,
Start / Stop / Reset buttons; Elmetron card with live cond/temp/pH/O₂
readings; live event log; E-STOP button); map + 2×2 chart grid on the
right for collected measurements.

## Run

Requirements: Python 3.10+ and the drone's serial adapter visible to
the OS. On Linux that's `/dev/ttyUSB0` (CP210x bench) or `/dev/ttyUSB1`
(FTDI radio); on Windows it's a `COMx` port — find it in **Device
Manager → Ports (COM & LPT)**.

> Running this from the **VS Code integrated terminal** on Windows? It
> defaults to **PowerShell**, so use the PowerShell block. Check the
> shell via the **˅ dropdown** next to the `+` in the terminal panel —
> if it says *Command Prompt* use the cmd block, if *Git Bash* / *WSL*
> use the Linux/macOS block.

**Linux / macOS:**

```bash
cd groundstation
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt

# Default: CP210x bench (deploy / mock firmware on UART0), /dev/ttyUSB0 @ 115200
.venv/bin/uvicorn main:app

# Real radio link
SERIAL_PORT=/dev/ttyUSB1 SERIAL_BAUD=57600 .venv/bin/uvicorn main:app
```

**Windows (PowerShell):**

```powershell
cd groundstation
python -m venv .venv
.venv\Scripts\pip install -r requirements.txt

# Default: CP210x bench. Set the port to your CP210x COM number first.
$env:SERIAL_PORT="COM5"; .venv\Scripts\uvicorn main:app

# Real radio link
$env:SERIAL_PORT="COM6"; $env:SERIAL_BAUD="57600"; .venv\Scripts\uvicorn main:app
```

**Windows (cmd.exe):**

```cmd
cd groundstation
python -m venv .venv
.venv\Scripts\pip install -r requirements.txt

set SERIAL_PORT=COM5
set SERIAL_BAUD=57600
.venv\Scripts\uvicorn main:app
```

> On PowerShell, `$env:VAR="value"` sets the variable for the rest of
> the session — chaining with `;` keeps it to one line. On `cmd.exe`,
> `set VAR=value` persists until the window closes, so run the `set`
> lines once then start uvicorn. Unlike Linux, you can't inline
> `VAR=value command` on a single Windows line.

Open <http://127.0.0.1:8000/static/> in a browser.

> **You usually don't need the env vars.** Serial port/baud (and the
> level-poll interval) are saved in the **Settings page** and reloaded
> on startup, so once you've set them there, plain `uvicorn main:app`
> works. The env vars above are just one-off overrides. See
> [Settings & provisioning](#settings--provisioning).

Useful sister URLs:

- <http://127.0.0.1:8000/health> — quick check that the radio thread is
  alive and which port/baud it opened.
- <http://127.0.0.1:8000/docs> — FastAPI auto-generated API explorer
  (you can fire commands from there too).
- <http://127.0.0.1:8000/api/state/now> — raw JSON snapshot of the live
  state model.
- <http://127.0.0.1:8000/api/stream/live> — opens the SSE stream as
  plain text in the browser; handy when debugging.

## Configuration

| Env var                 | Default        | Notes                                                            |
|-------------------------|----------------|------------------------------------------------------------------|
| `SERIAL_PORT`           | `/dev/ttyUSB0` | Path, `/dev/serial/by-id/...`, or `COMx` on Windows.            |
| `SERIAL_BAUD`           | `115200`       | `115200` for CP210x bench builds; `57600` for the RFD868 radio. |
| `LEVEL_POLL_INTERVAL_S` | `1.5`          | Seconds between per-tank `CMD,LEVEL` polls (round-robin, so each tank refreshes every ~3× this). Lower = snappier wet/dry pills but more radio chatter. Set to `0` to disable the poller (e.g. if the radio link is congested). |

Commands sent from the dashboard (or `POST /api/cmd/{verb}`) are
**confirmed and retried**: the backend waits for the matching
`ACK`/`NACK` and resends on timeout (default 0.7 s × 3), since radio
frames get dropped. The command-status pill in the top bar shows
*sending… / OK / rejected / no response*. The per-tank STATE badges
remain the source of truth for what the drone actually did.

The SQLite file `drone_data.db` is created in the working directory on
first run. Schema migrations are idempotent (`CREATE TABLE IF NOT
EXISTS`) so running against an existing DB just adds the new tables.

## Operator panel

```
┌──────────────────────────────────────────────────────────────────────────┐
│ Title  SYSTEM badge  link / battery / FW / GPS indicators       E-STOP   │
├─────────────────────────────────┬────────────────────────────────────────┤
│ Tank C1   [SAMPLING][WET]       │                                        │
│           step: PUMPING         │       Map (Leaflet + OSM)              │
│           [Start][Stop][Reset]  │       • Per-tank collection pins       │
│                                 │         (C1 blue, C2 green, C3 orange) │
│ Tank C2   [EMPTY][DRY]   ...    │       • Measurement-trace overlay      │
│ Tank C3   ...                   ├────────────────────────────────────────┤
│                                 │       Charts (2×2 grid)                │
│ Elmetron  [MEASURING]           │       pH    │  O₂                      │
│   step: IN_WATER                │       Temp  │  Cond                    │
│   cond/temp/pH/O₂  ●valid       │                                        │
│                                 │                                        │
│ ┌─ Live event log ─────────────┐│                                        │
│ │ 14:32:01  EVT,SYS,ACK,...    ││                                        │
│ │ 14:32:00  EVT,C1,STEP,...    ││                                        │
│ └──────────────────────────────┘│                                        │
└─────────────────────────────────┴────────────────────────────────────────┘
```

- **SYSTEM badge** — colour-coded mode (IDLE grey / SAMPLING blue /
  MEASURING green / E_STOP red / UNKNOWN yellow until the first
  STATE event arrives).
- **Per-tank cards** show the latest STATE (EMPTY / SAMPLING / FULL /
  FAULT), the active STEP, and a live **WET / DRY** pill driven by a
  background poll (`CMD,LEVEL,N` round-robins through all three tanks
  every ~2 s).
- **Button enable rules** mirror the firmware's `is_busy()` and the
  "concurrent sampling prohibited" project constraint. Start is
  disabled while anything else is sampling/measuring; Stop is enabled
  only when this subsystem is active; Reset is enabled only when this
  subsystem is in FAULT (or FULL for tanks).
- **E-STOP** is always enabled, always red, top-right. Latches the
  system in `E_STOP` mode until every faulted subsystem is reset.
- **Live event log** shows EVT frames in real time, colour-coded
  (ACK green, NACK / ERROR red). The `LEVEL` poll noise is filtered
  out so other events aren't drowned.
- **Map** shows one **collection pin per tank** at the GPS position
  the drone reported when the tank reached FULL. Per-tank colours.
  Measurement-trace dots are a separate layer that survives chart
  refreshes.

## Settings & provisioning

The **Settings page** (`/static/settings.html`, linked from the
dashboard's top bar) decouples the *logical* tanks (C1/C2/C3) from the
*physical* channels they're wired to, so a rig plugged together
differently is remapped here instead of rewiring or reflashing.

Per tank you set:

- **Enabled** — uncheck for modules that aren't mounted (e.g. only
  tanks 1 and 3 present). Disabled tanks are greyed out and locked in
  the operator panel and aren't level-polled.
- **Pump + sensor channel** (1/2/3) — the fixed PUMP+TOPCN pair on the
  board the tank is wired to. They move as a unit.
- **Winch servo id** — the SC-09 bus address of that tank's winch servo.

Plus globals: pumping timeout, the GCS serial **port** (a picker that
auto-detects attached adapters cross-platform, or type one) and
**baud**, and the level-poll interval.

The active config is saved to `config.json` (gitignored; defaults live
in `app/config.py`). **Save & provision** validates (no two enabled
tanks may share a channel or servo id), persists, reconnects the serial
link if the port/baud changed, and **provisions** the config to the
ESP. A status pill (top bar `CFG:` and the settings banner) shows the
result:

| Status        | Meaning                                                    |
|---------------|------------------------------------------------------------|
| `provisioned` | ESP echoed back a config matching ours (readback verified) |
| `mismatch`    | ESP echoed a different config / no readback                |
| `unsupported` | ESP firmware has no `CFG` support yet (see note below)     |
| `rejected`    | ESP NACK'd for another reason                              |
| `no_response` | no ACK after retries (link down?)                          |
| `pending`     | not attempted yet                                          |

Provisioning is pushed on GCS startup, on every save, and automatically
on every `EVT,SYS,BOOT` (so a mid-mission ESP reboot re-syncs — the ESP
holds config in RAM only, no firmware persistence).

> **Firmware side is not built yet.** The provisioning *protocol*
> (`CMD,CFG,to=<sec>,C1=<en>:<ch>:<servo>,…` set + `CMD,CFG_GET` →
> `EVT,SYS,CFG,…` readback) is implemented on the GCS, but current
> firmware doesn't understand `CFG` and will NACK it — so the status
> shows `unsupported` until the firmware phase lands. The GCS-side
> behaviour (enable/disable, channel-aware level polling, persisted
> serial config) works today regardless.

## What gets logged where

Three DB tables, plus an in-memory live state for the operator panel
and an SSE stream pushing events to the browser as they arrive.

| Table          | One row per                                          | Driven by                                       |
|----------------|------------------------------------------------------|-------------------------------------------------|
| `measurements` | TLM frame **while system mode is MEASURING**         | Live SSE → write-through                        |
| `collections`  | `EVT,Cx,COLLECTED` (tank physically filled + geo-tagged) | Live SSE → write-through                    |
| `events`       | Every EVT frame (audit log)                          | Live SSE → write-through                        |
| `telemetry`    | (legacy, no longer written to)                       | Preserved so old data isn't destroyed           |

Heartbeat TLMs received outside MEASURING are intentionally **not**
persisted — they only serve "drone is alive" and that's tracked in
the live link indicator. Keeps the measurements table lean.

## API

| Method | Path                              | Purpose                                                                |
|--------|-----------------------------------|------------------------------------------------------------------------|
| GET    | `/health`                         | Liveness + port/baud                                                   |
| GET    | `/api/state/now`                  | Snapshot of the live state (system mode, tanks, elmetron, etc.)       |
| GET    | `/api/stream/live`                | **Server-Sent Events** stream: every TLM, every EVT, plus initial snapshot on connect |
| GET    | `/api/measurements/recent`        | Historical measurements (`?limit=` / `?valid_only=true` / `?start=` / `?end=`) |
| GET    | `/api/collections/recent`         | Historical collections                                                 |
| GET    | `/api/events/recent`              | Audit log                                                              |
| POST   | `/api/cmd/{verb}?args=a,b,c`      | Build an Adler-32 framed CMD and write it to the serial port. Waits for ACK/NACK, retries on timeout; returns the outcome. |
| GET    | `/api/config`                     | Active site config (tank↔channel/servo, timeout, serial, poll).       |
| GET    | `/api/config/defaults`            | Factory defaults.                                                     |
| POST   | `/api/config`                     | Validate + save + apply (reconnect serial if changed, re-provision). 422 with an error list on invalid input. |
| POST   | `/api/config/reset`               | Reset to factory defaults and apply.                                  |
| POST   | `/api/config/provision`           | Re-push the current config to the ESP without changing it.            |
| GET    | `/api/serial/ports`               | Enumerate serial ports the OS sees (cross-platform). Powers the settings port picker. |
| GET    | `/api/data/all_readings`          | Legacy alias; returns measurement rows under the old key names.       |

Examples:

```bash
# Send a PING
curl -X POST http://127.0.0.1:8000/api/cmd/PING

# Start tank 1
curl -X POST http://127.0.0.1:8000/api/cmd/START_C1

# Probe the level sensor for tank 2
curl -X POST 'http://127.0.0.1:8000/api/cmd/LEVEL?args=2'

# Re-id servo from 3 to 2 (see the embedded README for full procedure)
curl -X POST 'http://127.0.0.1:8000/api/cmd/SERVO_SET_ID?args=3,2'

# Manual servo move
curl -X POST 'http://127.0.0.1:8000/api/cmd/SERVO_MOVE?args=2,500'

# Subscribe to the live stream (Ctrl+C to exit)
curl -N http://127.0.0.1:8000/api/stream/live
```

> **Windows / PowerShell gotcha:** in PowerShell, `curl` is an alias
> for `Invoke-WebRequest`, which does **not** accept `-X`/`-N` and will
> error. Use `curl.exe` explicitly (the `.exe` bypasses the alias —
> works on Windows 10+):
>
> ```powershell
> curl.exe -X POST http://127.0.0.1:8000/api/cmd/START_C1
> curl.exe -X POST "http://127.0.0.1:8000/api/cmd/LEVEL?args=2"
> ```
>
> Or stay native with `Invoke-RestMethod`:
>
> ```powershell
> Invoke-RestMethod -Method Post http://127.0.0.1:8000/api/cmd/START_C1
> ```
>
> Easiest of all: the dashboard buttons and <http://127.0.0.1:8000/docs>
> (the "Try it out" panel) send these for you — no shell quoting at all.

The full CMD verb list lives in [`docs/protocol.md`](../docs/protocol.md).

## Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│ FastAPI process                                                      │
│                                                                      │
│  RadioService background thread (radio.py)                           │
│  ┌────────────────────────────────────────────────────────────────┐  │
│  │ serial.Serial(port, baud)                                      │  │
│  │  readline() → Adler-32 verify → parse TLM/EVT → dispatch       │  │
│  │    → update LiveState (under lock)                             │  │
│  │    → insert_measurement / insert_collection / insert_event     │  │
│  │    → broadcast({type, ...}) → call_soon_threadsafe per subscriber │  │
│  └────────────────────────────────────────────────────────────────┘  │
│                       │                ▲                             │
│                       │   write_lock   │                             │
│                       ▼                │                             │
│                  ┌────────────────┐    │ asyncio.Queue per SSE       │
│                  │ send_cmd()     │    │ subscriber                  │
│                  └────────────────┘    │                             │
│                       ▲                │                             │
│                       │                │                             │
│              POST /api/cmd/{verb}      │ /api/stream/live (SSE)      │
│                       ▲                │ /api/state/now              │
│                       │                │ /api/{measurements,...}     │
│                       │                │                             │
│  Background tasks:                     │                             │
│  • status_poller (CMD,STATUS every 30 s — battery refresh)           │
│  • level_poller  (CMD,LEVEL,1/2/3 round-robin every 0.7 s)           │
└──────────────────────────────────────────────────────────────────────┘
                                        ▲
                                        │ SSE
                                        │
                          ┌─────────────────────────────┐
                          │ Browser (static/index.html) │
                          │ vanilla JS + Leaflet +      │
                          │ Chart.js. Operator panel    │
                          │ + historical dashboard.     │
                          └─────────────────────────────┘
```

Threading: the radio thread mutates `LiveState` and posts events; SSE
subscribers consume them in the FastAPI event loop. Cross-thread
signalling uses `loop.call_soon_threadsafe(queue.put_nowait, event)`
so the asyncio queues stay sound. Serial writes are gated by a
`threading.Lock` so concurrent CMDs (multiple browsers, automation,
the level/status pollers) can't interleave bytes on the wire.

## Repository layout

```
groundstation/
├── main.py                      FastAPI app, lifespan, pollers, env config
├── requirements.txt
├── drone_data.db                SQLite (gitignored, created on first run)
├── app/
│   ├── api/endpoints.py         all /api/* routes
│   ├── database.py              schema + insert/query helpers
│   ├── schemas/frame_reading.py legacy Pydantic model
│   └── services/radio.py        serial I/O, frame parser, LiveState, SSE pub/sub
└── static/
    └── index.html               operator panel + dashboard (vanilla JS)
```

## Notes for contributors

- **Frame parsing** dispatches by `TLM,` / `EVT,` prefix at the start
  of the line. `EVT` details may contain commas (e.g.
  `EVT,Cx,COLLECTED,lat=...,lon=...,utc=...`), so the parser splits on
  the first three commas only and treats the rest as opaque details
  for the kind-specific handler to parse. See `parse_kv` for the
  `k1=v1,k2=v2` helper.
- **Adler-32** matches the firmware's `zlib.adler32`. Reference
  vectors in [`docs/protocol.md`](../docs/protocol.md) §3.
- **Lat/Lon** arrive scaled by 10⁷ on the wire and are divided to
  decimal degrees in `_handle_tlm` and `_handle_collected`. Don't
  scale again downstream.
- **SSE keepalive** — the stream emits an `event: ping` line every
  15 s of silence so corporate proxies don't drop the connection.
- **CORS** is wide open (`allow_origins=["*"]`) for local development;
  tighten before deploying.
