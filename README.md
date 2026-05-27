# H2O Water-Quality Drone

A small drone that goes to a body of water, takes physical water samples
into three on-board tanks, and measures water quality (conductivity,
temperature, pH, dissolved oxygen) in-situ with an Elmetron probe. An
operator on the ground drives the mission from a laptop over a radio
link.

This repository contains the **drone firmware** (an ESP32-S3 board on
the drone), the **ground station** (FastAPI backend + browser
dashboard), and the **wire protocol** they speak to each other.

```
┌───────────────────────────────────────┐                  ┌──────────────────────────────┐
│  Drone (ESP32-S3 + RFD868 radio)      │  ◀── radio ──▶   │  Ground station (laptop)     │
│                                       │  57600 8N1       │                              │
│  • Three sampling tanks (C1/C2/C3)    │                  │  FastAPI server              │
│    each with a winch, pump, level     │                  │   ├─ serial listener thread  │
│    sensor                             │                  │   ├─ SQLite (measurements,   │
│  • Elmetron probe on its own winch    │                  │   │   collections, events)  │
│  • NEO-M8U GPS                        │                  │   ├─ /api/cmd/<verb>         │
│  • 3S LiPo + battery monitor          │                  │   └─ SSE stream → browser    │
│                                       │                  │                              │
│  Emits:  TLM @ 2 Hz, EVT on events    │                  │  Browser dashboard           │
│  Accepts: CMD (Start, Stop, Reset...) │                  │   • Live operator panel      │
│                                       │                  │   • Map + 4 charts           │
└───────────────────────────────────────┘                  └──────────────────────────────┘
```

## Two-minute quick start

The fastest path is the **mock build** — real firmware on a bare ESP32-S3
with no peripherals connected, plus the ground station on your laptop.
Lets you exercise the whole operator UI before any of the physical drone
hardware is in front of you.

> **Windows users:** commands below are given for both Linux/macOS and
> Windows. If you drive PlatformIO through the **VS Code extension**
> rather than the CLI, see
> [`embedded/README.md` → "VS Code extension"](embedded/README.md#vs-code-extension-no-cli)
> — you click toolbar buttons instead of typing `pio`.

**1.** Flash the firmware to any spare ESP32-S3 (CP210x USB adapter
plugged into UART0):

```bash
# Linux / macOS
cd embedded
~/.platformio/penv/bin/pio run -e esp32s3wroom1-mock -t upload
```

```powershell
# Windows (PowerShell) — pio is on PATH inside the PlatformIO Core CLI terminal
cd embedded
pio run -e esp32s3wroom1-mock -t upload
```

**2.** Run the ground station against the same USB port:

```bash
# Linux / macOS
cd ../groundstation
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/uvicorn main:app
```

```powershell
# Windows (PowerShell)
cd ..\groundstation
python -m venv .venv
.venv\Scripts\pip install -r requirements.txt
.venv\Scripts\uvicorn main:app
```

**3.** Open <http://127.0.0.1:8000/static/> in a browser. Within a few
seconds the operator panel should populate (system mode IDLE, tanks
EMPTY, Elmetron DOCKED). Click **Start C1** and watch the FSM run
through DESCENDING / IN_WATER / PUMPING / ASCENDING / HOME in ~7 s.
Click **Start Elmetron** and watch the measurement-valid dot flip
green after the settle window.

For real-hardware deployment, see [`embedded/README.md`](embedded/README.md)
for build-env selection and flash instructions, and
[`groundstation/README.md`](groundstation/README.md) for radio-link
configuration.

## Repository layout

```
README.md                  this file
docs/
  protocol.md              wire-format contract (CMD / EVT / TLM, Adler-32 framing)
  hardware/                board pinout PDF, schematic PDF, hardware notes
embedded/                  ESP32-S3 firmware (PlatformIO)
groundstation/             FastAPI + browser dashboard
```

## Where to look for what

| Question                                                    | Read this                                          |
|-------------------------------------------------------------|----------------------------------------------------|
| Wire format / commands / events / TLM fields                | [`docs/protocol.md`](docs/protocol.md)             |
| Which firmware env do I flash for X situation?              | [`embedded/README.md`](embedded/README.md) §"Build envs" |
| How does the operator UI behave?                            | [`groundstation/README.md`](groundstation/README.md) §"Operator panel" |
| Pinout / which GPIO does what                               | [`docs/hardware/board_pinout.pdf`](docs/hardware/board_pinout.pdf) |
| What's still mocked vs. real on the firmware side?          | [`embedded/README.md`](embedded/README.md) §"Mock vs real" |
| What changed recently?                                      | `git log --oneline` and the protocol doc's version section |

## Glossary

- **`Cx`** — a sampling tank (C1, C2, C3). Each has its own winch
  (SC-09 servo), pump, and FS-IR12 optical level sensor.
- **Elmetron** — the in-situ water-quality probe. Lives on its own
  winch (brushed DC + H-bridge — driver pending).
- **TLM** — 2 Hz telemetry heartbeat from the drone (GPS, sensor
  readings, water flag, measurement-valid flag).
- **EVT** — event frames from the drone (state changes, ACK/NACK to
  commands, errors, boot).
- **CMD** — command frames from the GCS to the drone.
- **STATE** — the high-level disposition of a subsystem
  (EMPTY/SAMPLING/FULL/FAULT for tanks; DOCKED/MEASURING/FAULT for
  Elmetron; IDLE/SAMPLING/MEASURING/E_STOP for the system).
- **STEP** — the current sub-phase during an active operation
  (DESCENDING / IN_WATER / PUMPING / ASCENDING / HOME, plus HOMING
  for Elmetron).
