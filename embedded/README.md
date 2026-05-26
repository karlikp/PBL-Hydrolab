# Embedded firmware — H2O drone

ESP32-S3 firmware for the water-quality drone. The command/event
pipeline, state machines, wire protocol, GPS, level sensors, pumps and
the SC-09 servo bus are all live on hardware. The Elmetron probe and
its H-bridge winch are still stubs (driver pending hardware
integration — see [Mock vs real](#mock-vs-real)).

## Build envs at a glance

Pick the env that matches what you have in front of you. All envs are
declared in [`platformio.ini`](platformio.ini).

| Env                       | Protocol link        | When to use                                                                                        |
|---------------------------|----------------------|----------------------------------------------------------------------------------------------------|
| `esp32s3wroom1`           | RFD868 radio @ 57600 | **Production deploy.** Full hardware: servos, pumps, level sensors, GPS, radio.                    |
| `esp32s3wroom1-uartlink`  | CP210x USB @ 115200  | **Bench testing with real peripherals.** No radio in the loop. Pumps / sensors / GPS still live.   |
| `esp32s3wroom1-mock`      | CP210x USB @ 115200  | **GCS development without any peripheral hardware.** Bare ESP32-S3, GPS+level sensor mocked.       |
| `esp32s3wroom1-servotest` | none (JTAG only)     | Standalone SC-09 servo bring-up. UART0 dedicated to the servo bus.                                 |
| `esp32s3wroom1-radiotest` | CP210x USB           | RFD868 TX/RX-orientation diagnostic. Rotates pin mapping every 5 s.                                |
| `esp32doit-devkit-v1`     | CP210x USB           | Legacy classic ESP32 dev board. Same protocol/FSM, no S3-specific peripherals.                     |

## Flash / monitor

If `pio` on your `$PATH` resolves to an old 4.x install, use the
explicit modern path: `~/.platformio/penv/bin/pio`.

```bash
# Build only (sanity check)
pio run -e esp32s3wroom1

# Build + upload
pio run -e esp32s3wroom1 -t upload

# Serial monitor (CP210x adapter, 115200 baud)
pio device monitor -p /dev/ttyUSB0 -b 115200

# Radio listen (FTDI on the V5 radio, 57600 baud)
pio device monitor -p /dev/ttyUSB1 -b 57600
```

The deploy env (`esp32s3wroom1`) pins the upload port to the CP210x
adapter via a stable `by-id` path so PIO can't accidentally try to
flash through the FTDI radio adapter. Other envs that don't depend on
this can use plain auto-detect.

### If a flash hangs partway

esptool's flasher stub has an internal watchdog that occasionally
times out mid-write on this board/cable combo ("chip stopped
responding"). All four S3 envs that you'd normally flash already pin
`upload_flags = --no-stub` to bypass the stub. Flashing is slightly
slower but rock-solid.

If you still get a hang, manual bootloader entry: hold **BOOT**,
press **EN**, release **EN**, release **BOOT**, then re-run the
upload command.

## Quick demo — flash the mock, drive from the host

You don't need radio, pumps, sensors, GPS, or batteries for this.
Just a spare ESP32-S3 board on your laptop USB.

```bash
# 1. Flash the mock build
pio run -e esp32s3wroom1-mock -t upload

# 2. Watch the protocol stream live
pio device monitor -p /dev/ttyUSB0 -b 115200
# You should see EVT,SYS,BOOT,... once, then TLM frames every 500 ms.

# 3. From another terminal, fire commands
python3 tools/send_cmd.py -p /dev/ttyUSB0 -b 115200 STATUS
python3 tools/send_cmd.py -p /dev/ttyUSB0 -b 115200 --watch 12 START_C1
python3 tools/send_cmd.py -p /dev/ttyUSB0 -b 115200 --watch 25 START_ELMETRON
```

A C1 sample cycle runs in ~7.5 s on mock timings, an Elmetron
measurement cycle in ~20.5 s including HOMING. Both end with the
appropriate STATE return (FULL for tanks; DOCKED for Elmetron) plus a
geo-tagged `EVT,Cx,COLLECTED` if a tank reached FULL.

## Tests

Unit tests run **on-target** — they compile, flash, and execute on a
real ESP32. There's no host-side runner; the tests use the same
toolchain and headers as the firmware.

```bash
pio test -e esp32doit-devkit-v1                       # run all
pio test -e esp32doit-devkit-v1 -f test_frame_codec   # one suite
pio test -e esp32doit-devkit-v1 -f test_sampler
pio test -e esp32doit-devkit-v1 -f test_elmetron
```

Coverage today:

| Suite             | Cases | What it exercises                                                                  |
|-------------------|-------|------------------------------------------------------------------------------------|
| `test_frame_codec`| 21    | Adler-32 reference vectors, frame build/parse roundtrip, every rejection path.     |
| `test_sampler`    | 17    | Per-tank FSM transitions, abort/reset semantics, change-flag consumption.          |
| `test_elmetron`   | 21    | Elmetron FSM transitions including HOMING, synthetic reading evolution, abort/reset. |

## Architecture

```
                 ┌────────────────────────────────────────────────┐
                 │              main.cpp (composition root)        │
                 │  instantiates Clock, RadioLink, MissionControl, │
                 │  Telemetry, GpsLink, ServoBus, PumpControl,     │
                 │  LevelSensor, BatteryMonitor, Elmetron.         │
                 └─────────────────────┬───────────────────────────┘
                                       ▼
        ┌────────────────────────────────────────────────────────────┐
        │                  MissionControl                            │
        │  • Owns 3× Sampler FSMs (tanks) + 1× Elmetron FSM          │
        │  • Dispatches CMD frames (handle_command)                  │
        │  • Emits ACK/NACK/STATE/STEP/COLLECTED events              │
        │  • Tracks system mode (IDLE / SAMPLING / MEASURING / E_STOP)│
        │  • Drives servos, pumps, telemetry pushes                  │
        └────────────────────────────────────────────────────────────┘
              │                       │                  │
              ▼                       ▼                  ▼
    ┌────────────────────┐   ┌─────────────────┐   ┌───────────────────┐
    │ Sampler ×3         │   │ Elmetron        │   │ Telemetry         │
    │ EMPTY/SAMPLING/    │   │ DOCKED/MEAS/    │   │ TLM @ 2 Hz with   │
    │ FULL/FAULT         │   │ FAULT           │   │ measurement_valid │
    └────────────────────┘   └─────────────────┘   └───────────────────┘
              │                       │                  ▲
              ▼                       ▼                  │
    ┌────────────────────┐   ┌─────────────────┐   ┌─────┴──────┐
    │ ServoBus, Pump-    │   │ GpsLink (used   │   │ GpsLink    │
    │ Control, Level-    │   │ for geo-tagging │   │ pushes lat/│
    │ Sensor             │   │ collections)    │   │ lon/UTC    │
    └────────────────────┘   └─────────────────┘   └────────────┘
```

The wire protocol — every command, every event, every TLM field —
lives in [`docs/protocol.md`](../docs/protocol.md).

## Module reference

```
embedded/
├── platformio.ini      build envs (see table above)
├── lib/                pure modules — testable in isolation
│   ├── Clock/          time-source abstraction (Real + Test)
│   ├── FrameCodec/     wire-frame build/parse + Adler-32
│   ├── Sampler/        per-tank state machine (one instance per Cx)
│   └── Elmetron/       measurement-side state machine (one instance)
├── include/            project headers
│   ├── MissionControl.h    top-level state + CMD dispatcher
│   ├── Telemetry.h         2 Hz TLM broadcaster (10 fields incl. measurement_valid)
│   ├── RadioLink.h         framed bidirectional UART transport
│   ├── ServoBus.h          SC-09 half-duplex bus (1 Mbps on UART0)
│   ├── PumpControl.h       three on/off pump GPIOs
│   ├── LevelSensor.h       FS-IR12 optical "tank full" sensors
│   ├── BatteryMonitor.h    3S LiPo voltage + soft-power latch shutdown
│   ├── GpsLink.h           NEO-M8U over I2C
│   └── RadioTransport.h    RFD868 link bring-up
├── src/                project sources (mostly thin glue around the headers)
└── tools/
    ├── send_cmd.py     host helper: send a CMD frame, wait for ACK
    ├── watch_adc.py
    └── watch_battery.py
```

## Mock vs real

| Module           | Today                                                     | Real-hardware swap-in plan                        |
|------------------|-----------------------------------------------------------|---------------------------------------------------|
| FrameCodec       | Real                                                      | —                                                 |
| RadioLink        | Real on Serial1 (RFD868) or Serial (CP210x bench)         | —                                                 |
| MissionControl   | Real                                                      | —                                                 |
| Sampler          | Real; PUMPING exits on level sensor (sensor-driven) or hard 90 s timeout. DESCENDING/ASCENDING/HOME still timer-based until LIM1/2/3 switches are wired. | LIM1/2/3 wired into per-step exit conditions.    |
| ServoBus         | Real for tank winches (SC-09 on UART0)                    | —                                                 |
| PumpControl      | Real                                                      | —                                                 |
| LevelSensor      | Real (post-rework: external pull-up to 3.3 V)             | —                                                 |
| BatteryMonitor   | Real but **temporarily disabled in deploy env** (`-DDISABLE_BATTERY_MONITOR` in `platformio.ini`) because R18/R19 divider saturates the ADC above ~9 V. | Once HW team confirms divider rework, drop the flag and restore the 11.0 V / 10.5 V thresholds in `BatteryMonitor.h`. |
| GpsLink          | Real (u-blox NEO-M8U over I2C). Mock build synthesizes a fixed fix near Warsaw. | —                                       |
| Telemetry        | Real (probe fields zero outside MEASURING)                | —                                                 |
| Elmetron probe   | Synthetic ramp readings (no probe driver yet)             | Add real UART driver on ELE_TX (IO18) / ELE_RX (IO17). Cond reading can drive descent-trigger logic (see `project_elmetron_descent_timeout`). |
| Elmetron winch   | FSM transitions but motor isn't driven                    | WinchH driver: H_EN_L/H_EN_R direction, H_PWM speed, H_LIMIT for HOMING exit. TODOs documented in `drive_elmetron_servo_for_step`. |

## Known hardware caveats

- **SC-09 half-duplex echo.** The library reads its own TX echo on RX,
  so `Ack()` returns are unreliable for write/read operations.
  `ServoBus::move()` is fire-and-forget; `ServoBus::read_position()`
  may return garbage. Verify servo writes by observing motion, not by
  return code. See the comment block above `set_id` in `ServoBus.cpp`.
- **Battery divider saturates above ~9 V.** Effective ratio is ~0.35,
  not the schematic's 0.248. Until HW team reworks R18/R19, the deploy
  env disables BatteryMonitor entirely.
- **GPIO 4 is the soft-power latch.** Firmware must drive it HIGH
  within the first few ms after boot or the board powers itself off.
  `early_board_init()` in `main.cpp` does this — keep it at the top of
  every S3 build's `setup()`.
- **UART0 pin swap for the SC-09 bus.** TX = GPIO 44, RX = GPIO 43
  (opposite of the chip default). See
  [`docs/hardware/servo_uart_pinout.md`](../docs/hardware/servo_uart_pinout.md).
- **Bench env saturates the battery ADC on USB power.** `-uartlink`
  and `-mock` builds skip BatteryMonitor entirely — USB on the ADC
  pin would trip CRITICAL and drop the power latch.
