# Embedded firmware — H2O drone

ESP32 firmware for the H2O water-sampling drone. Currently in **skeleton
phase**: the command/event pipeline, state machines, and wire protocol
are real and exercised end-to-end on hardware. Pumps, motors and sensors
are mocked with timer-driven transitions; real hardware drivers slot in
when the hardware team confirms the schematic interpretation (see
`docs/hardware/pytania_do_hw.md`).

## Quick orientation

```
embedded/
├── platformio.ini      build env(s)
├── lib/                pure modules (testable in isolation)
│   ├── FrameCodec/     wire-frame build/parse + Adler-32
│   ├── Sampler/        per-tank state machine
│   └── Clock/          time-source abstraction (Real + Test)
├── include/            project headers (Arduino-touching)
│   ├── MissionControl.h    top-level state + CMD dispatcher
│   ├── RadioLink.h         bidirectional UART transport
│   └── ...                 (legacy headers, unused for now)
├── src/                project sources
│   ├── main.cpp        thin glue: instantiates the system, ticks it
│   ├── MissionControl.cpp
│   ├── RadioLink.cpp
│   └── ...             (legacy sources, unused for now)
├── test/
│   ├── test_frame_codec/   Unity tests for FrameCodec
│   └── test_sampler/       Unity tests for Sampler FSM
└── tools/
    └── send_cmd.py     host helper: send a CMD frame, print replies
```

## Architecture in one paragraph

`main.cpp` instantiates a `Clock`, a `RadioLink` (over USB Serial in
skeleton mode), and a `MissionControl`. The main loop pumps the radio
(reads bytes, validates frames via `FrameCodec`, dispatches `CMD` frames
to `MissionControl::handle_command`) and ticks `MissionControl` (which
drives three `Sampler` FSMs and emits `EVT` frames for state/step
changes). The wire protocol — every frame's structure, every command,
every event — lives in `docs/protocol.md`. The mock timing for the
sampling FSM lives in `Sampler.h::mock_timing`.

## Build / flash / monitor

```bash
# from embedded/
pio run                             # build
pio run -t upload                   # build + flash
pio device monitor                  # open serial monitor (115200 baud)
```

**Note for this machine:** there are two PlatformIO installs and `pio`
on `$PATH` may resolve to a broken legacy 4.3.4. Use the modern one
explicitly if needed:

```bash
~/.platformio/penv/bin/pio run
```

## Tests

Tests run **on-target** — they compile, flash, and execute on the real
ESP32. Each cycle is ~15–30 seconds. There is no host-side test runner
(deliberately: keeps infrastructure minimal, and the tests stay honest
about running on the same toolchain as the firmware).

```bash
pio test -e esp32doit-devkit-v1                       # run all tests
pio test -e esp32doit-devkit-v1 -f test_frame_codec   # one test target
```

Currently covered:

- `test_frame_codec` — 21 cases. Adler-32 reference vectors, build/parse
  roundtrip, every rejection path (bad checksum, missing star, oversized,
  short input, non-hex). Source: `test/test_frame_codec/test_main.cpp`.
- `test_sampler` — 17 cases. Every Sampler state transition,
  abort/reset semantics, change-flag consumption, string names.

When real hardware drivers land, add tests for any pure logic that can
be exercised without GPIO (e.g. command parsing edge cases in
MissionControl). Driver code itself is validated by the demo run, not
by unit tests.

## Demo (no GCS needed)

The skeleton is flashed and the board is connected. From this directory:

```bash
# Ask the drone its current state
python3 tools/send_cmd.py STATUS

# Sample tank C1 (~9 second cycle)
python3 tools/send_cmd.py --watch 12 START_C1

# Reset and try again
python3 tools/send_cmd.py RESET_C1
python3 tools/send_cmd.py --watch 12 START_C1

# Trigger and recover from E-STOP
python3 tools/send_cmd.py --watch 2 START_C1   # start sampling
python3 tools/send_cmd.py E_STOP               # abort mid-sample
python3 tools/send_cmd.py RESET_C1             # clear the fault
```

Every command and reply is a properly-checksummed protocol frame. See
`docs/protocol.md` for the verb / kind catalogue.

## What's mocked vs real

| Module           | Skeleton state                                | Real-hardware swap-in plan        |
|------------------|-----------------------------------------------|-----------------------------------|
| FrameCodec       | Real — final implementation                   | No change                         |
| RadioLink        | Real on USB Serial                            | Swap to `Serial1` (or HW UART) wired to RFD module |
| MissionControl   | Real — full state machine + CMD dispatch      | No change                         |
| Sampler          | Real FSM, **timer-driven step transitions**   | Step transitions become sensor-driven (`SUBx`, `TOPCNx`, `LIMx`); pump/winch actuation calls land in `enter_step()` |
| Clock            | Real (`millis()`)                             | No change                         |
| Telemetry        | Real broadcast path, **zeroed sample data**   | `Telemetry::update(Sample)` called by sensor + GPS modules with real readings; broadcast path unchanged |
| Elmetron, GPS    | Not wired yet                                 | New modules emit STATE/STEP/ERROR events via the same protocol; data flows into Telemetry::update() |

Reference implementations from the previous semester live in `legacy/`
(separate tree, not compiled). See `legacy/README.md` for what's in
there and how to fold each module back in.

## Skeleton scope and known limitations

- **Single UART used for both directions.** In skeleton mode the USB
  Serial port handles both incoming CMDs and outgoing TLM/EVT. When the
  RFD module is wired up, swap to a dedicated UART (`Serial1` or
  `Serial2`) at 57600 baud — see the note at the top of `main.cpp`.
- **TLM carries zeros until sensors land.** The 2 Hz TLM broadcast is
  wired up and continuously firing; sensor and GPS fields are zeroed,
  which the GCS already interprets as "sensor not ready" per the
  protocol. The path is real — only the data source is a stub.
- **Single writer, no mutex.** The main loop is the only producer of
  outgoing frames today. When tasks land that emit frames concurrently
  (e.g. a FreeRTOS sensor task), wrap `RadioLink::send()` in a mutex to
  prevent byte-interleaving on the wire — see protocol-robustness notes
  in `docs/protocol.md`.
- **Mock step timings are arbitrary.** `Sampler.h::mock_timing` gives
  a ~9-second cycle; pick whatever feels good for testing. With real
  sensors, timings disappear entirely (transitions fire on sensor input).
