# Legacy embedded modules

Reference implementations from the previous semester's firmware. Not
compiled into the current build (PlatformIO doesn't pick this directory
up). Kept around because they contain working, lab-tested logic for
hardware we'll re-integrate once the new architecture is ready for it:

- `SensorManager.{cpp,h}` — Elmetron CX-401 driver. Single-query
  `#0#0#0#` protocol that returns all four channels (pH, O₂, conductivity,
  temperature) in one ETX-terminated response. State machine: REQUEST →
  WAIT 500 ms → PROCESS.

- `GPSManager.{cpp,h}` — u-blox NEO-M9N wrapper over I²C using the
  SparkFun GNSS library.

- `motor_control.{cpp,h}` — MCPWM-based H-Bridge driver with an explicit
  arm/disarm gate (`init_motor_safe`, `motor_arm`, `motor_stop`). The
  motor cannot spin until `motor_arm(true)` is called. Includes a small
  deadband to prevent buzzing at near-zero PWM. **Keep this when wiring
  up the real winch driver — the arming gate is a real safety property.**

- `pid_ctr.{cpp,h}` — minimal PID class with `update()`, `tune()`,
  `reset()`. Used by previous semester's closed-loop winch position
  control. May or may not be needed on the new board — open question
  whether there's still an encoder on the winch motor (see
  `docs/hardware/pytania_do_hw.md`).

- `config.h`, `pinout_def.h`, `TelemetryData.h` — constants and structs
  for the previous-semester ESP32-classic pin map. Replace wholesale
  when the new ESP32-S3 pinout is final; do not copy-paste pin numbers,
  they're wrong for the new board.

When integrating any of these, **read the file first** rather than
copying it. The previous-semester code uses module-level globals and
file-scope `HardwareSerial` objects, which we're moving away from in
the new architecture. Each integration should rewrap the driver as a
small class with a clean constructor and a `tick()` method, in the
style of `Sampler` / `MissionControl`.
