# H2O Drone — Communication Protocol

**Version:** 1.0
**Audience:** anyone working on the ground station ↔ drone link, on either side.

This document describes the messages that go back and forth between the
drone and the ground station. It's the contract — both sides need to
match it for the system to work. If you're implementing either side,
this is everything you need.

---

## 1. The big picture

The drone and the ground station are connected by a **radio link that
behaves like a serial cable**. Bytes go in one end, come out the other.
Both sides can send and receive at the same time.

Three kinds of messages flow over this link:

```
                    ─────── drone sends ─────►
   ╔════════╗      TLM  — telemetry every half-second        ╔═════╗
   ║  drone ║      EVT  — events ("tank 1 is full", "ack")   ║ GCS ║
   ╚════════╝                                                ╚═════╝
                   ◄────── GCS sends ────────
                   CMD  — commands ("sample tank 1", "stop")
```

- **`TLM`** is a steady heartbeat from the drone — sensor readings, GPS,
  every 500 ms. Use it to draw charts and update the map.
- **`EVT`** is the drone telling you something happened — a tank just
  filled up, a button was acknowledged, an error occurred. Use it to
  update the operator's UI.
- **`CMD`** is you (the GCS) telling the drone what to do — start
  sampling, emergency stop, etc.

Each message is one line of text, ending with a newline (`\n`). At the
end of every line there's a small checksum so corrupted messages can be
spotted and dropped.

---

## 2. What does a message look like?

Every message has this shape:

```
<the actual content>*<8 hex digits>\n
```

The `*` separates the content from the checksum. The 8 hex digits are
an Adler-32 checksum of the content. The newline ends the message.

Here's a real telemetry frame, annotated:

```
TLM,2026-05-15 14:30:00,521230000,210110000,432.10,18.50,7.21,8.30,1*0F123ABC
└─┬─┘ └────────┬────────┘ └────┬────┘ └────┬────┘ └──┬──┘ ...    └───┬────┘
type  timestamp        latitude    longitude   conductivity      checksum
```

Reading order: split on `*`, check the checksum matches, look at the
first comma-separated field to know what kind of message it is, then
parse the rest accordingly.

If the checksum doesn't match, **drop the message silently**. Don't
ask the sender to resend, don't show an error — just move on. The next
valid message will pick things up.

**Maximum length:** 256 bytes including the newline. Longer than that =
drop.

---

## 3. The checksum

We use **Adler-32**, computed over the content (everything before the
`*`). Here's the algorithm in plain Python — drop it into your codebase
and you're done:

```python
def adler32(payload: str) -> str:
    """Return Adler-32 of payload as 8-char uppercase hex."""
    a, b = 1, 0
    for byte in payload.encode("utf-8"):
        a = (a + byte) % 65521
        b = (b + a) % 65521
    return f"{(b << 16) | a:08X}"
```

(Python's standard library `zlib.adler32` produces the same numbers if
you prefer.)

**Test it works:** run these through your implementation, you should
get exactly these outputs:

| Input (string)   | Expected checksum |
|------------------|-------------------|
| `""`             | `00000001`        |
| `"a"`            | `00620062`        |
| `"abc"`          | `024D0127`        |
| `"TLM,test"`     | `0BBF02DA`        |

If any of these don't match, your implementation is wrong somewhere
(usually: byte encoding, or you included the `*` in the input).

---

## 4. `TLM` — telemetry from the drone

The drone broadcasts a telemetry frame **every 500 ms** (2 Hz) while
it's powered on. It looks like this:

```
TLM,<timestamp>,<lat>,<lon>,<cond>,<temp>,<ph>,<oxygen>,<water_flag>,<measurement_valid>*<checksum>
```

| Field                 | What it is                                       | Example          |
|-----------------------|--------------------------------------------------|------------------|
| `timestamp`           | Date and time from the drone's GPS               | `2026-05-15 14:30:00` |
| `lat`                 | Latitude as integer ×10,000,000                  | `521230000` = 52.1230000° |
| `lon`                 | Longitude as integer ×10,000,000                 | `210110000` = 21.0110000° |
| `cond`                | Conductivity, mS/cm                              | `432.10`         |
| `temp`                | Water temperature, °C                            | `18.50`          |
| `ph`                  | pH                                               | `7.21`           |
| `oxygen`              | Dissolved oxygen, mg/L                           | `8.30`           |
| `water_flag`          | `0` = sensor in air, `1` = sensor under water    | `1`              |
| `measurement_valid`   | `1` = probe has settled, the four sensor fields are trustworthy; `0` = probe absent or still settling, fields are informational only | `1` |

> ⚠️ **Heads-up:** lat/lon are scaled integers, not normal decimals.
> Divide by 10,000,000 to get degrees. This is a leftover convention
> from the existing system — don't try to "fix" it on one side without
> the other.

> ⚠️ **Heads-up:** if any sensor field is `0.00`, treat the frame as
> "sensor not ready" — store it but don't show alarms. This is how the
> drone signals "I'm warming up". Existing dashboard already does this.
> The `measurement_valid` flag is a stronger signal: `0` means "do not
> trust these readings yet" even when the values look plausible. Plot
> them if you want a settle curve, but don't surface them as the
> measurement of record. `1` means the probe has reached steady state
> and the four sensor fields can be relied on.

---

## 5. `EVT` — events from the drone

The drone sends event messages when something interesting happens. Use
these to update the live UI: tank state cards, progress indicators,
notifications, etc.

```
EVT,<source>,<kind>,<details>*<checksum>
```

### Who sent it? (the `source` field)

| Source      | Meaning                                  |
|-------------|------------------------------------------|
| `C1`        | The tank-1 sampler                       |
| `C2`        | The tank-2 sampler                       |
| `C3`        | The tank-3 sampler                       |
| `ELMETRON`  | The Elmetron measurement subsystem       |
| `WINCH`     | The shared winch (motor + servo)         |
| `SYS`       | The overall drone — top-level state, acks, errors |

### What kind of event? (the `kind` field)

| Kind     | Sent by                  | What `details` contains             |
|----------|--------------------------|-------------------------------------|
| `STATE`  | `C1`/`C2`/`C3`           | `EMPTY`, `SAMPLING`, `FULL`, `FAULT` |
| `STATE`  | `ELMETRON`               | `DOCKED`, `MEASURING`, `FAULT`      |
| `STATE`  | `SYS`                    | `IDLE`, `SAMPLING`, `MEASURING`, `E_STOP` |
| `CFG`    | `SYS`                    | Active tank config readback (reply to `CMD,CFG_GET` / `CMD,STATUS`) |
| `CFG_ELE`| `SYS`                    | Active Elmetron config readback (reply to `CMD,CFG_ELE_GET` / `CMD,STATUS`) |
| `STEP`   | `C1`/`C2`/`C3`/`ELMETRON`| `HOMING` (ELMETRON only), `DESCENDING`, `IN_WATER`, `PUMPING` (C1–C3 only), `ASCENDING`, `HOME` |
| `ACK`    | `SYS`                    | The command that was accepted (e.g. `START_C1`) |
| `NACK`   | `SYS`                    | The command and why it was rejected (e.g. `START_C2:busy`) |
| `ERROR`  | any                      | A short error description (e.g. `limit_switch_timeout`) |
| `BOOT`   | `SYS`                    | Firmware version (e.g. `1.0.0`)     |

**Quick mental model:**
- `STATE` is the **current overall status** of something — show it as a
  big colour-coded badge.
- `STEP` is **what's happening right now** during a multi-step operation
  — show it as a progress bar or step indicator.
- `ACK` / `NACK` is the drone responding to **the most recent CMD you
  sent** — use it to confirm the button click worked.
- `ERROR` is bad — show it prominently to the operator.
- `BOOT` means the drone just (re)started — re-sync your UI state.

### Examples

```
EVT,C1,STATE,SAMPLING*01F2FF11           ← tank 1 just started sampling
EVT,C1,STEP,IN_WATER*01EE7766            ← tank 1 has reached the water
EVT,C1,STATE,FULL*01E10044               ← tank 1 finished, it's full
EVT,SYS,ACK,START_C1*0211EE99            ← drone accepted the START_C1 command
EVT,SYS,NACK,START_C2:busy*02A1BC03      ← drone rejected START_C2: another op is running
EVT,WINCH,ERROR,limit_switch_timeout*0344DEAD  ← winch hasn't reached its limit in time
EVT,SYS,BOOT,1.0.0*01122334              ← drone just booted, firmware 1.0.0
```

> 💡 **Implementation tip:** the `details` field never contains commas.
> If you split the message on commas, you'll get exactly four pieces:
> `EVT`, source, kind, details.

---

## 6. `CMD` — commands you send to the drone

Send these when the operator clicks a button. Each command is one-shot —
fire it and watch the events to see what happens.

```
CMD,<command>,<arguments>*<checksum>
```

Most commands don't take arguments, in which case the arguments part is
empty — note the trailing comma before the `*`:

```
CMD,START_C1,*04AB1234
       ↑    ↑
   command  empty args (just leave it)
```

### Available commands

| Command            | What it does                                                  |
|--------------------|---------------------------------------------------------------|
| `START_C1`         | Start sampling tank 1                                         |
| `START_C2`         | Start sampling tank 2                                         |
| `START_C3`         | Start sampling tank 3                                         |
| `START_ELMETRON`   | Start an Elmetron measurement at current position             |
| `STOP_C1`          | Stop tank 1 in place (FAULT) without latching system E-STOP. Cable / pump halt where they are; tank can be `RESET_C1`-ed independently. Other subsystems keep running. |
| `STOP_C2`          | (same for tank 2)                                             |
| `STOP_C3`          | (same for tank 3)                                             |
| `STOP_ELMETRON`    | Stop Elmetron in place (FAULT) without latching system E-STOP |
| `E_STOP`           | Emergency stop — abort everything, move any in-progress op to FAULT, latch the system in `E_STOP` until **all** faulted subsystems are reset. |
| `RESET_C1`         | Clear tank 1's `FAULT` and move it back to `EMPTY`            |
| `RESET_C2`         | (same for tank 2)                                             |
| `RESET_C3`         | (same for tank 3)                                             |
| `RESET_ELMETRON`   | Clear Elmetron's `FAULT` and move it back to `DOCKED`         |
| `JOG_C<n>_UP`<br>`JOG_C<n>_DOWN` | Manual one-shot winch jog on tank n (1–3). Drives PWM in the rewind (UP) or unroll (DOWN) direction until the encoder cumulative has moved ≈15° (~51 counts), then coasts. Used to home the spool by hand before pressing `START_C<n>`. Time safety cap is 2 s (fires only if ReadPos fails). NACKs `busy` while any sampling is in progress, `winch_busy` while a previous jog is still finishing. |
| `STATUS`           | Ask the drone for a snapshot — it replies with one `EVT,STATE` per subsystem plus `EVT,SYS,CFG` |
| `PING`             | Check the link is alive — drone responds `EVT,SYS,ACK,PING`   |
| `CFG`              | Provision the tank mapping + global timeout. Format: `CMD,CFG,to=<sec>,Cx=<en>:<ch>:<servo>:<unroll_ms>:<roll_ms>:<pwm>,...` — per tank: enabled, pump/sensor channel, SC-09 servo id, and winch PWM/wheel-mode tuning (unroll/roll durations in ms, signed PWM duty -1023..1023; sign sets unroll direction). E.g. `CMD,CFG,to=90,C1=1:1:1:4000:4000:600,C2=1:2:2:4000:4000:-600,C3=0:3:3:4000:4000:600`. Rejected while busy. RAM-only — re-sent by the GCS on every boot. The ESP puts every enabled tank's servo into PWM/wheel mode on provisioning (writes 0/0 to angle-limit EEPROM regs — persists across power cycles). |
| `CFG_GET`          | Drone replies `EVT,SYS,CFG,...` with its active tank config (readback) |
| `CFG_ELE`          | Provision Elmetron tuning: `CMD,CFG_ELE,wt=<mS/cm>,wd=<duty%>,dto=<s>,ato=<s>,hto=<s>,mto=<s>,cw=<s>,ct=<%>,di=<0\|1>,lal=<0\|1>` (water threshold, winch duty, descent/ascent/homing/measure timeouts, convergence window+tolerance, direction-invert, limit-active-low). Timeouts ceilinged so the descent safety cap can't be disabled. |
| `CFG_ELE_GET`      | Drone replies `EVT,SYS,CFG_ELE,...` with its active Elmetron config (readback) |
| `ELE`              | Diagnostic — drone replies `EVT,SYS,ELE,cond=<mS/cm>,temp=<C>,ph=<>,o2=<mg/L>,water=0\|1,present=0\|1` with the latest raw Elmetron probe reading (`present=0` if no probe is answering on the bus) |

### What happens after you send a CMD

The drone **always replies with exactly one of these**:

- ✅ `EVT,SYS,ACK,<command>` — the drone accepted the command and is
  now executing it. Subsequent `STEP` and `STATE` events from the
  affected subsystem will follow as it runs.
- ❌ `EVT,SYS,NACK,<command>:<reason>` — the drone refused. Nothing
  happens.

So your code should: send the CMD, then watch for either ACK or NACK
within ~500 ms. If it's NACK, show why to the operator. If it's ACK,
keep listening for the follow-up `STEP` and `STATE` events.

### Why might a command be NACKed?

| Reason              | What it means                                                       |
|---------------------|---------------------------------------------------------------------|
| `busy`              | Another sampling or measurement is in progress — wait for it to finish |
| `tank_full`         | Tank is already full — empty it manually, or pick a different tank   |
| `fault`             | Tank is in fault state — send `RESET_Cx` first                       |
| `not_running`       | `STOP_*` was sent against a subsystem that isn't currently active   |
| `not_in_fault`      | `RESET_ELMETRON` was sent while Elmetron wasn't in `FAULT`           |
| `not_implemented`   | The targeted subsystem isn't attached on this build                 |
| `disabled`          | `START_Cx` for a tank disabled in the provisioned config            |
| `dup_channel` / `dup_servo` | `CMD,CFG` had two enabled tanks sharing a channel or servo id |
| `out_of_range` / `bad_timeout` | `CMD,CFG` value outside the valid range              |
| `e_stop_active`     | Drone is in E-STOP mode — only `RESET_*` and `STATUS` work until reset |
| `unknown_command`   | The drone doesn't recognise this command (version mismatch?)         |

---

## 7. How a typical session looks

The flow you'll implement on the GCS side:

```
1. GCS connects to the serial port
2. GCS sends:    CMD,STATUS,                  ← "what's your current state?"
3. Drone replies with a burst of EVT,STATE messages, one per subsystem
4. GCS shows the current state on the UI

5. Operator clicks "Sample C1":
   GCS sends:    CMD,START_C1,
   Drone:        EVT,SYS,ACK,START_C1
   Drone:        EVT,SYS,STATE,SAMPLING
   Drone:        EVT,C1,STATE,SAMPLING
   Drone:        EVT,C1,STEP,DESCENDING
   Drone:        EVT,C1,STEP,IN_WATER
   Drone:        EVT,C1,STEP,PUMPING
   Drone:        EVT,C1,STEP,ASCENDING
   Drone:        EVT,C1,STEP,HOME
   Drone:        EVT,C1,STATE,FULL
   Drone:        EVT,SYS,STATE,IDLE
   (TLM frames continue arriving at 2 Hz throughout)

6. If the radio link drops and reconnects:
   GCS:    CMD,STATUS,            ← resync
   Drone:  EVT,STATE,... (burst, like at startup)

7. If the drone reboots unexpectedly:
   Drone:  EVT,SYS,BOOT,1.0.0    ← GCS sees this → send CMD,STATUS again
```

> 💡 **Implementation tip:** PING/STATUS every few seconds is a great
> way to detect a dead link. If the drone hasn't responded to a `PING`
> within ~3 seconds, show "RADIO OFFLINE" in the UI.

---

## 8. When things go wrong

| Situation                            | What to do                                       |
|--------------------------------------|--------------------------------------------------|
| Received a frame with bad checksum   | Drop it silently. Don't log loudly — corrupted frames happen on radio. |
| Received a frame you can't parse     | Drop silently. Maybe log at debug level.         |
| Sent a CMD, no ACK/NACK within 500ms | Resend once. Still nothing? Show "no response" to operator. |
| `EVT,SYS,BOOT` arrives mid-session   | Drone restarted. Reset your UI state and send `CMD,STATUS`. |
| Frame longer than 256 bytes         | Drop. Don't try to "fix" by partial parsing.    |

---

## 9. Full worked example — start a sample, then E-STOP

Operator clicks "Sample C1":

```
→ CMD,START_C1,*04AB1234
← EVT,SYS,ACK,START_C1*0211EE99
← EVT,SYS,STATE,SAMPLING*01D1AA22
← EVT,C1,STATE,SAMPLING*01F2FF11
← EVT,WINCH,STEP,SELECT_C1*01ABCD01
← EVT,C1,STEP,DESCENDING*01F09988
← EVT,C1,STEP,IN_WATER*01EE7766
```

Mid-operation, operator panics and hits E-STOP:

```
→ CMD,E_STOP,*0392DDEE
← EVT,SYS,ACK,E_STOP*0211EEFE
← EVT,C1,STATE,FAULT*01F2EEAA
← EVT,SYS,STATE,E_STOP*01D1CC00
```

To recover, operator clears the fault:

```
→ CMD,RESET_C1,*0123ABCD
← EVT,SYS,ACK,RESET_C1*0211ABCD
← EVT,C1,STATE,EMPTY*01F2EEBB
→ CMD,STATUS,*0392DEAD
← EVT,SYS,STATE,IDLE*01D1AABB
← EVT,C1,STATE,EMPTY*01F2EEBB
← EVT,C2,STATE,EMPTY*01F2EECC
← EVT,C3,STATE,EMPTY*01F2EEDD
← EVT,ELMETRON,STATE,DOCKED*01F2EEEE
```

Operator can now retry `CMD,START_C1`.

---

## 10. Versioning

This is **version 1.0** of the protocol. The drone reports its firmware
version in `EVT,SYS,BOOT,<version>` — match the major version (the
first number) and you're compatible. Major-version mismatches mean
some frames may not work — surface a warning to the operator.

**Forward-compatibility rule:** if a message has extra fields you don't
recognise (because a future version added them), **just ignore the
extras**. Don't reject the message. This lets us add new fields
without breaking old GCS code.

---

## Notes for the GCS team

A few practical tips when you implement this:

- **Use a single thread / task for reading from the serial port.** It
  reads lines, validates checksums, and pushes parsed frames into a
  queue or pub/sub. Other code (FastAPI handlers, UI updaters) consume
  from there.
- **Use a `Lock` around writing to the serial port.** When the operator
  sends `CMD,START_C1` and you're also responding to a `PING` from the
  drone (you're not — they don't ping us — but the principle stands),
  you don't want the two writes interleaving on the wire.
- **Don't trust the checksum to catch every error.** Adler-32 catches
  most random bit flips but isn't bulletproof. Sanity-check parsed
  values (e.g. lat between -90 and 90 after scaling).
- **Keep a buffer of recent EVTs** in memory so the dashboard can show
  "what just happened" without polling state every frame. ~50 events
  is plenty.
- **The drone is the source of truth for state.** Don't infer tank
  state from your own logic ("I sent START_C1 so it must be sampling
  now") — wait for the `EVT,C1,STATE,SAMPLING` to confirm.

That's it. If anything is unclear or seems to contradict itself, ask —
better to clarify here than in code.
