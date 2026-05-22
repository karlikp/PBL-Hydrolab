# Battery voltage divider — ADC saturation issue

## Symptom

The on-board ADC reading on GPIO 5 (SUPADC) saturates at the maximum
value (`raw=4095`, `mv≈3146`) for any battery voltage above ~9 V.
Because of this, the firmware cannot detect crossing the project's
10.5 V cutoff threshold — by the time the ADC un-saturates, the LiPo
is already in its damage zone.

## What we measured (2026-05-22)

Probing the ADC pin with a multimeter while sweeping bench supply:

| Supply (V) | V_adc real (multimeter) | ADC reads (mV)     | Raw count       |
|-----------:|-------------------------|--------------------|-----------------|
| 8.0        | 2.80                    | 2900               | 3484  (~85 %)   |
| 9.0        | ≈ 3.15 (calc)           | 3104               | 3900  (~95 %)   |
| 11.0       | ≈ 3.85 (calc, sat.)     | 3146 (capped)      | 4095  (sat.)    |

Effective divider ratio measured: **0.35**.

## What the schematic says

> R18 = 100 kΩ (upper, from VBATT)
> R19 = 33 kΩ (lower, to GND)
> R17 = 2.2 kΩ (small series filter R between divider tap and ADC pin)
>
> Expected ratio: R19 / (R18 + R19) = 33 / 133 = **0.248**.

At ratio 0.248, V_adc at 11 V battery = 2.73 V — well within the
ESP32-S3 ADC's 11 dB-attenuation linear range (~0.15–2.45 V) and
under its saturation point (~3.1 V). The divider would work fine.

## So either…

1. **The mounted resistor values are different from the schematic**
   (e.g. R18 actually 56 kΩ, or R19 actually 56 kΩ), giving the
   observed 0.35 ratio.
2. **There's an additional path** loading one leg of the divider
   (parallel resistor, leakage, mis-routed trace) that effectively
   shifts the ratio.
3. **The schematic mis-labels R18/R19** and the as-built values are
   simply different.

## Action needed from HW team

- Visually read the markings on R17, R18, R19 on the assembled PCB
  and confirm against the schematic values.
- If they match, trace the SUPADC net to see if there's an unintended
  parallel load.
- Target ratio for a 3S LiPo monitored on the 11 dB-atten ADC is
  about **0.18** — gives V_adc ≈ 2.27 V at 12.6 V full charge,
  comfortably inside the linear range. Examples: R18 = 220 kΩ, R19 =
  33 kΩ; or R18 = 100 kΩ, R19 = 22 kΩ.

## Firmware state until then

- Module: `embedded/include/BatteryMonitor.h` + `.cpp`
- Pin: GPIO 5 (confirmed via `ADC_SCAN` diagnostic — see
  `embedded/src/MissionControl.cpp::cmd_adc_scan` and
  `embedded/tools/watch_adc.py`)
- Thresholds: stopgap WARNING=8.5 V, CRITICAL=8.0 V (highest values
  the firmware can actually detect on the current hardware — below
  LiPo safe-operating range, so this is a backstop only)
- `EVT,SYS,BATTERY,<V>/raw=<R>/mv=<M>` in STATUS responses includes
  the raw ADC count and millivolts so future debugging can see
  whether the ADC is saturated regardless of computed voltage.

## When the HW fix lands

1. Update `DIVIDER_RATIO` in `BatteryMonitor.h` to match the new
   resistor values: `R_lower / (R_upper + R_lower)`.
2. Restore intended thresholds: `WARNING_V = 11.0f`, `CRITICAL_V = 10.5f`.
3. Verify with `python3 tools/watch_adc.py` while sweeping supply —
   all four values should track linearly with no saturation in the
   8–13 V band.
4. Remove this file (or mark it resolved with date).
