// BatteryMonitor — protect the LiPo from over-discharge.
//
// The drone's 3S LiPo feeds the rest of the system through a soft-power
// latch held by GPIO 4 (see main.cpp). If we keep drawing once cell
// voltage drops below ~3.0 V/cell the battery is damaged — so the
// firmware samples the battery via a divider on GPIO 5 and, when it
// crosses safety thresholds, first warns the operator (EVT,SYS,ERROR)
// and then releases the latch (drive GPIO 4 LOW), powering the board
// down cleanly.
//
// !!! KNOWN HARDWARE ISSUE — battery protection is currently NOT
// effective. See docs/hardware/battery_divider_issue.md for details.
//
// The schematic says the divider is R18(100k) upper, R19(33k) lower,
// which gives ratio 0.248 → V_adc=2.73V at 11V battery (well in ADC
// range). But empirical measurement on the assembled board shows an
// effective ratio of ~0.35: at 8V supply the ADC pin sees 2.8V real,
// at 9V it sees ~3.15V (right at the saturation knee of 11dB-atten
// ADC), and above ~9V the ADC saturates entirely. So the 10.5V
// safety threshold cannot be detected — the firmware can only read
// below ~9V battery, which is already inside the LiPo damage zone.
//
// HW team needs to verify R18/R19 markings on the board and either
// confirm the schematic values are correct (then we look elsewhere
// for the saturation cause) or swap to a divider with ratio ~0.18.
// Until then, this module emits readings but they're meaningless
// above ~9V and the shutdown threshold is set to a stopgap value
// below the saturation knee — minimal protection, better than none.

#pragma once

#include <stdint.h>

class Clock;
class RadioLink;

class BatteryMonitor {
public:
    enum class State : uint8_t {
        OK,
        WARNING,    // below WARNING_V — operator notified
        CRITICAL,   // below CRITICAL_V — shutdown initiated
    };

    BatteryMonitor(Clock& clock, RadioLink& radio,
                   int adc_pin, int latch_pin);

    // Initialise the ADC and latch pins. Call from setup() AFTER the
    // latch is already driven HIGH so we don't accidentally drop it.
    void begin();

    // Sample + threshold-check on the SAMPLE_INTERVAL_MS cadence.
    // Call from the main loop.
    void tick();

    State    state()         const { return state_; }
    float    last_voltage()  const { return last_voltage_; }
    // For diagnostics: raw ADC millivolts and raw 12-bit ADC count
    // (i.e. before divider math and calibration).
    uint32_t last_mv()       const { return last_mv_; }
    uint16_t last_raw()      const { return last_raw_; }

private:
    Clock&     clock_;
    RadioLink& radio_;
    const int  adc_pin_;
    const int  latch_pin_;

    State    state_           = State::OK;
    uint32_t last_sample_ms_  = 0;
    float    last_voltage_    = 0.0f;
    uint32_t last_mv_         = 0;
    uint16_t last_raw_        = 0;

    // 3S LiPo (3 cells × 3.7 V nominal = 11.1 V). Intended thresholds
    // per HW team safety requirement:
    //   warning       11.0 V  (3.67 V/cell)
    //   critical      10.5 V  (3.50 V/cell)  → shutdown
    //
    // Currently neither is reachable through the ADC because of the
    // divider issue described above — the ADC saturates above ~9V.
    // Stopgap values below sit inside the LiPo damage zone but are
    // the highest the firmware can actually detect on the current
    // hardware. Restore to the intended thresholds (11.0 / 10.5)
    // immediately after the HW divider is fixed.
    static constexpr float WARNING_V  =  8.5f;
    static constexpr float CRITICAL_V =  8.0f;

    // Voltage divider feeding the ADC: VBATT — R18 (100k) — node —
    // R19 (33k) — GND. R17 (2.2k) is a small series filter R between
    // the divider tap and the ADC pin paired with a cap; it does not
    // change the DC ratio. Confirmed by HW team.
    static constexpr float DIVIDER_RATIO = 33.0f / (100.0f + 33.0f);

    static constexpr uint32_t SAMPLE_INTERVAL_MS = 2000;  // 0.5 Hz

    // Unused now that we read via analogReadMilliVolts(); kept for
    // anyone who needs raw ADC math reference.
    static constexpr int ADC_MAX = 4095;

    float read_voltage();
    void  shutdown();
};
