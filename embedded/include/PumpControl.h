// PumpControl — three on/off pump GPIOs, one per tank.
//
// Per the production-board pinout PDF, the three sampling-pump
// control lines land on these chip GPIOs:
//
//     PUMP1 = IO14  (drives tank C1 pump)
//     PUMP2 = IO21  (drives tank C2 pump)
//     PUMP3 = IO47  (drives tank C3 pump)
//
// Active polarity is assumed HIGH = pump ON; flip the constant if
// the H-bridge / driver on the board turns out to be inverting.
//
// This class is intentionally as dumb as possible: a `set(id, on)`
// that toggles the matching GPIO. Higher-level logic (PUMPING-step
// hook, abort safety, etc.) lives in MissionControl. There is no
// timeout in here — callers are responsible for turning pumps off.
// A `stop_all()` is provided as a safety primitive (used by E-STOP).

#pragma once

#include <stdint.h>

class PumpControl {
public:
    // Sampler IDs are 1..3 ; PumpControl uses the same id-space so a
    // tank's id maps directly to its pump.
    static constexpr uint8_t TANK_COUNT  = 3;
    static constexpr int     PUMP1_PIN   = 14;
    static constexpr int     PUMP2_PIN   = 21;
    static constexpr int     PUMP3_PIN   = 47;
    static constexpr bool    ACTIVE_HIGH = true;  // flip if HW inverts

    PumpControl() = default;

    // Configure GPIOs as outputs, ensure all pumps are off.
    void begin();

    // Drive pump <tank_id> on or off. tank_id is 1..3. Out-of-range
    // ids are silently ignored.
    void set(uint8_t tank_id, bool on);

    // Force every pump off. Use for E-STOP / abort safety paths.
    void stop_all();

    // Read back what we last commanded (not a true sensor read).
    bool last_state(uint8_t tank_id) const;

private:
    static int pin_for(uint8_t tank_id);

    bool state_[TANK_COUNT] = {false, false, false};
};
