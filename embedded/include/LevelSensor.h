// LevelSensor — read the per-tank optical liquid-level sensors.
//
// Each tank has one FS-IR12 IR optical sensor wired to the production
// board's TOPCN net:
//
//     TOPCN1 = IO40  (tank C1)
//     TOPCN2 = IO41  (tank C2)
//     TOPCN3 = IO42  (tank C3)
//
// Hardware: each TOPCN net has an external pull-up resistor on the
// PCB tied to the 3.3 V logic rail. The sensor is an open-collector
// NPN — its output sinks the line to GND when submerged, and is
// high-Z when dry (the external pull-up gives HIGH). So:
//
//     wet  -> line LOW   (full = 1)
//     dry  -> line HIGH  (full = 0)
//     disconnected -> line HIGH (== not full), safe default.
//
// We still enable the internal pull-up as a redundant backup in case
// the PCB pull-up is ever depopulated; it's swamped by the external
// resistor at normal operating impedances.
//
// Implementation note: IO40/41/42 are the ESP32-S3's default JTAG
// pins. Arduino's pinMode() is supposed to reroute them to plain GPIO
// but the IO_MUX is sometimes flaky on these pins. We use ESP-IDF's
// gpio_config() directly to forcefully take them away from JTAG.

#pragma once

#include <stdint.h>

class LevelSensor {
public:
    static constexpr uint8_t TANK_COUNT  = 3;
    static constexpr int     TOPCN1_PIN  = 40;
    static constexpr int     TOPCN2_PIN  = 41;
    static constexpr int     TOPCN3_PIN  = 42;
    // External PCB pull-up + open-collector NPN sensor: wet sinks the
    // line LOW, dry leaves it HIGH. So "tank full" == raw reads LOW.
    static constexpr bool    ACTIVE_LOW = true;

    LevelSensor() = default;

    // Configure TOPCN1/2/3 as INPUT with internal pull-up enabled
    // (redundant backup to the external PCB pull-up), taking them
    // away from the default JTAG IO_MUX function in the process.
    void begin();

    // Returns true when tank is full (sensor sees liquid). tank_id is
    // 1..3 ; out-of-range tank_ids return false.
    bool is_full(uint8_t tank_id) const;

    // Raw digitalRead — useful for bench diagnostics so the operator
    // can tell whether the sensor itself is reachable. Returns -1 on
    // out-of-range tank_id.
    int raw(uint8_t tank_id) const;

private:
    static int pin_for(uint8_t tank_id);
};
