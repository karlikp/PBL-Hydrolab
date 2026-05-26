// Elmetron — measurement-side FSM (water-quality probe on the shared
// winch, lowered into open water at the current GPS position).
//
//     DOCKED ──request_start()──► MEASURING
//                                     │
//                                     │ runs through internal steps
//                                     │ (HOMING → DESCENDING →
//                                     │  IN_WATER → ASCENDING → HOME)
//                                     │ on timed transitions driven
//                                     │ by tick(). Returns straight
//                                     │ back to DOCKED at the end of
//                                     │ HOME.
//                                     ▼
//                                  DOCKED
//
// HOMING is a safe-start phase that runs before DESCENDING: drive the
// winch UP until H_LIMIT trips (cable fully retracted), then begin the
// descent. Without this, a previous abort that left the cable at a
// mid-position would cause the next descent to over-extend — the
// H-bridge driver is time-based and doesn't know absolute position.
// A hard time bound HOMING_MAX_MS faults the cycle out if the limit
// never reads, since the operator-facing failure mode of "drive up
// forever and snap the cable" is worse than "explicit fault."
//
// At any point, abort() forces FAULT (used by E-STOP). reset() clears
// FAULT back to DOCKED. The protocol's documented step vocabulary
// (DESCENDING / IN_WATER / PUMPING / ASCENDING / HOME) is reused; the
// Elmetron skips PUMPING since there is nothing to pump — it just
// dwells in IN_WATER long enough to settle and gather samples.
//
// IN_WATER is internally split into:
//
//   - a settle period of SETTLE_MS where the probe is wet but its
//     readings are still drifting toward equilibrium — TLM samples
//     during this window carry measurement_valid=0 so the GCS can
//     plot the convergence curve without trusting it.
//
//   - a measurement window of MEASURING_WINDOW_MS where the probe
//     has settled — TLM samples carry measurement_valid=1. The GCS
//     averages or summarises these into a per-spot reading (firmware
//     does no averaging itself; the GCS journals raw samples).
//
// Bring-up note (2026-05-26): SETTLE_MS / MEASURING_WINDOW_MS /
// steady-state values are placeholders. The whole point of the
// flag-and-emit telemetry approach is so we can plot a real probe's
// convergence curve and tune these against actual hardware.
//
// The synthetic reading model linearly ramps from zero to the
// steady-state target over SETTLE_MS so bench tests of the protocol
// path see a non-trivial curve even without a real probe attached.

#pragma once

#include <stdint.h>

#include "Clock.h"

enum class ElmetronState : uint8_t {
    DOCKED,
    MEASURING,
    FAULT,
};

enum class ElmetronStep : uint8_t {
    NONE,           // not currently measuring
    HOMING,         // safe-start: drive winch UP until H_LIMIT trips
    DESCENDING,
    IN_WATER,
    ASCENDING,
    HOME,
};

namespace elmetron_timing {
    // Hard time bound on the HOMING step. The real winch should reach
    // its home limit in well under this; if it doesn't, something is
    // wrong (broken limit switch, jammed mechanism, missing H-bridge
    // driver) and we'd rather fault out than keep driving the motor
    // and risk snapping the cable. Tune against real hardware once
    // the WinchH driver lands.
    constexpr uint32_t HOMING_MAX_MS       = 4000;

    // Time to lower the probe to the water from its docked position.
    constexpr uint32_t DESCENDING_MS       = 3000;

    // Time the probe spends in water before its readings are
    // considered steady. TLM during this period carries
    // measurement_valid=0.
    constexpr uint32_t SETTLE_MS           = 5000;

    // Additional time the probe spends in water collecting
    // measurement_valid=1 samples that the GCS averages.
    constexpr uint32_t MEASURING_WINDOW_MS = 5000;

    // Time to retract the probe back to dock.
    constexpr uint32_t ASCENDING_MS        = 3000;

    // Brief dwell at HOME before returning to DOCKED. Mirrors the
    // Sampler FSM so the protocol's STEP,HOME event is visible to the
    // GCS as a distinct moment.
    constexpr uint32_t HOME_MS             = 500;
}

namespace elmetron_synthetic {
    // Placeholder steady-state readings used until a real driver
    // lands. Values picked to match the example TLM frame in
    // docs/protocol.md §4 so bench tests look realistic.
    constexpr float COND_STEADY   = 432.10f;   // mS/cm
    constexpr float TEMP_STEADY   = 18.50f;    // °C
    constexpr float PH_STEADY     = 7.21f;
    constexpr float OXYGEN_STEADY = 8.30f;     // mg/L
}

class Elmetron {
public:
    struct Reading {
        float cond   = 0.0f;
        float temp   = 0.0f;
        float ph     = 0.0f;
        float oxygen = 0.0f;
        bool  valid  = false;
    };

    explicit Elmetron(Clock& clock);

    // Begin a new measurement cycle. Returns false if state isn't
    // DOCKED (already MEASURING, or in FAULT).
    bool request_start();

    // Force the FSM to FAULT immediately. No-op if already FAULT.
    // Used by E-STOP. Step clears to NONE.
    void abort();

    // Clear FAULT back to DOCKED. Refuses to reset while MEASURING —
    // use abort() first. Returns false on no-op.
    bool reset();

    // Drive the FSM forward. Call frequently from the main loop.
    void tick();

    ElmetronState state() const { return state_; }
    ElmetronStep  step()  const { return step_;  }

    // Probe reading at the current instant. During IN_WATER the
    // values evolve over time (synthetic ramp until a real driver
    // takes over); elsewhere they're zero with valid=false.
    Reading last_reading() const;

    // One-shot change indicators — same idiom as Sampler. Reading
    // returns true clears the flag so the orchestrator emits exactly
    // one event per change.
    bool consume_state_change();
    bool consume_step_change();

private:
    Clock&        clock_;
    ElmetronState state_           = ElmetronState::DOCKED;
    ElmetronStep  step_            = ElmetronStep::NONE;
    uint32_t      step_entered_ms_ = 0;
    bool          state_changed_   = false;
    bool          step_changed_    = false;

    void enter_state(ElmetronState s);
    void enter_step(ElmetronStep s);
    uint32_t elapsed_in_step() const;
};

// String helpers for protocol frame emission.
const char* state_name(ElmetronState s);
const char* step_name(ElmetronStep s);
