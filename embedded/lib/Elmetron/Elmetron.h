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
    // ---- Safety caps (HARDWARE mode) ----
    // In hardware mode each moving step advances on a real signal
    // (limit switch / water detection / convergence); these are the
    // backstops if that signal never comes. Sized for ELMETRON winch
    // duty in MissionControl — rescale if the drive duty changes
    // (slower drive ⇒ longer cap). HOMING/DESCENDING/ASCENDING faults
    // on cap; IN_WATER just finishes with whatever the last reading is.
    constexpr uint32_t DESCENT_MAX_MS      = 4000;   // unroll cap (mechanical)
    constexpr uint32_t ASCENT_MAX_MS       = 5000;   // retract cap
    constexpr uint32_t MEASURE_MAX_MS      = 90000;  // strict measure cap

    // Hard time bound on the HOMING step (both modes). The real winch
    // should reach its home limit well under this; if it doesn't,
    // something is wrong and we'd rather fault than keep driving.
    constexpr uint32_t HOMING_MAX_MS       = 4000;

    // ---- Synthetic timings (no-hardware mode, e.g. dev-kit/mock) ----
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

    // Hardware mode: when a real probe + winch are attached,
    // MissionControl drives the step transitions via the notify
    // triggers below (with the safety caps as backstops). When off
    // (no hardware — dev-kit/mock), the FSM runs the synthetic
    // timer-and-ramp path so the cycle is still exercisable.
    void set_hardware_mode(bool on) { hardware_mode_ = on; }
    bool hardware_mode() const { return hardware_mode_; }

    // Provision the hardware-mode safety caps (ms). Defaults are the
    // elmetron_timing constants. These only bound the HARDWARE path;
    // the synthetic path keeps its own fixed timings.
    void set_hw_timeouts(uint32_t descent_ms, uint32_t ascent_ms,
                         uint32_t homing_ms, uint32_t measure_ms) {
        descent_max_ms_ = descent_ms;
        ascent_max_ms_  = ascent_ms;
        homing_max_ms_  = homing_ms;
        measure_max_ms_ = measure_ms;
    }

    // Notify triggers (hardware mode). Each advances the FSM only when
    // it's in the matching step; otherwise ignored. Idempotent.
    void at_home();           // HOMING / ASCENDING reached the home limit
    void water_detected();    // DESCENDING — probe shows water
    void measurement_done();  // IN_WATER — readings converged

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
    bool          hardware_mode_   = false;
    bool          advance_         = false;  // pending hardware trigger
    uint32_t      descent_max_ms_  = elmetron_timing::DESCENT_MAX_MS;
    uint32_t      ascent_max_ms_   = elmetron_timing::ASCENT_MAX_MS;
    uint32_t      homing_max_ms_   = elmetron_timing::HOMING_MAX_MS;
    uint32_t      measure_max_ms_  = elmetron_timing::MEASURE_MAX_MS;

    void enter_state(ElmetronState s);
    void enter_step(ElmetronStep s);
    uint32_t elapsed_in_step() const;
    void tick_hardware(uint32_t elapsed);
    void tick_synthetic(uint32_t elapsed);
};

// String helpers for protocol frame emission.
const char* state_name(ElmetronState s);
const char* step_name(ElmetronStep s);
