// Sampler — per-tank sampling state machine.
//
// Owns the lifecycle of a single sample collection on one tank:
//
//     EMPTY ──request_start()──► SAMPLING
//                                    │
//                                    │ runs through internal steps
//                                    │ (DESCENDING → IN_WATER → PUMPING
//                                    │  → ASCENDING → HOME) on timed
//                                    │ transitions driven by tick()
//                                    ▼
//                                  FULL
//                                    │
//                                    │ reset()
//                                    ▼
//                                  EMPTY
//
// At any point, abort() forces the tank to FAULT (used by E-STOP).
// reset() clears FULL or FAULT back to EMPTY.
//
// In the SKELETON phase the step transitions fire on mock timers — no
// real pump or sensor is driven. When real hardware drivers land, the
// step-entry code in Sampler.cpp will call those drivers and the
// step-exit conditions will read real sensor inputs instead of timing
// out. The FSM structure itself doesn't change.

#pragma once

#include <stdint.h>

#include "Clock.h"

enum class TankState : uint8_t {
    EMPTY,
    SAMPLING,
    FULL,
    FAULT,
};

enum class TankStep : uint8_t {
    NONE,        // not currently sampling
    DESCENDING,
    IN_WATER,
    PUMPING,
    ASCENDING,
    HOME,
};

// Mock step durations used during the skeleton phase. With real
// hardware, sensor inputs replace these timers.
namespace mock_timing {
    constexpr uint32_t DESCENDING_MS = 3000;
    constexpr uint32_t IN_WATER_MS   = 500;
    constexpr uint32_t PUMPING_MS    = 2000;
    constexpr uint32_t ASCENDING_MS  = 3000;
    constexpr uint32_t HOME_MS       = 500;
}

class Sampler {
public:
    Sampler(uint8_t id, Clock& clock);

    // Request a new sampling cycle. Returns false if the tank is not
    // EMPTY (already SAMPLING, FULL, or FAULT).
    bool request_start();

    // Force this tank to FAULT immediately. No-op if already FAULT.
    // Used by E-STOP. Step is cleared to NONE.
    void abort();

    // Clear FULL or FAULT back to EMPTY. No-op if already EMPTY.
    // Refuses to reset a tank currently SAMPLING — use abort() first.
    bool reset();

    // Drive the FSM forward. Should be called from the main loop
    // frequently (every few ms is plenty).
    void tick();

    TankState state() const { return state_; }
    TankStep   step()  const { return step_;  }
    uint8_t    id()    const { return id_;    }

    // One-shot change indicators. Returning true clears the flag, so
    // the orchestrator emits exactly one event per change.
    bool consume_state_change();
    bool consume_step_change();

private:
    uint8_t id_;
    Clock&  clock_;
    TankState state_ = TankState::EMPTY;
    TankStep  step_  = TankStep::NONE;
    uint32_t  step_entered_ms_ = 0;
    bool state_changed_ = false;
    bool step_changed_  = false;

    void enter_state(TankState s);
    void enter_step(TankStep s);
    uint32_t elapsed_in_step() const;
};

// String helpers, used by the orchestrator when emitting EVT frames.
const char* state_name(TankState s);
const char* step_name(TankStep s);
