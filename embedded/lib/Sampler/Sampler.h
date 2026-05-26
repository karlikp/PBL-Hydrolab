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
// hardware, sensor inputs replace these timers — but PUMPING_MS is
// still used as a fallback when no level sensor is attached, and
// PUMPING_TIMEOUT_MS is the hard safety cap that ends PUMPING even
// when a sensor is attached but never fires (broken sensor, clogged
// hose, etc.).
namespace mock_timing {
    constexpr uint32_t DESCENDING_MS     = 3000;
    constexpr uint32_t IN_WATER_MS       = 500;
    constexpr uint32_t PUMPING_MS        = 2000;     // no-sensor fallback
    constexpr uint32_t PUMPING_TIMEOUT_MS = 30000;   // with-sensor hard cap
    constexpr uint32_t ASCENDING_MS      = 3000;
    constexpr uint32_t HOME_MS           = 500;
}

class Sampler {
public:
    // Optional per-tank "is this tank full?" probe. When set, the
    // PUMPING step ends as soon as this returns true (level sensor
    // reports liquid present) instead of after PUMPING_MS. A hard
    // PUMPING_TIMEOUT_MS still applies as a safety cap. When unset,
    // PUMPING falls back to the mock PUMPING_MS timer so unit tests
    // and the classic dev board still work unchanged.
    using LevelSensorFn = bool (*)(uint8_t tank_id, void* ctx);

    Sampler(uint8_t id, Clock& clock);

    // Request a new sampling cycle. Returns false if the tank is not
    // EMPTY (already SAMPLING, FULL, or FAULT).
    bool request_start();

    // Attach a level-sensor probe used during PUMPING. Pass nullptr
    // to detach and fall back to mock-timer behaviour.
    void set_level_sensor(LevelSensorFn fn, void* ctx);

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
    LevelSensorFn level_sensor_     = nullptr;
    void*         level_sensor_ctx_ = nullptr;

    void enter_state(TankState s);
    void enter_step(TankStep s);
    uint32_t elapsed_in_step() const;
};

// String helpers, used by the orchestrator when emitting EVT frames.
const char* state_name(TankState s);
const char* step_name(TankStep s);
