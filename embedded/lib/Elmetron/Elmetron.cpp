#include "Elmetron.h"

Elmetron::Elmetron(Clock& clock) : clock_(clock) {}

bool Elmetron::request_start() {
    if (state_ != ElmetronState::DOCKED) return false;
    enter_state(ElmetronState::MEASURING);
    // Always begin with HOMING so the cycle starts from a known
    // physical position regardless of where a previous abort left
    // the cable.
    enter_step(ElmetronStep::HOMING);
    return true;
}

void Elmetron::abort() {
    if (state_ == ElmetronState::FAULT) return;
    enter_state(ElmetronState::FAULT);
    enter_step(ElmetronStep::NONE);
}

bool Elmetron::reset() {
    if (state_ != ElmetronState::FAULT) return false;
    enter_state(ElmetronState::DOCKED);
    enter_step(ElmetronStep::NONE);
    return true;
}

void Elmetron::tick() {
    if (state_ != ElmetronState::MEASURING) return;

    const uint32_t elapsed = elapsed_in_step();
    switch (step_) {
        case ElmetronStep::HOMING:
            // Placeholder behaviour: dwell up to HOMING_MAX_MS then
            // proceed to DESCENDING. Once the WinchH driver lands,
            // this step also drives the H-bridge UP and exits early
            // as soon as H_LIMIT reads "at home" — and FAULTs out
            // if the limit never trips within the time bound. For
            // now the time bound IS the exit condition.
            // TODO(winch-h): drive H_EN_R / H_PWM here, read H_LIMIT,
            // transition to DESCENDING on limit-trip, FAULT on
            // timeout, leaving this fallback in for safety.
            if (elapsed >= elmetron_timing::HOMING_MAX_MS) {
                enter_step(ElmetronStep::DESCENDING);
            }
            break;
        case ElmetronStep::DESCENDING:
            if (elapsed >= elmetron_timing::DESCENDING_MS) {
                enter_step(ElmetronStep::IN_WATER);
            }
            break;
        case ElmetronStep::IN_WATER:
            // IN_WATER covers settle + measurement window. The step
            // ends when the full window has elapsed; the valid flag
            // in last_reading() flips internally at SETTLE_MS.
            if (elapsed >= elmetron_timing::SETTLE_MS
                         + elmetron_timing::MEASURING_WINDOW_MS) {
                enter_step(ElmetronStep::ASCENDING);
            }
            break;
        case ElmetronStep::ASCENDING:
            if (elapsed >= elmetron_timing::ASCENDING_MS) {
                enter_step(ElmetronStep::HOME);
            }
            break;
        case ElmetronStep::HOME:
            if (elapsed >= elmetron_timing::HOME_MS) {
                enter_state(ElmetronState::DOCKED);
                enter_step(ElmetronStep::NONE);
            }
            break;
        case ElmetronStep::NONE:
            // Unreachable while MEASURING; treat as fault recovery so
            // we don't loop forever on bad state.
            enter_state(ElmetronState::FAULT);
            break;
    }
}

Elmetron::Reading Elmetron::last_reading() const {
    Reading r;
    if (state_ != ElmetronState::MEASURING) return r;
    if (step_ != ElmetronStep::IN_WATER)    return r;

    const uint32_t elapsed = elapsed_in_step();
    if (elapsed < elmetron_timing::SETTLE_MS) {
        // Linear ramp from 0 to steady-state during the settle phase.
        // A real probe's curve is non-linear, but for bench tests of
        // the protocol path this gives the GCS a non-trivial signal
        // to plot.
        const float k = static_cast<float>(elapsed)
                      / static_cast<float>(elmetron_timing::SETTLE_MS);
        r.cond   = k * elmetron_synthetic::COND_STEADY;
        r.temp   = k * elmetron_synthetic::TEMP_STEADY;
        r.ph     = k * elmetron_synthetic::PH_STEADY;
        r.oxygen = k * elmetron_synthetic::OXYGEN_STEADY;
        r.valid  = false;
    } else {
        r.cond   = elmetron_synthetic::COND_STEADY;
        r.temp   = elmetron_synthetic::TEMP_STEADY;
        r.ph     = elmetron_synthetic::PH_STEADY;
        r.oxygen = elmetron_synthetic::OXYGEN_STEADY;
        r.valid  = true;
    }
    return r;
}

bool Elmetron::consume_state_change() {
    if (!state_changed_) return false;
    state_changed_ = false;
    return true;
}

bool Elmetron::consume_step_change() {
    if (!step_changed_) return false;
    step_changed_ = false;
    return true;
}

void Elmetron::enter_state(ElmetronState s) {
    if (state_ == s) return;
    state_ = s;
    state_changed_ = true;
}

void Elmetron::enter_step(ElmetronStep s) {
    if (step_ == s) return;
    step_ = s;
    step_entered_ms_ = clock_.now_ms();
    step_changed_ = true;
}

uint32_t Elmetron::elapsed_in_step() const {
    return clock_.now_ms() - step_entered_ms_;
}

const char* state_name(ElmetronState s) {
    switch (s) {
        case ElmetronState::DOCKED:    return "DOCKED";
        case ElmetronState::MEASURING: return "MEASURING";
        case ElmetronState::FAULT:     return "FAULT";
    }
    return "?";
}

const char* step_name(ElmetronStep s) {
    switch (s) {
        case ElmetronStep::NONE:       return "NONE";
        case ElmetronStep::HOMING:     return "HOMING";
        case ElmetronStep::DESCENDING: return "DESCENDING";
        case ElmetronStep::IN_WATER:   return "IN_WATER";
        case ElmetronStep::ASCENDING:  return "ASCENDING";
        case ElmetronStep::HOME:       return "HOME";
    }
    return "?";
}
