#include "Sampler.h"

Sampler::Sampler(uint8_t id, Clock& clock)
    : id_(id), clock_(clock) {}

void Sampler::set_level_sensor(LevelSensorFn fn, void* ctx) {
    level_sensor_     = fn;
    level_sensor_ctx_ = ctx;
}

bool Sampler::request_start() {
    if (state_ != TankState::EMPTY) return false;
    enter_state(TankState::SAMPLING);
    enter_step(TankStep::DESCENDING);
    return true;
}

void Sampler::abort() {
    if (state_ == TankState::FAULT) return;
    enter_state(TankState::FAULT);
    enter_step(TankStep::NONE);
}

bool Sampler::reset() {
    if (state_ == TankState::SAMPLING) return false;
    if (state_ == TankState::EMPTY)    return false;
    enter_state(TankState::EMPTY);
    enter_step(TankStep::NONE);
    return true;
}

void Sampler::tick() {
    if (state_ != TankState::SAMPLING) return;

    const uint32_t elapsed = elapsed_in_step();

    switch (step_) {
        case TankStep::DESCENDING:
            if (elapsed >= descending_ms_) enter_step(TankStep::IN_WATER);
            break;
        case TankStep::IN_WATER:
            if (elapsed >= mock_timing::IN_WATER_MS) enter_step(TankStep::PUMPING);
            break;
        case TankStep::PUMPING:
            if (level_sensor_) {
                // Real sensor wired: stop pumping as soon as the
                // sensor reports the tank is full. Safety cap so a
                // stuck/broken sensor can't keep the pump running
                // forever.
                const bool full = level_sensor_(id_, level_sensor_ctx_);
                if (full || elapsed >= pumping_timeout_ms_) {
                    enter_step(TankStep::ASCENDING);
                }
            } else {
                // No sensor: fall back to fixed mock duration.
                if (elapsed >= mock_timing::PUMPING_MS) {
                    enter_step(TankStep::ASCENDING);
                }
            }
            break;
        case TankStep::ASCENDING:
            if (elapsed >= ascending_ms_) enter_step(TankStep::HOME);
            break;
        case TankStep::HOME:
            if (elapsed >= mock_timing::HOME_MS) {
                enter_state(TankState::FULL);
                enter_step(TankStep::NONE);
            }
            break;
        case TankStep::NONE:
            // Unreachable while SAMPLING; treated as fault recovery.
            enter_state(TankState::FAULT);
            break;
    }
}

bool Sampler::consume_state_change() {
    if (!state_changed_) return false;
    state_changed_ = false;
    return true;
}

bool Sampler::consume_step_change() {
    if (!step_changed_) return false;
    step_changed_ = false;
    return true;
}

void Sampler::enter_state(TankState s) {
    if (state_ == s) return;
    state_ = s;
    state_changed_ = true;
}

void Sampler::enter_step(TankStep s) {
    if (step_ == s) return;
    step_ = s;
    step_entered_ms_ = clock_.now_ms();
    step_changed_ = true;
}

uint32_t Sampler::elapsed_in_step() const {
    return clock_.now_ms() - step_entered_ms_;
}

const char* state_name(TankState s) {
    switch (s) {
        case TankState::EMPTY:    return "EMPTY";
        case TankState::SAMPLING: return "SAMPLING";
        case TankState::FULL:     return "FULL";
        case TankState::FAULT:    return "FAULT";
    }
    return "?";
}

const char* step_name(TankStep s) {
    switch (s) {
        case TankStep::NONE:       return "NONE";
        case TankStep::DESCENDING: return "DESCENDING";
        case TankStep::IN_WATER:   return "IN_WATER";
        case TankStep::PUMPING:    return "PUMPING";
        case TankStep::ASCENDING:  return "ASCENDING";
        case TankStep::HOME:       return "HOME";
    }
    return "?";
}
