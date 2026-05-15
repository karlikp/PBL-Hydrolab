#include "MissionControl.h"

#include <stdio.h>
#include <string.h>

#include "RadioLink.h"
#include "Clock.h"

MissionControl::MissionControl(Clock& clock, RadioLink& radio)
    : clock_(clock),
      radio_(radio),
      tanks_{ Sampler(1, clock), Sampler(2, clock), Sampler(3, clock) } {}

void MissionControl::boot(const char* version) {
    emit_boot(version);
}

void MissionControl::tick() {
    for (auto& t : tanks_) t.tick();

    for (auto& t : tanks_) {
        if (t.consume_state_change()) emit_tank_state(t);
        if (t.consume_step_change())  emit_tank_step(t);
    }

    // When the last sampling tank finishes, drop back to IDLE.
    if (mode_ == SystemMode::SAMPLING && !any_tank_sampling()) {
        set_mode(SystemMode::IDLE);
    }

    if (mode_changed_) {
        mode_changed_ = false;
        emit_sys_state();
    }
}

void MissionControl::handle_command(const char* payload, size_t payload_len) {
    // Payload is "CMD,VERB,ARGS" — slice out the verb.
    if (payload_len < 5 || memcmp(payload, "CMD,", 4) != 0) return;

    const char* verb_begin = payload + 4;
    const char* end        = payload + payload_len;
    const char* verb_end   = verb_begin;
    while (verb_end < end && *verb_end != ',') ++verb_end;
    const size_t verb_len  = static_cast<size_t>(verb_end - verb_begin);

    // Null-terminated copy of the verb for snprintf in responses.
    char verb[24] = {0};
    if (verb_len == 0 || verb_len >= sizeof(verb)) {
        emit_nack("?", "unknown_command");
        return;
    }
    memcpy(verb, verb_begin, verb_len);

    auto matches = [&](const char* expected) -> bool {
        const size_t n = strlen(expected);
        return verb_len == n && memcmp(verb_begin, expected, n) == 0;
    };

    // E-STOP gate: while latched, refuse anything except RESET_*,
    // STATUS, PING, and (idempotently) E_STOP itself.
    if (mode_ == SystemMode::E_STOP
        && !matches("RESET_C1") && !matches("RESET_C2") && !matches("RESET_C3")
        && !matches("STATUS")   && !matches("PING")     && !matches("E_STOP")) {
        emit_nack(verb, "e_stop_active");
        return;
    }

    if      (matches("START_C1"))       cmd_start_tank(0, verb);
    else if (matches("START_C2"))       cmd_start_tank(1, verb);
    else if (matches("START_C3"))       cmd_start_tank(2, verb);
    else if (matches("START_ELMETRON")) emit_nack(verb, "not_implemented");
    else if (matches("E_STOP"))         cmd_estop(verb);
    else if (matches("RESET_C1"))       cmd_reset_tank(0, verb);
    else if (matches("RESET_C2"))       cmd_reset_tank(1, verb);
    else if (matches("RESET_C3"))       cmd_reset_tank(2, verb);
    else if (matches("STATUS"))         cmd_status(verb);
    else if (matches("PING"))           cmd_ping(verb);
    else                                 emit_nack(verb, "unknown_command");
}

void MissionControl::cmd_start_tank(uint8_t idx, const char* verb) {
    Sampler& t = tanks_[idx];

    if (any_tank_sampling())              { emit_nack(verb, "busy");      return; }
    if (t.state() == TankState::FULL)     { emit_nack(verb, "tank_full"); return; }
    if (t.state() == TankState::FAULT)    { emit_nack(verb, "fault");     return; }
    if (!t.request_start())               { emit_nack(verb, "busy");      return; }

    emit_ack(verb);
    set_mode(SystemMode::SAMPLING);
}

void MissionControl::cmd_estop(const char* verb) {
    emit_ack(verb);
    for (auto& t : tanks_) {
        if (t.state() == TankState::SAMPLING) t.abort();
    }
    set_mode(SystemMode::E_STOP);
}

void MissionControl::cmd_reset_tank(uint8_t idx, const char* verb) {
    Sampler& t = tanks_[idx];
    if (!t.reset()) {
        emit_nack(verb, t.state() == TankState::SAMPLING ? "busy" : "fault");
        return;
    }
    emit_ack(verb);

    // If E-STOP latched and there are no fault tanks left, drop to IDLE.
    if (mode_ == SystemMode::E_STOP && !any_tank_in_fault()) {
        set_mode(SystemMode::IDLE);
    }
}

void MissionControl::cmd_status(const char* verb) {
    emit_ack(verb);
    emit_sys_state();
    for (auto& t : tanks_) emit_tank_state(t);
    // Elmetron isn't implemented yet; report DOCKED placeholder so the
    // GCS has a complete snapshot to render.
    send_payload("EVT,ELMETRON,STATE,DOCKED");
}

void MissionControl::cmd_ping(const char* verb) {
    emit_ack(verb);
}

void MissionControl::set_mode(SystemMode m) {
    if (mode_ == m) return;
    mode_ = m;
    mode_changed_ = true;
}

bool MissionControl::any_tank_sampling() const {
    for (const auto& t : tanks_) {
        if (t.state() == TankState::SAMPLING) return true;
    }
    return false;
}

bool MissionControl::any_tank_in_fault() const {
    for (const auto& t : tanks_) {
        if (t.state() == TankState::FAULT) return true;
    }
    return false;
}

void MissionControl::emit_ack(const char* verb) {
    char buf[64];
    snprintf(buf, sizeof(buf), "EVT,SYS,ACK,%s", verb);
    send_payload(buf);
}

void MissionControl::emit_nack(const char* verb, const char* reason) {
    char buf[80];
    snprintf(buf, sizeof(buf), "EVT,SYS,NACK,%s:%s", verb, reason);
    send_payload(buf);
}

void MissionControl::emit_sys_state() {
    char buf[48];
    snprintf(buf, sizeof(buf), "EVT,SYS,STATE,%s", mode_name(mode_));
    send_payload(buf);
}

void MissionControl::emit_tank_state(Sampler& tank) {
    char buf[48];
    snprintf(buf, sizeof(buf), "EVT,%s,STATE,%s",
             tank_source(tank.id() - 1), state_name(tank.state()));
    send_payload(buf);
}

void MissionControl::emit_tank_step(Sampler& tank) {
    if (tank.step() == TankStep::NONE) return;
    char buf[48];
    snprintf(buf, sizeof(buf), "EVT,%s,STEP,%s",
             tank_source(tank.id() - 1), step_name(tank.step()));
    send_payload(buf);
}

void MissionControl::emit_boot(const char* version) {
    char buf[48];
    snprintf(buf, sizeof(buf), "EVT,SYS,BOOT,%s", version);
    send_payload(buf);
}

void MissionControl::send_payload(const char* payload) {
    radio_.send(payload);
}

const char* MissionControl::mode_name(SystemMode m) {
    switch (m) {
        case SystemMode::IDLE:      return "IDLE";
        case SystemMode::SAMPLING:  return "SAMPLING";
        case SystemMode::MEASURING: return "MEASURING";
        case SystemMode::E_STOP:    return "E_STOP";
    }
    return "?";
}

const char* MissionControl::tank_source(uint8_t idx) {
    switch (idx) {
        case 0: return "C1";
        case 1: return "C2";
        case 2: return "C3";
    }
    return "?";
}
