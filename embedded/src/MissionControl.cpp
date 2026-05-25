#include "MissionControl.h"

#include <stdio.h>
#include <string.h>
#include <Arduino.h>

#include "BatteryMonitor.h"
#include "RadioLink.h"
#include "Clock.h"
#include "ServoBus.h"
#include "PumpControl.h"

namespace {

// Per-tank winch endpoints, driven on Sampler step transitions.
// HOME = stowed/rolled-back, UNROLLED = fully extended for sampling.
// SC-09 position range is 0..1023; staying slightly inside the
// extremes avoids hitting mechanical end-stops. Tune these as the
// physical mechanism gets characterised — the firmware does no other
// magic, just commands the SC-09 to these absolute positions.
constexpr uint16_t SAMPLER_HOME_POSITION     = 0;
constexpr uint16_t SAMPLER_UNROLLED_POSITION = 1000;

}  // namespace

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
        if (t.consume_step_change()) {
            emit_tank_step(t);
            drive_servo_for_step(t);
            drive_pump_for_step(t);
        }
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
    else if (matches("ADC_SCAN"))       cmd_adc_scan(verb);
    else if (matches("SERVO_MOVE")) {
        const char* args = (verb_end < end) ? verb_end + 1 : verb_end;
        const size_t args_len = static_cast<size_t>(end - args);
        cmd_servo_move(verb, args, args_len);
    }
    else if (matches("SERVO_SET_ID")) {
        const char* args = (verb_end < end) ? verb_end + 1 : verb_end;
        const size_t args_len = static_cast<size_t>(end - args);
        cmd_servo_set_id(verb, args, args_len);
    }
    else if (matches("SERVO_BCAST_SET_ID")) {
        const char* args = (verb_end < end) ? verb_end + 1 : verb_end;
        const size_t args_len = static_cast<size_t>(end - args);
        cmd_servo_bcast_set_id(verb, args, args_len);
    }
    else if (matches("SERVO_PING")) {
        const char* args = (verb_end < end) ? verb_end + 1 : verb_end;
        const size_t args_len = static_cast<size_t>(end - args);
        cmd_servo_ping(verb, args, args_len);
    }
    else if (matches("PUMP")) {
        const char* args = (verb_end < end) ? verb_end + 1 : verb_end;
        const size_t args_len = static_cast<size_t>(end - args);
        cmd_pump(verb, args, args_len);
    }
    else if (matches("GPIO")) {
        const char* args = (verb_end < end) ? verb_end + 1 : verb_end;
        const size_t args_len = static_cast<size_t>(end - args);
        cmd_gpio(verb, args, args_len);
    }
    else                                 emit_nack(verb, "unknown_command");
}

// Diagnostic: read a handful of ADC1-capable GPIOs that aren't
// dedicated to other peripherals, emit one event per pin so we can
// spot which one tracks battery voltage. Used to pin down which GPIO
// the board's SUPADC net actually lands on.
void MissionControl::cmd_adc_scan(const char* verb) {
    emit_ack(verb);
    // Only scan pins that are NOT assigned to other peripherals in
    // the connector PDF (so we don't disturb running drivers). GPIO 4
    // is the power latch — also skipped.
    const int candidates[] = { 1, 2, 3, 5 };
    for (int pin : candidates) {
        analogSetPinAttenuation(pin, ADC_11db);
        pinMode(pin, INPUT);
        delay(2);
        const int raw = analogRead(pin);
        const uint32_t mv = analogReadMilliVolts(pin);
        char buf[80];
        snprintf(buf, sizeof(buf), "EVT,SYS,ADC,IO%d/raw=%d/mv=%lu",
                 pin, raw, static_cast<unsigned long>(mv));
        send_payload(buf);
    }
}

// CMD,SERVO_MOVE,<id>,<position> — sends a goal-position write to the
// servo bus. id=254 is broadcast (every servo on the bus moves).
// position is 0..1023.
void MissionControl::cmd_servo_move(const char* verb, const char* args, size_t args_len) {
    if (!servo_bus_) {
        emit_nack(verb, "no_servo_bus");
        return;
    }

    // Parse "<id>,<pos>" — copy to a stack buffer for atoi.
    char buf[32] = {0};
    if (args_len == 0 || args_len >= sizeof(buf)) {
        emit_nack(verb, "bad_args");
        return;
    }
    memcpy(buf, args, args_len);

    char* comma = strchr(buf, ',');
    if (!comma) {
        emit_nack(verb, "bad_args");
        return;
    }
    *comma = '\0';
    const int id = atoi(buf);
    const int pos = atoi(comma + 1);
    if (id < 0 || id > 254 || pos < 0 || pos > 1023) {
        emit_nack(verb, "out_of_range");
        return;
    }

    servo_bus_->move(static_cast<uint8_t>(id), static_cast<uint16_t>(pos));
    char ack[64];
    snprintf(ack, sizeof(ack), "EVT,SYS,SERVO_MOVE,id=%d,pos=%d", id, pos);
    send_payload(ack);
    emit_ack(verb);
}

// CMD,SERVO_SET_ID,<current_id>,<new_id> — re-assign one servo's ID.
// Run with ONLY that one servo connected to the bus (fresh servos all
// share ID=1, so daisy-chaining would collide).
void MissionControl::cmd_servo_set_id(const char* verb, const char* args, size_t args_len) {
    if (!servo_bus_) {
        emit_nack(verb, "no_servo_bus");
        return;
    }

    char buf[32] = {0};
    if (args_len == 0 || args_len >= sizeof(buf)) {
        emit_nack(verb, "bad_args");
        return;
    }
    memcpy(buf, args, args_len);

    char* comma = strchr(buf, ',');
    if (!comma) {
        emit_nack(verb, "bad_args");
        return;
    }
    *comma = '\0';
    const int cur = atoi(buf);
    const int nxt = atoi(comma + 1);
    if (cur < 1 || cur > 253 || nxt < 1 || nxt > 253) {
        emit_nack(verb, "out_of_range");  // 0xFE = broadcast, 0xFF = invalid
        return;
    }

    const bool ok = servo_bus_->set_id(static_cast<uint8_t>(cur),
                                       static_cast<uint8_t>(nxt));
    if (!ok) {
        emit_nack(verb, "set_id_failed");
        return;
    }

    char ack[64];
    snprintf(ack, sizeof(ack), "EVT,SYS,SERVO_SET_ID,from=%d,to=%d", cur, nxt);
    send_payload(ack);
    emit_ack(verb);
}

// CMD,SERVO_BCAST_SET_ID,<new_id> — broadcast write to the ID register
// of EVERY servo on the bus. No replies expected. Use to collapse
// multiple unknown-ID servos to a known shared ID, as a setup move
// for combinatorial unique-ID assignment.
void MissionControl::cmd_servo_bcast_set_id(const char* verb, const char* args, size_t args_len) {
    if (!servo_bus_) {
        emit_nack(verb, "no_servo_bus");
        return;
    }

    char buf[16] = {0};
    if (args_len == 0 || args_len >= sizeof(buf)) {
        emit_nack(verb, "bad_args");
        return;
    }
    memcpy(buf, args, args_len);
    const int nxt = atoi(buf);
    if (nxt < 1 || nxt > 253) {
        emit_nack(verb, "out_of_range");
        return;
    }

    servo_bus_->broadcast_set_id(static_cast<uint8_t>(nxt));
    char ack[64];
    snprintf(ack, sizeof(ack), "EVT,SYS,SERVO_BCAST_SET_ID,to=%d", nxt);
    send_payload(ack);
    emit_ack(verb);
}

// CMD,SERVO_PING,<id> — best-effort ping to confirm a servo is present.
// Returns EVT,SYS,SERVO_PING,id=<id>,reply=<id-or--1>.
void MissionControl::cmd_servo_ping(const char* verb, const char* args, size_t args_len) {
    if (!servo_bus_) {
        emit_nack(verb, "no_servo_bus");
        return;
    }

    char buf[16] = {0};
    if (args_len == 0 || args_len >= sizeof(buf)) {
        emit_nack(verb, "bad_args");
        return;
    }
    memcpy(buf, args, args_len);
    const int id = atoi(buf);
    if (id < 1 || id > 253) {
        emit_nack(verb, "out_of_range");
        return;
    }

    const int reply = servo_bus_->ping(static_cast<uint8_t>(id));
    char ev[64];
    snprintf(ev, sizeof(ev), "EVT,SYS,SERVO_PING,id=%d,reply=%d", id, reply);
    send_payload(ev);
    emit_ack(verb);
}

// CMD,GPIO,<pin>,<level> — drive an arbitrary GPIO HIGH or LOW.
// Diagnostic only — used to experimentally find which pin is TXEN by
// driving candidate GPIOs HIGH and seeing if the servo bus comes
// alive. Don't use this on production-assigned pins (it'll fight
// the actual drivers).
void MissionControl::cmd_gpio(const char* verb, const char* args, size_t args_len) {
    char buf[32] = {0};
    if (args_len == 0 || args_len >= sizeof(buf)) {
        emit_nack(verb, "bad_args");
        return;
    }
    memcpy(buf, args, args_len);
    char* comma = strchr(buf, ',');
    if (!comma) {
        emit_nack(verb, "bad_args");
        return;
    }
    *comma = '\0';
    const int pin = atoi(buf);
    const int lvl = atoi(comma + 1);
    if (pin < 0 || pin > 48) {
        emit_nack(verb, "bad_pin");
        return;
    }
    pinMode(pin, OUTPUT);
    digitalWrite(pin, lvl ? HIGH : LOW);
    char ack[64];
    snprintf(ack, sizeof(ack), "EVT,SYS,GPIO,pin=%d,level=%d", pin, lvl ? 1 : 0);
    send_payload(ack);
    emit_ack(verb);
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
    // Safety: force every pump off immediately, regardless of where
    // each Sampler was in its FSM. The step-change hook would do this
    // anyway as tanks transition to FAULT, but the explicit stop_all
    // is a belt-and-braces guard against any tank that was already
    // outside the SAMPLING path.
    if (pump_control_) pump_control_->stop_all();
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
    if (battery_) {
        char buf[80];
        snprintf(buf, sizeof(buf), "EVT,SYS,BATTERY,%.2fV/raw=%u/mv=%lu",
                 battery_->last_voltage(),
                 battery_->last_raw(),
                 static_cast<unsigned long>(battery_->last_mv()));
        send_payload(buf);
    }
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

// CMD,PUMP,<id>,<state> — turn a single pump on (state=1) or off
// (state=0). Used for bench testing the pump GPIOs directly; the
// Sampler FSM drives pumps automatically during the PUMPING step.
void MissionControl::cmd_pump(const char* verb, const char* args, size_t args_len) {
    if (!pump_control_) {
        emit_nack(verb, "no_pump_control");
        return;
    }

    char buf[16] = {0};
    if (args_len == 0 || args_len >= sizeof(buf)) {
        emit_nack(verb, "bad_args");
        return;
    }
    memcpy(buf, args, args_len);

    char* comma = strchr(buf, ',');
    if (!comma) {
        emit_nack(verb, "bad_args");
        return;
    }
    *comma = '\0';
    const int id    = atoi(buf);
    const int state = atoi(comma + 1);
    if (id < 1 || id > 3 || (state != 0 && state != 1)) {
        emit_nack(verb, "out_of_range");
        return;
    }

    pump_control_->set(static_cast<uint8_t>(id), state != 0);
    char ack[64];
    snprintf(ack, sizeof(ack), "EVT,SYS,PUMP,id=%d,state=%d", id, state);
    send_payload(ack);
    emit_ack(verb);
}

// Wire Sampler step transitions to the per-tank servo. Sampler IDs
// 1/2/3 line up 1:1 with servo IDs 1/2/3 (the IDs we programmed via
// CMD,SERVO_SET_ID). Only DESCENDING and ASCENDING steps actually
// move the servo; the IN_WATER / PUMPING / HOME steps are pure mock
// dwells while the rest of the mechanism (pump, end-stops) lands.
void MissionControl::drive_servo_for_step(const Sampler& tank) {
    if (!servo_bus_) return;
    switch (tank.step()) {
        case TankStep::DESCENDING:
            servo_bus_->move(tank.id(), SAMPLER_UNROLLED_POSITION);
            break;
        case TankStep::ASCENDING:
            servo_bus_->move(tank.id(), SAMPLER_HOME_POSITION);
            break;
        case TankStep::IN_WATER:
        case TankStep::PUMPING:
        case TankStep::HOME:
        case TankStep::NONE:
            break;  // no servo motion at these steps
    }
}

// Drive the pump for `tank` based on its current step. Pump on
// exactly when the tank is in PUMPING; off for every other step
// (including NONE on FAULT). Idempotent — safe to call on every
// step change.
void MissionControl::drive_pump_for_step(const Sampler& tank) {
    if (!pump_control_) return;
    const bool should_pump = (tank.step() == TankStep::PUMPING);
    pump_control_->set(tank.id(), should_pump);
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
