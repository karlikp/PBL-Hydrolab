#include "MissionControl.h"

#include <stdio.h>
#include <string.h>
#include <Arduino.h>

#include "BatteryMonitor.h"
#include "RadioLink.h"
#include "Clock.h"
#include "ServoBus.h"
#include "PumpControl.h"
#include "LevelSensor.h"
#include "GpsLink.h"
#include "Telemetry.h"
#include "ElmetronProbe.h"
#include "WinchH.h"

#include <driver/gpio.h>

namespace {

// Per-tank winch endpoints (SC-09 position 0..1023, HOME = stowed,
// UNROLLED = deployed) are now per-tank config (TankConfig::servo_home
// / servo_unrolled), provisioned + calibrated from the GCS. The
// TankConfig NSDMI defaults (0 / 1000) preserve the old hardcoded
// behaviour until the operator tunes them.

// Absolute ceilings on the provisioned Elmetron timeouts (seconds).
// The descent cap is the mechanical safety bound the operator flagged
// as non-negotiable — config can SHORTEN it but not raise it past this,
// so a bad config can't disable the protection.
constexpr uint16_t DESCENT_CEILING_S = 30;
constexpr uint16_t ASCENT_CEILING_S  = 30;
constexpr uint16_t HOMING_CEILING_S  = 30;
constexpr uint16_t MEASURE_CEILING_S = 600;

}  // namespace

MissionControl::MissionControl(Clock& clock, RadioLink& radio)
    : clock_(clock),
      radio_(radio),
      tanks_{ Sampler(1, clock), Sampler(2, clock), Sampler(3, clock) } {
    // Default config = historical 1:1 wiring (tank N → channel/servo N),
    // so an un-provisioned board behaves exactly as before the config
    // layer existed.
    for (uint8_t i = 0; i < 3; ++i) {
        config_.tanks[i].enabled  = true;
        config_.tanks[i].channel  = static_cast<uint8_t>(i + 1);
        config_.tanks[i].servo_id = static_cast<uint8_t>(i + 1);
    }
    apply_config();
}

// Bridge from Sampler's typed callback into MissionControl. The ctx
// carries the MissionControl* so the probe can map the logical tank id
// to its provisioned physical sensor channel before reading.
static bool sampler_level_probe(uint8_t tank_id, void* ctx) {
    auto* mc = static_cast<MissionControl*>(ctx);
    return mc && mc->tank_full(tank_id);
}

void MissionControl::set_level_sensor(LevelSensor* ls) {
    level_sensor_ = ls;
    for (auto& tank : tanks_) {
        tank.set_level_sensor(ls ? &sampler_level_probe : nullptr, this);
    }
}

// Map logical tank (1..3) → provisioned sensor channel, then read it.
bool MissionControl::tank_full(uint8_t tank_id) {
    if (!level_sensor_ || tank_id < 1 || tank_id > 3) return false;
    return level_sensor_->is_full(config_.tanks[tank_id - 1].channel);
}

void MissionControl::apply_config() {
    // Grace period after the winch should be done before the Sampler
    // transitions out of DESCENDING/ASCENDING — covers timing jitter
    // and lets the PWM=0 coast settle.
    constexpr uint32_t WINCH_GRACE_MS = 500;
    for (uint8_t i = 0; i < 3; ++i) {
        tanks_[i].set_pumping_timeout_ms(config_.pumping_timeout_ms);
        tanks_[i].set_descending_ms(config_.tanks[i].winch_unroll_ms + WINCH_GRACE_MS);
        tanks_[i].set_ascending_ms (config_.tanks[i].winch_roll_ms   + WINCH_GRACE_MS);
    }
    // Put each ENABLED tank's servo into PWM/wheel mode. Writes
    // EEPROM angle limits to 0/0; persists across power cycles, so
    // this is idempotent on subsequent boots. No-op if the servo bus
    // isn't attached yet (set_servo_bus re-runs apply_config).
    if (servo_bus_) {
        for (uint8_t i = 0; i < 3; ++i) {
            if (config_.tanks[i].enabled) {
                servo_bus_->set_pwm_mode(config_.tanks[i].servo_id);
            }
        }
    }
}

void MissionControl::boot(const char* version) {
    emit_boot(version);
}

void MissionControl::tick() {
    for (auto& t : tanks_) t.tick();
    service_winches();

    for (size_t i = 0; i < 3; ++i) {
        auto& t = tanks_[i];
        if (t.consume_state_change()) {
            emit_tank_state(t);
            // Capture geo-tag the instant a tank reaches FULL. GPS
            // poll is 1 Hz so the fix may be up to ~1 s stale; for
            // hydrology-scale sample provenance that's well below GPS
            // accuracy. fix_valid=false captures the (rare) case where
            // a collection finished before the module had a lock.
            if (t.state() == TankState::FULL) {
                capture_collection(static_cast<uint8_t>(i));
                emit_collection(static_cast<uint8_t>(i));
            }
        }
        if (t.consume_step_change()) {
            emit_tank_step(t);
            drive_servo_for_step(t);
            drive_pump_for_step(t);
        }
    }

    if (elmetron_) {
        // Feed hardware-derived triggers (limit switch / water / converge)
        // BEFORE ticking so the FSM can act on them this cycle.
        service_elmetron_hardware();
        elmetron_->tick();
        if (elmetron_->consume_state_change()) emit_elmetron_state();
        if (elmetron_->consume_step_change()) {
            emit_elmetron_step();
            // Reset the convergence window each time we (re)enter IN_WATER.
            if (elmetron_->step() == ElmetronStep::IN_WATER) conv_.reset();
            drive_elmetron_winch_for_step();
        }
        push_elmetron_reading();
    }

    // When the last sampling tank finishes, drop back to IDLE.
    if (mode_ == SystemMode::SAMPLING && !any_tank_sampling()) {
        set_mode(SystemMode::IDLE);
    }
    // When the Elmetron cycle finishes, drop back to IDLE.
    if (mode_ == SystemMode::MEASURING && !elmetron_measuring()) {
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
    // STATUS, PING, and (idempotently) E_STOP itself. Per-subsystem
    // STOPs are also refused — once the whole system is latched, you
    // can't selectively un-fault things; full reset path applies.
    if (mode_ == SystemMode::E_STOP
        && !matches("RESET_C1") && !matches("RESET_C2") && !matches("RESET_C3")
        && !matches("RESET_ELMETRON")
        && !matches("STATUS")   && !matches("PING")     && !matches("E_STOP")
        && !matches("CFG")      && !matches("CFG_GET")
        && !matches("CFG_ELE")  && !matches("CFG_ELE_GET")) {
        emit_nack(verb, "e_stop_active");
        return;
    }

    if      (matches("START_C1"))       cmd_start_tank(0, verb);
    else if (matches("START_C2"))       cmd_start_tank(1, verb);
    else if (matches("START_C3"))       cmd_start_tank(2, verb);
    else if (matches("START_ELMETRON")) cmd_start_elmetron(verb);
    else if (matches("E_STOP"))         cmd_estop(verb);
    else if (matches("STOP_C1"))        cmd_stop_tank(0, verb);
    else if (matches("STOP_C2"))        cmd_stop_tank(1, verb);
    else if (matches("STOP_C3"))        cmd_stop_tank(2, verb);
    else if (matches("STOP_ELMETRON"))  cmd_stop_elmetron(verb);
    else if (matches("JOG_C1_UP"))      cmd_jog(0, -1, verb);
    else if (matches("JOG_C1_DOWN"))    cmd_jog(0, +1, verb);
    else if (matches("JOG_C2_UP"))      cmd_jog(1, -1, verb);
    else if (matches("JOG_C2_DOWN"))    cmd_jog(1, +1, verb);
    else if (matches("JOG_C3_UP"))      cmd_jog(2, -1, verb);
    else if (matches("JOG_C3_DOWN"))    cmd_jog(2, +1, verb);
    else if (matches("RESET_C1"))       cmd_reset_tank(0, verb);
    else if (matches("RESET_C2"))       cmd_reset_tank(1, verb);
    else if (matches("RESET_C3"))       cmd_reset_tank(2, verb);
    else if (matches("RESET_ELMETRON")) cmd_reset_elmetron(verb);
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
    else if (matches("LEVEL")) {
        const char* args = (verb_end < end) ? verb_end + 1 : verb_end;
        const size_t args_len = static_cast<size_t>(end - args);
        cmd_level(verb, args, args_len);
    }
    else if (matches("LEVEL_DIAG")) {
        const char* args = (verb_end < end) ? verb_end + 1 : verb_end;
        const size_t args_len = static_cast<size_t>(end - args);
        cmd_level_diag(verb, args, args_len);
    }
    else if (matches("GPIO")) {
        const char* args = (verb_end < end) ? verb_end + 1 : verb_end;
        const size_t args_len = static_cast<size_t>(end - args);
        cmd_gpio(verb, args, args_len);
    }
    else if (matches("GPS")) {
        const char* args = (verb_end < end) ? verb_end + 1 : verb_end;
        const size_t args_len = static_cast<size_t>(end - args);
        cmd_gps(verb, args, args_len);
    }
    else if (matches("COLLECTIONS"))     cmd_collections(verb);
    else if (matches("ELE"))             cmd_ele(verb);
    else if (matches("CFG_GET"))         cmd_cfg_get(verb);
    else if (matches("CFG_ELE_GET"))     cmd_cfg_ele_get(verb);
    else if (matches("CFG_ELE")) {
        const char* args = (verb_end < end) ? verb_end + 1 : verb_end;
        const size_t args_len = static_cast<size_t>(end - args);
        cmd_cfg_ele(verb, args, args_len);
    }
    else if (matches("CFG")) {
        const char* args = (verb_end < end) ? verb_end + 1 : verb_end;
        const size_t args_len = static_cast<size_t>(end - args);
        cmd_cfg(verb, args, args_len);
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

    if (!config_.tanks[idx].enabled)      { emit_nack(verb, "disabled");  return; }
    if (is_busy())                        { emit_nack(verb, "busy");      return; }
    if (t.state() == TankState::FULL)     { emit_nack(verb, "tank_full"); return; }
    if (t.state() == TankState::FAULT)    { emit_nack(verb, "fault");     return; }
    if (!t.request_start())               { emit_nack(verb, "busy");      return; }

    emit_ack(verb);
    set_mode(SystemMode::SAMPLING);
}

void MissionControl::cmd_start_elmetron(const char* verb) {
    if (!elmetron_)                       { emit_nack(verb, "not_implemented"); return; }
    if (is_busy())                        { emit_nack(verb, "busy");            return; }
    if (elmetron_->state() == ElmetronState::FAULT) {
        emit_nack(verb, "fault");
        return;
    }
    if (!elmetron_->request_start())      { emit_nack(verb, "busy");            return; }

    emit_ack(verb);
    set_mode(SystemMode::MEASURING);
}

// CMD,STOP_Cx — abort one tank without latching the whole system to
// E_STOP. The tank goes to FAULT in place (servo frozen at its
// current physical position, pump drops off via the step-change
// hook). System mode settles back to IDLE on the next tick if
// nothing else is active — other tanks / Elmetron remain available
// to start once the user resets the faulted one. NACKs `not_running`
// if the tank isn't currently sampling, so the operator sees
// "nothing to stop" instead of putting an idle tank into FAULT.
void MissionControl::cmd_stop_tank(uint8_t idx, const char* verb) {
    Sampler& t = tanks_[idx];
    if (t.state() != TankState::SAMPLING) {
        emit_nack(verb, "not_running");
        return;
    }
    // Halt the winch immediately (PWM=0 → motor coasts). With the
    // SC-09 in wheel mode there's no holding torque on stop — if
    // bench-test shows the spool gravity-unrolls under load, this
    // is where we'd flip back into position mode and lock present-pos.
    stop_tank_winch(idx);
    t.abort();
    emit_ack(verb);
}

// CMD,STOP_ELMETRON — same semantics as STOP_Cx, for the Elmetron
// subsystem. Winch stops where it is (drive_elmetron_servo_for_step
// is a no-op outside DESCENDING/ASCENDING). System mode returns to
// IDLE on the next tick once Elmetron is no longer MEASURING.
void MissionControl::cmd_stop_elmetron(const char* verb) {
    if (!elmetron_) {
        emit_nack(verb, "not_implemented");
        return;
    }
    if (elmetron_->state() != ElmetronState::MEASURING) {
        emit_nack(verb, "not_running");
        return;
    }
    elmetron_->abort();
    emit_ack(verb);
}

void MissionControl::cmd_estop(const char* verb) {
    emit_ack(verb);
    for (auto& t : tanks_) {
        if (t.state() == TankState::SAMPLING) t.abort();
    }
    if (elmetron_ && elmetron_->state() == ElmetronState::MEASURING) {
        elmetron_->abort();
    }
    // Safety: halt the Elmetron winch immediately (the step-change hook
    // would also stop it as the FSM drops to NONE, but don't wait).
    if (winch_) winch_->stop();
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

    // RESET does NOT drive the winch. The autonomous rewind already
    // ran at the end of ASCENDING; if the spool isn't quite at home,
    // the operator uses the manual JOG buttons (15° per click, encoder-
    // gated) to position it. RESET = just clear state, ready for the
    // next sample. This avoids the over-wind that the cumulative-based
    // rewind safety cap caused when ReadPos was unreliable.

    // If E-STOP latched and there are no fault tanks left, drop to IDLE.
    // The Elmetron is implicitly cleared at the same time — the
    // protocol doesn't expose a RESET_ELMETRON, and FAULT today only
    // happens via E-STOP, so the tank-reset stream is the recovery
    // path for the whole system.
    if (mode_ == SystemMode::E_STOP && !any_tank_in_fault()) {
        if (elmetron_ && elmetron_->state() == ElmetronState::FAULT) {
            elmetron_->reset();
        }
        set_mode(SystemMode::IDLE);
    }
}

// CMD,ELE — dump the latest raw Elmetron probe reading. Diagnostic for
// bench bring-up of the CX-series UART before the FSM consumes it.
//   EVT,SYS,ELE,cond=<mS/cm>,temp=<C>,ph=<>,o2=<mg/L>,water=0|1,present=0|1
void MissionControl::cmd_ele(const char* verb) {
    if (!elmetron_probe_) { emit_nack(verb, "no_probe"); return; }
    const ElmetronProbe::Reading& r = elmetron_probe_->last_reading();
    char buf[128];
    snprintf(buf, sizeof(buf),
        "EVT,SYS,ELE,cond=%.2f,temp=%.2f,ph=%.2f,o2=%.2f,water=%d,present=%d",
        r.conductivity, r.temperature, r.ph, r.oxygen,
        r.water ? 1 : 0, elmetron_probe_->present() ? 1 : 0);
    send_payload(buf);
    emit_ack(verb);
}

// CMD,RESET_ELMETRON — clear Elmetron FAULT back to DOCKED. Needed
// now that STOP_ELMETRON can fault Elmetron without latching the
// whole system in E_STOP (the E-STOP recovery path is still the
// implicit way to clear Elmetron when the system itself was latched).
//
// Like RESET_Cx, the intent is "back to init position." The H-bridge
// winch is open-loop with no absolute position feedback — its only
// reference is H_LIMIT. When the WinchH driver lands, RESET should
// drive UP until H_LIMIT trips (same homing routine the HOMING step
// uses on START_ELMETRON), with a hard time bound and FAULT-on-
// timeout. Until then, RESET only flips state and the winch stays
// where STOP left it physically.
void MissionControl::cmd_reset_elmetron(const char* verb) {
    if (!elmetron_) {
        emit_nack(verb, "not_implemented");
        return;
    }
    if (!elmetron_->reset()) {
        // reset() refuses anywhere except FAULT — surface why.
        emit_nack(verb,
            elmetron_->state() == ElmetronState::MEASURING ? "busy" : "not_in_fault");
        return;
    }
    emit_ack(verb);
    // TODO(winch-h): drive UP until H_LIMIT to restore init position.
    // Without it, the operator gets "fault cleared, but winch is still
    // wherever STOP froze it" — they'd need to START a measurement
    // to trigger HOMING. Once the H-bridge driver exists, add the
    // homing call here so RESET matches the tank-side behaviour.
}

void MissionControl::cmd_status(const char* verb) {
    emit_ack(verb);
    emit_sys_state();
    for (auto& t : tanks_) emit_tank_state(t);
    // Per-tank geo-tag history (only tanks that have ever been
    // collected this session). Lets a reconnecting GCS rebuild the
    // collection table without having to replay the EVT log.
    for (uint8_t i = 0; i < 3; ++i) {
        if (collections_[i].valid) emit_collection(i);
    }
    // Live Elmetron state if attached, else the historical DOCKED
    // placeholder so the GCS snapshot is always complete.
    if (elmetron_) {
        emit_elmetron_state();
    } else {
        send_payload("EVT,ELMETRON,STATE,DOCKED");
    }
    // Active provisioned config, so a reconnecting GCS resyncs its
    // provisioning status from a plain STATUS without a separate CFG_GET.
    emit_cfg();
    emit_cfg_ele();
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

bool MissionControl::elmetron_measuring() const {
    return elmetron_ && elmetron_->state() == ElmetronState::MEASURING;
}

bool MissionControl::elmetron_in_fault() const {
    return elmetron_ && elmetron_->state() == ElmetronState::FAULT;
}

bool MissionControl::is_busy() const {
    return any_tank_sampling() || elmetron_measuring();
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

void MissionControl::emit_elmetron_state() {
    if (!elmetron_) return;
    char buf[48];
    snprintf(buf, sizeof(buf), "EVT,ELMETRON,STATE,%s",
             state_name(elmetron_->state()));
    send_payload(buf);
}

void MissionControl::emit_elmetron_step() {
    if (!elmetron_ || elmetron_->step() == ElmetronStep::NONE) return;
    char buf[48];
    snprintf(buf, sizeof(buf), "EVT,ELMETRON,STEP,%s",
             step_name(elmetron_->step()));
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

// CMD,LEVEL_DIAG,<id> — probe the level-sensor GPIO under three
// different pull configurations to figure out what is actually driving
// the line.
//
// Emits  EVT,SYS,LEVEL_DIAG,id=N,nopull=A,pullup=B,pulldown=C
//
// How to read the result (assume sensor connected and we ran this in
// one state — say, dry):
//
//   A=0, B=1, C=0  -> nothing driving the line; reading is determined
//                     entirely by the internal pull. Sensor is in
//                     high-Z or disconnected.
//   A=1, B=1, C=1  -> something is ACTIVELY driving the line HIGH
//                     (~5 V); our internal ~45 kΩ pulls can't fight
//                     it. If this stays the same wet vs. dry, the
//                     sensor is stuck (not switching) or something
//                     other than the sensor is asserting HIGH.
//   A=0, B=0, C=0  -> something is actively driving the line LOW.
//   A=1, B=1, C=0  -> conflicting; usually means a weak external
//                     drive that pull-down can fight but pull-up
//                     reinforces. Hardware issue worth poking.
//
// After the probe the pin is restored to LevelSensor's default
// configuration so subsequent CMD,LEVEL calls work as before.
void MissionControl::cmd_level_diag(const char* verb,
                                    const char* args,
                                    size_t args_len) {
    if (!level_sensor_) {
        emit_nack(verb, "no_level_sensor");
        return;
    }
    char buf[8] = {0};
    if (args_len == 0 || args_len >= sizeof(buf)) {
        emit_nack(verb, "bad_args");
        return;
    }
    memcpy(buf, args, args_len);
    const int id = atoi(buf);
    if (id < 1 || id > 3) {
        emit_nack(verb, "out_of_range");
        return;
    }

    // Match LevelSensor's pin layout.
    int pin = -1;
    switch (id) {
        case 1: pin = LevelSensor::TOPCN1_PIN; break;
        case 2: pin = LevelSensor::TOPCN2_PIN; break;
        case 3: pin = LevelSensor::TOPCN3_PIN; break;
    }

    auto probe = [pin](gpio_pullup_t pu, gpio_pulldown_t pd) -> int {
        gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << pin,
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = pu,
            .pull_down_en = pd,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&cfg);
        delay(5);                // let the line settle
        return digitalRead(pin);
    };

    const int a = probe(GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE);  // no pull
    const int b = probe(GPIO_PULLUP_ENABLE,  GPIO_PULLDOWN_DISABLE);  // pull-up
    const int c = probe(GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_ENABLE);   // pull-down

    // Restore to LevelSensor's preferred config (pull-down).
    probe(GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_ENABLE);

    char ev[96];
    snprintf(ev, sizeof(ev),
             "EVT,SYS,LEVEL_DIAG,id=%d,nopull=%d,pullup=%d,pulldown=%d",
             id, a, b, c);
    send_payload(ev);
    emit_ack(verb);
}

// CMD,LEVEL,<id> — read the FS-IR12 optical level sensor for tank
// <id> and emit `EVT,SYS,LEVEL,id=N,full=0|1,raw=0|1`. The `full`
// field is the interpreted state (after ACTIVE_LOW inversion);
// `raw` is the actual digitalRead value so the operator can tell a
// missing/disconnected sensor (reads HIGH=1 due to the external
// pull-up on the PCB) apart from a sensor reading "not full".
void MissionControl::cmd_level(const char* verb, const char* args, size_t args_len) {
    if (!level_sensor_) {
        emit_nack(verb, "no_level_sensor");
        return;
    }

    char buf[8] = {0};
    if (args_len == 0 || args_len >= sizeof(buf)) {
        emit_nack(verb, "bad_args");
        return;
    }
    memcpy(buf, args, args_len);
    const int id = atoi(buf);
    if (id < 1 || id > 3) {
        emit_nack(verb, "out_of_range");
        return;
    }

    const bool full = level_sensor_->is_full(static_cast<uint8_t>(id));
    const int  raw  = level_sensor_->raw(static_cast<uint8_t>(id));
    char ev[64];
    snprintf(ev, sizeof(ev),
             "EVT,SYS,LEVEL,id=%d,full=%d,raw=%d",
             id, full ? 1 : 0, raw);
    send_payload(ev);
    emit_ack(verb);
}

// Wire Sampler step transitions to the per-tank servo. DESCENDING
// starts an unroll PWM run; ASCENDING starts a rewind. Both are pure
// time-based with separate durations (winch_unroll_ms / winch_roll_ms),
// so unroll/rewind speed asymmetry is handled by the operator dialing
// in different times per direction. IN_WATER/PUMPING/HOME are dwells —
// the winch finishes on its own via service_winches.
void MissionControl::drive_servo_for_step(const Sampler& tank) {
    const uint8_t idx = tank.id() - 1;
    switch (tank.step()) {
        case TankStep::DESCENDING: start_winch_unroll(idx); break;
        case TankStep::ASCENDING:  start_winch_rewind(idx); break;
        case TankStep::IN_WATER:
        case TankStep::PUMPING:
        case TankStep::HOME:
        case TankStep::NONE:
            break;  // no winch motion at these steps
    }
}

void MissionControl::start_winch_unroll(uint8_t idx) {
    if (!servo_bus_ || idx >= 3) return;
    const TankConfig& tc = config_.tanks[idx];
    servo_bus_->write_pwm(tc.servo_id, tc.winch_pwm);
    winch_active_[idx]     = true;
    winch_stop_at_ms_[idx] = clock_.now_ms() + tc.winch_unroll_ms;
}

void MissionControl::start_winch_rewind(uint8_t idx) {
    if (!servo_bus_ || idx >= 3) return;
    const TankConfig& tc = config_.tanks[idx];
    // Drive in the OPPOSITE direction of unroll.
    servo_bus_->write_pwm(tc.servo_id, static_cast<int16_t>(-tc.winch_pwm));
    winch_active_[idx]     = true;
    winch_stop_at_ms_[idx] = clock_.now_ms() + tc.winch_roll_ms;
}

// CMD,JOG_C{n}_{UP,DOWN} — manual one-shot jog used to position the
// spool by hand (e.g. homing it before sampling). Drives PWM for a
// fixed short duration; operator clicks again for more travel.
//
// sign convention matches start_winch_*: +1 = unroll direction (DOWN),
// -1 = rewind direction (UP). "UP/DOWN" labels follow the marine-winch
// convention (line drops on unroll, rises on rewind).
void MissionControl::cmd_jog(uint8_t idx, int8_t sign, const char* verb) {
    if (is_busy())                              { emit_nack(verb, "busy");          return; }
    if (!servo_bus_)                            { emit_nack(verb, "no_servo_bus");  return; }
    if (idx >= 3 || !config_.tanks[idx].enabled){ emit_nack(verb, "tank_disabled"); return; }
    if (winch_active_[idx])                     { emit_nack(verb, "winch_busy");    return; }

    constexpr uint32_t JOG_MS = 300;       // one click ≈ half a rev at default pwm
    const TankConfig& tc = config_.tanks[idx];
    const int16_t pwm = static_cast<int16_t>(sign) * tc.winch_pwm;
    servo_bus_->write_pwm(tc.servo_id, pwm);
    winch_active_[idx]     = true;
    winch_stop_at_ms_[idx] = clock_.now_ms() + JOG_MS;
    emit_ack(verb);
}

// Per-tick: stop any winch whose configured duration has elapsed.
void MissionControl::service_winches() {
    if (!servo_bus_) return;
    const uint32_t now = clock_.now_ms();
    for (uint8_t i = 0; i < 3; ++i) {
        if (!winch_active_[i]) continue;
        if ((int32_t)(now - winch_stop_at_ms_[i]) >= 0) {
            servo_bus_->stop_pwm(config_.tanks[i].servo_id);
            winch_active_[i] = false;
        }
    }
}

// Halt this tank's winch immediately (PWM=0 = coast). Used by STOP.
void MissionControl::stop_tank_winch(uint8_t idx) {
    if (!servo_bus_ || idx >= 3) return;
    servo_bus_->stop_pwm(config_.tanks[idx].servo_id);
    winch_active_[idx] = false;
}

// Drive the pump for `tank` based on its current step. Pump on
// exactly when the tank is in PUMPING; off for every other step
// (including NONE on FAULT). Idempotent — safe to call on every
// step change.
void MissionControl::drive_pump_for_step(const Sampler& tank) {
    if (!pump_control_) return;
    const bool should_pump = (tank.step() == TankStep::PUMPING);
    pump_control_->set(config_.tanks[tank.id() - 1].channel, should_pump);
}

// Recompute Elmetron hardware mode: it's hardware-driven only when BOTH
// a real probe and a winch are attached. Otherwise the FSM runs its
// synthetic timer/ramp path so the cycle stays exercisable on the
// dev-kit / mock builds.
void MissionControl::update_elmetron_mode() {
    if (elmetron_) elmetron_->set_hardware_mode(elmetron_probe_ && winch_);
    // Push the current Elmetron config to whatever just attached.
    apply_elmetron_config();
}

void MissionControl::apply_elmetron_config() {
    if (elmetron_probe_) {
        elmetron_probe_->set_water_threshold_ms(ele_config_.water_threshold_ms);
    }
    if (winch_) {
        winch_->set_direction_invert(ele_config_.winch_dir_invert);
        winch_->set_limit_active_low(ele_config_.limit_active_low);
    }
    if (elmetron_) {
        elmetron_->set_hw_timeouts(
            static_cast<uint32_t>(ele_config_.descent_timeout_s) * 1000u,
            static_cast<uint32_t>(ele_config_.ascent_timeout_s)  * 1000u,
            static_cast<uint32_t>(ele_config_.homing_timeout_s)  * 1000u,
            static_cast<uint32_t>(ele_config_.measure_timeout_s) * 1000u);
    }
    conv_.set_params(
        static_cast<uint32_t>(ele_config_.convergence_window_s) * 1000u,
        static_cast<float>(ele_config_.convergence_tol_pct) / 100.0f);
}

// Drive the winch motor for the current Elmetron step. Set-and-forget:
// the H-bridge holds the last command until the next step change, so we
// only act on transitions. STOP on IN_WATER / HOME / NONE (NONE covers
// abort + FAULT, so the motor always halts when the cycle ends).
void MissionControl::drive_elmetron_winch_for_step() {
    if (!winch_ || !elmetron_) return;
    switch (elmetron_->step()) {
        case ElmetronStep::HOMING:
        case ElmetronStep::ASCENDING:
            winch_->drive(WinchH::Direction::UP, ele_config_.winch_duty_pct);
            break;
        case ElmetronStep::DESCENDING:
            winch_->drive(WinchH::Direction::DOWN, ele_config_.winch_duty_pct);
            break;
        case ElmetronStep::IN_WATER:
        case ElmetronStep::HOME:
        case ElmetronStep::NONE:
            winch_->stop();
            break;
    }
}

// Read the winch limit + probe each tick and fire the FSM's notify
// triggers for the current step. No-op unless we're in hardware mode
// and actively measuring.
void MissionControl::service_elmetron_hardware() {
    if (!elmetron_ || !winch_ || !elmetron_probe_) return;
    if (!elmetron_->hardware_mode()) return;
    if (elmetron_->state() != ElmetronState::MEASURING) return;

    switch (elmetron_->step()) {
        case ElmetronStep::HOMING:
        case ElmetronStep::ASCENDING:
            if (winch_->at_home()) elmetron_->at_home();
            break;
        case ElmetronStep::DESCENDING:
            if (elmetron_probe_->last_reading().water) elmetron_->water_detected();
            break;
        case ElmetronStep::IN_WATER:
            // Sample the conductivity into the convergence window only on
            // genuinely-new probe frames (~2 Hz), not every loop tick.
            if (elmetron_probe_->consume_changed()) {
                conv_.add(clock_.now_ms(),
                          elmetron_probe_->last_reading().conductivity);
            }
            if (conv_.converged(clock_.now_ms())) elmetron_->measurement_done();
            break;
        case ElmetronStep::HOME:
        case ElmetronStep::NONE:
            break;
    }
}

// Push the Elmetron's current reading into Telemetry. While the
// probe is in MEASURING the synthetic ramp (or, later, a real
// driver) drives cond/temp/ph/oxygen + measurement_valid; in every
// other state we explicitly clear the probe-side TLM fields so the
// stream doesn't carry a stale post-measurement value indefinitely.
void MissionControl::push_elmetron_reading() {
    if (!telemetry_ || !elmetron_) return;

    // Real probe attached: stream its live readings into the heartbeat
    // ALWAYS (even idle/docked), so the operator can see at a glance
    // whether the probe is answering. measurement_valid flips true only
    // once we're settled in water (IN_WATER + converged) — the GCS
    // only logs to the measurements table while MEASURING anyway, so
    // idle readings show in the live panel without polluting the DB.
    if (elmetron_->hardware_mode() && elmetron_probe_) {
        const ElmetronProbe::Reading& r = elmetron_probe_->last_reading();
        const bool valid = elmetron_->state() == ElmetronState::MEASURING
                           && elmetron_->step() == ElmetronStep::IN_WATER
                           && conv_.converged(clock_.now_ms());
        telemetry_->update_measurement(r.conductivity, r.temperature,
                                       r.ph, r.oxygen, valid);
        return;
    }

    // Synthetic (no hardware): ramp during MEASURING, zeros when idle.
    if (elmetron_->state() == ElmetronState::MEASURING) {
        const Elmetron::Reading r = elmetron_->last_reading();
        telemetry_->update_measurement(r.cond, r.temp, r.ph, r.oxygen, r.valid);
    } else {
        telemetry_->clear_measurement();
    }
}

// CMD,GPS,<sub> — sub is STATUS / RAW / RESET.
//   STATUS → EVT,SYS,GPS,fix=0|1,sats=N,lat=...,lon=...,utc=YYYY-MM-DD HH:MM:SS
//   RAW    → EVT,SYS,GPS_RAW,lat=...,lon=...,sats=N,present=0|1
//   RESET  → re-run the u-blox bring-up sequence on the existing bus.
void MissionControl::cmd_gps(const char* verb, const char* args, size_t args_len) {
    if (!gps_) { emit_nack(verb, "no_gps"); return; }

    char sub[12] = {0};
    if (args_len == 0 || args_len >= sizeof(sub)) {
        emit_nack(verb, "bad_args");
        return;
    }
    memcpy(sub, args, args_len);

    if (strcmp(sub, "STATUS") == 0) {
        const auto& f = gps_->last_fix();
        char ev[128];
        snprintf(ev, sizeof(ev),
                 "EVT,SYS,GPS,fix=%d,sats=%u,lat=%ld,lon=%ld,utc=%04u-%02u-%02u %02u:%02u:%02u",
                 f.fix_valid ? 1 : 0, f.sat_count, f.lat, f.lon,
                 f.year, f.month, f.day, f.hour, f.minute, f.second);
        send_payload(ev);
        emit_ack(verb);
    } else if (strcmp(sub, "RAW") == 0) {
        const auto& f = gps_->last_fix();
        char ev[96];
        snprintf(ev, sizeof(ev),
                 "EVT,SYS,GPS_RAW,lat=%ld,lon=%ld,sats=%u,present=%d",
                 f.lat, f.lon, f.sat_count, gps_->present() ? 1 : 0);
        send_payload(ev);
        emit_ack(verb);
    } else if (strcmp(sub, "RESET") == 0) {
        const bool ok = gps_->reinit();
        char ev[48];
        snprintf(ev, sizeof(ev), "EVT,SYS,GPS_RESET,ok=%d", ok ? 1 : 0);
        send_payload(ev);
        if (ok) emit_ack(verb); else emit_nack(verb, "reinit_failed");
    } else {
        emit_nack(verb, "bad_args");
    }
}

// CMD,COLLECTIONS — dump the per-tank geo-tag captured at the moment
// each tank reached FULL this session. Tanks that haven't been
// collected emit valid=0 with zeroed fields.
void MissionControl::cmd_collections(const char* verb) {
    emit_ack(verb);
    for (uint8_t i = 0; i < 3; ++i) emit_collection(i);
}

// Snapshot the current GPS fix into the tank's Collection slot.
// Captures even when fix_valid=false so the GCS can record that a
// pre-fix collection happened (rare in field use, common during
// bench bring-up).
void MissionControl::capture_collection(uint8_t idx) {
    if (idx >= 3) return;
    Collection& c = collections_[idx];
    if (gps_) {
        const auto& f = gps_->last_fix();
        c.lat       = f.lat;
        c.lon       = f.lon;
        c.year      = f.year;
        c.month     = f.month;
        c.day       = f.day;
        c.hour      = f.hour;
        c.minute    = f.minute;
        c.second    = f.second;
        c.fix_valid = f.fix_valid;
    } else {
        c = Collection{};   // no GPS attached: leave everything zero
    }
    c.valid = true;
}

void MissionControl::emit_collection(uint8_t idx) {
    if (idx >= 3) return;
    const Collection& c = collections_[idx];
    char buf[128];
    snprintf(buf, sizeof(buf),
             "EVT,%s,COLLECTED,valid=%d,fix=%d,lat=%ld,lon=%ld,utc=%04u-%02u-%02u %02u:%02u:%02u",
             tank_source(idx),
             c.valid ? 1 : 0, c.fix_valid ? 1 : 0,
             c.lat, c.lon,
             c.year, c.month, c.day, c.hour, c.minute, c.second);
    send_payload(buf);
}

// CMD,CFG,to=<sec>,C1=<en>:<ch>:<servo>,C2=...,C3=... — provision the
// logical-tank → physical-channel/servo mapping + global timeout. One
// atomic frame; the whole config lands or none of it. Rejected while
// busy (don't remap mid-operation). Validated for channel/servo range
// and uniqueness among enabled tanks before it's committed.
void MissionControl::cmd_cfg(const char* verb, const char* args, size_t args_len) {
    if (is_busy()) { emit_nack(verb, "busy"); return; }

    char buf[200];
    if (args_len == 0 || args_len >= sizeof(buf)) {
        emit_nack(verb, "bad_args");
        return;
    }
    memcpy(buf, args, args_len);
    buf[args_len] = '\0';

    // Parse into a temp copy; commit only if the whole thing validates.
    SystemConfig tmp = config_;
    char* save = nullptr;
    for (char* tok = strtok_r(buf, ",", &save); tok;
         tok = strtok_r(nullptr, ",", &save)) {
        if (strncmp(tok, "to=", 3) == 0) {
            const int sec = atoi(tok + 3);
            if (sec <= 0) { emit_nack(verb, "bad_timeout"); return; }
            tmp.pumping_timeout_ms = static_cast<uint32_t>(sec) * 1000u;
        } else if (tok[0] == 'C' && tok[1] >= '1' && tok[1] <= '3'
                   && tok[2] == '=') {
            const int idx = tok[1] - '1';
            // Cx = enabled:channel:servo[:unroll_ms:roll_ms:pwm]. The
            // winch fields are optional (older 3-field form keeps the
            // current values, so legacy provisioners still parse). PWM
            // is SIGNED — sign sets the unroll direction.
            int en = 0, ch = 0, sv = 0, ups = -1, rls = -1, pw = -9999;
            const int got = sscanf(tok + 3, "%d:%d:%d:%d:%d:%d",
                                   &en, &ch, &sv, &ups, &rls, &pw);
            if (got < 3) { emit_nack(verb, "bad_args"); return; }
            if (ch < 1 || ch > 3 || sv < 1 || sv > 253) {
                emit_nack(verb, "out_of_range");
                return;
            }
            if (got >= 4 && (ups < 0 || ups > 30000))      { emit_nack(verb, "bad_winch_ms"); return; }
            if (got >= 5 && (rls < 0 || rls > 30000))      { emit_nack(verb, "bad_winch_ms"); return; }
            if (got >= 6 && (pw < -1023 || pw > 1023))     { emit_nack(verb, "bad_winch_pwm"); return; }
            tmp.tanks[idx].enabled  = (en != 0);
            tmp.tanks[idx].channel  = static_cast<uint8_t>(ch);
            tmp.tanks[idx].servo_id = static_cast<uint8_t>(sv);
            if (got >= 4) tmp.tanks[idx].winch_unroll_ms = static_cast<uint16_t>(ups);
            if (got >= 5) tmp.tanks[idx].winch_roll_ms   = static_cast<uint16_t>(rls);
            if (got >= 6) tmp.tanks[idx].winch_pwm       = static_cast<int16_t>(pw);
        }
        // Unknown tokens ignored (forward-compat per protocol §10).
    }

    // No two ENABLED tanks may share a channel or a servo id.
    for (int i = 0; i < 3; ++i) {
        for (int j = i + 1; j < 3; ++j) {
            if (!tmp.tanks[i].enabled || !tmp.tanks[j].enabled) continue;
            if (tmp.tanks[i].channel == tmp.tanks[j].channel) {
                emit_nack(verb, "dup_channel"); return;
            }
            if (tmp.tanks[i].servo_id == tmp.tanks[j].servo_id) {
                emit_nack(verb, "dup_servo"); return;
            }
        }
    }

    config_ = tmp;
    apply_config();
    emit_ack(verb);
}

// CMD,CFG_GET — dump the active config so the GCS can verify what
// actually landed (readback closes the provisioning loop over a lossy
// radio link).
void MissionControl::cmd_cfg_get(const char* verb) {
    emit_cfg();
    emit_ack(verb);
}

// CMD,CFG_ELE,wt=..,wd=..,dto=..,ato=..,hto=..,mto=..,cw=..,ct=..,di=..,lal=..
// — provision the Elmetron tuning (water threshold, winch duty, safety
// timeouts, convergence, polarity flips). Rejected while busy. Timeouts
// are ceilinged so a bad value can't disable the mechanical safety caps.
void MissionControl::cmd_cfg_ele(const char* verb, const char* args, size_t args_len) {
    if (is_busy()) { emit_nack(verb, "busy"); return; }

    char buf[200];
    if (args_len == 0 || args_len >= sizeof(buf)) { emit_nack(verb, "bad_args"); return; }
    memcpy(buf, args, args_len);
    buf[args_len] = '\0';

    ElmetronConfig tmp = ele_config_;
    char* save = nullptr;
    for (char* tok = strtok_r(buf, ",", &save); tok;
         tok = strtok_r(nullptr, ",", &save)) {
        char* eq = strchr(tok, '=');
        if (!eq) continue;
        *eq = '\0';
        const char* k = tok;
        const char* v = eq + 1;
        if      (!strcmp(k, "wt"))  tmp.water_threshold_ms   = atof(v);
        else if (!strcmp(k, "wd"))  tmp.winch_duty_pct        = (uint8_t)atoi(v);
        else if (!strcmp(k, "dto")) tmp.descent_timeout_s     = (uint16_t)atoi(v);
        else if (!strcmp(k, "ato")) tmp.ascent_timeout_s      = (uint16_t)atoi(v);
        else if (!strcmp(k, "hto")) tmp.homing_timeout_s      = (uint16_t)atoi(v);
        else if (!strcmp(k, "mto")) tmp.measure_timeout_s     = (uint16_t)atoi(v);
        else if (!strcmp(k, "cw"))  tmp.convergence_window_s  = (uint16_t)atoi(v);
        else if (!strcmp(k, "ct"))  tmp.convergence_tol_pct   = (uint8_t)atoi(v);
        else if (!strcmp(k, "di"))  tmp.winch_dir_invert      = atoi(v) != 0;
        else if (!strcmp(k, "lal")) tmp.limit_active_low      = atoi(v) != 0;
        // unknown keys ignored (forward-compat)
    }

    if (tmp.water_threshold_ms <= 0.0f || tmp.water_threshold_ms > 200.0f) { emit_nack(verb, "bad_wt"); return; }
    if (tmp.winch_duty_pct < 1 || tmp.winch_duty_pct > 100)                { emit_nack(verb, "bad_duty"); return; }
    if (tmp.descent_timeout_s < 1 || tmp.descent_timeout_s > DESCENT_CEILING_S) { emit_nack(verb, "descent_ceiling"); return; }
    if (tmp.ascent_timeout_s  < 1 || tmp.ascent_timeout_s  > ASCENT_CEILING_S)  { emit_nack(verb, "bad_ato"); return; }
    if (tmp.homing_timeout_s  < 1 || tmp.homing_timeout_s  > HOMING_CEILING_S)  { emit_nack(verb, "bad_hto"); return; }
    if (tmp.measure_timeout_s < 1 || tmp.measure_timeout_s > MEASURE_CEILING_S) { emit_nack(verb, "bad_mto"); return; }
    if (tmp.convergence_window_s < 1 || tmp.convergence_window_s > 120)     { emit_nack(verb, "bad_cw"); return; }
    if (tmp.convergence_tol_pct  < 1 || tmp.convergence_tol_pct  > 50)      { emit_nack(verb, "bad_ct"); return; }

    ele_config_ = tmp;
    apply_elmetron_config();
    emit_ack(verb);
}

void MissionControl::cmd_cfg_ele_get(const char* verb) {
    emit_cfg_ele();
    emit_ack(verb);
}

// EVT,SYS,CFG_ELE,... — must match the GCS-built string byte-for-byte
// for the provisioning readback to compare equal.
void MissionControl::emit_cfg_ele() {
    char buf[160];
    snprintf(buf, sizeof(buf),
        "EVT,SYS,CFG_ELE,wt=%.2f,wd=%d,dto=%d,ato=%d,hto=%d,mto=%d,cw=%d,ct=%d,di=%d,lal=%d",
        ele_config_.water_threshold_ms, ele_config_.winch_duty_pct,
        ele_config_.descent_timeout_s, ele_config_.ascent_timeout_s,
        ele_config_.homing_timeout_s, ele_config_.measure_timeout_s,
        ele_config_.convergence_window_s, ele_config_.convergence_tol_pct,
        ele_config_.winch_dir_invert ? 1 : 0, ele_config_.limit_active_low ? 1 : 0);
    send_payload(buf);
}

// EVT,SYS,CFG,to=<sec>,C1=<en>:<ch>:<servo>:<unroll_ms>:<roll_ms>:<pwm>,...
// — must match the GCS-built string byte-for-byte for readback to
// compare equal. Timeout reported in whole seconds.
void MissionControl::emit_cfg() {
    char buf[200];
    snprintf(buf, sizeof(buf),
        "EVT,SYS,CFG,to=%lu,"
        "C1=%d:%d:%d:%d:%d:%d,C2=%d:%d:%d:%d:%d:%d,C3=%d:%d:%d:%d:%d:%d",
        static_cast<unsigned long>(config_.pumping_timeout_ms / 1000u),
        config_.tanks[0].enabled ? 1 : 0, config_.tanks[0].channel, config_.tanks[0].servo_id,
        config_.tanks[0].winch_unroll_ms, config_.tanks[0].winch_roll_ms, config_.tanks[0].winch_pwm,
        config_.tanks[1].enabled ? 1 : 0, config_.tanks[1].channel, config_.tanks[1].servo_id,
        config_.tanks[1].winch_unroll_ms, config_.tanks[1].winch_roll_ms, config_.tanks[1].winch_pwm,
        config_.tanks[2].enabled ? 1 : 0, config_.tanks[2].channel, config_.tanks[2].servo_id,
        config_.tanks[2].winch_unroll_ms, config_.tanks[2].winch_roll_ms, config_.tanks[2].winch_pwm);
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
