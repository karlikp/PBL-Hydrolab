// MissionControl — top-level system state + command dispatcher.
//
// Owns the three Samplers and the system-wide mode (IDLE / SAMPLING /
// MEASURING / E_STOP). It's the single decision point for:
//
//   - Whether a CMD,START_Cx is allowed right now (busy? fault?
//     e-stop active? tank already full?).
//   - Emitting ACK/NACK in response to every CMD.
//   - Emitting STATE and STEP events whenever a Sampler transitions.
//   - Maintaining the global mode (anyone sampling => SAMPLING; an
//     E_STOP latches us in until all tanks are RESET).
//
// The controller never talks to hardware directly. It manipulates
// Samplers (which today contain mock timing, tomorrow contain real
// pumps and motors) and emits frames through the RadioLink.

#pragma once

#include <stdint.h>
#include <stddef.h>

#include "Sampler.h"
#include "Elmetron.h"
#include "ConvergenceDetector.h"

class Clock;
class RadioLink;
class BatteryMonitor;
class ServoBus;
class PumpControl;
class LevelSensor;
class GpsLink;
class Telemetry;
class ElmetronProbe;
class WinchH;

enum class SystemMode : uint8_t {
    IDLE,
    SAMPLING,
    MEASURING,    // reserved for Elmetron; not driven this iteration
    E_STOP,
};

// Runtime-provisioned mapping from a logical tank to the physical
// channel it's wired to. Provisioned by the GCS via CMD,CFG so a rig
// plugged together differently can be remapped without reflashing.
// Defaults match the historical 1:1 wiring (tank N → channel/servo N),
// so an un-provisioned board behaves exactly as before.
//
// EXPERIMENTAL — winch in PWM/wheel mode (multi-revolution unspool).
// The SC-09 has a hard ~300° single-turn range in position mode, which
// isn't enough line for some rigs. PWM mode disables position control:
// instead of "go to position X", we drive a continuous-rotation duty
// for the unroll, and use ENCODER WRAP COUNTING during the rewind to
// land exactly back at home regardless of any unroll/rewind speed
// asymmetry.
//   winch_unroll_ms : drive duration on DESCENDING (time-based)
//   winch_roll_ms   : safety cap on the rewind (cumulative-position
//                     based stop is the primary; this just guards
//                     against ReadPos failure)
//   winch_pwm       : signed duty (-1023..1023). Sign = which direction
//                     counts as "unroll" (lets the user flip without
//                     reorienting the spool). Magnitude = motor speed.
struct TankConfig {
    bool     enabled  = true;
    uint8_t  channel  = 1;        // 1..3 — the fixed PUMP+TOPCN pair
    uint8_t  servo_id = 1;        // SC-09 bus address of this tank's winch
    uint16_t winch_unroll_ms = 4000;
    uint16_t winch_roll_ms   = 12000;  // safety cap; rewind usually < this
    int16_t  winch_pwm       = 600;
};

struct SystemConfig {
    uint32_t   pumping_timeout_ms = 90000;
    TankConfig tanks[3];
};

// Runtime-provisioned Elmetron tuning (CMD,CFG_ELE). Defaults match the
// module compile-time defaults, so an un-provisioned board is unchanged.
// Timeouts are seconds on the wire; converted to ms when applied.
struct ElmetronConfig {
    float    water_threshold_ms   = 0.7f;  // descent trigger (cond mS/cm)
    uint8_t  winch_duty_pct       = 60;    // drive speed
    uint16_t descent_timeout_s    = 4;     // safety cap (ceilinged in firmware)
    uint16_t ascent_timeout_s     = 5;
    uint16_t homing_timeout_s     = 4;
    uint16_t measure_timeout_s    = 90;    // strict measurement cap
    uint16_t convergence_window_s = 10;
    uint8_t  convergence_tol_pct  = 5;
    bool     winch_dir_invert     = false; // flip DOWN/UP if wrong way
    bool     limit_active_low     = true;
};

class MissionControl {
public:
    MissionControl(Clock& clock, RadioLink& radio);

    // Call once after radio is up. Emits EVT,SYS,BOOT,<version>.
    void boot(const char* firmware_version);

    // Call from main loop frequently. Drives Samplers, emits any
    // pending STATE/STEP events, and updates system mode when a
    // sampling cycle finishes.
    void tick();

    // Called by RadioLink when a valid CMD frame arrives.
    // `payload` points at the verified payload (no checksum, no LF);
    // it must begin with "CMD,".
    void handle_command(const char* payload, size_t payload_len);

    SystemMode mode() const { return mode_; }

    // Called by the Sampler level-probe bridge: is the tank "full"?
    // Maps the logical tank (1..3) to its provisioned physical sensor
    // channel before reading. Public so the file-scope bridge function
    // in the .cpp can reach it.
    bool tank_full(uint8_t tank_id);

    // Optional: attach the battery monitor so STATUS responses include
    // current battery voltage. nullptr is fine (e.g. on classic env).
    void set_battery_monitor(BatteryMonitor* bm) { battery_ = bm; }

    // Optional: attach the servo bus so CMD,SERVO_MOVE,<id>,<pos> works.
    // Re-applies the active config (sets PWM mode on every enabled
    // tank's servo) since this is the first point at which we can.
    void set_servo_bus(ServoBus* sb) { servo_bus_ = sb; apply_config(); }

    // Optional: attach the pump controller so the PUMPING step actually
    // turns a pump on, and CMD,PUMP,<id>,<state> works for bench tests.
    void set_pump_control(PumpControl* pc) { pump_control_ = pc; }

    // Optional: attach the level sensor cluster. When attached, the
    // PUMPING step ends as soon as the matching tank's sensor reports
    // "full" instead of running for the full mock duration, and
    // CMD,LEVEL,<id> reads back a single sensor.
    void set_level_sensor(LevelSensor* ls);

    // Optional: attach the GPS module. When attached, a Sampler
    // transitioning to FULL captures the current GPS fix into the
    // tank's Collection record (geo-tag), and CMD,GPS,STATUS /
    // CMD,GPS,RAW / CMD,GPS,RESET work for bench tests.
    void set_gps_link(GpsLink* gps) { gps_ = gps; }

    // Optional: attach the Elmetron measurement subsystem. When
    // attached, CMD,START_ELMETRON drives a settle-and-measure cycle
    // and the system mode tracks MEASURING; STATUS reports its live
    // state. Without it, START_ELMETRON NACKs "not_implemented".
    void set_elmetron(Elmetron* el) { elmetron_ = el; }

    // Optional: attach the Telemetry singleton so probe readings are
    // pushed into the TLM stream while the Elmetron is in IN_WATER.
    // Without it the Elmetron still runs but TLM carries no probe
    // data (probe fields stay zero, measurement_valid stays 0).
    void set_telemetry(Telemetry* t) { telemetry_ = t; }

    // Optional: attach the real Elmetron probe (CX-series over UART2).
    // When present, CMD,ELE reports live probe readings; the FSM
    // integration (conductance-triggered descent, real readings to
    // telemetry) builds on this. Absent on hardware-less builds, where
    // the Elmetron FSM falls back to synthetic readings.
    void set_elmetron_probe(ElmetronProbe* p) { elmetron_probe_ = p; update_elmetron_mode(); }

    // Optional: attach the Elmetron winch (H-bridge). With both a probe
    // and a winch attached, the Elmetron FSM switches to hardware mode:
    // transitions are driven by the limit switch / water detection /
    // convergence rather than synthetic timers.
    void set_winch(WinchH* w) { winch_ = w; update_elmetron_mode(); }

private:
    // Per-tank geo-tag captured at the moment the Sampler transitions
    // to FULL. `valid` stays false until the tank has been collected
    // at least once this session (no NVS persistence — the GCS
    // journals the EVT,Cx,COLLECTED stream into its DB).
    struct Collection {
        long     lat        = 0;
        long     lon        = 0;
        uint16_t year       = 0;
        uint8_t  month      = 0;
        uint8_t  day        = 0;
        uint8_t  hour       = 0;
        uint8_t  minute     = 0;
        uint8_t  second     = 0;
        bool     fix_valid  = false;   // GPS fix at moment of capture
        bool     valid      = false;   // any collection happened yet
    };

    Clock&     clock_;
    RadioLink& radio_;
    SystemMode mode_ = SystemMode::IDLE;
    bool mode_changed_ = false;
    Sampler tanks_[3];
    Collection collections_[3];
    SystemConfig   config_;
    ElmetronConfig ele_config_;
    BatteryMonitor* battery_ = nullptr;
    ServoBus*       servo_bus_ = nullptr;
    PumpControl*    pump_control_ = nullptr;
    LevelSensor*    level_sensor_ = nullptr;
    GpsLink*        gps_ = nullptr;
    Elmetron*       elmetron_ = nullptr;
    Telemetry*      telemetry_ = nullptr;
    ElmetronProbe*  elmetron_probe_ = nullptr;
    WinchH*         winch_ = nullptr;
    ConvergenceDetector conv_;  // conductivity stability during IN_WATER

    // Per-tank winch (PWM mode) state. Cumulative-position tracking
    // lets rewind land exactly at home regardless of unroll/roll speed
    // asymmetry: poll present-position at 20 Hz, detect wraps, and stop
    // the rewind PWM when the accumulator returns to ~0. winch_roll_ms
    // is the safety cap that fires if ReadPos is unreliable.
    //   winch_active_       : motor is being driven on this tank
    //   winch_direction_    : +1 unroll, -1 rewind, 0 idle
    //   winch_safety_stop_  : hard PWM-off deadline (ms)
    //   winch_cumulative_   : signed encoder counts from home (wraps
    //                         accumulated; can exceed ±1023 across turns)
    //   winch_last_raw_     : last validated present-position 0..1023
    //                         (-1 = no successful poll yet this run)
    //   winch_last_poll_ms_ : rate-limits the ReadPos polls
    bool     winch_active_[3]          = {false, false, false};
    int8_t   winch_direction_[3]       = {0, 0, 0};
    uint32_t winch_safety_stop_[3]     = {0, 0, 0};
    int32_t  winch_cumulative_[3]      = {0, 0, 0};
    int16_t  winch_last_raw_[3]        = {-1, -1, -1};
    uint32_t winch_last_poll_ms_[3]    = {0, 0, 0};

    // Command handlers
    void cmd_start_tank(uint8_t idx, const char* verb);
    void cmd_start_elmetron(const char* verb);
    void cmd_estop(const char* verb);
    void cmd_stop_tank(uint8_t idx, const char* verb);
    void cmd_stop_elmetron(const char* verb);
    void cmd_reset_tank(uint8_t idx, const char* verb);
    void cmd_reset_elmetron(const char* verb);
    void cmd_status(const char* verb);
    void cmd_ping(const char* verb);
    void cmd_adc_scan(const char* verb);
    void cmd_servo_move(const char* verb, const char* args, size_t args_len);
    void cmd_servo_set_id(const char* verb, const char* args, size_t args_len);
    void cmd_servo_bcast_set_id(const char* verb, const char* args, size_t args_len);
    void cmd_servo_ping(const char* verb, const char* args, size_t args_len);
    void cmd_pump(const char* verb, const char* args, size_t args_len);
    void cmd_level(const char* verb, const char* args, size_t args_len);
    void cmd_level_diag(const char* verb, const char* args, size_t args_len);
    void cmd_gpio(const char* verb, const char* args, size_t args_len);
    void cmd_gps(const char* verb, const char* args, size_t args_len);
    void cmd_collections(const char* verb);
    void cmd_cfg(const char* verb, const char* args, size_t args_len);
    void cmd_cfg_get(const char* verb);
    void cmd_cfg_ele(const char* verb, const char* args, size_t args_len);
    void cmd_cfg_ele_get(const char* verb);
    void cmd_ele(const char* verb);

    void set_mode(SystemMode m);
    bool any_tank_sampling() const;
    bool any_tank_in_fault() const;
    bool elmetron_measuring() const;
    bool elmetron_in_fault() const;
    bool is_busy() const;  // any tank sampling OR elmetron measuring

    // EVT emission helpers
    void emit_ack(const char* verb);
    void emit_nack(const char* verb, const char* reason);
    void emit_sys_state();
    void emit_tank_state(Sampler& tank);
    void emit_tank_step(Sampler& tank);
    void emit_elmetron_state();
    void emit_elmetron_step();
    void emit_boot(const char* version);
    void emit_cfg();      // EVT,SYS,CFG,... active tank config
    void emit_cfg_ele();  // EVT,SYS,CFG_ELE,... active Elmetron config

    // Push the active Elmetron config into the probe / winch / FSM /
    // convergence detector. Guarded — applies only to attached modules.
    void apply_elmetron_config();

    // Push the active config into the per-tank Samplers (timeout) and
    // anywhere else that caches it. Call after config_ changes.
    void apply_config();

    // Drive the tank's servo on DESCENDING/ASCENDING step entries.
    // In PWM/wheel mode: start the motor at the configured signed PWM
    // and schedule a stop after winch_unroll_ms / winch_roll_ms.
    void drive_servo_for_step(const Sampler& tank);

    // Start an unroll run on tank `idx`: PWM forward at the configured
    // signed duty, time-based stop at winch_unroll_ms. Cumulative
    // position is tracked throughout so the matching rewind knows how
    // far to come back.
    void start_winch_unroll(uint8_t idx);

    // Start a rewind run on tank `idx`: PWM in the OPPOSITE direction
    // of unroll. Stops when cumulative_pos returns to ~0 (precise undo,
    // independent of any unroll/rewind speed asymmetry) OR when the
    // winch_roll_ms safety cap fires (in case the encoder polls fail).
    void start_winch_rewind(uint8_t idx);

    // Service the per-tank winch state — poll present-position at the
    // configured rate (updates cumulative), and apply per-direction
    // stop conditions. Called every tick().
    void service_winches();

    // Update winch_cumulative_ from a fresh present-position read.
    // Handles wraps and rejects glitched readings.
    void poll_winch_position(uint8_t idx);

    // Halt the tank's winch immediately: PWM=0, clear winch_active_.
    // Used by STOP. PWM=0 coasts — no holding torque. The cumulative
    // accumulator is preserved so a subsequent RESET can still rewind
    // to home from wherever the motor coasted to.
    void stop_tank_winch(uint8_t idx);

    // Drive the tank's pump on PUMPING entry, off on any other step.
    void drive_pump_for_step(const Sampler& tank);

    // Drive the Elmetron winch motor for the current step (UP for
    // HOMING/ASCENDING, DOWN for DESCENDING, STOP otherwise). No-op
    // without a winch attached.
    void drive_elmetron_winch_for_step();

    // Hardware-mode service: read the winch limit switch + probe each
    // tick and fire the Elmetron FSM's notify triggers (at_home /
    // water_detected / measurement_done). No-op outside hardware mode.
    void service_elmetron_hardware();

    // Recompute whether the Elmetron FSM should run in hardware mode
    // (both probe + winch attached). Called from the setters.
    void update_elmetron_mode();

    // Push the current Elmetron reading (synthetic ramp during settle,
    // steady-state thereafter) into the telemetry pipeline. Called
    // every tick while elmetron_ is attached; no-op without telemetry_.
    void push_elmetron_reading();

    // Geo-tag the per-tank collection on STATE -> FULL and emit
    // EVT,Cx,COLLECTED,... . No-op if GPS not attached.
    void capture_collection(uint8_t idx);
    void emit_collection(uint8_t idx);

    void send_payload(const char* payload);

    static const char* mode_name(SystemMode m);
    static const char* tank_source(uint8_t idx);
};
