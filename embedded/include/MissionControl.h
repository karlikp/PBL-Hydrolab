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

class Clock;
class RadioLink;
class BatteryMonitor;
class ServoBus;
class PumpControl;
class LevelSensor;
class GpsLink;

enum class SystemMode : uint8_t {
    IDLE,
    SAMPLING,
    MEASURING,    // reserved for Elmetron; not driven this iteration
    E_STOP,
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

    // Optional: attach the battery monitor so STATUS responses include
    // current battery voltage. nullptr is fine (e.g. on classic env).
    void set_battery_monitor(BatteryMonitor* bm) { battery_ = bm; }

    // Optional: attach the servo bus so CMD,SERVO_MOVE,<id>,<pos> works.
    void set_servo_bus(ServoBus* sb) { servo_bus_ = sb; }

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
    BatteryMonitor* battery_ = nullptr;
    ServoBus*       servo_bus_ = nullptr;
    PumpControl*    pump_control_ = nullptr;
    LevelSensor*    level_sensor_ = nullptr;
    GpsLink*        gps_ = nullptr;

    // Command handlers
    void cmd_start_tank(uint8_t idx, const char* verb);
    void cmd_estop(const char* verb);
    void cmd_reset_tank(uint8_t idx, const char* verb);
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

    void set_mode(SystemMode m);
    bool any_tank_sampling() const;
    bool any_tank_in_fault() const;

    // EVT emission helpers
    void emit_ack(const char* verb);
    void emit_nack(const char* verb, const char* reason);
    void emit_sys_state();
    void emit_tank_state(Sampler& tank);
    void emit_tank_step(Sampler& tank);
    void emit_boot(const char* version);

    // Drive the tank's servo on DESCENDING/ASCENDING step entries.
    void drive_servo_for_step(const Sampler& tank);

    // Drive the tank's pump on PUMPING entry, off on any other step.
    void drive_pump_for_step(const Sampler& tank);

    // Geo-tag the per-tank collection on STATE -> FULL and emit
    // EVT,Cx,COLLECTED,... . No-op if GPS not attached.
    void capture_collection(uint8_t idx);
    void emit_collection(uint8_t idx);

    void send_payload(const char* payload);

    static const char* mode_name(SystemMode m);
    static const char* tank_source(uint8_t idx);
};
