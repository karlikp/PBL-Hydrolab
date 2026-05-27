// ElmetronProbe — driver for the Elmetron CX-series multiparameter probe.
//
// The probe is a request/response instrument on a dedicated UART:
//
//     ELE_RX = IO17  (ESP receives — wired to the probe's TX)
//     ELE_TX = IO18  (ESP transmits — wired to the probe's RX)
//     115200 baud, 8E1   (even parity — confirm on the bench; flip
//                         SERIAL_8E1 ↔ SERIAL_8N1 if the link is silent)
//
// One query returns ALL parameters in a single response frame (this is
// the "read all at once" generation, not the older one-value-per-query
// units). Protocol reverse-engineered from the working legacy driver
// (embedded/legacy/src/SensorManager.cpp, a CX-401):
//
//   Query :  <SOH 0x01> "#0#0#0#" <ETX 0x03>
//   Reply :  control-char-wrapped, '#'-delimited tokens, each a value
//            followed/labelled by its unit:
//              "...pH"   -> pH
//              "...O2"   -> dissolved oxygen (mg/L)
//              "...S/cm" -> conductivity (µS/cm; ÷1000 → mS/cm if >10)
//              "...C"    -> temperature °C  (but NOT the "CX" model token)
//
// Water is considered "detected" once conductivity exceeds
// WATER_THRESHOLD_MS — used as the descent trigger (probe has reached
// the water) with a time-based safety cap upstream.
//
// tick() runs a non-blocking REQUEST → WAIT → PROCESS cycle off the
// Clock, so it never stalls the main loop. UART2 is independent of the
// radio (Serial1) and the SC-09 servo bus (UART0).

#pragma once

#include <stdint.h>
#include <stddef.h>

class HardwareSerial;
class Clock;

class ElmetronProbe {
public:
    struct Reading {
        float conductivity = 0.0f;   // mS/cm
        float temperature  = 0.0f;   // °C
        float ph           = 0.0f;
        float oxygen       = 0.0f;   // mg/L
        bool  water        = false;  // conductivity > WATER_THRESHOLD_MS
        bool  valid        = false;  // a complete frame has been parsed
    };

    ElmetronProbe(HardwareSerial& serial, Clock& clock);

    // Open the UART (115200 8E1) on the given pins and start polling.
    void begin(int rx_pin, int tx_pin);

    // Drive the request/response cycle. Call from the main loop.
    void tick();

    const Reading& last_reading() const { return last_; }

    // True once at least one complete frame has been parsed (i.e. the
    // probe is actually answering on the bus).
    bool present() const { return present_; }

    // One-shot: true if a fresh frame arrived since the last call.
    bool consume_changed();

    static constexpr uint32_t BAUD              = 115200;
    static constexpr float    WATER_THRESHOLD_MS = 0.7f;   // mS/cm
    static constexpr uint32_t READ_DELAY_MS     = 500;     // wait for reply

private:
    enum class Step : uint8_t { REQUEST, WAIT, PROCESS };

    HardwareSerial& serial_;
    Clock&          clock_;
    Step            step_         = Step::REQUEST;
    uint32_t        last_step_ms_ = 0;
    char            buf_[160];
    size_t          len_          = 0;
    Reading         last_;
    bool            changed_      = false;
    bool            present_      = false;

    void send_query();
    void process_incoming();
    void parse_frame(char* s);
};
