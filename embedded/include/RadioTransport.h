// RadioTransport — owns the HardwareSerial setup for the RFD868 radio.
//
// The radio is the operator comms channel in deployment. It lives on
// its own UART (separate from UART0, which is the servo bus). This
// class encapsulates the pinout + baud constants and the begin() call
// so that main.cpp doesn't have to know them, mirroring the ServoBus
// pattern.
//
// RadioLink takes a Stream& (transport-agnostic protocol layer), so
// callers do:
//
//     RadioTransport transport(Serial1);
//     RadioLink      radio(transport.stream());
//
//     void setup() {
//         transport.begin();
//         ...
//     }
//
// See docs/hardware/radio_uart_pinout.md.

#pragma once

#include <stdint.h>

class HardwareSerial;

class RadioTransport {
public:
    // RFD radio wiring on the production ESP32-S3 board.
    //
    // The board labels these nets `RFD_RX` (IO10) and `RFD_TX` (IO11)
    // — from the RADIO MODULE's perspective. That means `RFD_RX` is
    // an input to the radio, which has to be driven by the ESP's TX.
    // So from the ESP side the assignments cross over:
    //
    //     ESP TX = IO10  (board net "RFD_RX", into radio's RX)
    //     ESP RX = IO11  (board net "RFD_TX", from radio's TX)
    //
    // Same gotcha as the servo bus (see hw_servo_uart_pin_swap memory).
    static constexpr int      RX_PIN = 11;
    static constexpr int      TX_PIN = 10;
    static constexpr uint32_t BAUD   = 57600;  // V5 module verified; matches host RFD

    explicit RadioTransport(HardwareSerial& serial) : serial_(serial) {}

    // Open the underlying UART with the radio's pinout + baud.
    // Idempotent in practice — HardwareSerial::begin re-initialises
    // safely if called again.
    void begin();

    // Hand off the configured stream to RadioLink (or anyone else
    // that wants to read/write bytes on the radio UART).
    HardwareSerial& stream() { return serial_; }

private:
    HardwareSerial& serial_;
};
