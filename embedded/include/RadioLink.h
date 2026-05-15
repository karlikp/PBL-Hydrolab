// RadioLink — bidirectional UART transport for the wire protocol.
//
// Owns one HardwareSerial port. Accumulates incoming bytes into a line
// buffer; on newline, validates the frame via FrameCodec and dispatches
// to a registered callback. Outgoing frames are built from a payload
// string and written to the same port.
//
// For the skeleton phase, this is wired to the USB Serial port — that
// way you can type CMD frames in the serial monitor and watch the
// EVT replies stream back in the same window, no radio hardware needed.
// In production, instantiate with Serial1 (or the appropriate HW UART)
// connected to the RFD/SiK radio module.
//
// Threading: today there is a single writer (the main loop), so send()
// has no synchronisation. When FreeRTOS tasks start emitting frames
// concurrently, wrap send()'s build+write step in a mutex to prevent
// byte interleaving on the wire (see comms-robustness notes).

#pragma once

#include <stdint.h>
#include <stddef.h>

#include "FrameCodec.h"

class HardwareSerial;

class RadioLink {
public:
    using FrameHandler = void (*)(void* ctx,
                                  frame::Type type,
                                  const char* payload,
                                  size_t payload_len);

    explicit RadioLink(HardwareSerial& serial);

    // Open the underlying serial at the given baud rate.
    void begin(uint32_t baud);

    // Register the callback invoked for every received frame that
    // passes checksum validation. Invoked from tick().
    void on_frame(FrameHandler handler, void* ctx);

    // Build "<payload>*<adler>\n" and write it to the serial port.
    // Returns true on success, false on payload-too-long / build error.
    bool send(const char* payload);

    // Pump RX. Call from the main loop frequently.
    void tick();

private:
    HardwareSerial& serial_;
    FrameHandler handler_ = nullptr;
    void* handler_ctx_ = nullptr;
    char rx_buf_[frame::MAX_FRAME];
    size_t rx_pos_ = 0;

    void handle_line(size_t len);
};
