// RadioLink — bidirectional serial transport for the wire protocol.
//
// Takes any Arduino Stream — HardwareSerial on classic ESP32 / UART
// builds, USBCDC when CDC-on-boot is enabled on S3, or any future
// transport that satisfies the Stream interface (available/read/write).
// Caller is responsible for opening the underlying port (Serial.begin)
// before passing it in.
//
// Accumulates incoming bytes into a line buffer; on newline, validates
// the frame via FrameCodec and dispatches to a registered callback.
// Outgoing frames are built from a payload string and written to the
// same stream.
//
// Threading: today there is a single writer (the main loop), so send()
// has no synchronisation. When FreeRTOS tasks start emitting frames
// concurrently, wrap send()'s build+write step in a mutex to prevent
// byte interleaving on the wire (see comms-robustness notes).

#pragma once

#include <stdint.h>
#include <stddef.h>

#include "FrameCodec.h"

class Stream;

class RadioLink {
public:
    using FrameHandler = void (*)(void* ctx,
                                  frame::Type type,
                                  const char* payload,
                                  size_t payload_len);

    explicit RadioLink(Stream& serial);

    // Register the callback invoked for every received frame that
    // passes checksum validation. Invoked from tick().
    void on_frame(FrameHandler handler, void* ctx);

    // Build "<payload>*<adler>\n" and write it to the stream.
    // Returns true on success, false on payload-too-long / build error.
    bool send(const char* payload);

    // Pump RX. Call from the main loop frequently.
    void tick();

private:
    Stream& serial_;
    FrameHandler handler_ = nullptr;
    void* handler_ctx_ = nullptr;
    char rx_buf_[frame::MAX_FRAME];
    size_t rx_pos_ = 0;

    void handle_line(size_t len);
};
