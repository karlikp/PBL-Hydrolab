#include "RadioLink.h"

#include <Arduino.h>

RadioLink::RadioLink(HardwareSerial& serial)
    : serial_(serial) {}

void RadioLink::begin(uint32_t baud) {
    serial_.begin(baud);
}

void RadioLink::on_frame(FrameHandler handler, void* ctx) {
    handler_ = handler;
    handler_ctx_ = ctx;
}

bool RadioLink::send(const char* payload) {
    char buf[frame::MAX_FRAME];
    size_t n = frame::build(payload, buf, sizeof(buf));
    if (n == 0) return false;
    size_t written = serial_.write(reinterpret_cast<const uint8_t*>(buf), n);
    return written == n;
}

void RadioLink::tick() {
    while (serial_.available() > 0) {
        int c = serial_.read();
        if (c < 0) break;

        if (c == '\n') {
            handle_line(rx_pos_);
            rx_pos_ = 0;
        } else if (rx_pos_ < sizeof(rx_buf_) - 1) {
            rx_buf_[rx_pos_++] = static_cast<char>(c);
        } else {
            // Line longer than MAX_FRAME — discard in-progress buffer
            // and wait for the next newline to resync.
            rx_pos_ = 0;
        }
    }
}

void RadioLink::handle_line(size_t len) {
    if (len == 0) return;
    frame::Parsed p;
    if (!frame::parse(rx_buf_, len, &p)) return;  // bad checksum -> drop
    if (handler_) handler_(handler_ctx_, p.type, p.payload, p.payload_len);
}
