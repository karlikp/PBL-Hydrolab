// FrameCodec — wire-frame encode/decode for the drone <-> GCS link.
//
// See docs/protocol.md for the on-the-wire format. In short, every
// frame on the link is:
//
//     <payload>*<8 hex digits of Adler-32>\n
//
// This module is pure C++ (no Arduino.h) so it can be unit-tested
// without dragging in the Arduino runtime. It's small, allocation-
// free, and used by both the sender (build) and receiver (parse) on
// the embedded side.

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace frame {

// Max length of a complete frame, including the trailing newline.
// The receiver MUST drop any line longer than this.
constexpr size_t MAX_FRAME = 256;

// Max length of the payload (everything before the '*'). Leaves room
// for the 10-byte trailer ("*XXXXXXXX\n").
constexpr size_t MAX_PAYLOAD = MAX_FRAME - 10;

enum class Type : uint8_t {
    TLM,
    EVT,
    CMD,
    UNKNOWN,
};

// Result of a successful parse(). Pointers reference the original
// input line — no allocation, no copy. Caller must keep the input
// buffer alive while reading from the parsed result.
struct Parsed {
    Type type;
    const char* payload;
    size_t payload_len;
};

// Adler-32 checksum over [data, data+len). Matches docs/protocol.md
// and the reference Python in groundstation/app/services/radio.py.
uint32_t adler32(const char* data, size_t len);

// Build a complete frame "<payload>*<adler>\n" into outBuf.
//
// Returns the number of bytes written (excluding the null terminator),
// or 0 on failure. Failure cases:
//   - payload_len > MAX_PAYLOAD
//   - outBuf is too small (need payload_len + 11 bytes minimum)
//
// outBuf is null-terminated on success for convenience with C string
// APIs (Serial.print etc).
size_t build(const char* payload, size_t payload_len,
             char* outBuf, size_t outCap);

// Convenience overload for null-terminated payloads.
size_t build(const char* payload, char* outBuf, size_t outCap);

// Validate and parse a received line.
//
// The line MUST NOT have leading whitespace. Trailing \r and \n are
// tolerated (stripped before parsing).
//
// Returns true on success. On success, *out is filled in; payload
// pointer references the input buffer (no copy).
//
// Returns false if any of:
//   - line shorter than the minimum valid frame
//   - line longer than MAX_FRAME
//   - missing '*' separator
//   - checksum field not exactly 8 hex chars
//   - computed Adler-32 doesn't match the received checksum
//
// Receivers MUST drop the frame on false.
bool parse(const char* line, size_t lineLen, Parsed* out);

}  // namespace frame
