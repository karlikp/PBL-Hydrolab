#include "FrameCodec.h"

#include <stdio.h>
#include <string.h>

namespace frame {

uint32_t adler32(const char* data, size_t len) {
    uint32_t a = 1;
    uint32_t b = 0;
    constexpr uint32_t MOD = 65521;
    for (size_t i = 0; i < len; ++i) {
        a = (a + static_cast<uint8_t>(data[i])) % MOD;
        b = (b + a) % MOD;
    }
    return (b << 16) | a;
}

size_t build(const char* payload, size_t payload_len,
             char* outBuf, size_t outCap) {
    if (payload_len > MAX_PAYLOAD) return 0;
    // Need: payload + '*' + 8 hex + '\n' + null = payload_len + 11
    if (outCap < payload_len + 11) return 0;

    uint32_t cs = adler32(payload, payload_len);
    memcpy(outBuf, payload, payload_len);
    outBuf[payload_len] = '*';

    // 9 chars for "XXXXXXXX\n" + null
    int written = snprintf(outBuf + payload_len + 1,
                           outCap - payload_len - 1,
                           "%08X\n", cs);
    if (written != 9) return 0;  // truncation = silent failure, refuse

    return payload_len + 1 + 9;
}

size_t build(const char* payload, char* outBuf, size_t outCap) {
    return build(payload, strlen(payload), outBuf, outCap);
}

static bool hex_byte(char c, uint32_t* out) {
    if (c >= '0' && c <= '9') { *out = c - '0';      return true; }
    if (c >= 'A' && c <= 'F') { *out = 10 + c - 'A'; return true; }
    if (c >= 'a' && c <= 'f') { *out = 10 + c - 'a'; return true; }
    return false;
}

static bool hex8(const char* s, uint32_t* out) {
    uint32_t v = 0;
    for (int i = 0; i < 8; ++i) {
        uint32_t d;
        if (!hex_byte(s[i], &d)) return false;
        v = (v << 4) | d;
    }
    *out = v;
    return true;
}

bool parse(const char* line, size_t lineLen, Parsed* out) {
    // Strip trailing \r and \n
    while (lineLen > 0 && (line[lineLen-1] == '\n' || line[lineLen-1] == '\r')) {
        --lineLen;
    }

    // Bounds: shortest meaningful frame is "*XXXXXXXX" (9 chars).
    if (lineLen < 9) return false;
    if (lineLen > MAX_FRAME) return false;

    // Find the last '*'. The checksum is always the trailing 8 chars.
    if (line[lineLen - 9] != '*') return false;
    size_t star = lineLen - 9;

    uint32_t recv_cs;
    if (!hex8(line + star + 1, &recv_cs)) return false;

    uint32_t calc_cs = adler32(line, star);
    if (calc_cs != recv_cs) return false;

    // Determine frame type from the leading prefix "TYP,".
    Type t = Type::UNKNOWN;
    if (star >= 4 && line[3] == ',') {
        if      (memcmp(line, "TLM", 3) == 0) t = Type::TLM;
        else if (memcmp(line, "EVT", 3) == 0) t = Type::EVT;
        else if (memcmp(line, "CMD", 3) == 0) t = Type::CMD;
    }

    out->type = t;
    out->payload = line;
    out->payload_len = star;
    return true;
}

}  // namespace frame
