// Unit tests for FrameCodec.
//
// Run on-target:
//     pio test -e esp32doit-devkit-v1 -f test_frame_codec
//
// Tests cover the four protocol test vectors (matching docs/protocol.md),
// build/parse roundtrip, and the corruption / truncation rejection paths.

#include <Arduino.h>
#include <unity.h>
#include <string.h>

#include "FrameCodec.h"

void setUp(void) {}
void tearDown(void) {}

// ---------- Adler-32 reference vectors (docs/protocol.md §3) ----------

void test_adler32_empty(void) {
    TEST_ASSERT_EQUAL_HEX32(0x00000001u, frame::adler32("", 0));
}

void test_adler32_a(void) {
    TEST_ASSERT_EQUAL_HEX32(0x00620062u, frame::adler32("a", 1));
}

void test_adler32_abc(void) {
    TEST_ASSERT_EQUAL_HEX32(0x024D0127u, frame::adler32("abc", 3));
}

void test_adler32_tlm_test(void) {
    const char* s = "TLM,test";
    TEST_ASSERT_EQUAL_HEX32(0x0BBF02DAu, frame::adler32(s, strlen(s)));
}

// ---------- build() ----------

void test_build_simple_payload(void) {
    char buf[64];
    size_t n = frame::build("TLM,test", buf, sizeof(buf));
    TEST_ASSERT_EQUAL_size_t(strlen("TLM,test*0BBF02DA\n"), n);
    TEST_ASSERT_EQUAL_STRING("TLM,test*0BBF02DA\n", buf);
}

void test_build_appends_newline(void) {
    char buf[64];
    size_t n = frame::build("CMD,PING,", buf, sizeof(buf));
    TEST_ASSERT_GREATER_THAN(0, n);
    TEST_ASSERT_EQUAL('\n', buf[n - 1]);
}

void test_build_refuses_oversized_payload(void) {
    char big[frame::MAX_PAYLOAD + 1];
    memset(big, 'X', sizeof(big));
    char buf[frame::MAX_FRAME];
    size_t n = frame::build(big, sizeof(big), buf, sizeof(buf));
    TEST_ASSERT_EQUAL_size_t(0, n);
}

void test_build_refuses_small_outbuf(void) {
    char buf[10];  // need at least payload_len + 11
    size_t n = frame::build("TLM,test", buf, sizeof(buf));
    TEST_ASSERT_EQUAL_size_t(0, n);
}

// ---------- parse() ----------

void test_parse_valid_tlm(void) {
    const char* line = "TLM,test*0BBF02DA";
    frame::Parsed p;
    TEST_ASSERT_TRUE(frame::parse(line, strlen(line), &p));
    TEST_ASSERT_EQUAL((int)frame::Type::TLM, (int)p.type);
    TEST_ASSERT_EQUAL_size_t(8, p.payload_len);
    TEST_ASSERT_EQUAL_STRING_LEN("TLM,test", p.payload, p.payload_len);
}

void test_parse_valid_cmd(void) {
    char buf[64];
    size_t n = frame::build("CMD,START_C1,", buf, sizeof(buf));
    frame::Parsed p;
    TEST_ASSERT_TRUE(frame::parse(buf, n, &p));
    TEST_ASSERT_EQUAL((int)frame::Type::CMD, (int)p.type);
}

void test_parse_valid_evt(void) {
    char buf[64];
    size_t n = frame::build("EVT,C1,STATE,FULL", buf, sizeof(buf));
    frame::Parsed p;
    TEST_ASSERT_TRUE(frame::parse(buf, n, &p));
    TEST_ASSERT_EQUAL((int)frame::Type::EVT, (int)p.type);
}

void test_parse_rejects_bad_checksum(void) {
    const char* line = "TLM,test*DEADBEEF";
    frame::Parsed p;
    TEST_ASSERT_FALSE(frame::parse(line, strlen(line), &p));
}

void test_parse_rejects_missing_star(void) {
    const char* line = "TLM,test";
    frame::Parsed p;
    TEST_ASSERT_FALSE(frame::parse(line, strlen(line), &p));
}

void test_parse_rejects_short_input(void) {
    frame::Parsed p;
    TEST_ASSERT_FALSE(frame::parse("X", 1, &p));
    TEST_ASSERT_FALSE(frame::parse("", 0, &p));
}

void test_parse_rejects_non_hex_checksum(void) {
    const char* line = "TLM,test*0BBFZZZZ";
    frame::Parsed p;
    TEST_ASSERT_FALSE(frame::parse(line, strlen(line), &p));
}

void test_parse_unknown_type_still_validates(void) {
    char buf[64];
    size_t n = frame::build("XYZ,foo", buf, sizeof(buf));
    frame::Parsed p;
    TEST_ASSERT_TRUE(frame::parse(buf, n, &p));
    TEST_ASSERT_EQUAL((int)frame::Type::UNKNOWN, (int)p.type);
}

void test_parse_strips_lf(void) {
    const char* line = "TLM,test*0BBF02DA\n";
    frame::Parsed p;
    TEST_ASSERT_TRUE(frame::parse(line, strlen(line), &p));
}

void test_parse_strips_crlf(void) {
    const char* line = "TLM,test*0BBF02DA\r\n";
    frame::Parsed p;
    TEST_ASSERT_TRUE(frame::parse(line, strlen(line), &p));
}

void test_parse_accepts_lowercase_hex(void) {
    const char* line = "TLM,test*0bbf02da";
    frame::Parsed p;
    TEST_ASSERT_TRUE(frame::parse(line, strlen(line), &p));
}

void test_parse_rejects_oversized(void) {
    char big[frame::MAX_FRAME + 5];
    memset(big, 'X', sizeof(big));
    big[sizeof(big) - 9] = '*';
    memcpy(big + sizeof(big) - 8, "00000001", 8);  // fake checksum
    frame::Parsed p;
    TEST_ASSERT_FALSE(frame::parse(big, sizeof(big), &p));
}

// ---------- roundtrip ----------

void test_roundtrip(void) {
    const char* original = "EVT,C1,STATE,SAMPLING";
    char buf[64];
    size_t n = frame::build(original, buf, sizeof(buf));
    TEST_ASSERT_GREATER_THAN(0, n);

    frame::Parsed p;
    TEST_ASSERT_TRUE(frame::parse(buf, n, &p));
    TEST_ASSERT_EQUAL((int)frame::Type::EVT, (int)p.type);
    TEST_ASSERT_EQUAL_size_t(strlen(original), p.payload_len);
    TEST_ASSERT_EQUAL_STRING_LEN(original, p.payload, p.payload_len);
}

void setup() {
    delay(2000);  // give the serial monitor time to attach
    UNITY_BEGIN();

    RUN_TEST(test_adler32_empty);
    RUN_TEST(test_adler32_a);
    RUN_TEST(test_adler32_abc);
    RUN_TEST(test_adler32_tlm_test);

    RUN_TEST(test_build_simple_payload);
    RUN_TEST(test_build_appends_newline);
    RUN_TEST(test_build_refuses_oversized_payload);
    RUN_TEST(test_build_refuses_small_outbuf);

    RUN_TEST(test_parse_valid_tlm);
    RUN_TEST(test_parse_valid_cmd);
    RUN_TEST(test_parse_valid_evt);
    RUN_TEST(test_parse_rejects_bad_checksum);
    RUN_TEST(test_parse_rejects_missing_star);
    RUN_TEST(test_parse_rejects_short_input);
    RUN_TEST(test_parse_rejects_non_hex_checksum);
    RUN_TEST(test_parse_unknown_type_still_validates);
    RUN_TEST(test_parse_strips_lf);
    RUN_TEST(test_parse_strips_crlf);
    RUN_TEST(test_parse_accepts_lowercase_hex);
    RUN_TEST(test_parse_rejects_oversized);

    RUN_TEST(test_roundtrip);

    UNITY_END();
}

void loop() {}
