// Unit tests for the Sampler per-tank FSM.
//
// Run on-target:
//     pio test -e esp32doit-devkit-v1 -f test_sampler
//
// The Sampler is driven by a Clock — tests advance a TestClock manually
// instead of waiting for real time. This makes the whole transition
// matrix runnable in milliseconds.

#include <Arduino.h>
#include <unity.h>

#include "Clock.h"
#include "Sampler.h"

static TestClock clk;

void setUp(void) {
    clk.set(0);
}
void tearDown(void) {}

// Convenience: drain the change flags so subsequent tests start clean.
static void drain_changes(Sampler& s) {
    (void)s.consume_state_change();
    (void)s.consume_step_change();
}

// Drive Sampler through one full happy-path cycle.
static void run_to_full(Sampler& s) {
    TEST_ASSERT_TRUE(s.request_start());
    clk.advance(mock_timing::DESCENDING_MS); s.tick();
    clk.advance(mock_timing::IN_WATER_MS);   s.tick();
    clk.advance(mock_timing::PUMPING_MS);    s.tick();
    clk.advance(mock_timing::ASCENDING_MS);  s.tick();
    clk.advance(mock_timing::HOME_MS);       s.tick();
}

// ---------- initial state ----------

void test_initial_state(void) {
    Sampler s(1, clk);
    TEST_ASSERT_EQUAL((int)TankState::EMPTY, (int)s.state());
    TEST_ASSERT_EQUAL((int)TankStep::NONE,   (int)s.step());
}

// ---------- request_start ----------

void test_start_from_empty(void) {
    Sampler s(1, clk);
    TEST_ASSERT_TRUE(s.request_start());
    TEST_ASSERT_EQUAL((int)TankState::SAMPLING,   (int)s.state());
    TEST_ASSERT_EQUAL((int)TankStep::DESCENDING,  (int)s.step());
}

void test_start_rejected_when_sampling(void) {
    Sampler s(1, clk);
    s.request_start();
    drain_changes(s);
    TEST_ASSERT_FALSE(s.request_start());
}

void test_start_rejected_when_full(void) {
    Sampler s(1, clk);
    run_to_full(s);
    TEST_ASSERT_EQUAL((int)TankState::FULL, (int)s.state());
    TEST_ASSERT_FALSE(s.request_start());
}

void test_start_rejected_when_fault(void) {
    Sampler s(1, clk);
    s.abort();
    TEST_ASSERT_FALSE(s.request_start());
}

// ---------- step transitions ----------

void test_step_progresses_after_descending_timer(void) {
    Sampler s(1, clk);
    s.request_start();
    drain_changes(s);

    clk.advance(mock_timing::DESCENDING_MS - 1);
    s.tick();
    TEST_ASSERT_EQUAL((int)TankStep::DESCENDING, (int)s.step());

    clk.advance(1);
    s.tick();
    TEST_ASSERT_EQUAL((int)TankStep::IN_WATER, (int)s.step());
}

void test_full_happy_path(void) {
    Sampler s(1, clk);
    run_to_full(s);
    TEST_ASSERT_EQUAL((int)TankState::FULL, (int)s.state());
    TEST_ASSERT_EQUAL((int)TankStep::NONE,  (int)s.step());
}

void test_step_emits_change_per_transition(void) {
    Sampler s(1, clk);
    s.request_start();
    // After start: state EMPTY->SAMPLING and step NONE->DESCENDING both changed
    TEST_ASSERT_TRUE(s.consume_state_change());
    TEST_ASSERT_TRUE(s.consume_step_change());
    // Flags consumed — second read returns false.
    TEST_ASSERT_FALSE(s.consume_state_change());
    TEST_ASSERT_FALSE(s.consume_step_change());

    // Advance one full step
    clk.advance(mock_timing::DESCENDING_MS);
    s.tick();
    TEST_ASSERT_TRUE(s.consume_step_change());
    TEST_ASSERT_FALSE(s.consume_state_change());  // still SAMPLING
}

// ---------- abort / reset ----------

void test_abort_from_sampling(void) {
    Sampler s(1, clk);
    s.request_start();
    s.abort();
    TEST_ASSERT_EQUAL((int)TankState::FAULT, (int)s.state());
    TEST_ASSERT_EQUAL((int)TankStep::NONE,   (int)s.step());
}

void test_abort_from_full(void) {
    Sampler s(1, clk);
    run_to_full(s);
    s.abort();
    TEST_ASSERT_EQUAL((int)TankState::FAULT, (int)s.state());
}

void test_abort_is_idempotent(void) {
    Sampler s(1, clk);
    s.abort();
    drain_changes(s);
    s.abort();
    TEST_ASSERT_FALSE(s.consume_state_change());
}

void test_reset_from_full(void) {
    Sampler s(1, clk);
    run_to_full(s);
    TEST_ASSERT_TRUE(s.reset());
    TEST_ASSERT_EQUAL((int)TankState::EMPTY, (int)s.state());
}

void test_reset_from_fault(void) {
    Sampler s(1, clk);
    s.abort();
    TEST_ASSERT_TRUE(s.reset());
    TEST_ASSERT_EQUAL((int)TankState::EMPTY, (int)s.state());
}

void test_reset_refused_while_sampling(void) {
    Sampler s(1, clk);
    s.request_start();
    TEST_ASSERT_FALSE(s.reset());
    TEST_ASSERT_EQUAL((int)TankState::SAMPLING, (int)s.state());
}

void test_reset_refused_when_already_empty(void) {
    Sampler s(1, clk);
    TEST_ASSERT_FALSE(s.reset());
}

// ---------- string helpers ----------

void test_state_names(void) {
    TEST_ASSERT_EQUAL_STRING("EMPTY",    state_name(TankState::EMPTY));
    TEST_ASSERT_EQUAL_STRING("SAMPLING", state_name(TankState::SAMPLING));
    TEST_ASSERT_EQUAL_STRING("FULL",     state_name(TankState::FULL));
    TEST_ASSERT_EQUAL_STRING("FAULT",    state_name(TankState::FAULT));
}

void test_step_names(void) {
    TEST_ASSERT_EQUAL_STRING("DESCENDING", step_name(TankStep::DESCENDING));
    TEST_ASSERT_EQUAL_STRING("IN_WATER",   step_name(TankStep::IN_WATER));
    TEST_ASSERT_EQUAL_STRING("PUMPING",    step_name(TankStep::PUMPING));
    TEST_ASSERT_EQUAL_STRING("ASCENDING",  step_name(TankStep::ASCENDING));
    TEST_ASSERT_EQUAL_STRING("HOME",       step_name(TankStep::HOME));
}

void setup() {
    delay(2000);
    UNITY_BEGIN();

    RUN_TEST(test_initial_state);

    RUN_TEST(test_start_from_empty);
    RUN_TEST(test_start_rejected_when_sampling);
    RUN_TEST(test_start_rejected_when_full);
    RUN_TEST(test_start_rejected_when_fault);

    RUN_TEST(test_step_progresses_after_descending_timer);
    RUN_TEST(test_full_happy_path);
    RUN_TEST(test_step_emits_change_per_transition);

    RUN_TEST(test_abort_from_sampling);
    RUN_TEST(test_abort_from_full);
    RUN_TEST(test_abort_is_idempotent);

    RUN_TEST(test_reset_from_full);
    RUN_TEST(test_reset_from_fault);
    RUN_TEST(test_reset_refused_while_sampling);
    RUN_TEST(test_reset_refused_when_already_empty);

    RUN_TEST(test_state_names);
    RUN_TEST(test_step_names);

    UNITY_END();
}

void loop() {}
