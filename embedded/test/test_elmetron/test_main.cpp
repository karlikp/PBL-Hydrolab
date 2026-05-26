// Unit tests for the Elmetron measurement-side FSM.
//
// Run on-target:
//     pio test -e esp32doit-devkit-v1 -f test_elmetron
//
// Drives the FSM through its lifecycle and pokes at the synthetic
// reading model (linear ramp during settle, steady-state and
// measurement_valid=true thereafter, zeros / invalid in every other
// state). Same TestClock idiom as test_sampler.

#include <Arduino.h>
#include <unity.h>
#include <math.h>

#include "Clock.h"
#include "Elmetron.h"

static TestClock clk;

void setUp(void) {
    clk.set(0);
}
void tearDown(void) {}

static void drain_changes(Elmetron& e) {
    (void)e.consume_state_change();
    (void)e.consume_step_change();
}

// Advance through one full happy-path cycle: DOCKED → MEASURING →
// HOMING → DESCENDING → IN_WATER (settle + window) → ASCENDING →
// HOME → DOCKED.
static void run_to_done(Elmetron& e) {
    TEST_ASSERT_TRUE(e.request_start());
    clk.advance(elmetron_timing::HOMING_MAX_MS);                 e.tick();
    clk.advance(elmetron_timing::DESCENDING_MS);                 e.tick();
    clk.advance(elmetron_timing::SETTLE_MS
              + elmetron_timing::MEASURING_WINDOW_MS);           e.tick();
    clk.advance(elmetron_timing::ASCENDING_MS);                  e.tick();
    clk.advance(elmetron_timing::HOME_MS);                       e.tick();
}

// ---------- initial state ----------

void test_initial_state(void) {
    Elmetron e(clk);
    TEST_ASSERT_EQUAL((int)ElmetronState::DOCKED, (int)e.state());
    TEST_ASSERT_EQUAL((int)ElmetronStep::NONE,    (int)e.step());
}

void test_initial_reading_is_zero_and_invalid(void) {
    Elmetron e(clk);
    const auto r = e.last_reading();
    TEST_ASSERT_FALSE(r.valid);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r.cond);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r.temp);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r.ph);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r.oxygen);
}

// ---------- request_start ----------

void test_start_from_docked(void) {
    Elmetron e(clk);
    TEST_ASSERT_TRUE(e.request_start());
    TEST_ASSERT_EQUAL((int)ElmetronState::MEASURING,  (int)e.state());
    // Cycle always starts with HOMING — safe-start phase that runs
    // before DESCENDING (see Elmetron.h preamble).
    TEST_ASSERT_EQUAL((int)ElmetronStep::HOMING,      (int)e.step());
}

void test_homing_progresses_to_descending(void) {
    Elmetron e(clk);
    e.request_start();
    drain_changes(e);

    clk.advance(elmetron_timing::HOMING_MAX_MS - 1);
    e.tick();
    TEST_ASSERT_EQUAL((int)ElmetronStep::HOMING, (int)e.step());

    clk.advance(1);
    e.tick();
    TEST_ASSERT_EQUAL((int)ElmetronStep::DESCENDING, (int)e.step());
}

void test_start_rejected_when_measuring(void) {
    Elmetron e(clk);
    e.request_start();
    TEST_ASSERT_FALSE(e.request_start());
}

void test_start_rejected_when_fault(void) {
    Elmetron e(clk);
    e.abort();
    TEST_ASSERT_FALSE(e.request_start());
}

// ---------- step transitions ----------

// Helper: advance past HOMING so the FSM lands in DESCENDING. Used
// by the tests below that exercise post-HOMING behaviour.
static void advance_past_homing(Elmetron& e) {
    clk.advance(elmetron_timing::HOMING_MAX_MS);
    e.tick();
}

void test_descending_progresses_to_in_water(void) {
    Elmetron e(clk);
    e.request_start();
    advance_past_homing(e);
    drain_changes(e);

    clk.advance(elmetron_timing::DESCENDING_MS - 1);
    e.tick();
    TEST_ASSERT_EQUAL((int)ElmetronStep::DESCENDING, (int)e.step());

    clk.advance(1);
    e.tick();
    TEST_ASSERT_EQUAL((int)ElmetronStep::IN_WATER, (int)e.step());
}

void test_in_water_holds_through_settle_and_window(void) {
    Elmetron e(clk);
    e.request_start();
    advance_past_homing(e);
    clk.advance(elmetron_timing::DESCENDING_MS); e.tick();
    drain_changes(e);

    // Still IN_WATER one tick before the full window elapses.
    clk.advance(elmetron_timing::SETTLE_MS
              + elmetron_timing::MEASURING_WINDOW_MS - 1);
    e.tick();
    TEST_ASSERT_EQUAL((int)ElmetronStep::IN_WATER, (int)e.step());

    clk.advance(1);
    e.tick();
    TEST_ASSERT_EQUAL((int)ElmetronStep::ASCENDING, (int)e.step());
}

void test_full_cycle_returns_to_docked(void) {
    Elmetron e(clk);
    run_to_done(e);
    TEST_ASSERT_EQUAL((int)ElmetronState::DOCKED, (int)e.state());
    TEST_ASSERT_EQUAL((int)ElmetronStep::NONE,    (int)e.step());
}

void test_step_change_emits_one_pulse_per_transition(void) {
    Elmetron e(clk);
    e.request_start();
    TEST_ASSERT_TRUE(e.consume_state_change());
    TEST_ASSERT_TRUE(e.consume_step_change());
    TEST_ASSERT_FALSE(e.consume_state_change());
    TEST_ASSERT_FALSE(e.consume_step_change());

    // HOMING → DESCENDING transition: step change pulses, no state change.
    clk.advance(elmetron_timing::HOMING_MAX_MS);
    e.tick();
    TEST_ASSERT_TRUE(e.consume_step_change());
    TEST_ASSERT_FALSE(e.consume_state_change());  // still MEASURING
}

// ---------- reading evolution ----------

void test_reading_invalid_during_descending(void) {
    Elmetron e(clk);
    e.request_start();
    advance_past_homing(e);
    clk.advance(elmetron_timing::DESCENDING_MS / 2);
    e.tick();
    const auto r = e.last_reading();
    TEST_ASSERT_FALSE(r.valid);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r.cond);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r.temp);
}

void test_reading_invalid_during_homing(void) {
    Elmetron e(clk);
    e.request_start();
    clk.advance(elmetron_timing::HOMING_MAX_MS / 2);
    const auto r = e.last_reading();
    // HOMING is before the probe touches water — zeros, invalid.
    TEST_ASSERT_FALSE(r.valid);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r.cond);
}

void test_reading_ramps_during_settle(void) {
    Elmetron e(clk);
    e.request_start();
    advance_past_homing(e);
    clk.advance(elmetron_timing::DESCENDING_MS); e.tick();
    // We are now at the start of IN_WATER. Halfway into the settle
    // window, the synthetic reading should be ~half the steady-state.
    clk.advance(elmetron_timing::SETTLE_MS / 2);
    const auto r = e.last_reading();
    TEST_ASSERT_FALSE(r.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.5f * elmetron_synthetic::COND_STEADY * 0.05f,
                             0.5f * elmetron_synthetic::COND_STEADY, r.cond);
    TEST_ASSERT_FLOAT_WITHIN(0.5f * elmetron_synthetic::TEMP_STEADY * 0.05f,
                             0.5f * elmetron_synthetic::TEMP_STEADY, r.temp);
}

void test_reading_valid_after_settle(void) {
    Elmetron e(clk);
    e.request_start();
    advance_past_homing(e);
    clk.advance(elmetron_timing::DESCENDING_MS); e.tick();
    clk.advance(elmetron_timing::SETTLE_MS);
    const auto r = e.last_reading();
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_EQUAL_FLOAT(elmetron_synthetic::COND_STEADY,   r.cond);
    TEST_ASSERT_EQUAL_FLOAT(elmetron_synthetic::TEMP_STEADY,   r.temp);
    TEST_ASSERT_EQUAL_FLOAT(elmetron_synthetic::PH_STEADY,     r.ph);
    TEST_ASSERT_EQUAL_FLOAT(elmetron_synthetic::OXYGEN_STEADY, r.oxygen);
}

void test_reading_invalid_during_ascending(void) {
    Elmetron e(clk);
    e.request_start();
    advance_past_homing(e);
    clk.advance(elmetron_timing::DESCENDING_MS); e.tick();
    clk.advance(elmetron_timing::SETTLE_MS
              + elmetron_timing::MEASURING_WINDOW_MS); e.tick();
    // Now in ASCENDING. Probe out of water — reading should be zero
    // and invalid regardless of how long we wait.
    clk.advance(elmetron_timing::ASCENDING_MS / 2);
    const auto r = e.last_reading();
    TEST_ASSERT_FALSE(r.valid);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r.cond);
}

// ---------- abort / reset ----------

void test_abort_from_measuring(void) {
    Elmetron e(clk);
    e.request_start();
    e.abort();
    TEST_ASSERT_EQUAL((int)ElmetronState::FAULT, (int)e.state());
    TEST_ASSERT_EQUAL((int)ElmetronStep::NONE,   (int)e.step());
}

void test_abort_is_idempotent(void) {
    Elmetron e(clk);
    e.abort();
    drain_changes(e);
    e.abort();
    TEST_ASSERT_FALSE(e.consume_state_change());
}

void test_reset_from_fault(void) {
    Elmetron e(clk);
    e.abort();
    TEST_ASSERT_TRUE(e.reset());
    TEST_ASSERT_EQUAL((int)ElmetronState::DOCKED, (int)e.state());
}

void test_reset_refused_while_measuring(void) {
    Elmetron e(clk);
    e.request_start();
    TEST_ASSERT_FALSE(e.reset());
    TEST_ASSERT_EQUAL((int)ElmetronState::MEASURING, (int)e.state());
}

void test_reset_refused_when_docked(void) {
    Elmetron e(clk);
    TEST_ASSERT_FALSE(e.reset());
}

// ---------- string helpers ----------

void test_state_names(void) {
    TEST_ASSERT_EQUAL_STRING("DOCKED",    state_name(ElmetronState::DOCKED));
    TEST_ASSERT_EQUAL_STRING("MEASURING", state_name(ElmetronState::MEASURING));
    TEST_ASSERT_EQUAL_STRING("FAULT",     state_name(ElmetronState::FAULT));
}

void test_step_names(void) {
    TEST_ASSERT_EQUAL_STRING("HOMING",     step_name(ElmetronStep::HOMING));
    TEST_ASSERT_EQUAL_STRING("DESCENDING", step_name(ElmetronStep::DESCENDING));
    TEST_ASSERT_EQUAL_STRING("IN_WATER",   step_name(ElmetronStep::IN_WATER));
    TEST_ASSERT_EQUAL_STRING("ASCENDING",  step_name(ElmetronStep::ASCENDING));
    TEST_ASSERT_EQUAL_STRING("HOME",       step_name(ElmetronStep::HOME));
}

void setup() {
    delay(2000);
    UNITY_BEGIN();

    RUN_TEST(test_initial_state);
    RUN_TEST(test_initial_reading_is_zero_and_invalid);

    RUN_TEST(test_start_from_docked);
    RUN_TEST(test_start_rejected_when_measuring);
    RUN_TEST(test_start_rejected_when_fault);

    RUN_TEST(test_homing_progresses_to_descending);
    RUN_TEST(test_descending_progresses_to_in_water);
    RUN_TEST(test_in_water_holds_through_settle_and_window);
    RUN_TEST(test_full_cycle_returns_to_docked);
    RUN_TEST(test_step_change_emits_one_pulse_per_transition);

    RUN_TEST(test_reading_invalid_during_homing);
    RUN_TEST(test_reading_invalid_during_descending);
    RUN_TEST(test_reading_ramps_during_settle);
    RUN_TEST(test_reading_valid_after_settle);
    RUN_TEST(test_reading_invalid_during_ascending);

    RUN_TEST(test_abort_from_measuring);
    RUN_TEST(test_abort_is_idempotent);
    RUN_TEST(test_reset_from_fault);
    RUN_TEST(test_reset_refused_while_measuring);
    RUN_TEST(test_reset_refused_when_docked);

    RUN_TEST(test_state_names);
    RUN_TEST(test_step_names);

    UNITY_END();
}

void loop() {}
