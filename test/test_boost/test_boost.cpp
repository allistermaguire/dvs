// Host-side unit tests for the pure timed-boost logic: `pio test -e native`.
//
// These exercise the decision helpers in boost.cpp - no Arduino/bus/millis. They
// pin the powered-gated max-speed overlay, the wrap-safe active window and the
// round-up of the remaining-minutes report.

#include <unity.h>

#include "boost.h"

void setUp() {}
void tearDown() {}

// --- Transmit-speed overlay -------------------------------------------------

void test_tx_speed_boost_and_powered_goes_to_max() {
    // Boost active and the fan running: override the base target up to max.
    TEST_ASSERT_EQUAL_UINT8(84, dvsBoostTxSpeed(true, true, 30, 84));
}

void test_tx_speed_boost_but_fan_off_holds_base() {
    // Boost active but the fan is off (base 0): boost only elevates a running
    // fan, so nothing is driven onto the bus.
    TEST_ASSERT_EQUAL_UINT8(0, dvsBoostTxSpeed(true, false, 0, 84));
}

void test_tx_speed_no_boost_holds_base() {
    // No boost: transmit the commanded base speed regardless of power.
    TEST_ASSERT_EQUAL_UINT8(30, dvsBoostTxSpeed(false, true, 30, 84));
    TEST_ASSERT_EQUAL_UINT8(0, dvsBoostTxSpeed(false, false, 0, 84));
}

// --- Active window ----------------------------------------------------------

void test_active_false_when_no_deadline() {
    // untilMs == 0 means no boost, even far into uptime.
    TEST_ASSERT_FALSE(dvsBoostActive(1000, 0));
    TEST_ASSERT_FALSE(dvsBoostActive(0x80000000u, 0));
}

void test_active_true_before_deadline() {
    TEST_ASSERT_TRUE(dvsBoostActive(1000, 2000));
}

void test_active_false_at_and_after_deadline() {
    TEST_ASSERT_FALSE(dvsBoostActive(2000, 2000)); // exactly reached
    TEST_ASSERT_FALSE(dvsBoostActive(3000, 2000)); // past
}

void test_active_wrap_safe_across_millis_overflow() {
    // Deadline is ~66 ms after now but the counter wraps in between: now is just
    // below 2^32, until has already wrapped to a small value. Still "active".
    TEST_ASSERT_TRUE(dvsBoostActive(0xFFFFFFF0u, 50u));
}

// --- Remaining minutes (rounded up) -----------------------------------------

void test_remaining_zero_when_inactive() {
    TEST_ASSERT_EQUAL_UINT16(0, dvsBoostRemainingMin(5000, 0));   // no deadline
    TEST_ASSERT_EQUAL_UINT16(0, dvsBoostRemainingMin(2000, 2000)); // reached
}

void test_remaining_exact_minute() {
    // Exactly one minute left reports 1 (not 2).
    TEST_ASSERT_EQUAL_UINT16(1, dvsBoostRemainingMin(0, 60000));
}

void test_remaining_rounds_up() {
    // Any fraction over a whole minute rounds up.
    TEST_ASSERT_EQUAL_UINT16(2, dvsBoostRemainingMin(0, 60001));  // just over 1
    TEST_ASSERT_EQUAL_UINT16(2, dvsBoostRemainingMin(0, 90000));  // 1.5 min
    TEST_ASSERT_EQUAL_UINT16(1, dvsBoostRemainingMin(0, 1));      // ~0 -> 1
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_tx_speed_boost_and_powered_goes_to_max);
    RUN_TEST(test_tx_speed_boost_but_fan_off_holds_base);
    RUN_TEST(test_tx_speed_no_boost_holds_base);
    RUN_TEST(test_active_false_when_no_deadline);
    RUN_TEST(test_active_true_before_deadline);
    RUN_TEST(test_active_false_at_and_after_deadline);
    RUN_TEST(test_active_wrap_safe_across_millis_overflow);
    RUN_TEST(test_remaining_zero_when_inactive);
    RUN_TEST(test_remaining_exact_minute);
    RUN_TEST(test_remaining_rounds_up);
    return UNITY_END();
}
