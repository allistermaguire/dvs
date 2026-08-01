// Host-side unit tests for the shared argument parsers: `pio test -e native`.
//
// These pin the strict-rejection behaviour: a malformed numeric/mode/profile
// token must be rejected (leaving the caller's value untouched) rather than
// coerced to 0/defaults, so a bad MQTT payload cannot silently power the fan off
// or feed a bogus 0 C into auto control.

#include <unity.h>

#include "parse.h"

void setUp() {}
void tearDown() {}

// --- Tri-state (on/off/toggle) -----------------------------------------------

void test_tristate_on_off_toggle_invalid() {
    TEST_ASSERT_EQUAL_INT(DVS_TRI_ON, dvsParseTriState("on"));
    TEST_ASSERT_EQUAL_INT(DVS_TRI_ON, dvsParseTriState("1"));
    TEST_ASSERT_EQUAL_INT(DVS_TRI_OFF, dvsParseTriState("off"));
    TEST_ASSERT_EQUAL_INT(DVS_TRI_OFF, dvsParseTriState("LISTEN"));
    TEST_ASSERT_EQUAL_INT(DVS_TRI_TOGGLE, dvsParseTriState(""));
    TEST_ASSERT_EQUAL_INT(DVS_TRI_INVALID, dvsParseTriState("maybe"));
}

// --- Integer range parser ----------------------------------------------------

void test_int_range_accepts_valid() {
    long v = -1;
    TEST_ASSERT_TRUE(dvsParseIntRange("42", 0, 84, &v));
    TEST_ASSERT_EQUAL_INT(42, v);
    TEST_ASSERT_TRUE(dvsParseIntRange("0", 0, 84, &v));
    TEST_ASSERT_EQUAL_INT(0, v);
    TEST_ASSERT_TRUE(dvsParseIntRange("84", 0, 84, &v));
    TEST_ASSERT_EQUAL_INT(84, v);
}

void test_int_range_rejects_and_preserves_out() {
    long v = 7; // sentinel: must be untouched on every rejection
    TEST_ASSERT_FALSE(dvsParseIntRange("", 0, 84, &v));
    TEST_ASSERT_FALSE(dvsParseIntRange("abc", 0, 84, &v));
    TEST_ASSERT_FALSE(dvsParseIntRange("12x", 0, 84, &v)); // trailing junk
    TEST_ASSERT_FALSE(dvsParseIntRange("85", 0, 84, &v));  // above range
    TEST_ASSERT_FALSE(dvsParseIntRange("-1", 0, 84, &v));  // below range
    TEST_ASSERT_EQUAL_INT(7, v);
}

void test_int_range_allows_trailing_whitespace() {
    long v = -1;
    TEST_ASSERT_TRUE(dvsParseIntRange("42 \r\n", 0, 84, &v));
    TEST_ASSERT_EQUAL_INT(42, v);
}

// --- Float parser ------------------------------------------------------------

void test_float_accepts_valid() {
    float f = 0.0f;
    TEST_ASSERT_TRUE(dvsParseFloat("18.5", &f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 18.5f, f);
    TEST_ASSERT_TRUE(dvsParseFloat("-3.25", &f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -3.25f, f);
}

void test_float_rejects_and_preserves_out() {
    float f = 99.0f; // sentinel
    TEST_ASSERT_FALSE(dvsParseFloat("", &f));
    TEST_ASSERT_FALSE(dvsParseFloat("hot", &f));
    TEST_ASSERT_FALSE(dvsParseFloat("1.5c", &f)); // trailing junk
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 99.0f, f);
}

// --- Mode / profile parsers --------------------------------------------------

void test_mode_parser() {
    DvsControlMode m = DVS_MODE_AUTO;
    TEST_ASSERT_TRUE(dvsParseMode("manual", &m));
    TEST_ASSERT_EQUAL_INT(DVS_MODE_MANUAL, m);
    TEST_ASSERT_TRUE(dvsParseMode("AUTO", &m));
    TEST_ASSERT_EQUAL_INT(DVS_MODE_AUTO, m);
    TEST_ASSERT_FALSE(dvsParseMode("bogus", &m));
    TEST_ASSERT_EQUAL_INT(DVS_MODE_AUTO, m); // untouched on reject
}

void test_profile_parser_rejects_unknown() {
    DvsProfileSel s = DVS_PROFILE_SEL_WINTER;
    TEST_ASSERT_TRUE(dvsParseProfile("summer", &s));
    TEST_ASSERT_EQUAL_INT(DVS_PROFILE_SEL_SUMMER, s);
    TEST_ASSERT_TRUE(dvsParseProfile("auto", &s));
    TEST_ASSERT_EQUAL_INT(DVS_PROFILE_SEL_AUTO, s);
    // An unrecognised profile is rejected, not silently coerced to auto.
    TEST_ASSERT_FALSE(dvsParseProfile("spring", &s));
    TEST_ASSERT_EQUAL_INT(DVS_PROFILE_SEL_AUTO, s);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_tristate_on_off_toggle_invalid);
    RUN_TEST(test_int_range_accepts_valid);
    RUN_TEST(test_int_range_rejects_and_preserves_out);
    RUN_TEST(test_int_range_allows_trailing_whitespace);
    RUN_TEST(test_float_accepts_valid);
    RUN_TEST(test_float_rejects_and_preserves_out);
    RUN_TEST(test_mode_parser);
    RUN_TEST(test_profile_parser_rejects_unknown);
    return UNITY_END();
}
