// Host-side unit tests for the ventilation control profiles: `pio test -e native`.
//
// These exercise the pure decision logic in profile.cpp - no Arduino/bus. They
// pin the moisture-first winter behaviour (still ventilate at ~13 C), the cold
// and warm cutoffs, the summer free-cooling gates, hysteresis and auto-select.

#include <unity.h>

#include "profile.h"

void setUp() {}
void tearDown() {}

// Wellington defaults (mirrors the boot config in config.h). Kept local so the
// tests document the intended setpoints independently of the firmware header.
static DvsProfileConfig cfg() {
    DvsProfileConfig c;
    c.deadbandOnC = 1.5f;
    c.deadbandOffC = 0.5f;
    c.spanC = 5.0f;
    c.comfortHighC = 21.0f;
    c.comfortLowC = 18.0f;
    c.minSupplyC = 12.0f;
    c.maxSupplyC = 24.0f;
    c.minSpeed = 12;
    c.maxSpeed = 84;
    return c;
}

static DvsProfileInputs winter(float roof, float inside, bool moisture) {
    DvsProfileInputs in;
    in.roofTempC = roof;
    in.insideTempC = inside;
    in.moistureFloor = moisture;
    in.profile = DVS_PROFILE_WINTER;
    return in;
}

static DvsProfileInputs summer(float roof, float inside) {
    DvsProfileInputs in;
    in.roofTempC = roof;
    in.insideTempC = inside;
    in.moistureFloor = false;
    in.profile = DVS_PROFILE_SUMMER;
    return in;
}

// --- Winter: moisture floor -------------------------------------------------

void test_winter_moisture_runs_at_13C_not_adding_heat() {
    // Roof 13 C, inside 19 C: not adding heat, but above the 12 C cutoff, so the
    // moisture floor keeps a trickle going (matches the real unit's behaviour).
    const uint8_t s = dvsProfileSpeed(winter(13.0f, 19.0f, true), cfg(), 0);
    TEST_ASSERT_EQUAL_UINT8(12, s); // exactly minSpeed, no heating boost
}

void test_winter_moisture_off_below_cold_cutoff() {
    // Roof 11 C is below minSupply (12 C): too cold to dump, fan off even with
    // the moisture floor enabled.
    TEST_ASSERT_EQUAL_UINT8(0, dvsProfileSpeed(winter(11.0f, 19.0f, true), cfg(), 30));
}

void test_winter_off_when_house_already_warm() {
    // Inside >= comfortHigh (21 C): stop, regardless of roof temp / moisture.
    TEST_ASSERT_EQUAL_UINT8(0, dvsProfileSpeed(winter(25.0f, 21.0f, true), cfg(), 40));
}

void test_winter_heat_only_no_floor_requires_benefit() {
    // Moisture floor OFF, roof 13 C, inside 19 C: no warming benefit -> off.
    TEST_ASSERT_EQUAL_UINT8(0, dvsProfileSpeed(winter(13.0f, 19.0f, false), cfg(), 0));
}

void test_winter_heating_boost_scales_with_benefit() {
    // Roof 24 C, inside 19 C: benefit = 5 C == span -> full boost to maxSpeed.
    const uint8_t s = dvsProfileSpeed(winter(24.0f, 19.0f, true), cfg(), 12);
    TEST_ASSERT_EQUAL_UINT8(84, s);
}

void test_winter_heating_partial_boost() {
    // Roof 21.5 C, inside 19 C: benefit = 2.5 C. frac = (2.5-0.5)/(5-0.5) = 0.444.
    // boost = round(0.444 * 72) = 32 -> 12 + 32 = 44.
    const uint8_t s = dvsProfileSpeed(winter(21.5f, 19.0f, true), cfg(), 12);
    TEST_ASSERT_EQUAL_UINT8(44, s);
}

// --- Summer: free cooling --------------------------------------------------

void test_summer_cooling_with_cool_air() {
    // Roof 16 C, inside 23 C: benefit = 7 C > span -> full cooling to maxSpeed.
    const uint8_t s = dvsProfileSpeed(summer(16.0f, 23.0f), cfg(), 0);
    TEST_ASSERT_EQUAL_UINT8(84, s);
}

void test_summer_blocks_hot_roof_air() {
    // Roof 30 C > maxSupply (24 C): never bring in hot midday air.
    TEST_ASSERT_EQUAL_UINT8(0, dvsProfileSpeed(summer(30.0f, 23.0f), cfg(), 50));
}

void test_summer_off_when_house_already_cool() {
    // Inside <= comfortLow (18 C): don't cool below the sleeping setpoint.
    TEST_ASSERT_EQUAL_UINT8(0, dvsProfileSpeed(summer(15.0f, 18.0f), cfg(), 40));
}

// --- Hysteresis -------------------------------------------------------------

void test_summer_hysteresis_start_vs_keep_running() {
    // Benefit = 1.0 C: between deadbandOff (0.5) and deadbandOn (1.5).
    // Stopped fan (cur=0) must NOT start; running fan (cur>0) must keep going.
    TEST_ASSERT_EQUAL_UINT8(0, dvsProfileSpeed(summer(21.0f, 22.0f), cfg(), 0));
    TEST_ASSERT_TRUE(dvsProfileSpeed(summer(21.0f, 22.0f), cfg(), 20) > 0);
}

// --- Auto profile selection -------------------------------------------------

void test_auto_selects_summer_when_warm() {
    TEST_ASSERT_EQUAL_INT(DVS_PROFILE_SUMMER,
                          dvsProfileAuto(21.0f, 19.0f, DVS_PROFILE_WINTER));
}

void test_auto_selects_winter_when_cool() {
    TEST_ASSERT_EQUAL_INT(DVS_PROFILE_WINTER,
                          dvsProfileAuto(17.0f, 19.0f, DVS_PROFILE_SUMMER));
}

void test_auto_holds_profile_in_deadband() {
    // Inside within +/-0.5 C of the 19 C target: keep whatever we had.
    TEST_ASSERT_EQUAL_INT(DVS_PROFILE_SUMMER,
                          dvsProfileAuto(19.2f, 19.0f, DVS_PROFILE_SUMMER));
    TEST_ASSERT_EQUAL_INT(DVS_PROFILE_WINTER,
                          dvsProfileAuto(19.2f, 19.0f, DVS_PROFILE_WINTER));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_winter_moisture_runs_at_13C_not_adding_heat);
    RUN_TEST(test_winter_moisture_off_below_cold_cutoff);
    RUN_TEST(test_winter_off_when_house_already_warm);
    RUN_TEST(test_winter_heat_only_no_floor_requires_benefit);
    RUN_TEST(test_winter_heating_boost_scales_with_benefit);
    RUN_TEST(test_winter_heating_partial_boost);
    RUN_TEST(test_summer_cooling_with_cool_air);
    RUN_TEST(test_summer_blocks_hot_roof_air);
    RUN_TEST(test_summer_off_when_house_already_cool);
    RUN_TEST(test_summer_hysteresis_start_vs_keep_running);
    RUN_TEST(test_auto_selects_summer_when_warm);
    RUN_TEST(test_auto_selects_winter_when_cool);
    RUN_TEST(test_auto_holds_profile_in_deadband);
    return UNITY_END();
}
