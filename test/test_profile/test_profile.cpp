// Host-side unit tests for the ventilation control profiles: `pio test -e native`.
//
// These exercise the pure decision logic in profile.cpp - no Arduino/bus. They
// pin the moisture-first winter behaviour (always ventilate at a basedline above 
// the cold cutoff), the optional heat-recover boost, the summer free-cooling 
// gates, hysteresis and auto-select.

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
    c.comfortHighC = 24.0f;
    c.comfortLowC = 18.0f;
    c.minSupplyC = 12.0f;
    c.maxSupplyC = 24.0f;
    c.supplyHystC = 1.5f;
    c.recoverStartC = -2.0f;
    c.recoverFullC = 4.0f;
    c.minSpeed = 12;
    c.moistureBaseSpeed = 10;
    c.maxSpeed = 84;
    return c;
}

static DvsProfileInputs winter(float roof, float inside, bool heatRecovery) {
    DvsProfileInputs in;
    in.roofTempC = roof;
    in.insideTempC = inside;
    in.heatRecovery = heatRecovery;
    in.profile = DVS_PROFILE_WINTER;
    return in;
}

static DvsProfileInputs summer(float roof, float inside) {
    DvsProfileInputs in;
    in.roofTempC = roof;
    in.insideTempC = inside;
    in.heatRecovery = false;
    in.profile = DVS_PROFILE_SUMMER;
    return in;
}

// --- Winter: moisture baseline + heat recovery -------------------------------

void test_winter_moisture_baseline_above_cutoff() {
    // Roof 13 C, inside 19 C: above the 12 C cutoff but too cold to recover heat,
    // so the always-on moisture baseline runs (matches the real unit venting for
    // condensation without adding heat).
    const uint8_t s = dvsProfileSpeed(winter(13.0f, 19.0f, true), cfg(), 0);
    TEST_ASSERT_EQUAL_UINT8(10, s); // moistureBaseSpeed, no heat-recovery boost
}

void test_winter_off_below_cold_cutoff() {
    // Roof 11 C is below minSupply (12 C): too cold to supply, fan off even with
    // heat recovery on (stopped fan uses the full cutoff).
    TEST_ASSERT_EQUAL_UINT8(0, dvsProfileSpeed(winter(11.0f, 19.0f, true), cfg(), 0));
}

void test_winter_supply_cutoff_hysteresis_no_flap() {
    // Roof 11.5 C: below minSupply (12 C) but within supplyHyst (1.5 C). A
    // running fan (cur>0) keeps its moisture baseline - it cooled its own reading
    // below the cutoff and must not flap off; a stopped fan (cur=0) stays off.
    TEST_ASSERT_EQUAL_UINT8(10, dvsProfileSpeed(winter(11.5f, 19.0f, true), cfg(), 10));
    TEST_ASSERT_EQUAL_UINT8(0, dvsProfileSpeed(winter(11.5f, 19.0f, true), cfg(), 0));
    // Below the hysteresis band (10 C < 12 - 1.5): even a running fan stops.
    TEST_ASSERT_EQUAL_UINT8(0, dvsProfileSpeed(winter(10.0f, 19.0f, true), cfg(), 10));
}

void test_winter_heat_recovery_off_holds_baseline() {
    // Heat recovery OFF with a warm roof (26 C vs 19 C): no boost, just hold the
    // low moisture baseline.
    TEST_ASSERT_EQUAL_UINT8(10, dvsProfileSpeed(winter(26.0f, 19.0f, false), cfg(), 40));
}

void test_winter_comfort_high_caps_boost() {
    // Inside >= comfortHigh (24 C): keep venting for moisture at the baseline but
    // do not add the heat-recovery boost (avoid actively overheating).
    TEST_ASSERT_EQUAL_UINT8(10, dvsProfileSpeed(winter(26.0f, 24.0f, true), cfg(), 40));
}

void test_winter_heat_recovery_similar_temp_boost() {
    // Roof 20.2 C, inside 21.2 C: differential -1 C ("similar"). frac =
    // (-1 - -2)/(4 - -2) = 1/6; boost = round(1/6 * 74) = 12 -> 10 + 12 = 22.
    // Similar temps still nudge the speed up rather than sitting at the baseline.
    const uint8_t s = dvsProfileSpeed(winter(20.2f, 21.2f, true), cfg(), 10);
    TEST_ASSERT_EQUAL_UINT8(22, s);
}

void test_winter_heat_recovery_not_full_at_2C() {
    // Roof 21 C, inside 19 C: differential +2 C. frac = (2 - -2)/6 = 0.667;
    // boost = round(0.667 * 74) = 49 -> 59. A 2 C warm roof must NOT hit maxSpeed.
    const uint8_t s = dvsProfileSpeed(winter(21.0f, 19.0f, true), cfg(), 10);
    TEST_ASSERT_EQUAL_UINT8(59, s);
    TEST_ASSERT_TRUE(s < 84);
}

void test_winter_heat_recovery_full_at_4C() {
    // Roof 23 C, inside 19 C: differential +4 C == recoverFull -> maxSpeed.
    const uint8_t s = dvsProfileSpeed(winter(23.0f, 19.0f, true), cfg(), 10);
    TEST_ASSERT_EQUAL_UINT8(84, s);
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

// --- Fail-safe freshness gate ----------------------------------------------

void test_fresh_when_roof_recent() {
    // Roof reading present and inside the stale window -> control allowed.
    TEST_ASSERT_TRUE(dvsProfileInputsFresh(true, 1000, 300000));
}

void test_stale_when_roof_missing() {
    // A never-received roof reading (have == false) always fails safe, even at age 0.
    TEST_ASSERT_FALSE(dvsProfileInputsFresh(false, 0, 300000));
}

void test_stale_when_roof_too_old() {
    // A Roof reading at/older than the stale window fails safe.
    TEST_ASSERT_FALSE(dvsProfileInputsFresh(true, 300000, 300000));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_winter_moisture_baseline_above_cutoff);
    RUN_TEST(test_winter_off_below_cold_cutoff);
    RUN_TEST(test_winter_supply_cutoff_hysteresis_no_flap);
    RUN_TEST(test_winter_heat_recovery_off_holds_baseline);
    RUN_TEST(test_winter_comfort_high_caps_boost);
    RUN_TEST(test_winter_heat_recovery_similar_temp_boost);
    RUN_TEST(test_winter_heat_recovery_not_full_at_2C);
    RUN_TEST(test_winter_heat_recovery_full_at_4C);
    RUN_TEST(test_summer_cooling_with_cool_air);
    RUN_TEST(test_summer_blocks_hot_roof_air);
    RUN_TEST(test_summer_off_when_house_already_cool);
    RUN_TEST(test_summer_hysteresis_start_vs_keep_running);
    RUN_TEST(test_auto_selects_summer_when_warm);
    RUN_TEST(test_auto_selects_winter_when_cool);
    RUN_TEST(test_auto_holds_profile_in_deadband);
    RUN_TEST(test_fresh_when_roof_recent);
    RUN_TEST(test_stale_when_roof_missing);
    RUN_TEST(test_stale_when_roof_too_old);
    return UNITY_END();
}
