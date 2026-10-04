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
    c.supplyHystC = 2.0f;
    c.recoverStartC = 0.0f;
    c.recoverFullC = 8.0f;
    c.retainStartC = 0.0f;
    c.retainFullC = -8.0f;
    c.minSpeed = 27;
    c.moistureBaseSpeed = 35;
    c.maxSpeed = 74;
    return c;
}

static DvsProfileInputs winter(float roof, float inside, bool heatRecovery,
                               bool retainHeat = true) {
    DvsProfileInputs in;
    in.roofTempC = roof;
    in.insideTempC = inside;
    in.heatRecovery = heatRecovery;
    in.retainHeat = retainHeat;
    in.profile = DVS_PROFILE_WINTER;
    return in;
}

static DvsProfileInputs summer(float roof, float inside) {
    DvsProfileInputs in;
    in.roofTempC = roof;
    in.insideTempC = inside;
    in.heatRecovery = false;
    in.retainHeat = false;
    in.profile = DVS_PROFILE_SUMMER;
    return in;
}

// --- Winter: moisture baseline + heat recovery -------------------------------
void test_winter_baseline_at_neutral() {
    // Roof == inside (19/19), above the cutoff: at the 0 pivot the fan holds the
    // moisture baseline - no heat-recovery boost, no retain-heat throttle.
    TEST_ASSERT_EQUAL_UINT8(35, dvsProfileSpeed(winter(19.0f, 19.0f, true), cfg(), 0));
}

void test_winter_off_below_cold_cutoff() {
    // Roof 11 C is below minSupply (12 C): too cold to supply, fan off with retain
    // heat on (the default) from a stopped start (full cutoff immediately).
    TEST_ASSERT_EQUAL_UINT8(0, dvsProfileSpeed(winter(11.0f, 19.0f, true), cfg(), 0));
}

void test_winter_retain_heat_throttle_ramps_down() {
    // Retain heat ON: above the cutoff a roof COLDER than inside throttles the
    // baseline down proportionally (cold-side mirror of heat recovery). range =
    // base - minSpeed = 8; cut = round((-delta / 8) * 8) = round(-delta).
    // delta -1 -> 34, delta -4 -> 31, delta -6 -> 29.
    TEST_ASSERT_EQUAL_UINT8(34, dvsProfileSpeed(winter(18.0f, 19.0f, true), cfg(), 35));
    TEST_ASSERT_EQUAL_UINT8(31, dvsProfileSpeed(winter(15.0f, 19.0f, true), cfg(), 35));
    TEST_ASSERT_EQUAL_UINT8(29, dvsProfileSpeed(winter(13.0f, 19.0f, true), cfg(), 35));
}

void test_winter_retain_heat_cutoff() {
    // Below minSupply (12 C) a RUNNING fan steps to minSpeed within the supplyHyst
    // band (2 C) then stops; a stopped fan uses the full cutoff.
    // Roof 11 C is inside the band (12 - 2 = 10): running -> minSpeed, stopped -> off.
    TEST_ASSERT_EQUAL_UINT8(27, dvsProfileSpeed(winter(11.0f, 19.0f, true), cfg(), 35));
    TEST_ASSERT_EQUAL_UINT8(0, dvsProfileSpeed(winter(11.0f, 19.0f, true), cfg(), 0));
    // Roof 11 C is inside the band (12 - 1.5): base 35 -> minSpeed, stopped -> off.
    TEST_ASSERT_EQUAL_UINT8(27, dvsProfileSpeed(winter(11.0f, 19.0f, true), cfg(), 35));
    TEST_ASSERT_EQUAL_UINT8(0, dvsProfileSpeed(winter(11.0f, 19.0f, true), cfg(), 0));
    // Roof 9 C is below the band: fully off regardless of running state.
    TEST_ASSERT_EQUAL_UINT8(0, dvsProfileSpeed(winter(9.0f, 19.0f, true), cfg(), 35));
}

void test_winter_retain_heat_off_holds_baseline() {
    // Retain heat OFF: moisture takes priority over heat, so a cold roof holds the
    // moisture baseline (no throttle) both above and below the cold cutoff.
    TEST_ASSERT_EQUAL_UINT8(35, dvsProfileSpeed(winter(15.0f, 19.0f, true, false), cfg(), 35));
    TEST_ASSERT_EQUAL_UINT8(35, dvsProfileSpeed(winter(9.0f, 19.0f, true, false), cfg(), 35));
    TEST_ASSERT_EQUAL_UINT8(35, dvsProfileSpeed(winter(9.0f, 19.0f, true, false), cfg(), 0));
}

void test_winter_custom_cutoff() {
    // The cold cutoff is runtime-settable via cfg.minSupplyC. Raising it to 15 C
    // stops the fan at roof 14 C, where the default 12 C cutoff would still vent
    // (throttled baseline).
    DvsProfileConfig c = cfg();
    c.minSupplyC = 15.0f;
    TEST_ASSERT_EQUAL_UINT8(0, dvsProfileSpeed(winter(14.0f, 19.0f, true), c, 0));
    TEST_ASSERT_TRUE(dvsProfileSpeed(winter(14.0f, 19.0f, true), cfg(), 0) > 0);
}

void test_winter_heat_recovery_off_holds_baseline() {
    // Heat recovery OFF with a warm roof (26 C vs 19 C): no boost, just hold the
    // low moisture baseline.
    TEST_ASSERT_EQUAL_UINT8(35, dvsProfileSpeed(winter(26.0f, 19.0f, false), cfg(), 40));
}

void test_winter_comfort_high_caps_boost() {
    // Inside >= comfortHigh (24 C): keep venting for moisture at the baseline but
    // do not add the heat-recovery boost (avoid actively overheating).
    TEST_ASSERT_EQUAL_UINT8(35, dvsProfileSpeed(winter(26.0f, 24.0f, true), cfg(), 40));
}

void test_winter_heat_recovery_not_full_at_2C() {
    // Roof 21 C, inside 19 C: differential +2 C. frac = 2 / 8 = 0.25;
    // boost = round(0.25 * 39) = 10 -> 45. A 2 C warm roof must NOT hit maxSpeed.
    const uint8_t s = dvsProfileSpeed(winter(21.0f, 19.0f, true), cfg(), 35);
    TEST_ASSERT_EQUAL_UINT8(45, s);
    TEST_ASSERT_TRUE(s < 74);
}

void test_winter_heat_recovery_full_at_8C() {
    // Roof 23 C, inside 19 C: differential +8 C == recoverFull -> maxSpeed.
    const uint8_t s = dvsProfileSpeed(winter(27.0f, 19.0f, true), cfg(), 35);
    TEST_ASSERT_EQUAL_UINT8(74, s);
}

// --- Summer: free cooling --------------------------------------------------

void test_summer_cooling_with_cool_air() {
    // Roof 16 C, inside 23 C: benefit = 7 C > span -> full cooling to maxSpeed.
    const uint8_t s = dvsProfileSpeed(summer(16.0f, 23.0f), cfg(), 0);
    TEST_ASSERT_EQUAL_UINT8(74, s);
}

void test_summer_floor_when_hot_roof() {
    // Roof 30 C > maxSupply (24 C): don't bring in hot midday air, but summer no
    // longer stops - it holds the minSpeed minimum-ventilation floor.
    TEST_ASSERT_EQUAL_UINT8(27, dvsProfileSpeed(summer(30.0f, 23.0f), cfg(), 50));
}

void test_summer_floor_when_house_already_cool() {
    // Inside <= comfortLow (18 C): no cooling demand, hold the minSpeed floor
    // rather than stopping.
    TEST_ASSERT_EQUAL_UINT8(27, dvsProfileSpeed(summer(15.0f, 18.0f), cfg(), 40));
}

// --- Hysteresis -------------------------------------------------------------

void test_summer_floor_and_boost_hysteresis() {
    // Benefit = 1.0 C: between deadbandOff (0.5) and deadbandOn (1.5). A stopped
    // fan (cur=0) holds the minSpeed floor (no boost yet); a running fan (cur>0)
    // adds the cooling boost on top of the floor.
    TEST_ASSERT_EQUAL_UINT8(27, dvsProfileSpeed(summer(21.0f, 22.0f), cfg(), 0));
    TEST_ASSERT_TRUE(dvsProfileSpeed(summer(21.0f, 22.0f), cfg(), 27) > 27);
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
    RUN_TEST(test_winter_baseline_at_neutral);
    RUN_TEST(test_winter_off_below_cold_cutoff);
    RUN_TEST(test_winter_retain_heat_throttle_ramps_down);
    RUN_TEST(test_winter_retain_heat_cutoff);
    RUN_TEST(test_winter_retain_heat_off_holds_baseline);
    RUN_TEST(test_winter_custom_cutoff);
    RUN_TEST(test_winter_heat_recovery_off_holds_baseline);
    RUN_TEST(test_winter_comfort_high_caps_boost);
    RUN_TEST(test_winter_heat_recovery_not_full_at_2C);
    RUN_TEST(test_winter_heat_recovery_full_at_8C);
    RUN_TEST(test_summer_cooling_with_cool_air);
    RUN_TEST(test_summer_floor_when_hot_roof);
    RUN_TEST(test_summer_floor_when_house_already_cool);
    RUN_TEST(test_summer_floor_and_boost_hysteresis);
    RUN_TEST(test_auto_selects_summer_when_warm);
    RUN_TEST(test_auto_selects_winter_when_cool);
    RUN_TEST(test_auto_holds_profile_in_deadband);
    RUN_TEST(test_fresh_when_roof_recent);
    RUN_TEST(test_stale_when_roof_missing);
    RUN_TEST(test_stale_when_roof_too_old);
    return UNITY_END();
}
