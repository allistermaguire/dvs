// Host-side unit tests for the ventilation control profiles: `pio test -e native`.
//
// These exercise the pure decision logic in profile.cpp - no Arduino/bus. They
// pin the moisture-first winter behaviour (always ventilate at a basedline above 
// the cold cutoff), the optional heat-recover boost, the summer free-cooling 
// toward the target, hysteresis and the fail-safe freshness gate.

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
    c.comfortTargetC = 19.0f;
    c.cutoffC = 12.0f;
    c.maxSupplyC = 24.0f;
    c.supplyHystC = 2.0f;
    c.recoverStartC = 0.0f;
    c.recoverFullC = 8.0f;
    c.retentionStartC = 0.0f;
    c.retentionFullC = -8.0f;
    c.minSpeed = 27;
    c.moistureBaseSpeed = 35;
    c.maxSpeed = 74;
    return c;
}

static DvsProfileInputs winter(float airflow, float inside, bool heatRecovery,
                               bool heatRetention = true) {
    DvsProfileInputs in;
    in.airflowTempC = airflow;
    in.insideTempC = inside;
    in.heatRecovery = heatRecovery;
    in.heatRetention = heatRetention;
    in.profile = DVS_PROFILE_WINTER;
    return in;
}

static DvsProfileInputs summer(float airflow, float inside) {
    DvsProfileInputs in;
    in.airflowTempC = airflow;
    in.insideTempC = inside;
    in.heatRecovery = false;
    in.heatRetention = false;
    in.profile = DVS_PROFILE_SUMMER;
    return in;
}

// --- Winter: moisture baseline + heat recovery -------------------------------

void test_winter_baseline_at_neutral() {
    // Airflow == inside (19/19), above the cutoff: at the 0 pivot the fan holds the
    // moisture baseline - no heat-recovery boost, no retain-heat throttle.
    TEST_ASSERT_EQUAL_UINT8(35, dvsProfileSpeed(winter(19.0f, 19.0f, true), cfg(), 0));
}

void test_winter_off_below_cold_cutoff() {
    // Airflow 11 C is below the cutoff (12 C): too cold to supply, fan off with heat
    // retention on (the default) from a stopped start (full cutoff, no step-down).
    TEST_ASSERT_EQUAL_UINT8(0, dvsProfileSpeed(winter(11.0f, 19.0f, true), cfg(), 0));
}

void test_winter_heat_retention_throttle_ramps_down() {
    // Heat retention ON: above the cutoff a airflow COLDER than inside throttles the
    // baseline down proportionally (cold-side mirror of heat recovery). range =
    // base - minSpeed = 8; cut = round((-delta / 8) * 8) = round(-delta).
    // delta -1 -> 34, delta -4 -> 31, delta -6 -> 29.
    TEST_ASSERT_EQUAL_UINT8(34, dvsProfileSpeed(winter(18.0f, 19.0f, true), cfg(), 35));
    TEST_ASSERT_EQUAL_UINT8(31, dvsProfileSpeed(winter(15.0f, 19.0f, true), cfg(), 35));
    TEST_ASSERT_EQUAL_UINT8(29, dvsProfileSpeed(winter(13.0f, 19.0f, true), cfg(), 35));
}

void test_winter_heat_retention_cutoff() {
    // Below the cutoff (12 C) a RUNNING fan steps to minSpeed within the supplyHyst
    // band (2 C) then stops; a stopped fan uses the full cutoff.
    // Roof 11 C is inside the band (12 - 2 = 10): running -> minSpeed, stopped -> off.
    TEST_ASSERT_EQUAL_UINT8(27, dvsProfileSpeed(winter(11.0f, 19.0f, true), cfg(), 35));
    TEST_ASSERT_EQUAL_UINT8(0, dvsProfileSpeed(winter(11.0f, 19.0f, true), cfg(), 0));
    // Roof 9 C is below the band: even a running fan stops.
    TEST_ASSERT_EQUAL_UINT8(0, dvsProfileSpeed(winter(9.0f, 19.0f, true), cfg(), 35));
}

void test_winter_heat_retention_off_holds_baseline() {
    // Heat retention OFF: moisture takes priority over heat, so a cold roof holds
    // the moisture baseline (no throttle) both above and below the cold cutoff.
    TEST_ASSERT_EQUAL_UINT8(35, dvsProfileSpeed(winter(15.0f, 19.0f, true, false), cfg(), 35));
    TEST_ASSERT_EQUAL_UINT8(35, dvsProfileSpeed(winter(9.0f, 19.0f, true, false), cfg(), 35));
    TEST_ASSERT_EQUAL_UINT8(35, dvsProfileSpeed(winter(9.0f, 19.0f, true, false), cfg(), 0));
}

void test_winter_custom_cutoff() {
    // The cold cutoff is runtime-settable via cfg.cutoffC. Raising it to 15 C
    // stops the fan at roof 14 C, where the default 12 C cutoff would still vent
    // (throttled baseline).
    DvsProfileConfig c = cfg();
    c.cutoffC = 15.0f;
    TEST_ASSERT_EQUAL_UINT8(0, dvsProfileSpeed(winter(14.0f, 19.0f, true), c, 0));
    TEST_ASSERT_TRUE(dvsProfileSpeed(winter(14.0f, 19.0f, true), cfg(), 0) > 0);
}

void test_winter_heat_recovery_off_holds_baseline() {
    // Heat recovery OFF with a warm roof (26 C vs 19 C): no boost, just hold the
    // low moisture baseline.
    TEST_ASSERT_EQUAL_UINT8(35, dvsProfileSpeed(winter(26.0f, 19.0f, false), cfg(), 40));
}

void test_winter_caps_boost_at_target() {
    // Inside has reached the target (19 C) with a warm roof (26 C): keep venting
    // for moisture at the baseline but add no heat-recovery boost (don't overheat
    // past the target).
    TEST_ASSERT_EQUAL_UINT8(35, dvsProfileSpeed(winter(26.0f, 19.0f, true), cfg(), 40));
}

void test_winter_heat_recovery_not_full_at_2C() {
    // Airflow 19 C, inside 17 C (below the 19 C target): differential +2 C. frac =
    // 2/8 = 0.25; boost = round(0.25 * 39) = 10 -> 45. A 2 C warm airflow must NOT
    // hit maxSpeed.
    const uint8_t s = dvsProfileSpeed(winter(19.0f, 17.0f, true), cfg(), 35);
    TEST_ASSERT_EQUAL_UINT8(45, s);
    TEST_ASSERT_TRUE(s < 74);
}

void test_winter_heat_recovery_full_at_8C() {
    // Airflow 25 C, inside 17 C (below target): differential +8 C == recoverFull ->
    // maxSpeed.
    const uint8_t s = dvsProfileSpeed(winter(25.0f, 17.0f, true), cfg(), 35);
    TEST_ASSERT_EQUAL_UINT8(74, s);
}

// -- Summer: free cooling toward the target --------------------------------

void test_summer_cooling_with_cool_air() {
    // Airflow 16 C, inside 23 C: benefit = 7 C > span -> full cooling to maxSpeed.
    const uint8_t s = dvsProfileSpeed(summer(16.0f, 23.0f), cfg(), 0);
    TEST_ASSERT_EQUAL_UINT8(74, s);
}

void test_summer_runs_when_cooling_possible() {
    // Inside 22 C (above the 19 C target), roof 18 C (cooler): benefit 4 C ->
    // proportional cooling above the minSpeed floor. frac = (4 - 0.5)/4.5 = 0.778;
    // boost = round(0.778 * 47) = 37 -> 27 + 37 = 64.
    TEST_ASSERT_EQUAL_UINT8(64, dvsProfileSpeed(summer(18.0f, 22.0f), cfg(), 0));
}

void test_summer_runs_like_winter_at_or_below_target() {
    // At/below the target, summer no longer stops - it runs the winter moisture
    // logic. With a cool roof at/below the target (16/19, 16/17) and retain heat
    // off (the summer default) it holds the moisture baseline rather than cutting
    // out.
    TEST_ASSERT_EQUAL_UINT8(35, dvsProfileSpeed(summer(16.0f, 19.0f), cfg(), 40));
    TEST_ASSERT_EQUAL_UINT8(35, dvsProfileSpeed(summer(16.0f, 17.0f), cfg(), 40));
}

void test_summer_heat_recovery_below_target() {
    // Below the target (inside 17) with a warm roof (23) and heat recovery on,
    // summer behaves like winter: moisture baseline + heat-recovery boost toward
    // the target. delta +6 -> boost -> 64, above the 35 baseline.
    DvsProfileInputs in = summer(23.0f, 17.0f);
    in.heatRecovery = true;
    TEST_ASSERT_EQUAL_UINT8(64, dvsProfileSpeed(in, cfg(), 40));
}

void test_summer_stop_when_cannot_cool() {
    // Inside 20 C (above target) but the roof is WARMER (22 C): venting would
    // import heat, not cool, so the fan stops.
    TEST_ASSERT_EQUAL_UINT8(0, dvsProfileSpeed(summer(22.0f, 20.0f), cfg(), 40));
}

void test_summer_stop_when_hot_roof() {
    // Roof 30 C > maxSupply (+hyst while running = 26 C): don't bring in hot midday
    // air - summer stops entirely rather than holding a floor.
    TEST_ASSERT_EQUAL_UINT8(0, dvsProfileSpeed(summer(30.0f, 23.0f), cfg(), 50));
}

// --- Hysteresis -------------------------------------------------------------

void test_summer_hysteresis() {
    // Inside 22 C above target, benefit = 1.0 C: between deadbandOff (0.5) and
    // deadbandOn (1.5). A stopped fan (cur=0) needs the higher deadbandOn to
    // start, so it stays OFF; a running fan (cur>0) keeps cooling above the floor.
    TEST_ASSERT_EQUAL_UINT8(0, dvsProfileSpeed(summer(21.0f, 22.0f), cfg(), 0));
    TEST_ASSERT_TRUE(dvsProfileSpeed(summer(21.0f, 22.0f), cfg(), 27) > 27);
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
    RUN_TEST(test_winter_heat_retention_throttle_ramps_down);
    RUN_TEST(test_winter_heat_retention_cutoff);
    RUN_TEST(test_winter_heat_retention_off_holds_baseline);
    RUN_TEST(test_winter_custom_cutoff);
    RUN_TEST(test_winter_heat_recovery_off_holds_baseline);
    RUN_TEST(test_winter_caps_boost_at_target);
    RUN_TEST(test_winter_heat_recovery_not_full_at_2C);
    RUN_TEST(test_winter_heat_recovery_full_at_8C);
    RUN_TEST(test_summer_cooling_with_cool_air);
    RUN_TEST(test_summer_runs_when_cooling_possible);
    RUN_TEST(test_summer_runs_like_winter_at_or_below_target);
    RUN_TEST(test_summer_heat_recovery_below_target);
    RUN_TEST(test_summer_stop_when_cannot_cool);
    RUN_TEST(test_summer_stop_when_hot_roof);
    RUN_TEST(test_summer_hysteresis);
    RUN_TEST(test_fresh_when_roof_recent);
    RUN_TEST(test_stale_when_roof_missing);
    RUN_TEST(test_stale_when_roof_too_old);
    return UNITY_END();
}
