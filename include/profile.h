#pragma once

#include <stdint.h>

// -----------------------------------------------------------------------------
// Ventilation control profiles for the DVS G3 (positive-pressure roof-air unit).
// 
// The unit pushes RAW roof/outside air into the home - there is no heat-recovery
// exchanger core, so the air actually entering the house is at the roof
// temperature (no tempering blend). Given that, control is a pure function of
// the roof (supply) temperature and the in-home temperature:
// 
//  WINTER (moisture-first, optional heat recovery): ALWAYS ventilate for
//      moisture/condensation control whenever the supply air is above a cold
//      cutoff (minSupply), at a low baseline speed (moistureBaseSpeed). This is
//      the unit's core job and needs no toggle - only the cold cutoff stops it.
//      When heat recovery is enabled (the default) and the roof air is as warm as
//      (or warmer than) the house, ramp the speed up proportionally to harvest
//      that free heat: the boost starts around recoverStartC of differential and
//      reaches maxSpeed once the roof is recoverFullC warmer than inside.
//      comfortHigh caps the boost (hold the baseline) once the house is already
//      warm, so we keep ventilating without actively overheating. With heat
//      recovery off, the fan simply holds the moisture baseline.
// 
// SUMMER (free cooling): ventilate when the roof/night air is cooler than
//      inside; block hot midday roof air above maxSupply and stop once the house
//      is already cool enough (comfortLow). No floor.
// 
// The functions here are PURE (no Arduino/hardware dependencies) so they are
// exercised directly by the native host unit tests, mirroring crc16/frame.
// -----------------------------------------------------------------------------

enum DvsProfile : uint8_t {
    DVS_PROFILE_SUMMER = 0,
    DVS_PROFILE_WINTER = 1,
};

// Live inputs to a control decision.
struct DvsProfileInputs {
    float roofTempC;    // raw roof/outside supply-air temperature (deg C)
    float insideTempC;  // in-home temperature (deg C)
    bool heatRecovery;  // winter: ramp speed up to harvest warm roof air
    DvsProfile profile; // active profile (summer or winter)
};

// Tuning constants (boot defaults live in config.h; all runtime-adjustable).
struct DvsProfileConfig {
    float deadbandOnC;  // favourable differential needed to START ventilating
    float deadbandOffC; // differential below which a running fan STOPS (< 0n)
    float spanC;        // differential at which maxSpeed is reached
    float comfortHighC; // winter: stop once inside is already this warm
    float comfortLowC;  // summer: stop once inside is already this cool
    float minSupplyC;   // winter: never supply raw roof air colder than this
    float maxSupplyC;   // summer: never supply raw roof air hotter than this
    float supplyHystC;  // slack below/above the supply cutoff for a RUNNING fan
    float recoverStartC; // winter: roof-minus-inside where heat-recovery boost begins
    float recoverFullC;  // winter: roof-minus-inside where boost reaches maxSpeed
    uint8_t minSpeed;   // summer floor / gentle trickle speed
    uint8_t moistureBaseSpeed; // winter: always-on moisture-ventilation baseline
    uint8_t maxSpeed;   // full/boost speed
};

// Compute the desired fan speed (0..maxSpeed) for the given inputs. curSpeed is
// the current commanded speed (>0 == running) and drives the start/stop
// hysteresis so the fan does not chatter around a threshold.
uint8_t dvsProfileSpeed(const DvsProfileInputs &in, const DvsProfileConfig &cfg,
                        uint8_t curSpeed);

// Pick the profile for `auto` mode from the inside temperature. A small deadband
// around comfortTargetC prevents flip-flopping in the shoulder seasons; the
// current profile is kept while inside sits inside the band.
DvsProfile dvsProfileAuto(float insideTempC, float comfortTargetC,
                          DvsProfile current);

// Auto-control fail-safe gate. The fan may only be driven when the roof readings
// exists and is fresher than staleMs; otherwise control must fail safe to OFF.
// Kept pure (ages passed in) so it can be tested off-target.
bool dvsProfileInputsFresh(bool haveRoof, uint32_t roofAgeMs, uint32_t staleMs);
