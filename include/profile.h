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
// WINTER (moisture-first): keep ventilating for moisture/condensation control
//      whenever the supply air is above a cold cutoff (minSupply), even when it
//      is not adding heat (the "moisture floor"); add a proportional speed boost
//      on top when the roof air is actually warmer than inside (free heating).
//      With the moisture floor on, the fan keeps a trickle going even once the
//      house is warm (comfortHigh only caps the free-heat boost, it does not stop
//      the fan); only the cold cutoff (minSupply) stops it. With the floor off
//      (heat-only) it stops once the house is warm enough (comfortHigh) or the
//      roof air is too cold.
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
    bool moistureFloor; // winter: keep venting for moisture down to minSupply
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
    uint8_t minSpeed;   // gentle trickle speed
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

// Auto-control fail-safe gate. The fan may only be driven when BOTH the roof and
// inside readings exist and are fresher than staleMs; otherwise control must
// fail safe to OFF. Kept pure (ages passed in) so it can be tested off-target.
bool dvsProfileInputsFresh(bool haveRoof, uint32_t roofAgeMs, bool haveInside,
                           uint32_t insideAgeMs, uint32_t staleMs);
