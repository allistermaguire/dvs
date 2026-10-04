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
//  WINTER (moisture-first, with symmetric warm/cold ramps): ALWAYS ventilate
//      for moisture/condensation control whenever the supply air is above the cold
//      cutoff (minSupply), at a baseline speed (moistureBaseSpeed). Around the
//      0 pivot (roof == inside) the fan ramps both ways: when heat recovery is on
//      and the roof is WARMER than inside it ramps UP from the baseline toward
//      maxSpeed (recoverStartC..recoverFullC) to harvest free heat (capped at the
//       baseline once inside reaches comfortHigh); when retain heat is on and the
//      roof is COLDER than inside it ramps DOWN from the baseline toward minSpeed
//      (retainStartC..retainFullC) to limit heat loss. Below the cold cutoff a
//      running fan steps to minSpeed within the supplyHyst band and then stops
//      (retain heat on), or holds the baseline (retain heat off). The cutoff
//      (minSupply) is runtime-configurable.
// 
//  SUMMER (free cooling over a minimum floor): ventilate harder when the
//      roof/night air is cooler than inside; block hot midday roof air above
//      maxSupply and ease off once the house is already cool (comfortLow). Unlike
//      before, the fan never fully stops - it holds a minSpeed minimum-ventilation
//      floor (mirroring winter's baseline) and adds the cooling boost on top.
//
//  FUTURE (designed, not yet implemented): a humidity channel. Given roof/inside
//      temperature AND humidity, compute dew points and add a dehumidify demand that
//      is max()-combined with the thermal boost, and that is allowed to OVERRIDE the
//      winter cold cutoff (vent cold-but-dry air to clear condensation). The winter
//      cold cutoff below is deliberately isolated so that override can slot in ahead
//      of its returns.
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
    bool retainHeat;    // winter: stop below the cold cutoff to keep house heat
    DvsProfile profile; // active profile (summer or winter)
    // FUTURE: roofRh / insideRh (and derived dew points) will be added here for
    // humidity-aware dehumidification; see the header comment above.
};

// Tuning constants (boot defaults live in config.h; all runtime-adjustable).
struct DvsProfileConfig {
    float deadbandOnC;  // favourable differential needed to START ventilating
    float deadbandOffC; // differential below which a running fan STOPS (< 0n)
    float spanC;        // differential at which maxSpeed is reached
    float comfortHighC; // winter: stop once inside is already this warm
    float comfortLowC;  // summer: stop once inside is already this cool
    float minSupplyC;   // winter: never supply raw roof air colder than this (runtime-set)
    float maxSupplyC;   // summer: never supply raw roof air hotter than this
    float supplyHystC;  // slack below/above the supply cutoff for a RUNNING fan
    float recoverStartC; // winter: roof-minus-inside where heat-recovery boost begins
    float recoverFullC;  // winter: roof-minus-inside where boost reaches maxSpeed
    float retainStartC;  // winter: roof-minus-inside where retain-heat throttle begins
    float retainFullC;   // winter: roof-minus-inside where throttle reaches minSpeed
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
