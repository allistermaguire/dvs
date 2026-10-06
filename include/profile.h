#pragma once

#include <stdint.h>

// -----------------------------------------------------------------------------
// Ventilation control profiles for the DVS G3 (positive-pressure roof-air unit).
// 
// The unit pushes RAW roof/outside air into the home - there is no heat-recovery
// exchanger core, so the air actually entering the house is at the airflow
// (supply) temperature (no tempering blend). Given that, control is a pure
// function of the airflow (supply) temperature and the in-home temperature:
// 
//  WINTER (moisture-first, with symmetric warm/cold ramps): ALWAYS ventilate
//      for moisture/condensation control whenever the supply air is above the cold
//      cutoff, at a baseline speed (moistureBaseSpeed). Around the
//      0 pivot (airflow == inside) the fan ramps both ways: when heat recovery is
//      on and the airflow is WARMER than inside it ramps UP from the baseline
//      toward maxSpeed (recoverStartC..recoverFullC) to harvest free heat (capped
//      back to the baseline once inside reaches the comfort target); when heat
//      retention is on and the airflow is COLDER than inside it ramps DOWN from the
//      baseline toward minSpeed (retentionStartC..retentionFullC) to limit heat
//      loss. Below the cold cutoff a running fan steps to minSpeed within the
//      supplyHyst band and then stops (heat retention on), or holds the baseline
//      (heat retention off). The cutoff is runtime-configurable.
// 
//  SUMMER (free cooling above the target; winter-like below it): while the house
//      is ABOVE the target, cool it by venting cooler airflow - run only when the
//      supply air is cooler than inside, ramping proportionally up to maxSpeed, and
//      stop if the airflow is too hot to cool with (maxSupply) or not cool enough
//      to help. Once cooling brings the house down to the target it reverts to the
//      winter moisture logic above (baseline + heat-recovery/heat-retention ramps)
//      rather than stopping dead - so at/below the target summer behaves like
//      winter.
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
    float airflowTempC; // raw airflow/supply-air temperature (deg C)
    float insideTempC;  // in-home temperature (deg C)
    bool heatRecovery;  // winter: ramp speed up to harvest warm airflow
    bool heatRetention; // winter: stop below the cold cutoff to keep house heat
    DvsProfile profile; // active profile (summer or winter)
    // FUTURE: roofRh / insideRh (and derived dew points) will be added here for
    // humidity-aware dehumidification; see the header comment above.
};

// Tuning constants (boot defaults live in config.h; all runtime-adjustable).
struct DvsProfileConfig {
    float deadbandOnC;  // favourable differential needed to START ventilating
    float deadbandOffC; // differential below which a running fan STOPS (< 0n)
    float spanC;        // differential at which maxSpeed is reached
    float comfortTargetC; // unified target inside temp: winter caps the heat-recovery boost here, summer stops here (runtime-set)
    float cutoffC;         // winter: never supply raw airflow colder than this (runtime-set)
    float maxSupplyC;      // summer: never supply raw airflow hotter than this
    float supplyHystC;  // slack below/above the supply cutoff for a RUNNING fan
    float recoverStartC; // winter: airflow-minus-inside where heat-recovery boost begins
    float recoverFullC;  // winter: airflow-minus-inside where boost reaches maxSpeed
    float retentionStartC; // winter: airflow-minus-inside where heat-retention throttle begins
    float retentionFullC;  // winter: airflow-minus-inside where throttle reaches minSpeed
    uint8_t minSpeed;   // summer floor / gentle trickle speed
    uint8_t moistureBaseSpeed; // winter: always-on moisture-ventilation baseline
    uint8_t maxSpeed;   // full/boost speed
};

// Compute the desired fan speed (0..maxSpeed) for the given inputs. curSpeed is
// the current commanded speed (>0 == running) and drives the start/stop
// hysteresis so the fan does not chatter around a threshold.
uint8_t dvsProfileSpeed(const DvsProfileInputs &in, const DvsProfileConfig &cfg,
                        uint8_t curSpeed);

// Auto-control fail-safe gate. The fan may only be driven when the airflow reading
// exists and is fresher than staleMs; otherwise control must fail safe to OFF.
// Kept pure (ages passed in) so it can be tested off-target.
bool dvsProfileInputsFresh(bool haveAirflow, uint32_t airflowAgeMs, uint32_t staleMs);
