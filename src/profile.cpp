#include "profile.h"

#include <math.h>

// Clamp a speed candidate into [lo, hi].
static uint8_t clampSpeed(long s, uint8_t lo, uint8_t hi) {
    if (s < (long)lo) {
        return lo;
    }
    if (s > (long)hi) {
        return hi;
    }
    return (uint8_t)s;
}

// Map a favourable differential ("benefit") to a proportional boost above the
// minimum speed. At/below deadbandOff the boost is 0; at/above span it is the
// full (maxSpeed - minSpeed) range.
static uint8_t proportionalBoost(float benefit, const DvsProfileConfig &cfg) {
    const float denom = cfg.spanC - cfg.deadbandOffC;
    float frac = (denom > 0.0f) ? (benefit - cfg.deadbandOffC) / denom : 1.0f;
    if (frac < 0.0f) {
        frac = 0.0f;
    } else if (frac > 1.0f) {
        frac = 1.0f;
    }
    const int range = (int)cfg.maxSpeed - (int)cfg.minSpeed;
    return (uint8_t)lroundf(frac * (float)(range > 0 ? range : 0));
}

// Winter heat-recovery boost: ramp from 0 (at recoverStartC of roof-minus-inside
// differential) up to the full (maxSpeed - base) range (at recoverFullC) as the
// roof air out-warms the house. "Similar" temperatures (a slightly negative
// differential) still earn a modest boost; full speed needs a real warm roof.
static uint8_t recoveryBoost(float delta, const DvsProfileConfig &cfg,
                             uint8_t base) {
    const float denom = cfg.recoverFullC - cfg.recoverStartC;
    float frac = (denom > 0.0f) ? (delta - cfg.recoverStartC) / denom : 1.0f;
    if (frac < 0.0f) {
        frac = 0.0f;
    } else if (frac > 1.0f) {
        frac = 1.0f;
    }
    const int range = (int)cfg.maxSpeed - (int)base;
    return (uint8_t)lroundf(frac * (float)(range > 0 ? range : 0));
}

// Winter retain-heat throttle: the cold-side mirror of recoveryBoost. As the roof
// air cools below the house (delta < 0) ramp the moisture baseline DOWN toward
// minSpeed to limit heat loss - 0 reduction at retainStartC (the 0 pivot) and the
// full (base - minSpeed) reduction at retainFullC (roof that many deg colder).
static uint8_t retainThrottle(float delta, const DvsProfileConfig &cfg,
                              uint8_t base) {
    const float denom = cfg.retainStartC - cfg.retainFullC; // start > full
    float frac = (denom > 0.0f) ? (cfg.retainStartC - delta) / denom : 1.0f;
    if (frac < 0.0f) {
        frac = 0.0f;
    } else if (frac > 1.0f) {
        frac = 1.0f;
    }
    const int range = (int)base - (int)cfg.minSpeed;
    return (uint8_t)lroundf(frac * (float)(range > 0 ? range : 0));
}

uint8_t dvsProfileSpeed(const DvsProfileInputs &in, const DvsProfileConfig &cfg,
                        uint8_t curSpeed) {
    // Hysteresis: a stopped fan needs the (higher) deadbandOn benefit to start;
    // a running fan keeps going until benefit drops below deadbandOff.
    const bool running = curSpeed > 0;
    const float startStop = running ? cfg.deadbandOffC : cfg.deadbandOnC;

    if (in.profile == DVS_PROFILE_WINTER) {
        // Moisture ventilation is the always-on baseline (the core job); the fan
        // ramps up (heat recovery) or down (retain heat) around the 0 pivot.
        const uint8_t base = cfg.moistureBaseSpeed;

        // Absolute cold cutoff (runtime-configurable minSupply): raw roof air
        // below it is too cold to supply (no exchanger core to temper it).
        // retainHeat ON stops the fan to keep house heat, stepping down through
        // minSpeed within the supplyHyst band first (anti-flap); OFF keeps the
        // moisture baseline running regardless (moisture priority).
        //
        // FUTURE: a humidity-demand override (high indoor RH + drier roof air)
        // will slot in HERE, ahead of these returns, to vent cold-but-dry air and
        // clear condensation even below the cold cutoff.
        if (in.roofTempC < cfg.minSupplyC) {
            if (!in.retainHeat) {
                return base; // moisture priority: keep venting despite the cold
            }
            if (running && in.roofTempC >= cfg.minSupplyC - cfg.supplyHystC) {
                return cfg.minSpeed; // transitional step-down before stopping
            }
            return 0;
        }

        const float delta = in.roofTempC - in.insideTempC; // >0 warm, <0 cold
        if (delta > 0.0f) {
            // Warm roof: heat-recovery boost above the baseline, capped once the
            // house is already warm.
            if (!in.heatRecovery || in.insideTempC >= cfg.comfortHighC) {
                return base;
            }
            const uint8_t boost = recoveryBoost(delta, cfg, base);
            return clampSpeed((long)base + boost, base, cfg.maxSpeed);
        }
        if (in.retainHeat) {
            // Cold roof (above the cutoff): throttle the baseline down proportionally.
            const uint8_t cut = retainThrottle(delta, cfg, base);
            return clampSpeed((long)base - cut, cfg.minSpeed, base);
        }
        return base; // retain heat off: hold the moisture baseline
    }

    // Summer: free cooling on top of a minimum-ventilation floor. Like winter's
    // moisture baseline, the fan never fully stops - it holds minSpeed even when
    // the roof is too hot or the house is already cool, then adds a proportional
    // cooling boost when the roof/night air is cooler than inside. The supplyHyst
    // slack above the hot cutoff keeps the boost from flapping near maxSupply.
    const float maxSupply =
        running ? cfg.maxSupplyC + cfg.supplyHystC : cfg.maxSupplyC;
    const float benefit = in.insideTempC - in.roofTempC; // >0 == free cooling
    if (in.roofTempC > maxSupply || in.insideTempC <= cfg.comfortLowC ||
        benefit < startStop) {
        return cfg.minSpeed; // minimum ventilation floor (never fully off)
    }
    const uint8_t boost = proportionalBoost(benefit, cfg);
    return clampSpeed((long)cfg.minSpeed + boost, cfg.minSpeed, cfg.maxSpeed);
}

DvsProfile dvsProfileAuto(float insideTempC, float comfortTargetC,
                          DvsProfile current) {
    // +/-0.5 C band around the target keeps the profile stable in the shoulder
    // seasons instead of flipping every tick.
    const float band = 0.5f;
    if (insideTempC > comfortTargetC + band) {
        return DVS_PROFILE_SUMMER;
    }
    if (insideTempC < comfortTargetC - band) {
        return DVS_PROFILE_WINTER;
    }
    return current;
}

bool dvsProfileInputsFresh(bool haveRoof, uint32_t roofAgeMs, uint32_t staleMs) {
    return haveRoof && roofAgeMs < staleMs;
}
