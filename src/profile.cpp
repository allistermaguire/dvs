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

uint8_t dvsProfileSpeed(const DvsProfileInputs &in, const DvsProfileConfig &cfg,
                        uint8_t curSpeed) {
    // Hysteresis: a stopped fan needs the (higher) deadbandOn benefit to start;
    // a running fan keeps going until benefit drops below deadbandOff.
    const bool running = curSpeed > 0;
    const float startStop = running ? cfg.deadbandOffC : cfg.deadbandOnC;

    if (in.profile == DVS_PROFILE_WINTER) {
        // Supply air == raw roof temp (no exchanger core). Never dump air colder
        // than minSupply. The running fan cools its own roof reading, so give it
        // supplyHyst of slack below the cutoff before it stopping (prevents on/off
        // flapping when the roof sits near minSupply, e.g. ~12 C).
        const float minSupply =
            running ? cfg.minSupplyC - cfg.supplyHystC : cfg.minSupplyC;
        if (in.roofTempC < minSupply) {
            return 0;
        }
        const float benefit = in.roofTempC - in.insideTempC; // >0 == free heat
        const bool warmEnough = in.insideTempC >= cfg.comfortHighC;
        if (in.moistureFloor) {
            // Moisture-first: keep venting for condensation wherever the supply
            // air is above the cold cutoff - even once the house is already warm
            // (comfortHigh does NOT stop it; only the cold cutoff can). Add the
            // free-heat boost only while below comfortHigh; at/above it hold the
            // bare trickle so we still get ventilation for moisture without actively
            // overheating.
            const uint8_t boost = warmEnough ? 0 : proportionalBoost(benefit, cfg);
            return clampSpeed((long)cfg.minSpeed + boost, cfg.minSpeed,
                              cfg.maxSpeed);
        }
        // Heat-only: back off once the house is already warm enough, and require 
        // a genuine warming benefit (with hysteresis).
        if (warmEnough || benefit < startStop) {
            return 0;
        }
        const uint8_t boost = proportionalBoost(benefit, cfg);
        return clampSpeed((long)cfg.minSpeed + boost, cfg.minSpeed, cfg.maxSpeed);
    }

    // Summer: free cooling with cool roof/night air; block hot midday roof air.
    // Symmetric slack: a running fan tolerates supplyHyst above the hot cutoff
    // before stopping, so it does not flap when the roof hovers near maxSupply.
    const float maxSupply =
        running ? cfg.maxSupplyC + cfg.supplyHystC : cfg.maxSupplyC;
    if (in.roofTempC > maxSupply || in.insideTempC <= cfg.comfortLowC) {
        return 0;
    }
    const float benefit = in.insideTempC - in.roofTempC; // >0 == free cooling
    if (benefit < startStop) {
        return 0;
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
