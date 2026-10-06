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

// Winter heat-recovery boost: ramp from 0 (at recoverStartC of airflow-minus-
// inside differential) up to the full (maxSpeed - base) range (at recoverFullC)
// as the airflow out-warms the house. "Similar" temperatures (a slightly negative
// differential) still earn a modest boost; full speed needs a real warm airflow.
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

// Winter heat-retention throttle: the cold-side mirror of recoveryBoost. As the
// airflow cools below the house (delta < 0) ramp the moisture baseline DOWN toward
// minSpeed to limit heat loss - 0 reduction at retentionStartC (the 0 pivot) and
// the full (base - minSpeed) reduction at retentionFullC (airflow that many deg
// colder).
static uint8_t retentionThrottle(float delta, const DvsProfileConfig &cfg,
                              uint8_t base) {
    const float denom = cfg.retentionStartC - cfg.retentionFullC; // start > full
    float frac = (denom > 0.0f) ? (cfg.retentionStartC - delta) / denom : 1.0f;
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

    // Winter - and summer once the house is AT or BELOW the target - runs the
    // moisture-first logic: an always-on baseline with heat-recovery (warm airflow)
    // and heat-retention (cold airflow) ramps around the 0 pivot. Summer only
    // switches to active cooling (further down) while the house is still ABOVE the
    // target; once cooling brings it down to the target it reverts to this
    // winter-like ventilation rather than stopping dead.
    if (in.profile == DVS_PROFILE_WINTER || in.insideTempC <= cfg.comfortTargetC) {
        // Moisture ventilation is the always-on baseline (the core job); the fan
        // ramps up (heat recovery) or down (retain heat) around the 0 pivot.
        const uint8_t base = cfg.moistureBaseSpeed;

        // Absolute cold cutoff (runtime-configurable): raw airflow below it is
        // too cold to supply (no exchanger core to temper it).
        // heatRetention ON stops the fan to keep house heat, stepping down through
        // minSpeed within the supplyHyst band first (anti-flap); OFF keeps the
        // moisture baseline running regardless (moisture priority).
        //
        // FUTURE: a humidity-demand override (high indoor RH + drier airflow)
        // will slot in HERE, ahead of these returns, to vent cold-but-dry air and
        // clear condensation even below the cold cutoff.
        if (in.airflowTempC < cfg.cutoffC) {
            if (!in.heatRetention) {
                return base; // moisture priority: keep venting despite the cold
            }
            if (running && in.airflowTempC >= cfg.cutoffC - cfg.supplyHystC) {
                return cfg.minSpeed; // transitional step-down before stopping
            }
            return 0;
        }

        const float delta = in.airflowTempC - in.insideTempC; // >0 warm, <0 cold
        if (delta > 0.0f) {
            // Warm airflow: heat-recovery boost above the baseline, capped back to
            // the baseline once the house has reached the comfort target.
            if (!in.heatRecovery || in.insideTempC >= cfg.comfortTargetC) {
                return base;
            }
            const uint8_t boost = recoveryBoost(delta, cfg, base);
            return clampSpeed((long)base + boost, base, cfg.maxSpeed);
        }
        if (in.heatRetention) {
            // Cold airflow (above the cutoff): throttle the baseline down proportionally.
            const uint8_t cut = retentionThrottle(delta, cfg, base);
            return clampSpeed((long)base - cut, cfg.minSpeed, base);
        }
        return base; // heat retention off: hold the moisture baseline
    }

    // Summer free cooling, house still ABOVE the target. Run only while the airflow
    // (supply) air is cooler than inside so venting actually cools, ramping
    // proportionally with the benefit. Stop when the airflow is above the absolute`
    // hot ceiling (maxSupply) or the air is not cool enough to help - bringing in
    // hotter air would only push the house further above the target. (At or below
    // the target the winter-like branch above has already taken over.) The
    // supplyHyst slack above maxSupply keeps a running fan from flapping there.
    const float maxSupply =
        running ? cfg.maxSupplyC + cfg.supplyHystC : cfg.maxSupplyC;
    const float benefit = in.insideTempC - in.airflowTempC; // >0 == supply cooler
    if (in.airflowTempC > maxSupply || benefit < startStop) {
        return 0; // air too hot or not cool enough to help -> stop
    }
    const uint8_t boost = proportionalBoost(benefit, cfg);
    return clampSpeed((long)cfg.minSpeed + boost, cfg.minSpeed, cfg.maxSpeed);
}

bool dvsProfileInputsFresh(bool haveAirflow, uint32_t airflowAgeMs, uint32_t staleMs) {
    return haveAirflow && airflowAgeMs < staleMs;
}
