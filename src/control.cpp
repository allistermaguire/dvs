// Fan control + automatic ventilation logic for the DVS controller.
//
// Owns the commanded fan state and the summer/winter auto-control decision. It
// never touches the RS485 transceiver directly: every transmit goes through the
// bus seam (bus.h, implemented in main.cpp), so this stays a policy layer over a
// single hardware chokepoint.
//
// Excluded from the native (host) unit-test build: it depends on the Arduino
// core (Serial/millis) and the bus/net seams. The pure decision maths it drives
// lives in profile.cpp, which is tested off-target.

#if !defined(UNIT_TEST)

#include <Arduino.h>

#include "boost.h"
#include "bus.h"
#include "config.h"
#include "control.h"
#include "net.h"

// --- Fan control state (commanded, i.e. what we last told the fan) -----------
static bool g_txEnabled = DVS_TX_ENABLED_DEFAULT; // false => listen-only
static bool g_fanPower = false;           // true while we intend the fan to run
static uint8_t g_fanSpeed = 0;            // last commanded target speed (0..84)
static uint8_t g_lastNonZeroSpeed = 0;    // remembered speed for a bare "power on"
static uint32_t g_boostUntilMs = 0;       // full-speed boost deadline (0 => none)
static bool g_standby = false;            // user-off intent: hold fan off + suspend auto

// --- Auto ventilation control state ------------------------------------------
static DvsProfileSel g_profileSel = DVS_PROFILE_SEL_MANUAL; // manual / winter / summer
static bool g_heatRecovery = DVS_HEAT_RECOVERY_DEFAULT;     // winter warm-airflow boost
static bool g_heatRetention = DVS_HEAT_RETENTION_DEFAULT;   // winter cold cutoff stops fan
static float g_insideTempC = 0.0f;          // last inside temp fed in (MQTT)
static bool g_haveInsideTemp = false;
static float g_airflowTempC = 0.0f;         // last airflow temp sniffed from the bus
static bool g_haveAirflowTemp = false;
static uint32_t g_airflowTempMs = 0;        // millis() of the last airflow-temp reading
static uint32_t g_lastControlTickMs = 0;    // last auto-tick time (0 => act now)

// Profile tuning (config.h boot defaults). comfortTargetC and cutoffC (the
// cold cutoff) are mutated at runtime by the setters below; the rest are fixed.
static DvsProfileConfig g_profileCfg = {
    DVS_PROFILE_DEADBAND_ON_C,    DVS_PROFILE_DEADBAND_OFF_C,
    DVS_PROFILE_SPAN_C,           DVS_PROFILE_COMFORT_TARGET_C,
    DVS_PROFILE_CUTOFF_C,         DVS_PROFILE_MAX_SUPPLY_C,
    DVS_PROFILE_SUPPLY_HYST_C,    DVS_PROFILE_RECOVER_START_C,
    DVS_PROFILE_RECOVER_FULL_C,   DVS_PROFILE_RETENTION_START_C,
    DVS_PROFILE_RETENTION_FULL_C, DVS_PROFILE_MIN_SPEED,
    DVS_PROFILE_MOISTURE_SPEED,   DVS_PROFILE_MAX_SPEED,
};

// Map the selected profile to the pure season logic. Only meaningful for a
// seasonal profile; manual maps to winter as a harmless default (auto-control
// never runs in manual, so the value is unused there).
static DvsProfile controlResolveProfile() {
    return g_profileSel == DVS_PROFILE_SEL_SUMMER ? DVS_PROFILE_SUMMER
                                                : DVS_PROFILE_WINTER;
}

// Stop the fan without recording a user-off intent: write speed target 0 and
// clear the running state, but leave g_standby alone. Used by auto-control when
// it idles the fan (cutoff/fail-safe) so it never suspends itself.
static void fanOff() {
    g_fanPower = false;
    g_fanSpeed = 0;
    const uint8_t zero = 0;
    dvsBusSend(REG_FAN_SPEED, &zero, 1);
}

// --- Fan control API (declared in control.h) ---------------------------------
void dvsControlSetSpeed(uint8_t speed) {
    if (!g_txEnabled) {
        Serial.println(F("[warn] listen-only: speed command ignored (tx off)"));
        return;
    }
    if (speed > DVS_MAX_SPEED) {
        speed = DVS_MAX_SPEED;
    }
    if (speed == 0) {
        dvsControlSetPower(false);
        return;
    }
    g_fanSpeed = speed;
    g_lastNonZeroSpeed = speed;
    g_fanPower = true;
    g_standby = false; // commanding a speed clears the user-off intent
    dvsBusSend(REG_FAN_SPEED, &speed, 1);
}

void dvsControlSetPower(bool on) {
    if (!g_txEnabled) {
        Serial.println(F("[warn] listen-only: power command ignored (tx off)"));
        return;
    }
    if (on) {
        // Clear the user-off intent and resume: a manual profile returns to its
        // last/default speed; a seasonal profile hands back to auto-control,
        // which recomputes the speed on the next tick.
        g_standby = false;
        if (g_profileSel == DVS_PROFILE_SEL_MANUAL) {
            const uint8_t s = g_lastNonZeroSpeed ? g_lastNonZeroSpeed : DVS_DEFAULT_SPEED;
            dvsControlSetSpeed(s); // sets state + transmits the speed
        } else {
            g_lastControlTickMs = 0; // let auto-control act promptly
        }
    } else {
        // User off: record the intent so auto-control stays suspended (it won't
        // re-power the fan), then stop. "Off" writes fan-speed target 0 (there is
        // no separate stop register); the poll cycle keeps re-writing 0.
        g_standby = true;
        fanOff();
    }
}

bool dvsControlGetPower() { return g_fanPower; }
uint8_t dvsControlGetSpeed() { return g_fanSpeed; }

void dvsControlSetStandby(bool standby) {
    g_standby = standby;
    if (standby) {
        fanOff(); // dvsBusSend is tx-guarded, so this is safe in listen-only
    } else {
        g_lastControlTickMs = 0; // resume auto-control on the next tick
    }
}
bool dvsControlGetStandby() { return g_standby; }

void dvsControlSetTxEnabled(bool enabled) {
    if (g_txEnabled == enabled) {
        return;
    }
    g_txEnabled = enabled;
    if (!enabled) {
        // Dropping to listen-only: forget any "on" intent so the keepalive loop
        // goes quiet. We do NOT transmit anything here - listen-only is silent.
        g_fanPower = false;
        g_fanSpeed = 0;
    } else {
        // Resuming transmit: restart the poll cycle cleanly on the 0x15 write so
        // stale in-flight state can't cause an immediate timeout.
        dvsBusResetPollCycle();
    }
    Serial.printf("[ok] tx=%s (%s)\n", g_txEnabled ? "on" : "off",
                  g_txEnabled ? "listen+send" : "listen-only");
}

bool dvsControlGetTxEnabled() { return g_txEnabled; }

void dvsControlReportObserved(uint8_t targetSpeed) {
    // Mirror a target speed seen on the bus into the reported state, without
    // transmitting or touching the keepalive. Used in listen-only mode so the
    // HA fan entity tracks the real fan (driven by the wall controller).
    g_fanSpeed = targetSpeed;
    g_fanPower = (targetSpeed > 0);
    if (targetSpeed > 0) {
        g_lastNonZeroSpeed = targetSpeed;
    }
}

void dvsControlReportAirflowTemp(float celsius) {
    // Cache every reading (not just published ones) so the auto-control tick has
    // a fresh airflow (supply) temperature and can judge staleness.
    g_airflowTempC = celsius;
    g_airflowTempMs = millis();
    g_haveAirflowTemp = true;
}

// ---- Timed boost (declared in control.h; pure helpers in boost.h) ------------
void dvsControlSetBoost(uint16_t minutes) {
    if (minutes == 0) {
        g_boostUntilMs = 0;
        Serial.println(F("[ok] boost off"));
        return;
    }
    g_boostUntilMs = millis() + (uint32_t)minutes * 60000UL;
    Serial.printf("[ok] boost=%u min -> full speed\n", (unsigned)minutes);
}

uint16_t dvsControlBoostRemainingMin() {
    return dvsBoostRemainingMin(millis(), g_boostUntilMs);
}

uint8_t dvsControlGetTxSpeed() {
    return (dvsBoostTxSpeed(dvsBoostActive(millis(), g_boostUntilMs), g_fanPower,
                           g_fanSpeed, DVS_MAX_SPEED));
}

bool dvsControlBoostTick() {
    // Deadline set but no longer active => it just reached its end.
    if (g_boostUntilMs != 0 && !dvsBoostActive(millis(), g_boostUntilMs)) {
        g_boostUntilMs = 0;
        Serial.println(F("[auto] boost expired"));
        return true;
    }
    return false;
}

// --- Automatic ventilation control (declared in control.h) -------------------
const char *dvsControlProfileName(DvsProfileSel sel) {
    switch (sel) {
        case DVS_PROFILE_SEL_WINTER: return "winter";
        case DVS_PROFILE_SEL_SUMMER: return "summer";
        default: return "manual";
    }
}

void dvsControlSetProfile(DvsProfileSel sel) {
    if (g_profileSel == sel) {
        return;
    }
    g_profileSel = sel;
    g_lastControlTickMs = 0; // act on the change promptly
    Serial.printf("[ok] profile=%s\n", dvsControlProfileName(sel));
}
DvsProfileSel dvsControlGetProfile() { return g_profileSel; }

void dvsControlSetHeatRecovery(bool on) { g_heatRecovery = on; }
bool dvsControlGetHeatRecovery() { return g_heatRecovery; }

void dvsControlSetHeatRetention(bool on) { g_heatRetention = on; }
bool dvsControlGetHeatRetention() { return g_heatRetention; }

void dvsControlSetCutoff(float celsius) { g_profileCfg.cutoffC = celsius; }
float dvsControlGetCutoff() { return g_profileCfg.cutoffC; }

void dvsControlSetComfortTarget(float celsius) {
    g_profileCfg.comfortTargetC = celsius;
}
float dvsControlGetComfortTarget() { return g_profileCfg.comfortTargetC; }

void dvsControlSetInsideTemp(float celsius) {
    g_insideTempC = celsius;
    g_haveInsideTemp = true;
}
bool dvsControlGetInsideTemp(float *celsius) {
    if (!g_haveInsideTemp) {
        return false;
    }
    if (celsius) {
        *celsius = g_insideTempC;
    }
    return true;
}

void dvsControlAutoTick() {
    if (g_standby || g_profileSel == DVS_PROFILE_SEL_MANUAL) {
        return;
    }
    const uint32_t nowMs = millis();
    if (g_lastControlTickMs != 0 &&
        (nowMs - g_lastControlTickMs) < DVS_CONTROL_TICK_MS) {
        return;
    }
    g_lastControlTickMs = nowMs;

    // Listen-only stays silent; the dvsBusSend chokepoint guards too.
    if (!g_txEnabled) {
        return;
    }

    // Fail safe: a stale/absent airflow reading, or an inside temp we have never
    // received, must not drive the fan. Only the airflow is age-checked (it is
    // bus-polled ~1s); the inside temp just has to exist - without it the profile
    // maths would treat a missing reading as 0 C and command a bogus full boost.
    const bool fresh =
        g_haveInsideTemp &&
        dvsProfileInputsFresh(g_haveAirflowTemp, nowMs - g_airflowTempMs, DVS_TEMP_STALE_MS);
    if (!fresh) {
        if (dvsControlGetPower()) {
            fanOff();
            netPublishState();
            Serial.println(F("[auto] temp missing/stale -> fan off (fail-safe)"));
        }
        return;
    }

    DvsProfileInputs in;
    in.airflowTempC = g_airflowTempC;
    in.insideTempC = g_insideTempC;
    in.heatRecovery = g_heatRecovery;
    in.heatRetention = g_heatRetention;
    in.profile = controlResolveProfile();

    const uint8_t target = dvsProfileSpeed(in, g_profileCfg, g_fanSpeed);
    if (target == 0) {
        if (dvsControlGetPower()) {
            fanOff();
            netPublishState();
        }
    } else if (target != g_fanSpeed || !dvsControlGetPower()) {
        dvsControlSetSpeed(target);
        netPublishState();
    }
}

#endif // !UNIT_TEST
