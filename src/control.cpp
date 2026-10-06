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

// --- Auto ventilation control state ------------------------------------------
static DvsControlMode g_controlMode = DVS_MODE_MANUAL;      // manual vs auto profile
static DvsProfileSel g_profileSel = DVS_PROFILE_SEL_WINTER; // summer/winter (no auto)
static DvsProfile g_effectiveProfile = DVS_PROFILE_WINTER;  // AUTO resolved value
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

// Map the selected profile to the one in effect. There is no AUTO option - the
// user (or an external automation such as a Home Assistant seasonal switch)
// chooses summer or winter directly.
static DvsProfile controlResolveProfile() {
    return g_profileSel == DVS_PROFILE_SEL_SUMMER ? DVS_PROFILE_SUMMER
                                                : DVS_PROFILE_WINTER;
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
    dvsBusSend(REG_FAN_SPEED, &speed, 1);
}

void dvsControlSetPower(bool on) {
    if (!g_txEnabled) {
        Serial.println(F("[warn] listen-only: power command ignored (tx off)"));
        return;
    }
    if (on) {
        const uint8_t s = g_lastNonZeroSpeed ? g_lastNonZeroSpeed : DVS_DEFAULT_SPEED;
        dvsControlSetSpeed(s); // sets state + transmits the speed
    } else {
        // "Off" matches the wall controller: write fan-speed target 0 (there is
        // no separate stop register in captures). Send one explicit 0 now for an
        // immediate response; the poll cycle then keeps re-writing the target
        // (now 0) at the controller cadence rather than going silent - on/off is
        // expressed purely through the target value the keepalive re-sends, not
        // by starting/stopping the transmit.
        g_fanPower = false;
        g_fanSpeed = 0;
        const uint8_t zero = 0;
        dvsBusSend(REG_FAN_SPEED, &zero, 1);
    }
}

bool dvsControlGetPower() { return g_fanPower; }
uint8_t dvsControlGetSpeed() { return g_fanSpeed; }

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
void dvsControlSetMode(DvsControlMode mode) {
    if (g_controlMode == mode) {
        return;
    }
    g_controlMode = mode;
    g_lastControlTickMs = 0; // recompute on the next loop iteration
    Serial.printf("[ok] mode=%s\n", mode == DVS_MODE_AUTO ? "auto" : "manual");
}
DvsControlMode dvsControlGetMode() { return g_controlMode; }

void dvsControlSetProfile(DvsProfileSel sel) {
    g_profileSel = sel;
    g_effectiveProfile = controlResolveProfile(); // report state immediately
    g_lastControlTickMs = 0;                      // act on the change promptly
    Serial.printf("[ok] profile=%s\n",
                  sel == DVS_PROFILE_SEL_SUMMER ? "summer" : "winter");
}
DvsProfileSel dvsControlGetProfile() { return g_profileSel; }
DvsProfile dvsControlGetEffectiveProfile() { return g_effectiveProfile; }

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
    if (g_controlMode != DVS_MODE_AUTO) {
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
            dvsControlSetPower(false);
            netPublishState();
            Serial.println(F("[auto] temp missing/stale -> fan off (fail-safe)"));
        }
        return;
    }

    g_effectiveProfile = controlResolveProfile();

    DvsProfileInputs in;
    in.airflowTempC = g_airflowTempC;
    in.insideTempC = g_insideTempC;
    in.heatRecovery = g_heatRecovery;
    in.heatRetention = g_heatRetention;
    in.profile = g_effectiveProfile;

    const uint8_t target = dvsProfileSpeed(in, g_profileCfg, g_fanSpeed);
    if (target == 0) {
        if (dvsControlGetPower()) {
            dvsControlSetPower(false);
            netPublishState();
        }
    } else if (target != g_fanSpeed || !dvsControlGetPower()) {
        dvsControlSetSpeed(target);
        netPublishState();
    }
}

#endif // !UNIT_TEST
