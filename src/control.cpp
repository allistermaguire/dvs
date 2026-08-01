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

#include "bus.h"
#include "config.h"
#include "control.h"
#include "net.h"

// --- Fan control state (commanded, i.e. what we last told the fan) -----------
static bool g_txEnabled = DVS_TX_ENABLED_DEFAULT; // false => listen-only
static bool g_fanPower = false;           // true while we intend the fan to run
static uint8_t g_fanSpeed = 0;            // last commanded target speed (0..84)
static uint8_t g_lastNonZeroSpeed = 0; // remembered speed for a bare "power on"

// --- Auto ventilation control state ------------------------------------------
static DvsControlMode g_controlMode = DVS_MODE_MANUAL;    // manual vs auto profile
static DvsProfileSel g_profileSel = DVS_PROFILE_SEL_AUTO; // summer/winter/auto
static DvsProfile g_effectiveProfile = DVS_PROFILE_WINTER; // AUTO resolved value
static bool g_moistureFloor = DVS_MOISTURE_FLOOR_DEFAULT; // winter moisture vent
static float g_insideTempC = 0.0f;        // last inside temp fed in (MQTT)
static bool g_haveInsideTemp = false;
static uint32_t g_insideTempMs = 0;       // millis() of the last inside-temp update
static float g_roofTempC = 0.0f;          // last roof temp sniffed from the bus
static bool g_haveRoofTemp = false;
static uint32_t g_roofTempMs = 0;         // millis() of the last roof-temp reading
static uint32_t g_lastControlTickMs = 0; // last auto-tick time (0 => act now)

// Profile tuning (config.h boot defaults), built once rather than per tick.
static const DvsProfileConfig g_profileCfg = {
    DVS_PROFILE_DEADBAND_ON_C, DVS_PROFILE_DEADBAND_OFF_C,
    DVS_PROFILE_SPAN_C,        DVS_PROFILE_COMFORT_HIGH_C,
    DVS_PROFILE_COMFORT_LOW_C, DVS_PROFILE_MIN_SUPPLY_C,
    DVS_PROFILE_MAX_SUPPLY_C,  DVS_PROFILE_SUPPLY_HYST_C,
    DVS_PROFILE_MIN_SPEED,     DVS_PROFILE_MAX_SPEED,
};

// Resolve the profile in effect (AUTO -> summer/winter by inside temp, with
// hysteresis inside dvsProfileAuto). With no inside reading yet, AUTO keeps the
// current effective profile rather than resolving against a bogus 0 C.
static DvsProfile controlResolveProfile() {
    switch (g_profileSel) {
        case DVS_PROFILE_SEL_SUMMER:
            return DVS_PROFILE_SUMMER;
        case DVS_PROFILE_SEL_WINTER:
            return DVS_PROFILE_WINTER;
        default:
            return g_haveInsideTemp
                ? dvsProfileAuto(g_insideTempC, DVS_PROFILE_COMFORT_TARGET_C,
                                 g_effectiveProfile)
                : g_effectiveProfile;
    }
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

void dvsControlReportRoofTemp(float celsius) {
    // Cache every reading (not just published ones) so the auto-control tick has
    // a fresh roof (supply) temperature and can judge staleness.
    g_roofTempC = celsius;
    g_roofTempMs = millis();
    g_haveRoofTemp = true;
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
}
DvsProfileSel dvsControlGetProfile() { return g_profileSel; }
DvsProfile dvsControlGetEffectiveProfile() { return g_effectiveProfile; }

void dvsControlSetMoistureFloor(bool on) { g_moistureFloor = on; }
bool dvsControlGetMoistureFloor() { return g_moistureFloor; }

void dvsControlSetInsideTemp(float celsius) {
    g_insideTempC = celsius;
    g_haveInsideTemp = true;
    g_insideTempMs = millis();
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

    // Fail safe: a stale/absent roof or inside reading must not hold the fan on.
    const bool roofFresh =
        g_haveRoofTemp && (nowMs - g_roofTempMs) < DVS_TEMP_STALE_MS;
    const bool insideFresh =
        g_haveInsideTemp && (nowMs - g_insideTempMs) < DVS_TEMP_STALE_MS;
    if (!roofFresh || !insideFresh) {
        if (dvsControlGetPower()) {
            dvsControlSetPower(false);
            netPublishState();
            Serial.println(F("[auto] temp stale -> fan off (fail-safe)"));
        }
        return;
    }

    g_effectiveProfile = controlResolveProfile();

    DvsProfileInputs in;
    in.roofTempC = g_roofTempC;
    in.insideTempC = g_insideTempC;
    in.moistureFloor = g_moistureFloor;
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
