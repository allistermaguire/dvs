#pragma once

#include <stdint.h>

#include "profile.h"

// Fan control API. Implemented in main.cpp (which owns the RS485 bus) so there
// is a single place that drives the MAX3485 DE/RE line and Serial2. The MQTT
// layer (net.cpp) calls these to act on incoming commands.
//
// These functions transmit on the DVS bus. See config.h for the caveats.

// Turn the fan on (resume last non-zero speed, or DVS_DEFAULT_SPEED) or off.
// "Off" writes fan-speed target 0 (register 0x15); there is no separate stop
// regsiter on this bus.
void dvsControlSetPower(bool on);

// Command a target fan speed 0..DVS_MAX_SPEED (register 0x15). Speed 0 is
// treated as power off.
void dvsControlSetSpeed(uint8_t speed);

// Current commanded state (what we last told the fan to do).
bool dvsControlGetPower();
uint8_t dvsControlGetSpeed();

// Transmit mode. When disabled the firmware is listen-only: control commands are
// ignored and nothing is ever driven onto the bus. When enabled it may transmit
// (listen + send). Switching to listen-only also stops the fan bookkeeping.
void dvsControlSetTxEnabled(bool enabled);
bool dvsControlGetTxEnabled();

// Mirror a fan speed target seen on the bus into the reported state (no
// transmit). In listen-only mode this lets the reported/MQTT fan state follow
// the real fan as commanded by the existing wall controller.
void dvsControlReportObserved(uint8_t targetSpeed);

// --- Automatic ventilation control (summer / winter profiles) --------------
// In `manual` mode the fan follows explicit power/speed commands (the behaviour
// above). In `auto` mode a periodic tick computes a target speed from the airflow
// (supply) temperature and the inside temperature via the profile logic and
// drives the fan - but only while transmit is enabled, and it fails safe to OFF
// when either temperature is stale.
enum DvsControlMode : uint8_t {
    DVS_MODE_MANUAL = 0,
    DVS_MODE_AUTO = 1,
};

// Profile selection: summer or winter, chosen explicitly (by the user or an
// external automation, e.g. a Home Assistant seasonal switch). There is no
// built-in auto season-switch: a single comfort setpoint can't tell a warm
// winter day from summer, so it would wrongly flip into active cooling.
enum DvsProfileSel : uint8_t {
    DVS_PROFILE_SEL_SUMMER = 0,
    DVS_PROFILE_SEL_WINTER = 1,
};

void dvsControlSetMode(DvsControlMode mode);
DvsControlMode dvsControlGetMode();

void dvsControlSetProfile(DvsProfileSel sel);
DvsProfileSel dvsControlGetProfile();
// The profile actually in effect right now (summer or winter).
DvsProfile dvsControlGetEffectiveProfile();

// Winter heat recovery: ramp the fan speed up above the moisture baseline to
// harvest warm airflow when it is as warm as, or warmer than, the house. With
// it off the winter fan just holds the low moisture-ventilation baseline.
void dvsControlSetHeatRecovery(bool on);
bool dvsControlGetHeatRecovery();

// Winter heat retention: when on, raw airflow below the cold cutoff is too cold
// to supply, so the fan steps down and stops to keep the house warm. With it off
// the fan keeps holding the moisture baseline below the cutoff (prioritise
// condensation control over heat retention). Winter profile only.
void dvsControlSetHeatRetention(bool on);
bool dvsControlGetHeatRetention();

// Winter cold cutoff (deg C): the airflow temperature below which the fan stops
// (heat retention on). Runtime-settable; boots to DVS_PROFILE_CUTOFF_C and falls
// back to it when unset.
void dvsControlSetCutoff(float celsius);
float dvsControlGetCutoff();

// Target (comfort) inside temperature (deg C): the single setpoint each profile
// aims for. Winter ramps up to it (heat recovery) then holds the moisture
// baseline; summer cools down to it then stops. Runtime-settable; boots to
// DVS_PROFILE_COMFORT_TARGET_C.
void dvsControlSetComfortTarget(float celsius);
float dvsControlGetComfortTarget();

// Feed the inside temperature (deg C), typically from a Home Assistant sensor
// over MQTT. Records the value and a freshness timestamp.
void dvsControlSetInsideTemp(float celsius);
// Latest inside temperature. Returns false if none has ever been received.
bool dvsControlGetInsideTemp(float *celsius);

// Feed the airflow (supply) temperature sniffed from the bus (deg C). Records the
// value and a freshness timestamp for the auto-control fail-safe.
void dvsControlReportAirflowTemp(float celsius);

// ---- Timed boost -----------------------------------------------------------
// Drive the fan to full speed for `minutes` (0 cancels), overlaid on the current
// manual/auto target and expiring automatically. Only reaches the bus while the
// fan is powered and transmit is enabled.
void dvsControlSetBoost(uint16_t minutes);
// Minutes remaining on an active boost, rounded up (0 when inactive).
uint16_t dvsControlBoostRemainingMin();
// The speed to transmit right now: full speed while a boost is active (and the
// fan is powered), otherwise the commanded speed. Used by the bus poll cycle.
uint8_t dvsControlGetTxSpeed();
// Clear a boost whose deadline has passed; returns true if it just expired so
// the caller can republish state.
bool dvsControlBoostTick();

// Run one non-blocking step of the auto-control loop. Call from loop(); it only
// acts on its own DVS_CONTROL_TICK_MS cadence and only in auto mode.
void dvsControlAutoTick();
