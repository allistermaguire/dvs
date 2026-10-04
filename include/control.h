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
// above). In `auto` mode a periodic tick computes a target speed from the roof
// (supply) temperature and the inside temperature via the profile logic and
// drives the fan - but only while transmit is enabled, and it fails safe to OFF
// when either temperature is stale.
enum DvsControlMode : uint8_t {
    DVS_MODE_MANUAL = 0,
    DVS_MODE_AUTO = 1,
};

// Profile selection. AUTO resolves to summer/winter each tick from the inside
// temperature (for the shoulder seasons); SUMMER/WINTER force a profile.
enum DvsProfileSel : uint8_t {
    DVS_PROFILE_SEL_SUMMER = 0,
    DVS_PROFILE_SEL_WINTER = 1,
    DVS_PROFILE_SEL_AUTO = 2,
};

void dvsControlSetMode(DvsControlMode mode);
DvsControlMode dvsControlGetMode();

void dvsControlSetProfile(DvsProfileSel sel);
DvsProfileSel dvsControlGetProfile();
// The profile actually in effect right now (AUTO resolved to summer/winter).
DvsProfile dvsControlGetEffectiveProfile();

// Winter heat recovery: ramp the fan speed up above the moisture baseline to
// harvest warm roof air when it is as warm as, or warmer than, the house. With
// it off the winter fan just holds the low moisture-ventilation baseline.
void dvsControlSetHeatRecovery(bool on);
bool dvsControlGetHeatRecovery();

// Winter retain heat: when on, raw roof air below the cold cutoff (minSupply) is
// too cold to supply, so the fan steps down and stops to keep the house warm.
// With it off the fan keeps holding the moisture baseline below the cutoff
// (prioritise condensation control over heat retention). Winter profile only.
void dvsControlSetRetainHeat(bool on);
bool dvsControlGetRetainHeat();

// Winter cold cutoff (minSupply, deg C): the roof temperature below which the fan
// stops (retain heat on). Runtime-settable; boots to DVS_PROFILE_MIN_SUPPLY_C and
// falls back to it when unset.
void dvsControlSetMinSupply(float celsius);
float dvsControlGetMinSupply();

// Feed the inside temperature (deg C), typically from a Home Assistant sensor
// over MQTT. Records the value and a freshness timestamp.
void dvsControlSetInsideTemp(float celsius);
// Latest inside temperature. Returns false if none has ever been received.
bool dvsControlGetInsideTemp(float *celsius);

// Feed the roof (supply) temperature sniffed from the bus (deg C). Records the
// value and a freshness timestamp for the auto-control fail-safe.
void dvsControlReportRoofTemp(float celsius);

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
