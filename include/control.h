#pragma once

#include <stdint.h>

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
