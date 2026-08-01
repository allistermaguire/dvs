#pragma once

#include <stddef.h>
#include <stdint.h>

// Bus seam implemented by the RS485 owner (main.cpp). The control layer
// (control.cpp) calls these to command the fan without owning the transceiver,
// keeping control a pure policy layer over a single hardware chokepoint.

// Build and transmit a command frame addressed to the fan. A no-op in
// listen-only mode (guarded at this single chokepoint), and echoes the frame to
// the console like a received frame.
void dvsBusSend(uint8_t reg, const uint8_t *payload, size_t payloadLen);

// Restart the transmit poll cycle cleanly. Called when transmit is (re)enabled
// so stale in-flight state cannot cause an immediate timeout.
void dvsBusResetPollCycle();
