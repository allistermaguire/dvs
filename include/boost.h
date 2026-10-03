#pragma once

#include <stdint.h>

// ----------------------------------------------------------------------------
// Pure timed-boost helpers (no Arduino/hardware dependencies), so the decision
// logic is host-tested like crc16/frame/profile. control.cpp owns the deadline
// (a millis() timestamp) and the fan state, and calls these to decide behaviour.
// ----------------------------------------------------------------------------

// True while a boost is running: a deadline is set (untilMs != 0) and now has not
// reached it. The comparison is wrap-safe, so it holds across millis() overflow.
bool dvsBoostActive(uint32_t nowMs, uint32_t untilMs);

// Minutes left on an active boost, rounded UP to whole minutes (0 when inactive).
uint16_t dvsBoostRemainingMin(uint32_t nowMs, uint32_t untilMs);

// Speed to transmit now: full speed while a boost is active AND the fan is
// powered, otherwise the commanded base speed.
uint8_t dvsBoostTxSpeed(bool active, bool powered, uint8_t baseSpeed,
                        uint8_t maxSpeed);
