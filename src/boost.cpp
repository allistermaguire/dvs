// Pure timed-boost decision logic for the DVS controller (see boost.h). Kept
// free of Arduino/hardware deps so it is exercised directly by the native host
// unit tests, mirroring crc16/frame/profile.

#include "boost.h"

bool dvsBoostActive(uint32_t nowMs, uint32_t untilMs) {
    return untilMs != 0 && (int32_t)(nowMs - untilMs) < 0;
}

uint16_t dvsBoostRemainingMin(uint32_t nowMs, uint32_t untilMs) {
    if (!dvsBoostActive(nowMs, untilMs)) {
        return 0;
    }
    const uint32_t remMs = untilMs - nowMs; // wrap-safe: active => now before until
    return (uint16_t)((remMs + 59999UL) / 60000UL);
}

uint8_t dvsBoostTxSpeed(bool active, bool powered, uint8_t baseSpeed,
                        uint8_t maxSpeed) {
    return (active && powered) ? maxSpeed : baseSpeed;
}
