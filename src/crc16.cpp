#include "crc16.h"

static uint8_t reflect8(uint8_t b) {
    b = (uint8_t)((b & 0xF0) >> 4 | (b & 0x0F) << 4);
    b = (uint8_t)((b & 0xCC) >> 2 | (b & 0x33) << 2);
    b = (uint8_t)((b & 0xAA) >> 1 | (b & 0x55) << 1);
    return b;
}

static uint16_t reflect16(uint16_t v) {
    uint16_t r = 0;
    for (int i = 0; i < 16; ++i) {
        if (v & (1u << i)) {
            r |= (uint16_t)(1u << (15 - i));
        }
    }
    return r;
}

uint16_t crc16_ccitt(const uint8_t *data, size_t len, const Crc16Params &params) {
    uint16_t crc = params.init;
    for (size_t i = 0; i < len; ++i) {
        uint8_t byte = params.reflected ? reflect8(data[i]) : data[i];
        crc ^= (uint16_t)byte << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    if (params.reflected) {
        crc = reflect16(crc);
    }
    return crc;
}

bool crc16_verify_frame(const uint8_t *frame, size_t len,
                        const Crc16Params &params, bool crcBigEndian) {
    if (len < 3) {
        return false; // need at least addr + register + 2 CRC bytes-ish
    }
    const size_t bodyLen = len - 2;
    const uint16_t computed = crc16_ccitt(frame, bodyLen, params);
    const uint8_t lo = frame[len - 2];
    const uint8_t hi = frame[len - 1];
    const uint16_t transmitted =
        crcBigEndian ? (uint16_t)((lo << 8) | hi) : (uint16_t)((hi << 8) | lo);
    return computed == transmitted;
}
