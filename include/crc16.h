#pragma once

#include <stddef.h>
#include <stdint.h>

// CRC16-CCITT as used on the DVS bus.
//
// The exact variant is confirmed empirically against captured frames:
// CRC-16/AUG-CCITT (poly 0x1021, init 0x1D0F, no reflection, no xor-out), with
// the two CRC bytes transmitted big-endian (MSB first). The defaults below
// reflect that confirmed variant.

struct Crc16Params {
    uint16_t init = 0x1D0F;
    bool reflected = false; // reflect input bytes and the final result
};

// Compute the CRC over `len` bytes of `data`.
uint16_t crc16_ccitt(const uint8_t *data, size_t len,
                     const Crc16Params &params = Crc16Params{});

// Verify a full frame whose last two bytes are the transmitted CRC.
// DVS transmits the checksum big-endian (MSB first); pass `crcBigEndian`
// false if captures show otherwise. Returns true when the CRC matches.
bool crc16_verify_frame(const uint8_t *frame, size_t len,
                        const Crc16Params &params = Crc16Params{},
                        bool crcBigEndian = true);
