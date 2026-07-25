#pragma once

#include <stddef.h>
#include <stdint.h>

#include "config.h"
#include "crc16.h"

// A single buffered frame captured from the bus.
struct DvsFrame {
    uint8_t bytes[MAX_FRAME_LEN];
    size_t len = 0;
    uint32_t startMicros = 0; // timestamp of the first byte
};

// Human-readable name for a known register, or "?" when unknown.
const char *dvs_register_name(uint8_t reg);

// True when the trailing two bytes are a valid CRC over the body.
bool dvs_frame_crc_ok(const DvsFrame &f, const Crc16Params &params = Crc16Params{},
                      bool crcBigEndian = false);

// Append a decoded, single-line human-readable summary of `f` into `out`.
// Writes at most `outLen` bytes (always NUL-terminated). Returns the number of
// characters that would have been written (snprintf semantics).
// Format: "<us>us len=<n> <hex...> CRC=<OK|BAD> addr=0x.. reg=0x.. (<name>) <decode>"
int dvs_frame_format(const DvsFrame &f, char *out, size_t outLen,
                     const Crc16Params &params = Crc16Params{},
                     bool crcBigEndian = false);
