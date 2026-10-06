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

// Frame-field accessors for the wire layout [addr:1] [reg:1] [payload:0+] [crc16:2].
// Only meaningful once the frame holds at least addr + reg + 2 CRC bytes; callers
// should check dvs_frame_has_body(f) first.
inline bool dvs_frame_has_body(const DvsFrame &f) { return f.len >= 4; }
inline uint8_t dvs_frame_register(const DvsFrame &f) { return f.bytes[1]; }
inline const uint8_t *dvs_frame_payload(const DvsFrame &f) { return &f.bytes[2]; }
inline size_t dvs_frame_payload_len(const DvsFrame &f) {
return f.len - 4; // minus addr, register, 2x CRC
}

// Convert a raw airflow (supply-air) sensor count (payload byte 0 of REG_TEMP) to deg C.
inline float dvs_temp_c_from_raw(int raw) {
    return raw * DVS_AIRFLOW_CAL_SCALE - DVS_AIRFLOW_CAL_OFFSET;
}

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

// Build a transmittable DVS frame `[addr][reg][payload...][crc16:2]` into `out`.
// The CRC is computed over addr+register+payload and appended in the byte order
// selected by `crcBigEndian` (the DVS wire order is big-endian). `payload` may be
// nullptr when `payloadLen` is 0. Returns the total number of bytes written, or 0
// if `out` is null or too small to hold the whole frame.
size_t dvs_frame_build(uint8_t addr, uint8_t reg,
                       const uint8_t *payload, size_t payloadLen,
                       uint8_t *out, size_t outLen,
                       const Crc16Params &params = Crc16Params{},
                       bool crcBigEndian = true);
