#include "frame.h"

#include <stdio.h>
#include <string.h>

const char *dvs_register_name(uint8_t reg) {
    switch (reg) {
        case REG_DEADMAN:     return "deadman";
        case REG_SET_ADDRESS: return "set-address";
        case REG_STOP:        return "stop";
        case REG_TEMP:        return "temperature";
        case REG_FAN_SPEED:   return "fan-speed";
        default:              return "?";
    }
}

bool dvs_frame_crc_ok(const DvsFrame &f, const Crc16Params &params, bool crcBigEndian) {
    return crc16_verify_frame(f.bytes, f.len, params, crcBigEndian);
}

// Little-endian integer from up to 4 payload bytes.
static uint32_t le_value(const uint8_t *p, size_t n) {
    uint32_t v = 0;
    for (size_t i = 0; i < n && i < 4; ++i) {
        v |= (uint32_t)p[i] << (8 * i);
    }
    return v;
}

// Append the register-specific decode to `out` at offset `n`.
static int append_decode(const DvsFrame &f, char *out, size_t outLen, int n) {
    // Body = everything except the trailing 2 CRC bytes; payload starts after
    // addr + register.
    if (!dvs_frame_has_body(f)) {
        return n;
    }
    const uint8_t reg = dvs_frame_register(f);
    const uint8_t *payload = dvs_frame_payload(f);
    const size_t payloadLen = dvs_frame_payload_len(f);

    switch (reg) {
        case REG_TEMP:
            if (payloadLen >= 2) {
                // Payload byte 0 is a raw airflow (supply-air) sensor count
                // (bytes 1,2 seen as 0). Least-squares linear fit to 8 live
                // points spanning 11-20 C: temp_C = 0.06807*raw - 1.039 (~14.7
                // counts/C), all residuals < 0.75 C (display is integer-rounded).
                // May need revisiting at extremes - no data below ~11 or above 20.
                const int raw = (int)le_value(payload, payloadLen);
                const float tc = dvs_temp_c_from_raw(raw);
                n += snprintf(out + n, (n < (int)outLen) ? outLen - n : 0,
                            " temp_raw=%d temp_c=%.1f", raw, (double)tc);
            }
            break;
        case REG_FAN_SPEED:
            // Register 0x15 appears in two payload formats with distinct
            // behaviour when the unit is switched on (inferred from live
            // capture, not from bus direction - this is a passive tap):
            //   - 1-byte payload steps instantly (0 -> 54) => commanded/target.
            //   - 2-byte payload ramps gradually (0,1,20,32,38...) => actual
            //     speed catching up to the target (soft-start).
            if (payloadLen == 1) {
                n += snprintf(out + n, (n < (int)outLen) ? outLen - n : 0,
                            " speed_target=%u", (unsigned)payload[0]);
            } else if (payloadLen >= 2) {
                n += snprintf(out + n, (n < (int)outLen) ? outLen - n : 0,
                            " speed_actual=%u",
                            (unsigned)le_value(payload, payloadLen));
            }
            break;
        default:
            break;
    }
    return n;
}

int dvs_frame_format(const DvsFrame &f, char *out, size_t outLen,
                     const Crc16Params &params, bool crcBigEndian) {
    if (outLen == 0) {
        return 0;
    }
    int n = 0;
    n += snprintf(out + n, outLen - (size_t)n, "%luus len=%u",
                  (unsigned long)f.startMicros, (unsigned)f.len);

    // Hex dump of raw bytes.
    for (size_t i = 0; i < f.len; ++i) {
        n += snprintf(out + n, (n < (int)outLen) ? outLen - (size_t)n : 0,
                      " %02X", f.bytes[i]);
    }

    const bool crcOk = dvs_frame_crc_ok(f, params, crcBigEndian);
    n += snprintf(out + n, (n < (int)outLen) ? outLen - (size_t)n : 0,
                  " CRC=%s", crcOk ? "OK" : "BAD");

    if (f.len >= 2) {
        const uint8_t addr = f.bytes[0];
        const uint8_t reg = f.bytes[1];
        n += snprintf(out + n, (n < (int)outLen) ? outLen - (size_t)n : 0,
                      " addr=0x%02X reg=0x%02X (%s)", addr, reg,
                      dvs_register_name(reg));
        n = append_decode(f, out, outLen, n);
    }
    return n;
}

size_t dvs_frame_build(uint8_t addr, uint8_t reg,
                       const uint8_t *payload, size_t payloadLen,
                       uint8_t *out, size_t outLen,
                       const Crc16Params &params, bool crcBigEndian) {
    const size_t bodyLen = 2 + payloadLen; // addr + register + payload
    const size_t total = bodyLen + 2;      // + 2 CRC bytes
    if (out == nullptr || outLen < total) {
        return 0;
    }
    out[0] = addr;
    out[1] = reg;
    for (size_t i = 0; i < payloadLen; ++i) {
        out[2 + i] = payload[i];
    }
    const uint16_t crc = crc16_ccitt(out, bodyLen, params);
    const uint8_t hi = (uint8_t)((crc >> 8) & 0xFF);
    const uint8_t lo = (uint8_t)(crc & 0xFF);
    if (crcBigEndian) {
        out[bodyLen] = hi;
        out[bodyLen + 1] = lo;
    } else {
        out[bodyLen] = lo;
        out[bodyLen + 1] = hi;
    }
    return total;
}
