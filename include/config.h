#pragma once

// -----------------------------------------------------------------------------
// DVS G3 RS485 passive sniffer - build/runtime configuration.
//
// Verified protocol facts (Ian Rees, "Reverse engineering the DVS bus"):
//   - RS485, 4800 baud, LSb first (== normal UART framing), assumed 8N1.
//   - Frame: [addr:1] [register:1] [payload:0+] [crc16:2]
//   - CRC16-CCITT over the preceding bytes (variant confirmed against captures).
//   - Controller polls the fan; the fan responds. Bad CRC => fan ignores frame.
// -----------------------------------------------------------------------------

#include <stdint.h>

// --- UART pin mapping (D1 Mini ESP32 / WROOM, UART2) -------------------------
// MAX3485 RO -> RS485_RX_PIN, MAX3485 DI -> RS485_TX_PIN (unused while listening).
static constexpr int RS485_RX_PIN = 16;
static constexpr int RS485_TX_PIN = 17;

// MAX3485 DE + RE tied together to this GPIO. LOW = listen-only
// (driver disabled, receiver enabled). Kept on a GPIO so TX can be added later.
static constexpr int RS485_DE_RE_PIN = 4;

// --- Serial line settings ----------------------------------------------------
static constexpr uint32_t USB_BAUD = 115200;      // debug/monitor console
static constexpr uint32_t DEFAULT_BUS_BAUD = 4800; // DVS bus (runtime-adjustable)

// --- Frame boundary detection ------------------------------------------------
// At 4800 8N1 one byte takes ~2.08 ms. An idle gap longer than this threshold
// is treated as the end of a frame. Runtime-adjustable via the USB console.
static constexpr uint32_t DEFAULT_GAP_THRESHOLD_US = 3500;

// Largest frame we will buffer before force-flushing.
static constexpr size_t MAX_FRAME_LEN = 64;

// --- Known DVS registers -----------------------------------------------------
enum DvsRegister : uint8_t {
    REG_DEADMAN     = 0x03, // write occasionally to keep the fan spinning
    REG_SET_ADDRESS = 0x04, // 1 byte payload -> becomes new peripheral address
    REG_STOP        = 0x05, // stop the fan
    REG_TEMP        = 0x10, // roof-space temp: payload byte0 = raw count (cal TODO)
    REG_FAN_SPEED   = 0x15, // fan speed 0..84: 1-byte payload=target, 2-byte=actual
};

// --- Two-sensor communication (resolved) -------------------------------------
// The G3 has a roof-space sensor and a controller (in-home) sensor. Live capture
// shows only ONE temperature on the bus: register 0x10 from the fan (addr 0x02)
// carries the ROOF-SPACE reading. The IN-HOME sensor does NOT appear on the
// RS485 bus - it is wired directly to the wall controller and read locally. So
// the two sensors are communicated by different means, not two bus registers.
//
// Temperature scaling: payload byte 0 is a raw roof-space sensor count. A
// least-squares fit to 8 live points over 11~20 C is linear (residuals < 0.75 C,
// display is integer-rounded):
//   temp_C = 0.06807 * raw - 1.039   (~14.7 counts per C)
// Points (raw -> C): 180->11, 212->14, 240->15, 248->16, 276->17, 280->18,
// 292->19, 304->20. TODO: confirm at extremes (no data yet below ~11 or above 20).
