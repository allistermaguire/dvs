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

#include <stddef.h>
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
    REG_DEADMAN     = 0x03, // NOT observed in live control captures (see below)
    REG_SET_ADDRESS = 0x04, // 1 byte payload -> becomes new peripheral address
    REG_STOP        = 0x05, // NOT observed; "off" is really speed 0 via 0x15
    REG_TEMP        = 0x10, // roof-space temp: payload byte0 = raw count (cal TODO)
    REG_FAN_SPEED   = 0x15, // fan speed 0..84: 1-byte payload=target, 2-byte=actual
};

// Periodic status-block registers the fans answers with (identity/flags/telemetry).
// Watched over MQTT to learn what triggers the block; see publishObserved().
static constexpr uint8_t DVS_STATUS_REGS[] = {0x01, 0x06, 0x12, 0x13};
static constexpr size_t DVS_STATUS_REG_COUNT =
    sizeof(DVS_STATUS_REGS) / sizeof(DVS_STATUS_REGS[0]);

// True when `reg` is one of the periodic status-block registers above.
static inline bool dvs_is_status_reg(uint8_t reg) {
    for (size_t i = 0; i < DVS_STATUS_REG_COUNT; ++i) {
        if (DVS_STATUS_REGS[i] == reg) {
            return true;
        }
    }
    return false;
}

// Roof-space temperature calibration: temp_C = raw * SCALE - OFFSET.
// Least-squares fit to 8 live points over 11-20 C (residuals < 0.75 C).
static constexpr float DVS_TEMP_CAL_SCALE   = 0.06807f;
static constexpr float DVS_TEMP_CAL_OFFSET  = 1.039f;

// --- Control (transmit) parameters ----------------------------------------
// The base firmware is listen-only; enabling MQTT control makes the ESP32 an
// active talker on the bus. These describe how commands are addressed and paced.
//
//   WARNING: transmitting shares the bus with the existing wall controller.
//   Collisions are possible. Only enable control on a system you can recover.
//
// Address of the fan/air-handler peripheral (register 0x10 is seen from addr
// 0x02 in live captures, so commands are written to that address).
static constexpr uint8_t DVS_FAN_ADDR = 0x02;

// Fan speed limits and the speed used when "power on" has no remembered speed.
static constexpr uint8_t DVS_MAX_SPEED = 84;      // measured 0 (off) .. 84 (boost)
static constexpr uint8_t DVS_DEFAULT_SPEED = 42;  // mid speed for a bare "on"

// Keep-alive cadence. Live captures show the wall controller does NOT use a
// separate deadman (0x03); instead it continuously re-writes the fan-speed
// target (0x15) about every ~250 ms. We mirror that: while the fan is meant to
// run we re-send the current target at this interval. "Off" is a single 0x15=0.
static constexpr uint32_t DVS_KEEPALIVE_INTERVAL_MS = 300;

// Transmit mode at boot: false = listen-only (never drive the bus),
// true = listen + send (control commands are allowed to transmit). Switchable at
// runtime via the `tx` console command or the dvs/tx/set MQTT topic.
static constexpr bool DVS_TX_ENABLED_DEFAULT = false;

// --- Network / MQTT timing ----------------------------------------------------
static constexpr uint32_t WIFI_RECONNECT_MS = 10000; // retry Wi-Fi join cadence
static constexpr uint32_t MQTT_RECONNECT_MS = 5000;  // retry broker connect cadence
// How often the current fan state is (re)published to MQTT, in addition to an
// immediate publish whenever a command is applied. Keeps retained topics fresh
// for clients that (re)connect between commands.
static constexpr uint32_t STATE_PUBLISH_MS  = 15000; // periodic state republish

// --- MQTT topics --------------------------------------------------------------
// Broker/credentials live in secrets.h; the topic tree is fixed here.
#define MQTT_TOPIC_BASE          "dvs"
#define MQTT_TOPIC_AVAILABILITY  MQTT_TOPIC_BASE "/status"       // online/offline (LWT)
#define MQTT_TOPIC_POWER_SET     MQTT_TOPIC_BASE "/power/set"    // in:  ON / OFF
#define MQTT_TOPIC_POWER_STATE   MQTT_TOPIC_BASE "/power/state"  // out: ON / OFF
#define MQTT_TOPIC_SPEED_SET     MQTT_TOPIC_BASE "/speed/set"    // in:  0..84
#define MQTT_TOPIC_SPEED_STATE   MQTT_TOPIC_BASE "/speed/state"  // out: commanded 0..84
#define MQTT_TOPIC_ACTUAL_SPEED  MQTT_TOPIC_BASE "/speed/actual" // out: sniffed actual
#define MQTT_TOPIC_TEMPERATURE   MQTT_TOPIC_BASE "/temperature"  // out: sniffed roof temp
#define MQTT_TOPIC_TX_SET        MQTT_TOPIC_BASE "/tx/set"       // in:  ON / OFF (send vs listen)
#define MQTT_TOPIC_TX_STATE      MQTT_TOPIC_BASE "/tx/state"     // out: ON / OFF
// Raw payload of the periodic status-block registers (0x01/0x06/0x12/0x13),
// published as hex whenever the fan responds. Register hex is appended, e.g.
// dvs/status/reg/06. Watch these to learn what triggers the block and how often.
#define MQTT_TOPIC_STATUS_BASE  MQTT_TOPIC_BASE "/status/reg/"  // + "%02X"

// --- Home Assistant MQTT discovery ------------------------------------------
// When enabled, retained discovery configs are published on connect so HA
// auto-creates a fan + sensors + switch grouped under one device. The prefix
// must match Home Assistant's MQTT discovery prefix (default "homeassistant").
static constexpr bool HA_DISCOVERY_ENABLED = true;
#define HA_DISCOVERY_PREFIX "homeassistant"
#define HA_DEVICE_ID        "dvs_sniffer"
#define HA_DEVICE_NAME      "DVS G3"
// Shared JSON fragments (HA short keys) reused by every discovery config.
#define HA_AVAIL \
  "\"avty_t\":\"" MQTT_TOPIC_AVAILABILITY "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\""
#define HA_DEVICE \
  "\"dev\":{\"ids\":[\"" HA_DEVICE_ID "\"],\"name\":\"" HA_DEVICE_NAME "\",\"mf\":\"DVS\",\"mdl\":\"G3 BC220\"}"



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
