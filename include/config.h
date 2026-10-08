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
static constexpr int RS485_DE_RE_PIN = 21;

// --- Serial line settings ----------------------------------------------------
static constexpr uint32_t USB_BAUD = 115200; // debug/monitor console
static constexpr uint32_t BUS_BAUD = 4800;   // DVS bus (confirmed 4800 8N1)

// --- Frame boundary detection ------------------------------------------------
// At 4800 8N1 one byte takes ~2.08 ms. An idle gap longer than this threshold
// is treated as the end of a frame. Runtime-adjustable via the USB console.
static constexpr uint32_t DEFAULT_GAP_THRESHOLD_US = 3500;

// Largest frame we will buffer before force-flushing.
static constexpr size_t MAX_FRAME_LEN = 64;

// Per-frame console tracing at boot: false = quiet (production), true = decode
// and print every frame (a reverse-engineering aid). Lifecycle/error logs
// (wifi/mqtt/err/auto) are unaffected. Toggle at runtime with the `decode`
// command. MQTT still publishes all sniffed value regardless of this.
static constexpr bool DVS_FRAME_TRACE_DEFAULT = false;

// --- Known DVS registers -----------------------------------------------------
enum DvsRegister : uint8_t {
    REG_DEADMAN     = 0x03, // NOT observed in live control captures (see below)
    REG_SET_ADDRESS = 0x04, // 1 byte payload -> becomes new peripheral address
    REG_STOP        = 0x05, // NOT observed; "off" is really speed 0 via 0x15
    REG_TEMP        = 0x10, // airflow temp: payload byte0 = raw count (cal TODO)
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

// Airflow (supply-air) temperature calibration: temp_C = raw * SCALE - OFFSET.
// Least-squares fit to 8 live points over 11-20 C (residuals < 0.75 C).
static constexpr float DVS_AIRFLOW_CAL_SCALE = 0.06807f;
static constexpr float DVS_AIRFLOW_CAL_OFFSET = 1.039f;

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

// --- Transmit-mode poll cycle ---
// While transmitting the firmware acts as the DVS controller. The fan tracks a
// SINGLE register conversation at a time: a steady 0x15 keepalive alone makes it
// stop answering the 0x10 temperature query. The wall controller avoids this by
// running a tight, response-driven 1:1 cycle - send a request, wait for the
// reply, then send the next register's request - keeping BOTH the speed and
// temperature conversations alive. We do the same: alternate a 0x15 write
// (keepalive; "off" is just target 0, there is no 0x03 deadman) and a 0x10 read
// one-at-a-time, resending once if no reply arrives (the controller's "doubled"
// frame) and advancing after the reply or a timeout so a silent register never
// stalls the keepalive.
//
// Resend the outstanding request if no reply by now (the doubled frame, ~72 ms).
static constexpr uint32_t DVS_POLL_RESEND_MS = 72;
// Give up waiting for a reply and advance to the next register after this long,
// so an unanswered read cannot stall the 0x15 keepalive. The worst-case 0x15 gap
// (~= speed_txn + temp_txn, each bounded by this timeout + DVS_POLL_GAP_MS) must
// stay under the fan's keepalive tolerance (~182 ms max gap seen from the
// controller; measured stable well past that).
static constexpr uint32_t DVS_POLL_REPLY_TIMEOUT_MS = 130;
// Quiet gap left on the bus between one transaction's reply and the next request
// (matches the controller's ~35-54 ms inter-pair spacing).
static constexpr uint32_t DVS_POLL_GAP_MS = 40;

// Transmit mode at boot: false = listen-only (never drive the bus),
// true = listen + send (control commands are allowed to transmit). Switchable at
// runtime via the `tx` console command or the dvs/tx/set MQTT topic.
static constexpr bool DVS_TX_ENABLED_DEFAULT = false;

// Debounce for mirroring an observed fan-speed target of 0 into the reported
// (MQTT/HA) power state. The wall controller interleaves a spurious target=0
// with the real target (~every 250 ms) while its screen sleeps, which would
// otherwise flap the HA power state on/off. A non-zero target is mirrored
// immediately; a target of 0 only reports "off" once no non-zero target has been
// seen for this long (a genuine wall-off still turns off after the window).
static constexpr uint32_t DVS_TARGET_OFF_DEBOUNCE_MS = 5000;

// ---- Timed boost -----------------------------------------------------------
// User-triggered override that drives the fan to full speed (DVS_MAX_SPEED) for
// a fixed number of minutes, then expires on its own (0 cancels). Overlaid on
// the manual/auto target; only reaches the bus while the fan is powered and
// transmit is enabled. Requests longer than this are rejected.
static constexpr uint16_t DVS_BOOST_MAX_MINUTES = 240;
// Duration used when boost is switched on via the simple ON/OFF switch (the HA
// switch / HomeKit / Alexa path, which has no duration field). The number entity
// and `boost <min>` command still set an explicit duration.
static constexpr uint16_t DVS_BOOST_DEFAULT_MINUTES = 60;

// --- Automatic ventilation profiles (summer / winter) ---
// When the control mode is `auto` (and transmit is enabled) the firmware acts on
// the airflow (supply) temperature sniffed from the bus and an inside temperature
// fed in over MQTT (dvs/inside_temp/set), computing a proportional fan speed via
// the pure logic in profile.cpp. The unit is positive-pressure roof air with no
// heat-recovery core, so the air entering the house is the RAW airflow temperature
// (there is no tempering blend). Defaults below are tuned for Wellington NZ,
// where the primary job is moisture/condensation control; all are runtime
// tunable over MQTT/console. See profile.h for the decision logic.
//
// Hysteresis: a stopped fan needs deadbandOn of favourable differential to
// start, a running fan keeps going until it drops below deadbandOff.
static constexpr float DVS_PROFILE_DEADBAND_ON_C = 1.5f;
static constexpr float DVS_PROFILE_DEADBAND_OFF_C = 0.5f;
// Favourable differential at which the fan reaches maxSpeed.
static constexpr float DVS_PROFILE_SPAN_C = 5.0f;
// Winter: never supply raw airflow colder than this (matches the observed unit
// still ventilating for moisture at ~13 C; the floor is checked on RAW airflow
// temp, since there is no exchanger core to warm the incoming air). This is only
// the BOOT default: the cutoff is runtime-settable over MQTT (dvs/cutoff_temp)
// and falls back to this value when unset or the broker is unavailable.
static constexpr float DVS_PROFILE_CUTOFF_C = 12.0f;
// Bounds for the runtime-settable cold cutoff; out-of-range payloads are rejected
// and the previous value kept.
static constexpr float DVS_CUTOFF_MIN_C = 0.0f;
static constexpr float DVS_CUTOFF_MAX_C = 20.0f;
// Summer: never bring in airflow hotter than this (block hot midday air).
static constexpr float DVS_PROFILE_MAX_SUPPLY_C = 24.0f;
// Supply-cutoff hysteresis: a RUNNING fan is allowed this much slack below
// the cutoff (and above maxSupply) before it stops. A running fan cools its own
// airflow-temp reading, dipping it under the cutoff; without slack the fan cuts
// out, the reading recovers, and it restarts - the classic on/off flap near the
// cutoff (e.g. ~12 C). Stopping still requires the full cutoff to be crossed.
static constexpr float DVS_PROFILE_SUPPLY_HYST_C = 2.0f;
// Winter heat-recovery ramp (airflow-minus-inside differential, deg C). The boost
// begins at recoverStart (0 == the pivot: the airflow must be genuinely WARMER
// than inside to add heat) and reaches maxSpeed once it is recoverFull warmer.
// recoverFull is kept generous (>= 8 C) so full speed needs genuinely warm
// airflow, not a marginal 1-2 C difference.
static constexpr float DVS_PROFILE_RECOVER_START_C = 0.0f;
static constexpr float DVS_PROFILE_RECOVER_FULL_C = 8.0f;
// Winter heat-retention throttle (airflow-minus-inside differential, deg C). The
// cold-side mirror of the heat-recovery ramp, sharing the 0 pivot: as the airflow
// cools below the house the fan throttles from the moisture baseline DOWN toward
// minSpeed - 0 reduction at retentionStart (airflow == inside) and the full
// (base - minSpeed) reduction at retentionFull (airflow this many deg colder).
// Below the absolute cold cutoff the fan still stops (heat retention on).
static constexpr float DVS_PROFILE_RETENTION_START_C = 0.0f;
static constexpr float DVS_PROFILE_RETENTION_FULL_C = -8.0f;
// Auto-mode minimum speed: the summer minimum-ventilation floor (the fan never
// fully stops in summer) and the winter cold-cutoff step-down speed. Set to the
// measured ~27 hardware floor (see the README fan-speed map) rather than an
// unrealistically low trickle.
static constexpr uint8_t DVS_PROFILE_MIN_SPEED = 27;
// Winter always-on moisture-ventilation baseline - the speed the fan holds purely
// for condensation control before heat recovery boosts it. Decoupled from (and
// above) the minimum: winter wants a touch more exchange for moisture, and this
// is what the fan holds when heat recovery is off.
static constexpr uint8_t DVS_PROFILE_MOISTURE_SPEED = 35;
// Auto-mode ceiling, heldd below the 84 hardware max (DVS_MAX_SPEED) to leave
// headroom above the profile output.
static constexpr uint8_t DVS_PROFILE_MAX_SPEED = 74;
// Unified target (comfort) inside temperature - the single setpoint each profile
// aims for: winter warms up to it (heat recovery) then holds the moisture
// baseline, summer cools down to it then stops. Only the BOOT default:
// runtime-settable over MQTT (dvs/target_temp).
static constexpr float DVS_PROFILE_COMFORT_TARGET_C = 19.0f;
// Bounds for the runtime-settable target; out-of-range payloads are rejected.
static constexpr float DVS_TARGET_MIN_C = 10.0f;
static constexpr float DVS_TARGET_MAX_C = 30.0f;

// Winter heat-recovery default: ON. Moisture ventilation always runs above the
// cold cutoff; heat recovery additionally boosts the speed to harvest warm
// airflow. Turn it off for a fixed low moisture-only baseline. Runtime-toggleable.
static constexpr bool DVS_HEAT_RECOVERY_DEFAULT = true;

// Winter "heat retention" default: ON. When ON, raw airflow below the cutoff is
// too cold to supply, so the fan steps down through minSpeed and stops to keep the
// house warm. When OFF, the fan keeps holding the moisture baseline below the
// cutoff (prioritise condensation control over heat retention). Winter only.
static constexpr bool DVS_HEAT_RETENTION_DEFAULT = true;

// How often the auto-control tick recomputes and applies a target speed.
static constexpr uint32_t DVS_CONTROL_TICK_MS = 10000;
// Auto control fails safe to OFF if the airflow temperature is older than this (a
// stale/absent reading must not silently hold the fan on).
static constexpr uint32_t DVS_TEMP_STALE_MS = 300000; // 5 min

// --- Network / MQTT timing ----------------------------------------------------
static constexpr uint32_t WIFI_RECONNECT_MS = 10000; // retry Wi-Fi join cadence
static constexpr uint32_t MQTT_RECONNECT_MS = 5000;  // retry broker connect cadence
// How often the current fan state is (re)published to MQTT, in addition to an
// immediate publish whenever a command is applied. Keeps retained topics fresh
// for clients that (re)connect between commands.
static constexpr uint32_t STATE_PUBLISH_MS  = 15000; // periodic state republish
// On the first broker connect after boot, how long to wait for the retained
// */state topics to arrive so mode/profile/tx/heat_recovery can be restored
// before the firmware resumes publishing its own state.
static constexpr uint32_t DVS_MQTT_RESTORE_WINDOW_MS = 1500;

// --- MQTT topics --------------------------------------------------------------
// Broker/credentials live in secrets.h; the topic tree is fixed here.
#define MQTT_TOPIC_BASE          "dvs"
#define MQTT_TOPIC_AVAILABILITY  MQTT_TOPIC_BASE "/status"       // online/offline (LWT)
#define MQTT_TOPIC_POWER_SET     MQTT_TOPIC_BASE "/power/set"    // in:  ON / OFF
#define MQTT_TOPIC_POWER_STATE   MQTT_TOPIC_BASE "/power/state"  // out: ON / OFF
#define MQTT_TOPIC_SPEED_SET     MQTT_TOPIC_BASE "/speed/set"    // in:  0..84
#define MQTT_TOPIC_SPEED_STATE   MQTT_TOPIC_BASE "/speed/state"  // out: commanded 0..84
#define MQTT_TOPIC_ACTUAL_SPEED  MQTT_TOPIC_BASE "/speed/actual" // out: sniffed actual
#define MQTT_TOPIC_TEMPERATURE   MQTT_TOPIC_BASE "/temperature"  // out: sniffed airflow temp
#define MQTT_TOPIC_TX_SET        MQTT_TOPIC_BASE "/tx/set"       // in:  ON / OFF (send vs listen)
#define MQTT_TOPIC_TX_STATE      MQTT_TOPIC_BASE "/tx/state"     // out: ON / OFF
// Automatic ventilation control. Inside temperature is fed in (e.g. from a Home
// Assistant sensor); the profile selects manual / winter / summer behaviour and
// heat recovery toggles the winter warm-airflow speed boost.
#define MQTT_TOPIC_INSIDE_SET     MQTT_TOPIC_BASE "/inside_temp/set"  // in:  deg C
#define MQTT_TOPIC_INSIDE_STATE   MQTT_TOPIC_BASE "/inside_temp/state"  // out: deg C (echo)
#define MQTT_TOPIC_PROFILE_SET    MQTT_TOPIC_BASE "/profile/set"   // in:  manual / winter / summer
#define MQTT_TOPIC_PROFILE_STATE  MQTT_TOPIC_BASE "/profile/state" // out: manual / winter / summer
#define MQTT_TOPIC_HEATREC_SET    MQTT_TOPIC_BASE "/heat_recovery/set"   // in:  ON / OFF
#define MQTT_TOPIC_HEATREC_STATE  MQTT_TOPIC_BASE "/heat_recovery/state" // out: ON / OFF
#define MQTT_TOPIC_RETENTION_SET   MQTT_TOPIC_BASE "/heat_retention/set"   // in:  ON / OFF
#define MQTT_TOPIC_RETENTION_STATE MQTT_TOPIC_BASE "/heat_retention/state" // out: ON / OFF
#define MQTT_TOPIC_CUTOFF_SET     MQTT_TOPIC_BASE "/cutoff_temp/set"     // in:  deg C
#define MQTT_TOPIC_CUTOFF_STATE   MQTT_TOPIC_BASE "/cutoff_temp/state"   // out: deg C
#define MQTT_TOPIC_TARGET_SET     MQTT_TOPIC_BASE "/target_temp/set"     // in:  deg C
#define MQTT_TOPIC_TARGET_STATE   MQTT_TOPIC_BASE "/target_temp/state"   // out: deg C
// Timed full-speed boost: set minutes (0 = cancel), state echoes minutes left.
#define MQTT_TOPIC_BOOST_SET        MQTT_TOPIC_BASE "/boost/set"        // in:  minutes (0 = cancel)
#define MQTT_TOPIC_BOOST_STATE      MQTT_TOPIC_BASE "/boost/state"      // out: minutes remaining
// Simple ON/OFF boost switch (for HA switch -> HomeKit/Alexa). ON runs a boost
// of DVS_BOOST_DEFAULT_MINUTES; OFF cancels. State is ON while a boost is active.
#define MQTT_TOPIC_BOOST_PWR_SET    MQTT_TOPIC_BASE "/boost/power/set"   // in:  ON / OFF
#define MQTT_TOPIC_BOOST_PWR_STATE  MQTT_TOPIC_BASE "/boost/power/state" // out: ON / OFF
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
