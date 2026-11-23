// DVS G3 RS485 passive sniffer for D1 Mini ESP32 + MAX3485.
//
// Listens (RX only) on the DVS bus in parallel with the existing controller,
// groups bytes into frames by inter-frame idle gaps, verifies the CRC and prints
// a decoded summary of every frame to the USB serial console. A small line-based
// command interface allows changing the frame gap threshold and toggling display
// and transmit options at runtime. Fan control and the auto-ventilation policy
// live in control.cpp; this file owns the RS485 transceiver and the frame
// sniffer/decoder, and provides the bus seam (bus.h) the control layer calls.

#if !defined(UNIT_TEST)

#include <Arduino.h>

#include "bus.h"
#include "config.h"
#include "control.h"
#include "frame.h"
#include "net.h"
#include "parse.h"

// --- Runtime-adjustable state ------------------------------------------------
static uint32_t g_gapThresholdUs = DEFAULT_GAP_THRESHOLD_US;
static bool g_frameTrace = DVS_FRAME_TRACE_DEFAULT; // stream frames to the console
static bool g_decode = true;         // when tracing, decode regs vs raw hex
static bool g_badOnly = false;       // print only CRC=BAD frames (bus-error tracking)
static uint32_t g_badFrameCount = 0; // running total of CRC=BAD frames seen

// Confirmed DVS wire parameters (see crc16.h). Fixed now that the protocol is
// reverse-engineered; no longer console-adjustable.
static const Crc16Params g_crcParams;        // CRC-16/AUG-CCITT (init 0x1D0F)
static constexpr bool g_crcBigEndian = true; // CRC transmitted MSB first

// --- Transmit-mode poll cycle state ---
static uint8_t g_pollReg = REG_FAN_SPEED; // register of the in-flight transaction
static bool g_pollActive = false;         // a request is currently outstanding
static bool g_pollResent = false;         // the doubled resend has been sent
static uint32_t g_pollSentMs = 0;         // when the request was first sent
static uint32_t g_pollGapUntilMs = 0;     // don't start the next request before this

// Frame assembly ---
static DvsFrame g_frame;
static uint32_t g_lastByteUs = 0;
static uint8_t g_rxReg = 0;             // register of the last CRC-OK frame received
static uint32_t g_rxRegMs = 0;          // millis() when it arrived (poll reply detect)

static void applyEnable() {
    // Passive listen level: DE+RE LOW = driver ddisabled,receiver enabled.
    digitalWrite(RS485_DE_RE_PIN, LOW);
}

static void configureBus() {
    Serial2.begin(BUS_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
}

// --- Transmit path -----------------------------------------------------------
// Drive the MAX3485 for the duration of a write, then return to the passive
// (listen) level. This is the only place that ever enables the driver.
static void busTransmit(const uint8_t *bytes, size_t len) {
    digitalWrite(RS485_DE_RE_PIN, HIGH); // enable driver
    delayMicroseconds(20);               // let DE settle before shifting bytes
    Serial2.write(bytes, len);
    Serial2.flush();                     // block until the last byte is on the wire
    delayMicroseconds(20);
    applyEnable();                       // back to the configured listen level
}

// Build and transmit a command frame addressed to the fan (bus.h seam).
void dvsBusSend(uint8_t reg, const uint8_t *payload, size_t payloadLen) {
    // Single hard chokepoint: never drive the bus while in listen-only mode,
    // regardless of caller. This perserves the passive-sniffer guarantee even
    // if some other path (e.g. the keepalive) sets the fan "on".
    if (!dvsControlGetTxEnabled()) {
        return;
    }
    uint8_t buf[MAX_FRAME_LEN];
    const size_t n = dvs_frame_build(DVS_FAN_ADDR, reg, payload, payloadLen, buf,
                                    sizeof(buf), g_crcParams, g_crcBigEndian);
    if (n == 0) {
        Serial.println(F("[err] tx frame too large"));
        return;
    }
    busTransmit(buf, n);

    // Echo the transmitted frame through the same decoder used for received
    // frames. We never hear our own transmit back (the driver is enabled and the
    // receiver muted while sending), so without this the console would only ever
    // show the fan's responses and never what we actually put on the wire.
    // Skipped when frame tracing is off (production quiet); honours `decode`.
    if (!g_frameTrace) {
        return;
    }
    DvsFrame txFrame;
    txFrame.startMicros = micros();
    txFrame.len = (n <= MAX_FRAME_LEN) ? n : MAX_FRAME_LEN;
    memcpy(txFrame.bytes, buf, txFrame.len);
    if (g_decode) {
        char line[256];
        dvs_frame_format(txFrame, line, sizeof(line), g_crcParams, g_crcBigEndian);
        Serial.printf("[tx] %s\n", line);
    } else {
        Serial.printf("[tx] len=%u", (unsigned)txFrame.len);
        for (size_t i = 0; i < txFrame.len; ++i) {
            Serial.printf(" %02X", txFrame.bytes[i]);
        }
        Serial.println();
    }
}

// Fan control + auto ventilation now lines in control.cpp (the policy layer).
// This file provides the bus seam (dvsBusSend above, dvsBusResetPollCycle below)
// that they call into.

static void printConfig() {
    Serial.printf(
        "[cfg] gapUs=%lu trace=%s decode=%s tx=%s pins(RX=%d TX=%d DE/RE=%d)\n",
        (unsigned long)g_gapThresholdUs, g_frameTrace ? "on" : "off",
        g_decode ? "on" : "off", dvsControlGetTxEnabled() ? "send" : "listen",
        RS485_RX_PIN, RS485_TX_PIN, RS485_DE_RE_PIN);
    Serial.printf("[cfg] badonly=%s badCount=%lu\n", g_badOnly ? "on" : "off",
                  (unsigned long)g_badFrameCount);
    float inside;
    const bool haveInside = dvsControlGetInsideTemp(&inside);
    Serial.printf("[cfg] power=%s profile=%s heatrec=%s retention=%s inside=%s\n",
                  dvsControlGetStandby() ? "off" : "on",
                  dvsControlProfileName(dvsControlGetProfile()),
                  dvsControlGetHeatRecovery() ? "on" : "off",
                  dvsControlGetHeatRetention() ? "on" : "off",
                  haveInside ? String(inside, 1).c_str() : "n/a");
    Serial.printf("[cfg] cutoff=%.1fC target=%.1fC\n",
                (double)dvsControlGetCutoff(),
                (double)dvsControlGetComfortTarget());
    Serial.printf("[cfg] boost=%u min remaining\n",
                  (unsigned)dvsControlBoostRemainingMin());
}

static void printHelp() {
    Serial.println(F("Commands:"));
    Serial.println(F("  gap <us>      set inter-frame gap threshold in microseconds"));
    Serial.println(F("  trace [on|off] stream decoded frames to the console (off = quiet)"));
    Serial.println(F("  decode        toggle register decoding on/off (when tracing)"));
    Serial.println(F("  badonly [on|off|reset] show only CRC=BAD frames / reset counter"));
    Serial.println(F("  tx [on|off]   enable/disable transmit (listen+send vs listen-only)"));
    Serial.println(F("  on            power the fan on (last/default speed)"));
    Serial.println(F("  off           power the fan off (stop)"));
    Serial.println(F("  speed <n>     set target fan speed 0..84"));
    Serial.println(F("  profile <manual|winter|summer> manual control vs auto season"));
    Serial.println(F("  heatrecovery [on|off]   winter warm-airflow speed boost"));
    Serial.println(F("  heatretention [on|off]  winter: stop fan below the cold cutoff"));
    Serial.println(F("  cutoff <degC>           winter cold-cutoff temperature"));
    Serial.println(F("  target <degC>           set (target) temperature"));
    Serial.println(F("  inside <degC>           feed the inside temperature"));
    Serial.println(F("  boost <min|off>         run at full speed for N minutes"));
    Serial.println(F("  status        print current configuration"));
    Serial.println(F("  help          show this help"));
}

// Publish sniffed sensor readings from a valid frame to MQTT. Runs regardless of
// the console `decode` toggle so the broker always sees fresh values.
//
// Temperature and actual speed are polled several times a second but rarely
// change, so they are only published when the value actually changes (retained,
// so the broker still serves the last value to new subscribers). The status
// block is left un-deduplicated so its trigger cadence can be observed.
static void publishObserved(const DvsFrame &f) {
    static int lastTempRaw = -1;       // last published raw temp count
    static int lastActualSpeed = -1;   // last published actual speed
    static int lastMirroredTarget = -1;     // last target mirrored to HA state
    static uint32_t lastNonZeroTargetMs = 0; // time of last non-zero target seen

    if (f.len < 4 || !dvs_frame_crc_ok(f, g_crcParams, g_crcBigEndian)) {
        return;
    }
    const uint8_t reg = dvs_frame_register(f);
    const uint8_t *payload = dvs_frame_payload(f);
    const size_t payloadLen = dvs_frame_payload_len(f);
    if (reg == REG_TEMP && payloadLen >= 1) {
        // Raw roof-space count is little-endian across the payload: above ~16 C
        // it exceeds one byte (byte 1 becomes non-zero), so read the full value
        // rather than byte 0 alone (byte-0-only wraps, e.g. 18 C -> 0.6 C).
        uint32_t raw = 0;
        for (size_t i = 0; i < payloadLen; ++i) {
            raw |= (uint32_t)payload[i] << (8 * i);
        }
        const float tc = dvs_temp_c_from_raw((int)raw);
        // Cache every reading (not just published ones) so the auto-control tick
        // has a fresh airflow (supply) temperature and can judge staleness.
        dvsControlReportAirflowTemp(tc);
        if ((int)raw != lastTempRaw) {
            // Only advance the dedupe cache once the value is actually sent,
            // so values seen before MQTT connects are not lost.
            if (netPublishTemperature(tc)) {
                lastTempRaw = (int)raw;
            }
        }
    } else if (reg == REG_FAN_SPEED && payloadLen == 1) {
        // 1-byte payload = commanded target on the bus. In listen-only mode we
        // are not the controller, so mirror this into the reported fan state so
        // the HA fan entity follows the real fan. (In listen+send mode our own
        // command is authoritative and we do not receive our own transmit.)
        //
        // The wall controller interleaves a spurious target=0 with the real
        // target (~every 250 ms) while its screen sleeps, so a raw mirror would
        // flap the HA power state on/off. Debounce: a non-zero target takes
        // effect at once; a target of 0 only turns the reported fan off after no
        // non-zero target for DVS_TARGET_OFF_DEBOUNCE_MS. The sniffed actual-speed
        // sensor still reports the true (ramping) speed independently.
        if (!dvsControlGetTxEnabled()) {
            const uint8_t target = payload[0];
            const uint32_t nowMs = millis();
            bool changed = false;
            if (target > 0) {
                lastNonZeroTargetMs = nowMs;
                if (target != lastMirroredTarget || !dvsControlGetPower()) {
                    dvsControlReportObserved(target);
                    lastMirroredTarget = target;
                    changed = true;
                }
            } else if (dvsControlGetPower() &&
                    (nowMs - lastNonZeroTargetMs) >= DVS_TARGET_OFF_DEBOUNCE_MS) {
                dvsControlReportObserved(0);
                lastMirroredTarget = 0;
                changed = true;
            }
            if (changed) {
                netPublishState();
            }
        }
    } else if (reg == REG_FAN_SPEED && payloadLen >= 2) {
        // 2-byte payload = actual (ramping) speed; low byte holds 0..84.
        if ((int)payload[0] != lastActualSpeed) {
            // Only advance the dedupe cache once the value is actually sent.
            if (netPublishActualSpeed(payload[0])) {
                lastActualSpeed = payload[0];
            }
        }
    } else if (dvs_is_status_reg(reg) && payloadLen > 0) {
        // Periodic status-block responses (identity/flags/telemetry). Publish
        // the raw payload so the block can be watched over MQTT. payloadLen > 0
        // skips the controller's empty query frames, keeping only the fan's data.
        netPublishStatusReg(reg, payload, payloadLen);
    }
}

static void flushFrame() {
    if (g_frame.len == 0) {
        return;
    }

    // Drop runt captures shorter than the minimum DVS frame (addr+reg+crc16 =
    // 4 bytes). These can't be valid and are almost always an RS485 turnaround
    // glitch - a stray byte the UART latches when we release the driver after
    // transmitting. Discarding them keeps them out of the CRC-BAD count/console
    // as false bus errors (genuine corruption still garbles full-length frames).
    if (!dvs_frame_has_body(g_frame)) {
        g_frame.len = 0;
        return;
    }

    publishObserved(g_frame);

    // Bus-error tracking: count every CRC=BAD frame and, in bad-only mode,
    // suppress the successful frames so the console shows just the errors (and
    // their running count) without the noise. publishObserved() above still runs
    // regardless, so MQTT is unaffected by this display filter.
    const bool crcOk = dvs_frame_crc_ok(g_frame, g_crcParams, g_crcBigEndian);
    if (!crcOk) {
        ++g_badFrameCount;
    }
    if (crcOk && g_frame.len >= 2) {
        // Note the fan's reply for the response-driven poll cycle: in transmit
        // mode every received frame is a reply to one of our requests.
        g_rxReg = dvs_frame_register(g_frame);
        g_rxRegMs = millis();
    }
    // Frame tracing off (production quiet): keep the bookkeeping above but emit
    // no per-frame lines. Events (wifi/mqtt/err/auto) and MQTT are unaffected.
    // badCount is still visible via `status`.
    if (!g_frameTrace) {
        g_frame.len = 0;
        return;
    }
    if (g_badOnly && crcOk) {
        g_frame.len = 0; // drop good frames from the console in bad-only mode
        return;
    }
    if (g_badOnly) {
        Serial.printf("[bad #%lu] ", (unsigned long)g_badFrameCount);
    }

    if (g_decode) {
        char line[256];
        dvs_frame_format(g_frame, line, sizeof(line), g_crcParams, g_crcBigEndian);
        Serial.println(line);
    } else {
        Serial.printf("%luus len=%u", (unsigned long)g_frame.startMicros,
                      (unsigned)g_frame.len);
        for (size_t i = 0; i < g_frame.len; ++i) {
            Serial.printf(" %02X", g_frame.bytes[i]);
        }
        Serial.println();
    }
    g_frame.len = 0;
}

static void handleCommand(const String &raw) {
    String cmd = raw;
    cmd.trim();
    if (cmd.isEmpty()) {
        return;
    }

    if (cmd.startsWith("gap")) {
        long v;
        if (dvsParseIntRange(cmd.substring(3).c_str(), 1, INT32_MAX, &v)) {
            g_gapThresholdUs = (uint32_t)v;
            Serial.printf("[ok] gapUs=%lu\n", (unsigned long)g_gapThresholdUs);
        } else {
            Serial.println(F("[err] usage: gap <us>"));
        }
    } else if (cmd == "decode") {
        g_decode = !g_decode;
        Serial.printf("[ok] decode=%s\n", g_decode ? "on" : "off");
    } else if (cmd.startsWith("trace")) {
        String arg = cmd.substring(5);
        arg.trim();
        switch (dvsParseTriState(arg.c_str())) {
            case DVS_TRI_ON: g_frameTrace = true; break;
            case DVS_TRI_OFF: g_frameTrace = false; break;
            case DVS_TRI_TOGGLE: g_frameTrace = !g_frameTrace; break;
            default:
                Serial.println(F("[err] usage: trace [on|off]"));
                return;
        }
        Serial.printf("[ok] trace=%s\n", g_frameTrace ? "on" : "off");
    } else if (cmd.startsWith("badonly")) {
        String arg = cmd.substring(7);
        arg.trim();
        if (arg == "reset") {
            g_badFrameCount = 0;
        } else {
            switch (dvsParseTriState(arg.c_str())) {
                case DVS_TRI_ON: g_badOnly = true; break;
                case DVS_TRI_OFF: g_badOnly = false; break;
                case DVS_TRI_TOGGLE: g_badOnly = !g_badOnly; break;
                default:
                    Serial.println(F("[err] usage: badonly [on|off|reset]"));
                    return;
            }
        }
        Serial.printf("[ok] badonly=%s badCount=%lu\n", g_badOnly ? "on" : "off",
                    (unsigned long)g_badFrameCount);
    } else if (cmd.startsWith("tx")) {
        String arg = cmd.substring(2);
        arg.trim();
        switch (dvsParseTriState(arg.c_str())) {
            case DVS_TRI_ON: dvsControlSetTxEnabled(true); break;
            case DVS_TRI_OFF: dvsControlSetTxEnabled(false); break;
            case DVS_TRI_TOGGLE:
                dvsControlSetTxEnabled(!dvsControlGetTxEnabled());
                break;
            default: Serial.println(F("[err] usage: tx [on|off]")); break;
        }
    } else if (cmd == "on") {
        dvsControlSetPower(true);
        Serial.printf("[ok] power on speed=%u\n", (unsigned)dvsControlGetSpeed());
    } else if (cmd == "off") {
        dvsControlSetPower(false);
        Serial.println(F("[ok] power off"));
    } else if (cmd.startsWith("speed")) {
        long v;
        if (dvsParseIntRange(cmd.substring(5).c_str(), 0, DVS_MAX_SPEED, &v)) {
            dvsControlSetSpeed((uint8_t)v);
            Serial.printf("[ok] speed=%ld\n", v);
        } else {
            Serial.printf("[err] usage: speed <0..%u>\n", (unsigned)DVS_MAX_SPEED);
        }
    } else if (cmd.startsWith("profile")) {
        String arg = cmd.substring(7);
        arg.trim();
        DvsProfileSel sel;
        if (dvsParseProfile(arg.c_str(), &sel)) {
            dvsControlSetProfile(sel);
        } else {
            Serial.println(F("[err] usage: profile <manual|winter|summer>"));
            return;
        }
    } else if (cmd.startsWith("heatrecovery")) {
        String arg = cmd.substring(12);
        arg.trim();
        switch (dvsParseTriState(arg.c_str())) {
            case DVS_TRI_ON: dvsControlSetHeatRecovery(true); break;
            case DVS_TRI_OFF: dvsControlSetHeatRecovery(false); break;
            case DVS_TRI_TOGGLE:
                dvsControlSetHeatRecovery(!dvsControlGetHeatRecovery());
                break;
            default:
                Serial.println(F("[err] usage: heatrecovery [on|off]"));
                return;
        }
        Serial.printf("[ok] heatrec=%s\n",
                      dvsControlGetHeatRecovery() ? "on" : "off");
    } else if (cmd.startsWith("heatretention ")) {
        String arg = cmd.substring(13);
        arg.trim();
        switch (dvsParseTriState(arg.c_str())) {
        case DVS_TRI_ON: dvsControlSetHeatRetention(true); break;
        case DVS_TRI_OFF: dvsControlSetHeatRetention(false); break;
        case DVS_TRI_TOGGLE:
            dvsControlSetHeatRetention(!dvsControlGetHeatRetention());
            break;
        default:
            Serial.println(F("[err] usage: heatretention [on|off]"));
            return;
        }
        Serial.printf("[ok] retention=%s\n",
                      dvsControlGetHeatRetention() ? "on" : "off");
    } else if (cmd.startsWith("cutoff")) {
        String arg = cmd.substring(6);
        arg.trim();
        float t;
        if (dvsParseFloat(arg.c_str(), &t) && t >= DVS_CUTOFF_MIN_C &&
            t <= DVS_CUTOFF_MAX_C) {
            dvsControlSetCutoff(t);
            Serial.printf("[ok] cutoff=%.1fC\n", (double)dvsControlGetCutoff());
        } else {
            Serial.println(F("[err] usage: cutoff <degC>"));
            return;
        }
    } else if (cmd.startsWith("target ")) {
        String arg = cmd.substring(7);
        arg.trim();
        float t;
        if (dvsParseFloat(arg.c_str(), &t) && t >= DVS_TARGET_MIN_C &&
            t <= DVS_TARGET_MAX_C) {
            dvsControlSetComfortTarget(t);
            Serial.printf("[ok] target=%.1fC\n",
                        (double)dvsControlGetComfortTarget());
        } else {
            Serial.println(F("[err] usage: target <degC>"));
            return;
        }
    } else if (cmd.startsWith("inside")) {
        String arg = cmd.substring(6);
        arg.trim();
        float t;
        if (dvsParseFloat(arg.c_str(), &t)) {
            dvsControlSetInsideTemp(t);
            Serial.printf("[ok] inside=%.1f\n", (double)t);
        } else {
            Serial.printf("[err] usage: inside <degC>");
        }
    } else if (cmd.startsWith("boost")) {
        String arg = cmd.substring(5);
        arg.trim();
        if (arg.equalsIgnoreCase("off")) {
            dvsControlSetBoost(0);
        } else {
            long v;
            if (dvsParseIntRange(arg.c_str(), 0, DVS_BOOST_MAX_MINUTES, &v)) {
                dvsControlSetBoost((uint16_t)v);
            } else {
                Serial.printf("[err] usage: boost <0..%u|off>\n",
                              (unsigned)DVS_BOOST_MAX_MINUTES);
            }
        }
    } else if (cmd == "status") {
        printConfig();
    } else if (cmd == "help") {
        printHelp();
    } else {
        Serial.printf("[err] unknown command: %s\n", cmd.c_str());
    }
}

static void pollUsbCommands() {
    static String line;
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (line.length() > 0) {
                handleCommand(line);
                line = "";
            }
        } else if (line.length() < 64) {
            line += c;
        }
    }
}

void setup() {
    Serial.begin(USB_BAUD);
    delay(200);

    // Listen-only: DE + RE LOW => driver disabled, receiver enabled.
    pinMode(RS485_DE_RE_PIN, OUTPUT);
    applyEnable();

    configureBus();

    Serial.println();
    Serial.println(F("=== DVS G3 RS485 sniffer + MQTT control ==="));
    Serial.println(F("Type 'help' for commands."));
    printConfig();

    netSetup();
}

// Restart the transmit poll cycle cleanly (bus.h seam; called when transmit is
// re-enabled so stale in-flight state can't cause an immediate timeout).
void dvsBusResetPollCycle() {
    g_pollActive = false;
    g_pollReg = REG_FAN_SPEED;
    g_pollGapUntilMs = 0;
}

// --- Option 2: response-driven poll cycle ------------------------------------
// Send the request for the current transaction's register.
static void pollSendRequest() {
    if (g_pollReg == REG_TEMP) {
        dvsBusSend(REG_TEMP, nullptr, 0);           // airflow-temperature query
    } else {
        const uint8_t sp = dvsControlGetTxSpeed();
        dvsBusSend(REG_FAN_SPEED, &sp, 1);          // speed target / keepalive
    }
}

// Finish the current transaction, leave a quiet gap, then switch registers so
// the 0x15 write and 0x10 read alternate 1:1 (like the wall controller).
static void pollEndTransaction(uint32_t nowMs) {
    g_pollActive = false;
    g_pollGapUntilMs = nowMs + DVS_POLL_GAP_MS;
    g_pollReg = (g_pollReg == REG_FAN_SPEED) ? (uint8_t)REG_TEMP
                                             : (uint8_t)REG_FAN_SPEED;
}

// One non-blocking step of the response-driven poll cycle (only while
// transmitting). Alternates a 0x15 write and a 0x10 read, one outstanding at a
// time; resends once if no reply arrives (the controller's doubled frame) and
// advances after the reply or a timeout so a silent register never stalls the
// keepalive. Keeping the two conversations 1:1 stops the fan locking onto the
// keepalive and ignoring the temperature query (the failure of the independent
// timers). Reply detection: in transmit mode every received frame is a fan reply
// (we never hear our own transmit), so a CRC-OK frame of the requested register
// that arrived after we sent is ours.
static void pollCycleStep() {
    const uint32_t nowMs = millis();

    if ((int32_t)(nowMs - g_pollGapUntilMs) < 0) {
        return; // quiet inter-transaction gap
    }

    if (!g_pollActive) {
        g_pollActive = true;
        g_pollResent = false;
        g_pollSentMs = nowMs;
        pollSendRequest();
        return;
    }

    if (g_rxReg == g_pollReg && (int32_t)(g_rxRegMs - g_pollSentMs) >= 0) {
        pollEndTransaction(nowMs); // reply received
        return;
    }

    if (!g_pollResent && (nowMs - g_pollSentMs) >= DVS_POLL_RESEND_MS) {
        g_pollResent = true;
        pollSendRequest(); // resend once (the doubled frame)
        return;
    }

    if ((nowMs - g_pollSentMs) >= DVS_POLL_REPLY_TIMEOUT_MS) {
        pollEndTransaction(nowMs); // give up, keep the cycle moving
    }
}

void loop() {
    pollUsbCommands();
    netLoop();

    // While an OTA flash is in progress, keep handing time to ArduinoOTA (via
    // netLoop above) and suspend all bus work: no transmit, no FIFO draining.
    if (netOtaInProgress()) {
        return;
    }

    // Drain the UART FIFO first. netLoop() (TLS publishes, WiFi/MQTT reconnects)
    // and the keepalive transmit can block loop() for longer than the inter-frame
    // gap threshold. Any bytes that arrived meanwhile are buffered in the UART;
    // reading them all here in one burst keeps a single frame contiguous. If the
    // gap check below ran first it would fire on our own scheduling delay and
    // slice one valid frame into two CRC-bad fragments (a false bus error).
    while (Serial2.available() > 0) {
        uint8_t b = (uint8_t)Serial2.read();
        const uint32_t t = micros();

        if (g_frame.len == 0) {
            g_frame.startMicros = t;
        }
        if (g_frame.len < MAX_FRAME_LEN) {
            g_frame.bytes[g_frame.len++] = b;
            if (g_frame.len == MAX_FRAME_LEN) {
                flushFrame(); // safety flush on overflow
            }
        }
        g_lastByteUs = t;
    }

    const uint32_t now = micros();

    // Drive the transmit-mode poll cycle: a response-driven 1:1 alternation of
    // the 0x15 keepalive and the 0x10 temperature read (see pollCycleStep /
    // config.h). tx-only: listen-only stays silent even though observed-state
    // mirroring can report the fan as "on" (dvsBusSend guards the bus as well).
    if (dvsControlGetTxEnabled()) {
        pollCycleStep();
    }

    // Auto ventilation control: recompute a target speed from the airflow and
    // inside temperatures on its own cadence (no-op in manual mode / listen-only).
    dvsControlAutoTick();

    // Expire a lapsed full-speed boost and republish state when it ends.
    if (dvsControlBoostTick()) {
        netPublishState();
    }

    // Close the current frame once the bus has been idle past the threshold. The
    // FIFO is already drained above, so this now reflects a real on-bus idle gap
    // rather than a loop stall.
    if (g_frame.len > 0 && (now - g_lastByteUs) > g_gapThresholdUs) {
        flushFrame();
    }
}

#endif // !UNIT_TEST
