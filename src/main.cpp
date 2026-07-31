// DVS G3 RS485 passive sniffer for D1 Mini ESP32 + MAX3485.
//
// Listens (RX only) on the DVS bus in parallel with the existing controller,
// groups bytes into frames by inter-frame idle gaps, verifies the CRC and prints
// a decoded summary of every frame to the USB serial console. A small line-based
// command interface allows changing the bus baud rate, gap threshold and CRC
// variant at runtime to help work out the protocol from live captures.

#if !defined(UNIT_TEST)

#include <Arduino.h>

#include "config.h"
#include "control.h"
#include "frame.h"
#include "net.h"

// --- Runtime-adjustable state ------------------------------------------------
static uint32_t g_busBaud = DEFAULT_BUS_BAUD;
static uint32_t g_gapThresholdUs = DEFAULT_GAP_THRESHOLD_US;
static bool g_decode = true;         // decode registers vs raw hex only
static bool g_badOnly = false;       // print only CRC=BAD frames (bus-error tracking)
static uint32_t g_badFrameCount = 0; // running total of CRC=BAD frames seen
static Crc16Params g_crcParams;      // defaults to CRC-16/CCITT-FALSE
static bool g_crcBigEndian = true;   // transmitted CRC byte order (MSB first)
// Level driven on the transceiver EN / DE-RE pin. On a textbook MAX3485 LOW is
// receive-only, but some breakouts invert EN, so this is runtime-adjustable to
// find the passive (non-driving) state for the actual board in use.
static bool g_enableHigh = false;    // false => pin LOW, true => pin HIGH

// --- Fan control state (commanded, i.e. what we last told the fan) -----------
static bool g_txEnabled = DVS_TX_ENABLED_DEFAULT; // false => listen-only
static bool g_fanPower = false;         // true while we intend the fan to run
static uint8_t g_fanSpeed = 0;          // last commanded target speed (0..84)
static uint8_t g_lastNonZeroSpeed = 0;  // remembered speed for a bare "power on"

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
    digitalWrite(RS485_DE_RE_PIN, g_enableHigh ? HIGH : LOW);
}

static void configureBus() {
    Serial2.begin(g_busBaud, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
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

// Build and transmit a command frame addressed to the fan.
static void sendCommandFrame(uint8_t reg, const uint8_t *payload, size_t payloadLen) {
    // Single hard chokepoint: never drive the bus while in listen-only mode,
    // regardless of caller. This guarantees the passive-sniffer guarantee even
    // if some other path (e.g. the keepalive) sets the fan "on".
    if (!g_txEnabled) {
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
    // Honours the `decode` toggle exactly like flushFrame() does for RX frames.
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

// --- Fan control API (declared in control.h) ----------------------------------
void dvsControlSetSpeed(uint8_t speed) {
    if (!g_txEnabled) {
        Serial.println(F("[warn] listen-only: speed command ignored (tx off)"));
        return;
    }
    if (speed > DVS_MAX_SPEED) {
        speed = DVS_MAX_SPEED;
    }
    if (speed == 0) {
        dvsControlSetPower(false);
        return;
    }
    g_fanSpeed = speed;
    g_lastNonZeroSpeed = speed;
    g_fanPower = true;
    sendCommandFrame(REG_FAN_SPEED, &speed, 1);
}

void dvsControlSetPower(bool on) {
    if (!g_txEnabled) {
        Serial.println(F("[warn] listen-only: power command ignored (tx off)"));
        return;
    }
    if (on) {
        const uint8_t s = g_lastNonZeroSpeed ? g_lastNonZeroSpeed : DVS_DEFAULT_SPEED;
        dvsControlSetSpeed(s); // sets state + transmits the speed
    } else {
        // "Off" matches the wall controller: write fan-speed target 0 (there is
        // no separate stop register in captures). Send one explicit 0 now for an
        // immediate response; the poll cycle then keeps re-writing the target
        // (now 0) at the controller cadence rather than going silent - on/off is
        // expressed purely through the target value the keepalive re-sends, not
        // by starting/stopping the transmit.
        g_fanPower = false;
        g_fanSpeed = 0;
        const uint8_t zero = 0;
        sendCommandFrame(REG_FAN_SPEED, &zero, 1);
    }
}

bool dvsControlGetPower() { return g_fanPower; }
uint8_t dvsControlGetSpeed() { return g_fanSpeed; }

void dvsControlSetTxEnabled(bool enabled) {
    if (g_txEnabled == enabled) {
        return;
    }
    g_txEnabled = enabled;
    if (!enabled) {
        // Dropping to listen-only: forget any "on" intent so the keepalive loop
        // goes quiet. We do NOT transmit anything here - listen-only is silent.
        g_fanPower = false;
        g_fanSpeed = 0;
    } else {
        // Resuming transmit: restart the poll cycle cleanly on the 0x15 write so
        // stale in-flight state can't cause an immediate timeout.
        g_pollActive = false;
        g_pollReg = REG_FAN_SPEED;
        g_pollGapUntilMs = 0;
    }
    Serial.printf("[ok] tx=%s (%s)\n", g_txEnabled ? "on" : "off",
                    g_txEnabled ? "listen+send" : "listen-only");
}

bool dvsControlGetTxEnabled() { return g_txEnabled; }

void dvsControlReportObserved(uint8_t targetSpeed) {
    // Mirror a target speed seen on the bus into the reported state, without
    // transmitting or touching the keepalive. Used in listen-only mode so the
    // HA fan entity tracks the real fan (driven by the wall controller).
    g_fanSpeed = targetSpeed;
    g_fanPower = (targetSpeed > 0);
    if (targetSpeed > 0) {
        g_lastNonZeroSpeed = targetSpeed;
    }
}

static void printConfig() {
    Serial.printf(
        "[cfg] busBaud=%lu gapUs=%lu decode=%s crcInit=0x%04X crcReflected=%s "
        "crcBigEndian=%s en=%s tx=%spins(RX=%d TX=%d DE/RE=%d)\n",
        (unsigned long)g_busBaud, (unsigned long)g_gapThresholdUs,
        g_decode ? "on" : "off", g_crcParams.init,
        g_crcParams.reflected ? "yes" : "no", g_crcBigEndian ? "yes" : "no",
        g_enableHigh ? "HIGH" : "LOW", g_txEnabled ? "send" : "listen",
        RS485_RX_PIN, RS485_TX_PIN, RS485_DE_RE_PIN);
    Serial.printf("[cfg] badOnly=%s badCount=%lu\n", g_badOnly ? "on" : "off",
                (unsigned long)g_badFrameCount);
}
static void printHelp() {
    Serial.println(F("Commands:"));
    Serial.println(F("  baud <n>      set bus baud rate (e.g. baud 4800)"));
    Serial.println(F("  gap <us>      set inter-frame gap threshold in microseconds"));
    Serial.println(F("  decode        toggle register decoding on/off"));
    Serial.println(F("  badonly [on|off|reset] show only CRC=BAD frames / reset counter"));
    Serial.println(F("  crcinit <h>   set CRC init value (0x0000 or 0xFFFF)"));
    Serial.println(F("  crcrefl       toggle CRC input/output reflection"));
    Serial.println(F("  crcend        toggle CRC byte order (little/big endian)"));
    Serial.println(F("  en [hi|lo]    set/toggle transceiver EN (DE/RE) pin level"));
    Serial.println(F("  tx [on|off]   enable/disable transmit (listen+send vs listen-only)"));
    Serial.println(F("  on            power the fan on (last/default speed)"));
    Serial.println(F("  off           power the fan off (stop)"));
    Serial.println(F("  speed <n>     set target fan speed 0..84"));
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
        // Byte 0 is the raw roof-space count; higher bytes are 0 in captures.
        if ((int)raw != lastTempRaw) {
            const float tc = dvs_temp_c_from_raw((int)raw);
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

    if (cmd.startsWith("baud")) {
        long v = cmd.substring(4).toInt();
        if (v > 0) {
            g_busBaud = (uint32_t)v;
            configureBus();
            Serial.printf("[ok] busBaud=%lu\n", (unsigned long)g_busBaud);
        } else {
            Serial.println(F("[err] usage: baud <n>"));
        }
    } else if (cmd.startsWith("gap")) {
        long v = cmd.substring(3).toInt();
        if (v > 0) {
            g_gapThresholdUs = (uint32_t)v;
            Serial.printf("[ok] gapUs=%lu\n", (unsigned long)g_gapThresholdUs);
        } else {
            Serial.println(F("[err] usage: gap <us>"));
        }
    } else if (cmd == "decode") {
        g_decode = !g_decode;
        Serial.printf("[ok] decode=%s\n", g_decode ? "on" : "off");
    } else if (cmd.startsWith("badonly")) {
        String arg = cmd.substring(7);
        arg.trim();
        if (arg == "on" || arg == "1") {
            g_badOnly = true;
        } else if (arg == "off" || arg == "0") {
            g_badOnly = false;
        } else if (arg == "reset") {
            g_badFrameCount = 0;
        } else if (arg.isEmpty()) {
            g_badOnly = !g_badOnly; // bare `badonly` toggles
        } else {
            Serial.println(F("[err] usage: badonly [on|off|reset]"));
            return;
        }
        Serial.printf("[ok] badonly=%s badCount=%lu\n", g_badOnly ? "on" : "off",
                    (unsigned long)g_badFrameCount);
    } else if (cmd.startsWith("crcinit")) {
        long v = strtol(cmd.substring(7).c_str(), nullptr, 0);
        g_crcParams.init = (uint16_t)v;
        Serial.printf("[ok] crcInit=0x%04X\n", g_crcParams.init);
    } else if (cmd == "crcrefl") {
        g_crcParams.reflected = !g_crcParams.reflected;
        Serial.printf("[ok] crcReflected=%s\n", g_crcParams.reflected ? "yes" : "no");
    } else if (cmd == "crcend") {
        g_crcBigEndian = !g_crcBigEndian;
        Serial.printf("[ok] crcBigEndian=%s\n", g_crcBigEndian ? "yes" : "no");
    } else if (cmd.startsWith("tx")) {
        String arg = cmd.substring(2);
        arg.trim();
        if (arg == "on" || arg == "send" || arg == "1") {
            dvsControlSetTxEnabled(true);
        } else if (arg == "off" || arg == "listen" || arg == "0") {
            dvsControlSetTxEnabled(false);
        } else if (arg.isEmpty()) {
            dvsControlSetTxEnabled(!dvsControlGetTxEnabled()); // bare `tx` toggles
        } else {
            Serial.println(F("[err] usage: tx [on|off]"));
        }
    } else if (cmd.startsWith("en")) {
        String arg = cmd.substring(2);
        arg.trim();
        if (arg == "hi" || arg == "high" || arg == "1") {
            g_enableHigh = true;
        } else if (arg == "lo" || arg == "low" || arg == "0") {
            g_enableHigh = false;
        } else if (!arg.isEmpty()) {
            g_enableHigh = !g_enableHigh; // bare `en` toggles
        } else {
            Serial.println(F("[err] usage: en [hi|lo]"));
            return;
        }
        applyEnable();
        Serial.printf("[ok] en=%s\n", g_enableHigh ? "HIGH" : "LOW");
    } else if (cmd == "on") {
        dvsControlSetPower(true);
        Serial.printf("[ok] power on speed=%u\n", (unsigned)dvsControlGetSpeed());
    } else if (cmd == "off") {
        dvsControlSetPower(false);
        Serial.println(F("[ok] power off"));
    } else if (cmd.startsWith("speed")) {
        long v = cmd.substring(5).toInt();
        if (v >= 0 && v <= DVS_MAX_SPEED) {
            dvsControlSetSpeed((uint8_t)v);
            Serial.printf("[ok] speed=%ld\n", v);
        } else {
            Serial.printf("[err] usage: speed <0..%u>\n", (unsigned)DVS_MAX_SPEED);
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

// --- Option 2: response-driven poll cycle ------------------------------------
// Send the request for the current transaction's register.
static void pollSendRequest() {
    if (g_pollReg == REG_TEMP) {
        sendCommandFrame(REG_TEMP, nullptr, 0);          // roof-temperature query
    } else {
        sendCommandFrame(REG_FAN_SPEED, &g_fanSpeed, 1); // speed target / keepalive
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
    // mirroring can set g_fanPower (sendCommandFrame guards it as well).
    if (g_txEnabled) {
        pollCycleStep();
    }

    // Close the current frame once the bus has been idle past the threshold. The
    // FIFO is already drained above, so this now reflects a real on-bus idle gap
    // rather than a loop stall.
    if (g_frame.len > 0 && (now - g_lastByteUs) > g_gapThresholdUs) {
        flushFrame();
    }
}

#endif // !UNIT_TEST
