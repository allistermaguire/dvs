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
#include "frame.h"

// --- Runtime-adjustable state ------------------------------------------------
static uint32_t g_busBaud = DEFAULT_BUS_BAUD;
static uint32_t g_gapThresholdUs = DEFAULT_GAP_THRESHOLD_US;
static bool g_decode = true;         // decode registers vs raw hex only
static Crc16Params g_crcParams;      // defaults to CRC-16/CCITT-FALSE
static bool g_crcBigEndian = true;   // transmitted CRC byte order (MSB first)
// Level driven on the transceiver EN / DE-RE pin. On a textbook MAX3485 LOW is
// receive-only, but some breakouts invert EN, so this is runtime-adjustable to
// find the passive (non-driving) state for the actual board in use.
static bool g_enableHigh = false;    // false => pin LOW, true => pin HIGH

// --- Frame assembly ----------------------------------------------------------
static DvsFrame g_frame;
static uint32_t g_lastByteUs = 0;

static void applyEnable() {
    digitalWrite(RS485_DE_RE_PIN, g_enableHigh ? HIGH : LOW);
}

static void configureBus() {
    Serial2.begin(g_busBaud, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
}

static void printConfig() {
    Serial.printf(
        "[cfg] busBaud=%lu gapUs=%lu decode=%s crcInit=0x%04X crcReflected=%s "
        "crcBigEndian=%s en=%s pins(RX=%d TX=%d DE/RE=%d)\n",
        (unsigned long)g_busBaud, (unsigned long)g_gapThresholdUs,
        g_decode ? "on" : "off", g_crcParams.init,
        g_crcParams.reflected ? "yes" : "no", g_crcBigEndian ? "yes" : "no",
        g_enableHigh ? "high" : "low", 
        RS485_RX_PIN, RS485_TX_PIN, RS485_DE_RE_PIN);
}

static void printHelp() {
    Serial.println(F("Commands:"));
    Serial.println(F("  baud <n>      set bus baud rate (e.g. baud 4800)"));
    Serial.println(F("  gap <us>      set inter-frame gap threshold in microseconds"));
    Serial.println(F("  decode        toggle register decoding on/off"));
    Serial.println(F("  crcinit <h>   set CRC init value (0x0000 or 0xFFFF)"));
    Serial.println(F("  crcrefl       toggle CRC input/output reflection"));
    Serial.println(F("  crcend        toggle CRC byte order (little/big endian)"));
    Serial.println(F("  en [hi|lo]    set/toggle transceiver EN (DE/RE) pin level"));
    Serial.println(F("  status        print current configuration"));
    Serial.println(F("  help          show this help"));
}

static void flushFrame() {
    if (g_frame.len == 0) {
        return;
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
    Serial.println(F("=== DVS G3 RS485 passive sniffer ==="));
    Serial.println(F("Listen-only. Type 'help' for commands."));
    printConfig();
}

void loop() {
    pollUsbCommands();

    const uint32_t now = micros();

    // Close the current frame if the bus has been idle past the threshold.
    if (g_frame.len > 0 && (now - g_lastByteUs) > g_gapThresholdUs) {
        flushFrame();
    }

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
}

#endif // !UNIT_TEST
