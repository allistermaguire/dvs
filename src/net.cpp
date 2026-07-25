// Wi-Fi + MQTT bridge for the DVS sniffer/controller.
//
// Excluded from the native (host) unit-test build: it depends on the ESP32
// Arduino core and PubSubClient, and there is no bus to drive off-target.

#if !defined(UNIT_TEST)

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#include <string.h>

#include "config.h"
#include "control.h"
#include "net.h"
#include "secrets.h"

static WiFiClientSecure s_wifiClient;
static PubSubClient s_mqtt(s_wifiClient);

static uint32_t s_lastWifiAttempt = 0;
static uint32_t s_lastMqttAttempt = 0;
static uint32_t s_lastStatePublish = 0;

void netPublishState() {
    if (!s_mqtt.connected()) {
        return;
    }
    s_mqtt.publish(MQTT_TOPIC_POWER_STATE, dvsControlGetPower() ? "ON" : "OFF",
                   true);
    char buf[8];
    snprintf(buf, sizeof(buf), "%u", (unsigned)dvsControlGetSpeed());
    s_mqtt.publish(MQTT_TOPIC_SPEED_STATE, buf, true);
    s_mqtt.publish(MQTT_TOPIC_TX_STATE, dvsControlGetTxEnabled() ? "ON" : "OFF",
                   true);
}

// Parse a truthy ON/1/true payload (case-insensitive).
static bool parseOnOff(const char *msg) {
    return (strcasecmp(msg, "ON") == 0) || (strcasecmp(msg, "true") == 0) ||
           (strcmp(msg, "1") == 0);
}

// Incoming command dispatch. Payloads are small (ON/OFF or a number), so a tiny
// stack buffer is enough.
static void onMessage(char *topic, uint8_t *payload, unsigned int len) {
    char msg[16];
    unsigned int n = (len < sizeof(msg) - 1) ? len : sizeof(msg) - 1;
    memcpy(msg, payload, n);
    msg[n] = '\0';

    if (strcmp(topic, MQTT_TOPIC_TX_SET) == 0) {
        dvsControlSetTxEnabled(parseOnOff(msg));
        netPublishState();
    } else if (strcmp(topic, MQTT_TOPIC_POWER_SET) == 0) {
        dvsControlSetPower(parseOnOff(msg));
        netPublishState();
    } else if (strcmp(topic, MQTT_TOPIC_SPEED_SET) == 0) {
        int v = atoi(msg);
        if (v < 0) {
            v = 0;
        } else if (v > DVS_MAX_SPEED) {
            v = DVS_MAX_SPEED;
        }
        dvsControlSetSpeed((uint8_t)v);
        netPublishState();
    }
}

// Publish Home Assistant MQTT discovery configs (retained) so HA auto-creates a
// fan, sensors and a switch under one device. Uses HA short keys to keep each
// JSON small enough for the (enlarged) PubSubClient buffer.
static void publishDiscovery() {
    if (!HA_DISCOVERY_ENABLED || !s_mqtt.connected()) {
        return;
    }
    char buf[512];

    // Fan: on/off via power topics, percentage via speed topics (range 1..max).
    snprintf(buf, sizeof(buf),
             "{\"name\":\"Ventilation\",\"uniq_id\":\"dvs_fan\","
             "\"cmd_t\":\"" MQTT_TOPIC_POWER_SET "\",\"stat_t\":\"" MQTT_TOPIC_POWER_STATE "\","
             "\"pct_cmd_t\":\"" MQTT_TOPIC_SPEED_SET "\",\"pct_stat_t\":\"" MQTT_TOPIC_SPEED_STATE "\","
             "\"spd_rng_min\":1,\"spd_rng_max\":%d,\"pl_on\":\"ON\",\"pl_off\":\"OFF\","
             HA_AVAIL "," HA_DEVICE "}",
             DVS_MAX_SPEED);
    s_mqtt.publish(HA_DISCOVERY_PREFIX "/fan/dvs/fan/config", buf, true);

    // Roof-space temperature sensor.
    s_mqtt.publish(
        HA_DISCOVERY_PREFIX "/sensor/dvs/temp/config",
        "{\"name\":\"Roof Temperature\",\"uniq_id\":\"dvs_temp\","
        "\"stat_t\":\"" MQTT_TOPIC_TEMPERATURE "\",\"unit_of_meas\":\"\\u00b0C\","
        "\"dev_cla\":\"temperature\",\"stat_cla\":\"measurement\","
        HA_AVAIL "," HA_DEVICE "}",
        true);

    // Actual (sniffed) fan speed sensor.
    s_mqtt.publish(
        HA_DISCOVERY_PREFIX "/sensor/dvs/speed_actual/config",
        "{\"name\":\"Actual Speed\",\"uniq_id\":\"dvs_speed_actual\","
        "\"stat_t\":\"" MQTT_TOPIC_ACTUAL_SPEED "\",\"ic\":\"mdi:fan\","
        "\"stat_cla\":\"measurement\","
        HA_AVAIL "," HA_DEVICE "}",
        true);

    // Transmit mode switch (listen-only vs listen+send).
    s_mqtt.publish(
        HA_DISCOVERY_PREFIX "/switch/dvs/tx/config",
        "{\"name\":\"Transmit\",\"uniq_id\":\"dvs_tx\","
        "\"cmd_t\":\"" MQTT_TOPIC_TX_SET "\",\"stat_t\":\"" MQTT_TOPIC_TX_STATE "\","
        "\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"ent_cat\":\"config\","
        HA_AVAIL "," HA_DEVICE "}",
        true);

    // Diagnostic sensors for each status-block register.
    for (uint8_t r : DVS_STATUS_REGS) {
        char topic[64];
        snprintf(topic, sizeof(topic),
                 HA_DISCOVERY_PREFIX "/sensor/dvs/reg_%02X/config", r);
        snprintf(buf, sizeof(buf),
                 "{\"name\":\"Status 0x%02X\",\"uniq_id\":\"dvs_reg_%02X\","
                 "\"stat_t\":\"" MQTT_TOPIC_STATUS_BASE "%02X\","
                 "\"ent_cat\":\"diagnostic\",\"ic\":\"mdi:information-outline\","
                 HA_AVAIL "," HA_DEVICE "}",
                 r, r, r);
        s_mqtt.publish(topic, buf, true);
    }
}

static bool mqttConnect() {
    if (WiFi.status() != WL_CONNECTED) {
        return false;
    }
    Serial.print(F("[mqtt] connecting... "));
    bool ok;
    if (strlen(MQTT_USERNAME) > 0) {
        ok = s_mqtt.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD,
                            MQTT_TOPIC_AVAILABILITY, 0, true, "offline");
    } else {
        ok = s_mqtt.connect(MQTT_CLIENT_ID, MQTT_TOPIC_AVAILABILITY, 0, true,
                            "offline");
    }

    if (ok) {
        Serial.println(F("connected"));
        s_mqtt.publish(MQTT_TOPIC_AVAILABILITY, "online", true);
        s_mqtt.subscribe(MQTT_TOPIC_POWER_SET);
        s_mqtt.subscribe(MQTT_TOPIC_SPEED_SET);
        s_mqtt.subscribe(MQTT_TOPIC_TX_SET);
        publishDiscovery();
        netPublishState();
    } else {
        Serial.printf("failed rc=%d\n", s_mqtt.state());
    }
    return ok;
}

void netSetup() {
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(MQTT_CLIENT_ID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // TLS to Mosquitto. With a CA cert we verify the broker; without one we fall
    // back to an encrypted-but-unauthenticated connection (setInsecure()).
    if (strlen(MQTT_CA_CERT) > 0) {
        s_wifiClient.setCACert(MQTT_CA_CERT);
    } else {
        s_wifiClient.setInsecure();
        Serial.println(F("[mqtt] TLS without CA verification (setInsecure)"));
    }

    s_mqtt.setServer(MQTT_HOST, MQTT_PORT);
    s_mqtt.setCallback(onMessage);
    // HA discovery JSON exceeds PubSubClient's default 256-byte buffer.
    s_mqtt.setBufferSize(768);
    Serial.printf("[wifi] connecting to \"%s\" as %s\n", WIFI_SSID,
                  MQTT_CLIENT_ID);
}

void netLoop() {
    const uint32_t now = millis();

    if (WiFi.status() != WL_CONNECTED) {
        if (now - s_lastWifiAttempt >= WIFI_RECONNECT_MS) {
            s_lastWifiAttempt = now;
            WiFi.disconnect();
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        }
        return;
    }

    if (!s_mqtt.connected()) {
        if (now - s_lastMqttAttempt >= MQTT_RECONNECT_MS) {
            s_lastMqttAttempt = now;
            mqttConnect();
        }
        return;
    }

    s_mqtt.loop();

    if (now - s_lastStatePublish >= STATE_PUBLISH_MS) {
        s_lastStatePublish = now;
        netPublishState();
    }
}

bool netPublishTemperature(float celsius) {
    if (!s_mqtt.connected()) {
        return false;
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", (double)celsius);
    return s_mqtt.publish(MQTT_TOPIC_TEMPERATURE, buf, true);
}

bool netPublishActualSpeed(uint8_t speed) {
    if (!s_mqtt.connected()) {
        return false;
    }
    char buf[8];
    snprintf(buf, sizeof(buf), "%u", (unsigned)speed);
    return s_mqtt.publish(MQTT_TOPIC_ACTUAL_SPEED, buf, true);
}

void netPublishStatusReg(uint8_t reg, const uint8_t *payload, size_t len) {
    if (!s_mqtt.connected()) {
        return;
    }
    char topic[40];
    snprintf(topic, sizeof(topic), MQTT_TOPIC_STATUS_BASE "%02X", reg);
    char val[3 * 16 + 1];
    int n = 0;
    for (size_t i = 0; i < len && i < 16; ++i) {
        n += snprintf(val + n, (n < (int)sizeof(val)) ? sizeof(val) - n : 0,
                      (i == 0) ? "%02X" : " %02X", payload[i]);
    }
    s_mqtt.publish(topic, val, true);
}

#endif // !UNIT_TEST
