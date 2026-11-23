// Wi-Fi + MQTT bridge for the DVS sniffer/controller.
//
// Excluded from the native (host) unit-test build: it depends on the ESP32
// Arduino core and PubSubClient, and there is no bus to drive off-target.

#if !defined(UNIT_TEST)

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

#include <string.h>

#include "config.h"
#include "control.h"
#include "net.h"
#include "parse.h"
#include "secrets.h"

static WiFiClientSecure s_wifiClient;
static PubSubClient s_mqtt(s_wifiClient);

static uint32_t s_lastWifiAttempt = 0;
static uint32_t s_lastMqttAttempt = 0;
static uint32_t s_lastStatePublish = 0;

// OTA is started once, after Wi-Fi first associates (ArduinoOTA.begin needs a
// live connection). s_otaInProgress gates the RS485 work in loop() during a flash.
static bool s_otaStarted = false;
static bool s_otaInProgress = false;

// One-shot restore of standby/profile/tx/heat_recovery/heat_retention/
// cutoff_temp/target_temp from the retained */state topics on first connect.
// While active we suspend our own state publishing so the retained values aren't
// clobbered before we read them back.
static bool s_restoreDone = false;
static bool s_restoreActive = false;
static uint32_t s_restoreDeadline = 0;

// Publish the currect boost state (minutes remaining + on/off). Non-retained:
// boost is transient and must default off after a power cycle, so no value is
// left on the broker to linger as ON while the device is offline.
static void publishBoostState() {
    if (!s_mqtt.connected()) {
        return;
    }
    const uint16_t remaining = dvsControlBoostRemainingMin();
    char buf[8];
    snprintf(buf, sizeof(buf), "%u", (unsigned)remaining);
    s_mqtt.publish(MQTT_TOPIC_BOOST_STATE, buf, false);
    s_mqtt.publish(MQTT_TOPIC_BOOST_PWR_STATE, remaining > 0 ? "ON" : "OFF", false);
}

void netPublishState() {
    if (!s_mqtt.connected()) {
        return;
    }
    // Power state is the ACTUAL fan run state (off while a season idles the fan),
    // so Homekit doesn't render an on-but-0s fan as 1%. The user-off intent is
    // carried separately on the standby topic (for persistence).
    s_mqtt.publish(MQTT_TOPIC_POWER_STATE, dvsControlGetPower() ? "ON" : "OFF", 
                   true);
    s_mqtt.publish(MQTT_TOPIC_STANDBY_STATE, dvsControlGetStandby() ? "ON" : "OFF", 
                   true);
    char buf[8];
    snprintf(buf, sizeof(buf), "%u", (unsigned)dvsControlGetSpeed());
    s_mqtt.publish(MQTT_TOPIC_SPEED_STATE, buf, true);
    s_mqtt.publish(MQTT_TOPIC_TX_STATE, dvsControlGetTxEnabled() ? "ON" : "OFF",
                   true);
    // Auto-control state.
    s_mqtt.publish(MQTT_TOPIC_PROFILE_STATE,
                   dvsControlProfileName(dvsControlGetProfile()), true);
    s_mqtt.publish(MQTT_TOPIC_HEATREC_STATE,
                   dvsControlGetHeatRecovery() ? "ON" : "OFF", true);
    s_mqtt.publish(MQTT_TOPIC_RETENTION_STATE,
                   dvsControlGetHeatRetention() ? "ON" : "OFF", true);
    snprintf(buf, sizeof(buf), "%.1f", (double)dvsControlGetCutoff());
    s_mqtt.publish(MQTT_TOPIC_CUTOFF_STATE, buf, true);
    snprintf(buf, sizeof(buf), "%.1f", (double)dvsControlGetComfortTarget());
    s_mqtt.publish(MQTT_TOPIC_TARGET_STATE, buf, true);
    publishBoostState();
    float inside;
    if (dvsControlGetInsideTemp(&inside)) {
        snprintf(buf, sizeof(buf), "%.1f", (double)inside);
        s_mqtt.publish(MQTT_TOPIC_INSIDE_STATE, buf, true);
    }
}

// Incoming command dispatch. Payloads are small (ON/OFF or a number), so a tiny
// stack buffer is enough.
static void onMessage(char *topic, uint8_t *payload, unsigned int len) {
    char msg[16];
    unsigned int n = (len < sizeof(msg) - 1) ? len : sizeof(msg) - 1;
    memcpy(msg, payload, n);
    msg[n] = '\0';

    // Boot restore: apply the retained */state values via the setters only (the
    // setters don't publish, so this can't loop back to us while subscribed).
    if (s_restoreActive) {
        if (strcmp(topic, MQTT_TOPIC_PROFILE_STATE) == 0) {
            DvsProfileSel sel;
            if (dvsParseProfile(msg, &sel)) {
                dvsControlSetProfile(sel);
            }
            return;
        } else if (strcmp(topic, MQTT_TOPIC_STANDBY_STATE) == 0) {
            dvsControlSetStandby(!dvsParseOnOff(msg));
            return;
        } else if (strcmp(topic, MQTT_TOPIC_TX_STATE) == 0) {
            dvsControlSetTxEnabled(dvsParseOnOff(msg));
            return;
        } else if (strcmp(topic, MQTT_TOPIC_HEATREC_STATE) == 0) {
            dvsControlSetHeatRecovery(dvsParseOnOff(msg));
            return;
        } else if (strcmp(topic, MQTT_TOPIC_RETENTION_STATE) == 0) {
            dvsControlSetHeatRetention(dvsParseOnOff(msg));
            return;
        } else if (strcmp(topic, MQTT_TOPIC_CUTOFF_STATE) == 0) {
            float t;
            if (dvsParseFloat(msg, &t) && t >= DVS_CUTOFF_MIN_C &&
                t <= DVS_CUTOFF_MAX_C) {
                dvsControlSetCutoff(t);
            }
            return;
        } else if (strcmp(topic, MQTT_TOPIC_TARGET_STATE) == 0) {
            float t;
            if (dvsParseFloat(msg, &t) && t >= DVS_TARGET_MIN_C &&
                t <= DVS_TARGET_MAX_C) {
                dvsControlSetComfortTarget(t);
            }
            return;
        }
    }

    if (strcmp(topic, MQTT_TOPIC_TX_SET) == 0) {
        dvsControlSetTxEnabled(dvsParseOnOff(msg));
        netPublishState();
    } else if (strcmp(topic, MQTT_TOPIC_POWER_SET) == 0) {
        dvsControlSetPower(dvsParseOnOff(msg));
        netPublishState();
    } else if (strcmp(topic, MQTT_TOPIC_SPEED_SET) == 0) {
        long v;
        if (dvsParseIntRange(msg, 0, DVS_MAX_SPEED, &v)) {
            dvsControlSetSpeed((uint8_t)v);
            netPublishState();
        }
	} else if (strcmp(topic, MQTT_TOPIC_INSIDE_SET) == 0) {
		float t;
		if (dvsParseFloat(msg, &t)) {
			dvsControlSetInsideTemp(t);
			netPublishState();
		}
	} else if (strcmp(topic, MQTT_TOPIC_PROFILE_SET) == 0) {
		DvsProfileSel sel;
		if (dvsParseProfile(msg, &sel)) {
			dvsControlSetProfile(sel);
			netPublishState();
		}
	} else if (strcmp(topic, MQTT_TOPIC_HEATREC_SET) == 0) {
		dvsControlSetHeatRecovery(dvsParseOnOff(msg));
		netPublishState();
    } else if (strcmp(topic, MQTT_TOPIC_RETENTION_SET) == 0) {
        dvsControlSetHeatRetention(dvsParseOnOff(msg));
		netPublishState();
    } else if (strcmp(topic, MQTT_TOPIC_CUTOFF_SET) == 0) {
        float t;
        if (dvsParseFloat(msg, &t) && t >= DVS_CUTOFF_MIN_C &&
            t <= DVS_CUTOFF_MAX_C) {
            dvsControlSetCutoff(t);
            netPublishState();
        }
    } else if (strcmp(topic, MQTT_TOPIC_TARGET_SET) == 0) {
        float t;
        if (dvsParseFloat(msg, &t) && t >= DVS_TARGET_MIN_C &&
            t <= DVS_TARGET_MAX_C) {
            dvsControlSetComfortTarget(t);
            netPublishState();
        }
	} else if (strcmp(topic, MQTT_TOPIC_BOOST_SET) == 0) {
        long v;
        if (dvsParseIntRange(msg, 0, DVS_BOOST_MAX_MINUTES, &v)) {
            dvsControlSetBoost((uint16_t)v);
            netPublishState();
        }
    } else if (strcmp(topic, MQTT_TOPIC_BOOST_PWR_SET) == 0) {
        dvsControlSetBoost(dvsParseOnOff(msg) ? DVS_BOOST_DEFAULT_MINUTES : 0);
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

    // Airflow (supply-air) temperature sensor.
    s_mqtt.publish(
        HA_DISCOVERY_PREFIX "/sensor/dvs/temp/config",
        "{\"name\":\"Airflow Temperature\",\"uniq_id\":\"dvs_airflow_temp\","
        "\"stat_t\":\"" MQTT_TOPIC_TEMPERATURE "\",\"unit_of_meas\":\"\\u00b0C\","
        "\"dev_cla\":\"temperature\",\"stat_cla\":\"measurement\","
        HA_AVAIL "," HA_DEVICE "}",
        true);

    // Actual (sniffed) fan speed sensor.
    s_mqtt.publish(
        HA_DISCOVERY_PREFIX "/sensor/dvs/speed_actual/config",
        "{\"name\":\"Fan Speed\",\"uniq_id\":\"dvs_speed_actual\","
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

    // Profile select: manual control vs automatic winter/summer seasons (merges
    // the old control-mode + season selectors into one axis).
	s_mqtt.publish(
		HA_DISCOVERY_PREFIX "/select/dvs/profile/config",
		"{\"name\":\"Profile\",\"uniq_id\":\"dvs_profile\","
		"\"cmd_t\":\"" MQTT_TOPIC_PROFILE_SET "\",\"stat_t\":\"" MQTT_TOPIC_PROFILE_STATE "\","
        "\"options\":[\"manual\",\"winter\",\"summer\"],\"ent_cat\":\"config\","
		HA_AVAIL "," HA_DEVICE "}",
		true);

	// Winter heat-recovery boost switch.
	s_mqtt.publish(
		HA_DISCOVERY_PREFIX "/switch/dvs/heat_recovery/config",
        "{\"name\":\"Heat Recovery\",\"uniq_id\":\"dvs_heat_recovery\","
        "\"cmd_t\":\"" MQTT_TOPIC_HEATREC_SET "\",\"stat_t\":\"" MQTT_TOPIC_HEATREC_STATE "\","
		"\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"ent_cat\":\"config\","
		HA_AVAIL "," HA_DEVICE "}",
		true);

    // Winter heat-retention switch (stop the fan below the cold cutoff to keep heat).
    s_mqtt.publish(
        HA_DISCOVERY_PREFIX "/switch/dvs/heat_retention/config",
        "{\"name\":\"Heat Retention\",\"uniq_id\":\"dvs_heat_retention\","
        "\"cmd_t\":\"" MQTT_TOPIC_RETENTION_SET "\",\"stat_t\":\"" MQTT_TOPIC_RETENTION_STATE "\","
        "\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"ent_cat\":\"config\","
        HA_AVAIL "," HA_DEVICE "}",
        true);

    // Winter cold-cutoff temperature number.
    snprintf(buf, sizeof(buf),
             "{\"name\":\"Cold Cutoff Temperature\",\"uniq_id\":\"dvs_cutoff_temp\","
             "\"cmd_t\":\"" MQTT_TOPIC_CUTOFF_SET "\",\"stat_t\":\"" MQTT_TOPIC_CUTOFF_STATE "\","
             "\"min\":%d,\"max\":%d,\"step\":0.5,\"unit_of_meas\":\"\\u00b0C\","
             "\"dev_cla\":\"temperature\",\"mode\":\"box\",\"ent_cat\":\"config\","
             HA_AVAIL "," HA_DEVICE "}",
             (int)DVS_CUTOFF_MIN_C, (int)DVS_CUTOFF_MAX_C);
    s_mqtt.publish(HA_DISCOVERY_PREFIX "/number/dvs/cutoff_temp/config", buf, true);

    // Unified target (comfort) inside temperature number.
    snprintf(buf, sizeof(buf),
        "{\"name\":\"Set Temperature\",\"uniq_id\":\"dvs_target_temp\","
        "\"cmd_t\":\"" MQTT_TOPIC_TARGET_SET "\",\"stat_t\":\"" MQTT_TOPIC_TARGET_STATE "\","
        "\"min\":%d,\"max\":%d,\"step\":0.5,\"unit_of_meas\":\"\\u00b0C\","
        "\"dev_cla\":\"temperature\",\"mode\":\"box\",\"ent_cat\":\"config\","
        HA_AVAIL "," HA_DEVICE "}",
        (int)DVS_TARGET_MIN_C, (int)DVS_TARGET_MAX_C);
    s_mqtt.publish(HA_DISCOVERY_PREFIX "/number/dvs/target_temp/config", buf, true);

    // Timed full-speed boost (minutes; 0 = off). State shows minutes remaining.
    snprintf(buf, sizeof(buf),
            "{\"name\":\"Boost Timer\",\"uniq_id\":\"dvs_boost_timer\","
            "\"cmd_t\":\"" MQTT_TOPIC_BOOST_SET "\",\"stat_t\":\"" MQTT_TOPIC_BOOST_STATE "\","
            "\"min\":0,\"max\":%u,\"step\":30,\"unit_of_meas\":\"min\","
            "\"ic\":\"mdi:fan-plus\",\"mode\":\"box\","
            HA_AVAIL "," HA_DEVICE "}",
            (unsigned)DVS_BOOST_MAX_MINUTES);
    s_mqtt.publish(HA_DISCOVERY_PREFIX "/number/dvs/boost/config", buf, true);

    // Simple boost on/off switch (bridges to HomeKit/Alexa as a toggle). ON runs
    // a boost of DVS_BOOST_DEFAULT_MINUTES; it reverts to OFF when the timer ends.
    s_mqtt.publish(
        HA_DISCOVERY_PREFIX "/switch/dvs/boost/config",
        "{\"name\":\"Fan Boost\",\"uniq_id\":\"dvs_fan_boost\","
        "\"cmd_t\":\"" MQTT_TOPIC_BOOST_PWR_SET "\",\"stat_t\":\"" MQTT_TOPIC_BOOST_PWR_STATE "\","
        "\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"ic\":\"mdi:fan-plus\","
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
		s_mqtt.subscribe(MQTT_TOPIC_INSIDE_SET);
		s_mqtt.subscribe(MQTT_TOPIC_PROFILE_SET);
		s_mqtt.subscribe(MQTT_TOPIC_HEATREC_SET);
        s_mqtt.subscribe(MQTT_TOPIC_RETENTION_SET);
        s_mqtt.subscribe(MQTT_TOPIC_CUTOFF_SET);
        s_mqtt.subscribe(MQTT_TOPIC_TARGET_SET);
        s_mqtt.subscribe(MQTT_TOPIC_BOOST_SET);
        s_mqtt.subscribe(MQTT_TOPIC_BOOST_PWR_SET);
        publishDiscovery();
        publishBoostState();
        if (!s_restoreDone) {
            // First connect after boot: read the retained settings back before
            // publishing our own (default) state over the top of them.
            s_mqtt.subscribe(MQTT_TOPIC_STANDBY_STATE);
            s_mqtt.subscribe(MQTT_TOPIC_PROFILE_STATE);
            s_mqtt.subscribe(MQTT_TOPIC_TX_STATE);
            s_mqtt.subscribe(MQTT_TOPIC_HEATREC_STATE);
            s_mqtt.subscribe(MQTT_TOPIC_RETENTION_STATE);
            s_mqtt.subscribe(MQTT_TOPIC_CUTOFF_STATE);
            s_mqtt.subscribe(MQTT_TOPIC_TARGET_STATE);
            s_restoreActive = true;
            s_restoreDeadline = millis() + DVS_MQTT_RESTORE_WINDOW_MS;
        } else {
            netPublishState();
        }
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

bool netOtaInProgress() {
    return s_otaInProgress;
}

// Configure and start ArduinoOTA. Called once from netLoop() after Wi-Fi first
// associates (begin() needs an active connection). The onStart callback flags
// the update so loop() suspends the RS485 poll cycle, and marks the device
// offline so Home Assistant shows it as updating.
static void otaBegin() {
    ArduinoOTA.setHostname(MQTT_CLIENT_ID);
    if (strlen(OTA_PASSWORD) > 0) {
        ArduinoOTA.setPassword(OTA_PASSWORD);
    }
    ArduinoOTA.onStart([]() {
        s_otaInProgress = true;
        if (s_mqtt.connected()) {
            s_mqtt.publish(MQTT_TOPIC_AVAILABILITY, "offline", true);
        }
        Serial.println(F("[ota] update starting"));
    });
    ArduinoOTA.onEnd([]() { Serial.println(F("\n[ota] update complete")); });
    ArduinoOTA.onError([](ota_error_t error) {
        s_otaInProgress = false;
        Serial.printf("[ota] error %u\n", (unsigned)error);
    });
    ArduinoOTA.begin();
    Serial.println(F("[ota] ready"));
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

    if (!s_otaStarted) {
        s_otaStarted = true;
        otaBegin();
    }
    ArduinoOTA.handle();

    if (!s_mqtt.connected()) {
        if (now - s_lastMqttAttempt >= MQTT_RECONNECT_MS) {
            s_lastMqttAttempt = now;
            mqttConnect();
        }
        return;
    }

    s_mqtt.loop();

    if (s_restoreActive && (int32_t)(now - s_restoreDeadline) >= 0) {
        // Restore window elapsed: stop listening to our own state topics, then
        // publish the restored (or default) state once more without loopback.
        s_restoreActive = false;
        s_restoreDone = true;
        s_mqtt.unsubscribe(MQTT_TOPIC_STANDBY_STATE);
        s_mqtt.unsubscribe(MQTT_TOPIC_PROFILE_STATE);
        s_mqtt.unsubscribe(MQTT_TOPIC_TX_STATE);
        s_mqtt.unsubscribe(MQTT_TOPIC_HEATREC_STATE);
        s_mqtt.unsubscribe(MQTT_TOPIC_RETENTION_STATE);
        s_mqtt.unsubscribe(MQTT_TOPIC_CUTOFF_STATE);
        s_mqtt.unsubscribe(MQTT_TOPIC_TARGET_STATE);
        netPublishState();
    }

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
