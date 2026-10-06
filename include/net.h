#pragma once

#include <stddef.h>
#include <stdint.h>

// Wi-Fi + MQTT bridge. Connects to the broker, subscribes to the command topics
// and publishes fan state plus sniffed sensor values. All work is done from
// netLoop(); nothing here blocks for long.

// Start Wi-Fi (station mode) and configure the MQTT client. Non-blocking.
void netSetup();

// Service Wi-Fi/MQTT: (re)connect as needed, pump the client, publish state.
// Call once per loop() iteration.
void netLoop();

// Publish the current fan/tx state (power, speed, tx mode) to MQTT.
void netPublishState();

// Publish a sniffed airflow (supply-air) temperature (deg C) to the temperature topic.
// Returns true if it was actually sent (MQTT connected), false otherwise.
bool netPublishTemperature(float celsius);

// Publish a sniffed actual fan speed (0..84) to the actual-speed topic.
// Returns true if it was actually sent (MQTT connected), false otherwise.
bool netPublishActualSpeed(uint8_t speed);

// Publish the raw payload of a status-block register (0x01/0x06/0x12/0x13) as a
// hex string to dvs/status/reg/<hh>, so the block can be watched over MQTT.
void netPublishStatusReg(uint8_t reg, const uint8_t *payload, size_t len);

// True while an OTA firmware update is in progress. loop() uses this to suspend
// the RS485 poll cycle and frame draining so the transfer isn't disturbed.
bool netOtaInProgress();
