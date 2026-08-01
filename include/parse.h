#pragma once

#include <string.h>

// Small ON/OFF/toggle argument parser shared by the USB console and the MQTT
// command layer, so the two do not each re-implement the same string matching.

enum DvsTriState {
    DVS_TRI_ON,
    DVS_TRI_OFF,
    DVS_TRI_TOGGLE,  // empty argument => caller should toggle
    DVS_TRI_INVALID, // non-empty but unrecognised
};

// Parse a trimmed argument. Accepts on/1/true/send/hi(gh) as ON and
// off/0/false/listen/lo(w) as OFF (case-insensitive); an empty string toggles.
inline DvsTriState dvsParseTriState(const char *arg) {
    if (arg == nullptr || arg[0] == '\0') {
        return DVS_TRI_TOGGLE;
    }
    if (strcasecmp(arg, "on") == 0 || strcmp(arg, "1") == 0 ||
        strcasecmp(arg, "true") == 0 || strcasecmp(arg, "send") == 0 ||
        strcasecmp(arg, "hi") == 0 || strcasecmp(arg, "high") == 0) {
        return DVS_TRI_ON;
    }
    if (strcasecmp(arg, "off") == 0 || strcmp(arg, "0") == 0 ||
        strcasecmp(arg, "false") == 0 || strcasecmp(arg, "listen") == 0 ||
        strcasecmp(arg, "lo") == 0 || strcasecmp(arg, "low") == 0) {
        return DVS_TRI_OFF;
    }
    return DVS_TRI_INVALID;
}

// Truthy test for payloads that only distinguish "on" from anything else.
inline bool dvsParseOnOff(const char *arg) {
    return dvsParseTriState(arg) == DVS_TRI_ON;
}
