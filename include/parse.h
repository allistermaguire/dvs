#pragma once

#include <stdlib.h>
#include <string.h>
#include <strings.h> // strcasecmp on host/native builds

#include "control.h" // DvsProfileSel for the profile parser

// Small argument parsers shared by the USB console and the MQTT command layer,
// so the two do not each re-implement the same string matching. The numeric and
// profile parsers are strict: they reject invalid input instead of coercing it
// to 0/defaults (a bad payload must not silently power the fan off or feed a
// bogus 0 C into auto control).

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

// Skip trailing whitespace, then require the string to have ended. Used by the
// numeric parsers so "12x" or "1.5 junk" is rejected rather than truncated.
inline bool dvsParseAtEnd(const char *end) {
    while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n') {
        ++end;
    }
    return *end == '\0';
}

// Parse a whole token as a base-10 integer within [lo, hi]. Returns false (and
// leaves *out untouched) on empty input, trailing garbage, or an out-of-range
// value, so callers can reject bad input rather than coerce it to 0.
inline bool dvsParseIntRange(const char *arg, long lo, long hi, long *out) {
    if (arg == nullptr || arg[0] == '\0') {
        return false;
    }
    char *end = nullptr;
    const long v = strtol(arg, &end, 10);
    if (end == arg || !dvsParseAtEnd(end) || v < lo || v > hi) {
        return false;
    }
    if (out != nullptr) {
        *out = v;
    }
    return true;
}

// Parse a whole token as a float. Returns false (leaving *out untouched) on
// empty input or trailing garbage.
inline bool dvsParseFloat(const char *arg, float *out) {
    if (arg == nullptr || arg[0] == '\0') {
        return false;
    }
    char *end = nullptr;
    const float v = strtof(arg, &end);
    if (end == arg || !dvsParseAtEnd(end)) {
        return false;
    }
    if (out != nullptr) {
        *out = v;
    }
    return true;
}

// Parse a profile selection (manual|winter|summer). Returns false on any other
// token, so an unrecognised profile (including the removed "auto") is rejected
// rather than silently coerced.
inline bool dvsParseProfile(const char *arg, DvsProfileSel *out) {
    if (arg == nullptr) {
        return false;
    }
    if (strcasecmp(arg, "manual") == 0) {
        if (out != nullptr) {
            *out = DVS_PROFILE_SEL_MANUAL;
        }
        return true;
    }
    if (strcasecmp(arg, "winter") == 0) {
        if (out != nullptr) {
            *out = DVS_PROFILE_SEL_WINTER;
        }
        return true;
    }
    if (strcasecmp(arg, "summer") == 0) {
        if (out != nullptr) {
            *out = DVS_PROFILE_SEL_SUMMER;
        }
        return true;
    }
    return false;
}
