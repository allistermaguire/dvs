// Host-side unit tests: run with `pio test -e native`.
//
// CRC correctness is anchored to the well-known "123456789" check strings for
// each CRC-16 variant, so these tests validate the implementation independently
// of the firmware itself. Frame tests then build valid/invalid frames and
// confirm verification + decoding behaviour.

#include <string.h>

#include <unity.h>

#include "crc16.h"
#include "frame.h"

static const uint8_t CHECK[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};

void setUp() {}
void tearDown() {}

// --- CRC standard check vectors ----------------------------------------------

void test_crc_ccitt_false() {
    // init 0xFFFF, poly 0x1021, no reflection => 0x29B1
    Crc16Params p;
    p.init = 0xFFFF;
    p.reflected = false;
    TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16_ccitt(CHECK, sizeof(CHECK), p));
}

void test_crc_xmodem() {
    // init 0x0000, no reflection => 0x31C3
    Crc16Params p;
    p.init = 0x0000;
    p.reflected = false;
    TEST_ASSERT_EQUAL_HEX16(0x31C3, crc16_ccitt(CHECK, sizeof(CHECK), p));
}

void test_crc_kermit() {
    // init 0x0000, reflected in/out => 0x2189
    Crc16Params p;
    p.init = 0x0000;
    p.reflected = true;
    TEST_ASSERT_EQUAL_HEX16(0x2189, crc16_ccitt(CHECK, sizeof(CHECK), p));
}

void test_crc_aug_ccitt() {
    // The confirmed DVS variant: init 0x1D0F, no reflection => 0xE5CC.
    Crc16Params p;
    p.init = 0x1D0F;
    p.reflected = false;
    TEST_ASSERT_EQUAL_HEX16(0xE5CC, crc16_ccitt(CHECK, sizeof(CHECK), p));
}

void test_crc_real_capture_frame() {
    // Captured DVS frame: 02 10 DC 00 00 | CRC BE 1F (big-endian on the wire).
    // Defaults are AUG-CCITT (init 0x1D0F); verify with crcBigEndian=true.
    const uint8_t frame[] = {0x02, 0x10, 0xDC, 0x00, 0x00, 0xBE, 0x1F};
    TEST_ASSERT_TRUE(
        crc16_verify_frame(frame, sizeof(frame), Crc16Params{}, /*big=*/true));
}

// --- Frame verification ------------------------------------------------------

void test_frame_crc_ok_little_endian() {
    DvsFrame f;
    f.bytes[0] = 0x01; // addr
    f.bytes[1] = 0x15; // register
    const uint16_t crc = crc16_ccitt(f.bytes, 2, Crc16Params{});
    f.bytes[2] = (uint8_t)(crc & 0xFF);        // LSB first
    f.bytes[3] = (uint8_t)((crc >> 8) & 0xFF); // MSB
    f.len = 4;

    TEST_ASSERT_TRUE(dvs_frame_crc_ok(f, Crc16Params{}, /*bigEndian=*/false));
}

void test_frame_crc_bad_when_corrupted() {
    DvsFrame f;
    f.bytes[0] = 0x01;
    f.bytes[1] = 0x15;
    const uint16_t crc = crc16_ccitt(f.bytes, 2, Crc16Params{});
    f.bytes[2] = (uint8_t)(crc & 0xFF);
    f.bytes[3] = (uint8_t)((crc >> 8) & 0xFF);
    f.len = 4;
    f.bytes[1] ^= 0xFF; // corrupt the register byte after CRC was computed

    TEST_ASSERT_FALSE(dvs_frame_crc_ok(f, Crc16Params{}, false));
}

// --- Frame building (transmit path) -------------------------------------------

void test_register_names() {
    TEST_ASSERT_EQUAL_STRING("temperature", dvs_register_name(0x10));
    TEST_ASSERT_EQUAL_STRING("fan-speed", dvs_register_name(0x15));
    TEST_ASSERT_EQUAL_STRING("deadman", dvs_register_name(0x03));
    TEST_ASSERT_EQUAL_STRING("?", dvs_register_name(0x99));
}

void test_frame_build_matches_capture() {
    // Building addr=0x02 reg=0x10 payload=DC 00 00 must reproduce the real
    // captured frame, CRC included, big-endian on the wire: BE 1F.
    const uint8_t payload[] = {0xDC, 0x00, 0x00};
    const uint8_t expected[] = {0x02, 0x10, 0xDC, 0x00, 0x00, 0xBE, 0x1F};
    uint8_t out[16];
    const size_t n = dvs_frame_build(0x02, 0x10, payload, sizeof(payload), out,
                                     sizeof(out), Crc16Params{}, /*big=*/true);
    TEST_ASSERT_EQUAL_UINT(sizeof(expected), n);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, sizeof(expected));
}

void test_frame_build_roundtrips_crc() {
    // A built frame must verify against the same CRC parameters.
    const uint8_t payload[] = {42};
    uint8_t out[16];
    const size_t n = dvs_frame_build(0x02, 0x15, payload, sizeof(payload), out,
                                     sizeof(out), Crc16Params{}, /*big=*/true);
    TEST_ASSERT_EQUAL_UINT(5, n); // addr+reg+1 payload+2 crc
    TEST_ASSERT_TRUE(crc16_verify_frame(out, n, Crc16Params{}, /*big=*/true));
}

void test_frame_build_rejects_small_buffer() {
    const uint8_t payload[] = {1, 2, 3};
    uint8_t out[4]; // needs 7 bytes, only 4 available
    TEST_ASSERT_EQUAL_UINT(0, dvs_frame_build(0x02, 0x10, payload,
                                              sizeof(payload), out, sizeof(out),
                                              Crc16Params{}, true));
}

// --- Decode / formatting ------------------------------------------------------

void test_format_contains_decode() {
    // addr=0x01 reg=0x10(temp) payload=0x34 0x12 0x00 + CRC
    DvsFrame f;
    f.bytes[0] = 0x01;
    f.bytes[1] = 0x10;
    f.bytes[2] = 0x34;
    f.bytes[3] = 0x12;
    f.bytes[4] = 0x00;
    const uint16_t crc = crc16_ccitt(f.bytes, 5, Crc16Params{});
    f.bytes[5] = (uint8_t)(crc & 0xFF);
    f.bytes[6] = (uint8_t)((crc >> 8) & 0xFF);
    f.len = 7;

    char out[256];
    dvs_frame_format(f, out, sizeof(out), Crc16Params{}, false);

    TEST_ASSERT_NOT_NULL(strstr(out, "CRC=OK"));
    TEST_ASSERT_NOT_NULL(strstr(out, "reg=0x10"));
    TEST_ASSERT_NOT_NULL(strstr(out, "temperature"));
    TEST_ASSERT_NOT_NULL(strstr(out, "temp_raw=4660")); // 0x1234
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_crc_ccitt_false);
    RUN_TEST(test_crc_xmodem);
    RUN_TEST(test_crc_kermit);
    RUN_TEST(test_crc_aug_ccitt);
    RUN_TEST(test_crc_real_capture_frame);
    RUN_TEST(test_frame_crc_ok_little_endian);
    RUN_TEST(test_frame_crc_bad_when_corrupted);
    RUN_TEST(test_register_names);
    RUN_TEST(test_frame_build_matches_capture);
    RUN_TEST(test_frame_build_roundtrips_crc);
    RUN_TEST(test_frame_build_rejects_small_buffer);
    RUN_TEST(test_format_contains_decode);
    return UNITY_END();
}
