#include <stdio.h>
#include <string.h>
#include "usb_parse.h"

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)

static const uint8_t MSC_CFG[] = {
    // config descriptor: wTotalLength=32, 1 interface, bConfigurationValue=1
    9, 2, 32, 0, 1, 1, 0, 0x80, 50,
    // interface: num 0, 2 eps, class 8 sub 6 proto 0x50
    9, 4, 0, 0, 2, 0x08, 0x06, 0x50, 0,
    // endpoint: 0x81 bulk, mps 64
    7, 5, 0x81, 0x02, 64, 0, 0,
    // endpoint: 0x02 bulk, mps 64
    7, 5, 0x02, 0x02, 64, 0, 0,
};

static const uint8_t HUB_CFG[] = {
    9, 2, 25, 0, 1, 1, 0, 0xE0, 50,
    9, 4, 0, 0, 1, 0x09, 0x00, 0x00, 0,
    7, 5, 0x81, 0x03, 1, 0, 0xFF,      // interrupt IN, mps 1, interval 255
};

static void test_parse_msc(void) {
    usb_cfg_info_t info;
    CHECK(usb_parse_config(MSC_CFG, sizeof MSC_CFG, &info));
    CHECK(info.is_msc && !info.is_hub);
    CHECK(info.config_value == 1);
    CHECK(info.msc_itf == 0);
    CHECK(info.bulk_in == 0x81 && info.bulk_in_mps == 64);
    CHECK(info.bulk_out == 0x02 && info.bulk_out_mps == 64);
}

static void test_parse_hub(void) {
    usb_cfg_info_t info;
    CHECK(usb_parse_config(HUB_CFG, sizeof HUB_CFG, &info));
    CHECK(info.is_hub && !info.is_msc);
    CHECK(info.hub_int_ep == 0x81 && info.hub_int_mps == 1 && info.hub_int_interval == 0xFF);
}

static void test_parse_malformed(void) {
    usb_cfg_info_t info;
    uint8_t zero_len[12];
    memcpy(zero_len, MSC_CFG, 12);
    zero_len[9] = 0;                              // interface bLength = 0
    CHECK(!usb_parse_config(zero_len, sizeof zero_len, &info));
    CHECK(!usb_parse_config(MSC_CFG, 3, &info));  // truncated header
    uint8_t overrun[sizeof MSC_CFG];
    memcpy(overrun, MSC_CFG, sizeof MSC_CFG);
    overrun[sizeof MSC_CFG - 7] = 200;            // last ep bLength runs past end
    CHECK(!usb_parse_config(overrun, sizeof overrun, &info));
}

// MSC interface with bInterfaceNumber = 3 (not 0)
static void test_parse_msc_nonzero_itf(void) {
    // Copy MSC_CFG and change bInterfaceNumber (byte index 11, offset 2 within interface desc)
    uint8_t cfg[sizeof MSC_CFG];
    memcpy(cfg, MSC_CFG, sizeof MSC_CFG);
    cfg[11] = 3;  // bInterfaceNumber = 3 (interface descriptor starts at byte 9)
    usb_cfg_info_t info;
    CHECK(usb_parse_config(cfg, sizeof cfg, &info));
    CHECK(info.is_msc && !info.is_hub);
    CHECK(info.msc_itf == 3);
    CHECK(info.bulk_in == 0x81 && info.bulk_in_mps == 64);
    CHECK(info.bulk_out == 0x02 && info.bulk_out_mps == 64);
}

// Composite: HID interface (class 3) with interrupt IN ep, then MSC interface with two bulk eps.
// The interrupt endpoint from the HID interface must not contaminate hub fields.
static void test_parse_composite(void) {
    static const uint8_t COMPOSITE_CFG[] = {
        // config descriptor: wTotalLength=48, 2 interfaces, bConfigurationValue=1
        9, 2, 48, 0, 2, 1, 0, 0x80, 50,
        // interface 0: class 3 (HID), 1 endpoint
        9, 4, 0, 0, 1, 0x03, 0x00, 0x00, 0,
        // endpoint: 0x83 interrupt IN, mps 8, interval 10
        7, 5, 0x83, 0x03, 8, 0, 10,
        // interface 1: MSC class 8 sub 6 proto 0x50, 2 endpoints
        9, 4, 1, 0, 2, 0x08, 0x06, 0x50, 0,
        // endpoint: 0x81 bulk IN, mps 64
        7, 5, 0x81, 0x02, 64, 0, 0,
        // endpoint: 0x02 bulk OUT, mps 64
        7, 5, 0x02, 0x02, 64, 0, 0,
    };
    // Verify size: 9 + 9 + 7 + 9 + 7 + 7 = 48... let me compute: 9+9+7+9+7+7=48
    // wTotalLength above says 53 but parser uses len not wTotalLength, so array size matters
    usb_cfg_info_t info;
    CHECK(usb_parse_config(COMPOSITE_CFG, sizeof COMPOSITE_CFG, &info));
    CHECK(info.is_msc && !info.is_hub);
    CHECK(info.bulk_in == 0x81 && info.bulk_in_mps == 64);
    CHECK(info.bulk_out == 0x02 && info.bulk_out_mps == 64);
    CHECK(info.hub_int_ep == 0);  // foreign interrupt ep must not contaminate hub fields
}

// First descriptor has bDescriptorType != 2 (not a config descriptor) -> parse fails
static void test_parse_wrong_type(void) {
    uint8_t bad[sizeof MSC_CFG];
    memcpy(bad, MSC_CFG, sizeof MSC_CFG);
    bad[1] = 0x01;  // bDescriptorType = 1 (device), not 2 (config)
    usb_cfg_info_t info;
    CHECK(!usb_parse_config(bad, sizeof bad, &info));
}

// Alt-setting test: MSC alt 0 with two bulk EPs, then same interface alt 1 with different EPs.
// Result should report alt 0's endpoints.
static void test_parse_alt_setting(void) {
    static const uint8_t ALT_CFG[] = {
        // config descriptor: wTotalLength=55, 1 interface (with alt), bConfigurationValue=1
        9, 2, 55, 0, 1, 1, 0, 0x80, 50,
        // interface 0, alt 0: MSC, 2 endpoints
        9, 4, 0, 0, 2, 0x08, 0x06, 0x50, 0,
        // endpoint: 0x81 bulk IN, mps 64
        7, 5, 0x81, 0x02, 64, 0, 0,
        // endpoint: 0x02 bulk OUT, mps 64
        7, 5, 0x02, 0x02, 64, 0, 0,
        // interface 0, alt 1: same class, different endpoints
        9, 4, 0, 1, 2, 0x08, 0x06, 0x50, 0,
        // endpoint: 0x83 bulk IN, mps 512 (alt 1 - should be ignored)
        7, 5, 0x83, 0x02, 0x00, 0x02, 0,
        // endpoint: 0x04 bulk OUT, mps 512 (alt 1 - should be ignored)
        7, 5, 0x04, 0x02, 0x00, 0x02, 0,
    };
    usb_cfg_info_t info;
    CHECK(usb_parse_config(ALT_CFG, sizeof ALT_CFG, &info));
    CHECK(info.is_msc && !info.is_hub);
    CHECK(info.msc_itf == 0);
    // Alt 0 endpoints should be reported, not alt 1
    CHECK(info.bulk_in == 0x81 && info.bulk_in_mps == 64);
    CHECK(info.bulk_out == 0x02 && info.bulk_out_mps == 64);
}

static void test_cbw_clamp(void) {
    uint8_t cbw[MSC_CBW_LEN];
    uint8_t big_cb[20];
    for (int i = 0; i < 20; i++) big_cb[i] = (uint8_t)(i + 1);  // 1..20
    msc_build_cbw(cbw, 1, 0, false, 0, big_cb, 20);
    CHECK(cbw[14] == 16);                           // bCBWCBLength must be clamped to 16
    CHECK(memcmp(cbw + 15, big_cb, 16) == 0);       // first 16 bytes copied
}

static void test_cbw_build(void) {
    uint8_t cbw[MSC_CBW_LEN];
    const uint8_t read10[10] = {0x28, 0, 0, 0, 0x12, 0x34, 0, 0, 0x40, 0};
    msc_build_cbw(cbw, 0xDEADBEEF, 0x8000, true, 0, read10, sizeof read10);
    CHECK(cbw[0] == 0x55 && cbw[1] == 0x53 && cbw[2] == 0x42 && cbw[3] == 0x43); // 'USBC'
    CHECK(cbw[4] == 0xEF && cbw[5] == 0xBE && cbw[6] == 0xAD && cbw[7] == 0xDE); // tag LE
    CHECK(cbw[8] == 0x00 && cbw[9] == 0x80 && cbw[10] == 0 && cbw[11] == 0);     // len LE
    CHECK(cbw[12] == 0x80);                                                       // IN
    CHECK(cbw[13] == 0 && cbw[14] == 10);                                         // lun, cb_len
    CHECK(memcmp(cbw + 15, read10, 10) == 0);
    CHECK(cbw[25] == 0 && cbw[30] == 0);                                          // zero padding
}

static void test_csw_parse(void) {
    uint8_t status;
    uint8_t csw[MSC_CSW_LEN] = {0x55, 0x53, 0x42, 0x53,            // 'USBS'
        0xEF, 0xBE, 0xAD, 0xDE, 0, 0, 0, 0, 0};
    CHECK(msc_parse_csw(csw, 0xDEADBEEF, &status) && status == 0);
    csw[12] = 1;
    CHECK(msc_parse_csw(csw, 0xDEADBEEF, &status) && status == 1);
    csw[12] = 3;
    CHECK(!msc_parse_csw(csw, 0xDEADBEEF, &status));   // invalid status
    csw[12] = 0;
    CHECK(!msc_parse_csw(csw, 0x12345678, &status));   // wrong tag
    csw[0] = 0x56;
    CHECK(!msc_parse_csw(csw, 0xDEADBEEF, &status));   // bad signature
}

int main(void) {
    test_parse_msc();
    test_parse_hub();
    test_parse_malformed();
    test_parse_msc_nonzero_itf();
    test_parse_composite();
    test_parse_wrong_type();
    test_parse_alt_setting();
    test_cbw_clamp();
    test_cbw_build();
    test_csw_parse();
    if (failures) {
        printf("FAILED (%d)\n", failures);
    } else {
        printf("all tests passed\n");
    }
    return failures ? 1 : 0;
}
