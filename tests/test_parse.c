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

int main(void) {
    test_parse_msc();
    test_parse_hub();
    test_parse_malformed();
    if (failures) {
        printf("FAILED (%d)\n", failures);
    } else {
        printf("all tests passed\n");
    }
    return failures ? 1 : 0;
}
