#pragma once
#include <stdint.h>
#include <stdbool.h>

#define USB_CLASS_HUB     0x09
#define USB_CLASS_MSC     0x08
#define MSC_SUBCLASS_SCSI 0x06
#define MSC_PROTO_BOT     0x50

typedef struct {
    uint8_t  config_value;    // bConfigurationValue
    bool     is_hub;
    bool     is_msc;
    // valid when is_msc:
    uint8_t  msc_itf;         // bInterfaceNumber
    uint8_t  bulk_in;         // endpoint address (0x8x)
    uint8_t  bulk_out;        // endpoint address (0x0x)
    uint16_t bulk_in_mps;
    uint16_t bulk_out_mps;
    // valid when is_hub:
    uint8_t  hub_int_ep;      // status-change endpoint address (0x8x)
    uint16_t hub_int_mps;
    uint8_t  hub_int_interval;
} usb_cfg_info_t;

// Walk a full configuration descriptor (config + interface + endpoint TLVs).
// Returns false on malformed/truncated input.
bool usb_parse_config(const uint8_t *d, uint16_t len, usb_cfg_info_t *out);
