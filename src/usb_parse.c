#include "usb_parse.h"
#include <string.h>

enum { DT_CONFIG = 2, DT_INTERFACE = 4, DT_ENDPOINT = 5 };

bool usb_parse_config(const uint8_t *d, uint16_t len, usb_cfg_info_t *out) {
    memset(out, 0, sizeof *out);
    if (len < 9 || d[0] < 9 || d[1] != DT_CONFIG) return false;
    out->config_value = d[5];

    uint8_t cur_itf = 0;
    bool in_msc_itf = false, in_hub_itf = false;
    uint16_t off = 0;
    while (off < len) {
        uint8_t dlen = d[off];
        if (dlen < 2 || (uint32_t)off + dlen > len) return false;
        const uint8_t *p = d + off;
        switch (p[1]) {
        case DT_INTERFACE:
            if (dlen < 9) return false;
            if (p[3] != 0) {
                // Non-zero bAlternateSetting: ignore this interface and its endpoints
                in_msc_itf = false;
                in_hub_itf = false;
                break;
            }
            cur_itf = p[2];
            in_msc_itf = (p[5] == USB_CLASS_MSC && p[6] == MSC_SUBCLASS_SCSI &&
                          p[7] == MSC_PROTO_BOT && !out->is_msc);
            in_hub_itf = (p[5] == USB_CLASS_HUB && !out->is_hub);
            if (in_msc_itf) { out->is_msc = true; out->msc_itf = cur_itf; }
            if (in_hub_itf) out->is_hub = true;
            break;
        case DT_ENDPOINT: {
            if (dlen < 7) return false;
            uint8_t  addr = p[2], attr = p[3] & 0x03;
            uint16_t mps  = (uint16_t)(p[4] | (p[5] << 8));
            if (in_msc_itf && attr == 0x02) {           // bulk
                if (addr & 0x80) { out->bulk_in = addr;  out->bulk_in_mps = mps; }
                else             { out->bulk_out = addr; out->bulk_out_mps = mps; }
            }
            if (in_hub_itf && attr == 0x03 && (addr & 0x80)) {  // interrupt IN
                out->hub_int_ep = addr;
                out->hub_int_mps = mps;
                out->hub_int_interval = p[6];
            }
            break;
        }
        default:
            break;
        }
        off += dlen;
    }
    if (out->is_msc && (!out->bulk_in || !out->bulk_out)) return false;
    if (out->is_hub && !out->hub_int_ep) return false;
    return out->is_msc || out->is_hub;
}
