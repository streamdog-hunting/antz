#include "garmin_usb_decoder.h"

#include <string.h>

namespace antz {

    namespace {

        uint16_t u16(const uint8_t* p) {
            return static_cast<uint16_t>(p[0] | (p[1] << 8));
        }

        uint32_t u32(const uint8_t* p) {
            return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                   (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
        }

        float f32(const uint8_t* p) {
            const uint32_t bits = u32(p);
            float out;
            memcpy(&out, &bits, sizeof out);
            return out;
        }

        double f64(const uint8_t* p) {
            const uint64_t bits = static_cast<uint64_t>(u32(p)) |
                                  (static_cast<uint64_t>(u32(p + 4)) << 32);
            double out;
            memcpy(&out, &bits, sizeof out);
            return out;
        }

        bool layer_valid(const uint8_t layer) {
            return layer == GARMIN_USB_LAYER_PROTOCOL || layer == GARMIN_USB_LAYER_APPLICATION;
        }

        constexpr double PI = 3.14159265358979323846;

        // Fixed part of the packets, from docs/protocol.md.
        constexpr size_t PVT_SIZE = 64;
        constexpr size_t DOG_MIN_SIZE = 40;
        constexpr size_t DOG_NAME_AT = 31;

        bool model_confirmed(const uint16_t product_id) {
            return product_id == GARMIN_USB_PRODUCT_ASTRO_320 ||
                   product_id == GARMIN_USB_PRODUCT_ALPHA_100;
        }

        // Confirmed against both handhelds' screens (status-test and
        // dogtype-test reports). Anything else, including the other model's
        // codes, is Unknown with the raw byte kept by the caller.
        antz_garmin_usb_dog_status_t dog_status(const uint16_t product_id, const uint8_t raw) {
            uint8_t code;
            if (product_id == GARMIN_USB_PRODUCT_ASTRO_320 && raw >= 1 && raw <= 4) {
                code = raw;
            } else if (product_id == GARMIN_USB_PRODUCT_ALPHA_100 && raw >= 33 && raw <= 36) {
                code = static_cast<uint8_t>(raw - 32);
            } else {
                return GARMIN_USB_DOG_STATUS_UNKNOWN;
            }
            switch (code) {
                case 1: return GARMIN_USB_DOG_STATUS_SITTING;
                case 2: return GARMIN_USB_DOG_STATUS_MOVING;
                case 3: return GARMIN_USB_DOG_STATUS_TREED;
                default: return GARMIN_USB_DOG_STATUS_POINTED;
            }
        }

        int header_at(const uint8_t* raw, antz_garmin_usb_header_t* out) {
            const uint8_t layer = raw[0];
            const uint32_t size = u32(raw + 8);
            if (!layer_valid(layer) || size > GARMIN_USB_MAX_PACKET_SIZE) return 1;
            out->layer = layer;
            out->pid = u16(raw + 4);
            out->size = size;
            return 0;
        }
    }

    double garmin_usb_semicircles_to_degrees(const int32_t semicircles) {
        return static_cast<double>(semicircles) * 180.0 / 2147483648.0;
    }

    int garmin_usb_decode_header(const uint8_t* raw, const size_t len, antz_garmin_usb_header_t* out) {
        if (!raw || !out || len < GARMIN_USB_FRAME_HEADER) return 1;
        return header_at(raw, out);
    }

    void garmin_usb_framer_init(antz_garmin_usb_framer_t* framer) {
        if (!framer) return;
        framer->have = 0;
        framer->skip = 0;
        framer->failed = 0;
    }

    int garmin_usb_framer_feed(antz_garmin_usb_framer_t* framer, const uint8_t* chunk, size_t len,
                               const antz_garmin_usb_packet_cb cb, void* ctx) {
        if (!framer || !chunk || !cb) return 1;
        if (framer->failed) return 1;

        while (len > 0) {
            // The payload of an oversize packet: drop it, byte for byte.
            if (framer->skip > 0) {
                const size_t n = len < framer->skip ? len : framer->skip;
                framer->skip -= static_cast<uint32_t>(n);
                chunk += n;
                len -= n;
                continue;
            }

            // Complete the header first.
            if (framer->have < GARMIN_USB_FRAME_HEADER) {
                const size_t want = GARMIN_USB_FRAME_HEADER - framer->have;
                const size_t n = len < want ? len : want;
                memcpy(framer->buf + framer->have, chunk, n);
                framer->have += static_cast<uint32_t>(n);
                chunk += n;
                len -= n;
                if (framer->have < GARMIN_USB_FRAME_HEADER) break;
            }

            antz_garmin_usb_header_t h;
            if (header_at(framer->buf, &h) != 0) {
                framer->failed = 1;
                return 1;
            }

            if (h.size > GARMIN_USB_FRAMER_BUFFER) {
                // Too large to hold: report the packet without its payload
                // and skip the bytes that follow.
                antz_garmin_usb_packet_t pk = {h.layer, h.pid, h.size, nullptr};
                cb(&pk, ctx);
                framer->have = 0;
                framer->skip = h.size;
                continue;
            }

            // Complete the payload.
            const size_t have_payload = framer->have - GARMIN_USB_FRAME_HEADER;
            const size_t want = h.size - have_payload;
            const size_t n = len < want ? len : want;
            memcpy(framer->buf + framer->have, chunk, n);
            framer->have += static_cast<uint32_t>(n);
            chunk += n;
            len -= n;
            if (framer->have - GARMIN_USB_FRAME_HEADER < h.size) break;

            antz_garmin_usb_packet_t pk = {h.layer, h.pid, h.size, framer->buf + GARMIN_USB_FRAME_HEADER};
            cb(&pk, ctx);
            framer->have = 0;
        }
        return 0;
    }

    int garmin_usb_decode_session_started(const uint8_t* data, const size_t len, uint32_t* unit_id) {
        if (!data || !unit_id || len < 4) return 1;
        *unit_id = u32(data);
        return 0;
    }

    int garmin_usb_decode_product(const uint8_t* data, const size_t len, antz_garmin_usb_product_t* out) {
        if (!data || !out || len < 4) return 1;
        out->product_id = u16(data);
        out->software_version_x100 = static_cast<int16_t>(u16(data + 2));
        return 0;
    }

    int garmin_usb_decode_pvt(const uint8_t* data, const size_t len, antz_garmin_usb_pvt_t* out) {
        if (!data || !out || len < PVT_SIZE) return 1;
        // alt epe eph epv fix tow lat lon east north up msl leap days
        //   0   4   8  12  16  18  26  34   42    46 50  54   58   60
        out->altitude_m = static_cast<double>(f32(data)) + static_cast<double>(f32(data + 54));
        out->epe_m = f32(data + 4);
        out->fix = u16(data + 16);
        out->latitude_deg = f64(data + 26) * 180.0 / PI;
        out->longitude_deg = f64(data + 34) * 180.0 / PI;
        return 0;
    }

    int garmin_usb_decode_dog(const uint8_t* data, const size_t len, const uint16_t product_id,
                              antz_garmin_usb_dog_t* out) {
        if (!data || !out || len < DOG_MIN_SIZE) return 1;

        antz_garmin_usb_dog_t d;
        memset(&d, 0, sizeof d);

        d.latitude_deg = garmin_usb_semicircles_to_degrees(static_cast<int32_t>(u32(data)));
        d.longitude_deg = garmin_usb_semicircles_to_degrees(static_cast<int32_t>(u32(data + 4)));
        d.altitude_m = f32(data + 8);
        d.garmin_time_s = u32(data + 12);
        d.unix_time_s = GARMIN_USB_EPOCH_UNIX_S + static_cast<int64_t>(d.garmin_time_s);

        d.status_raw = data[16];
        d.status = dog_status(product_id, d.status_raw);
        d.flags = data[17];

        d.id_bytes[0] = data[22];
        d.id_bytes[1] = data[23];
        d.battery_level_raw = data[25];
        d.gps_bars = data[26];
        d.comm_bars = data[27];

        if (model_confirmed(product_id)) {
            // Loss and recovery were confirmed on both handhelds.
            d.gps_lost_known = 1;
            d.gps_lost = (d.flags & 0x20) ? 1 : 0;
            d.gps_bars_max = product_id == GARMIN_USB_PRODUCT_ASTRO_320 ? 5 : 3;
            d.track_color_known = 1;
            d.track_color_code = data[24];
        }
        if (product_id == GARMIN_USB_PRODUCT_ASTRO_320) {
            d.configured_id_known = 1;
            d.configured_id = u16(data + 22);
        }

        // The name runs from byte 31 to its own NUL, or to the end of the
        // packet when there is none, and never past what the field holds.
        size_t n = 0;
        while (DOG_NAME_AT + n < len && n < GARMIN_USB_DOG_NAME_MAX && data[DOG_NAME_AT + n] != 0) ++n;
        memcpy(d.name, data + DOG_NAME_AT, n);
        d.name[n] = '\0';

        *out = d;
        return 0;
    }
}
