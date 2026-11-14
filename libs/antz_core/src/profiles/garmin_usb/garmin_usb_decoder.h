//
// Garmin Astro / Alpha handheld over USB: packet framing and decoding.
//
// The handheld speaks the Garmin Device Interface over its USB cable
// (VID 091e, PID 0003). Besides its own position it sends a 100-byte
// "dog" packet (pid 3078, undocumented by Garmin) for every collar, every
// five seconds. This file decodes that traffic. It reads bytes and returns
// values; it opens no port, keeps no identity and decides nothing about
// which collar is which. The cable is the host's.
//
// Every field here is held to what Arnfinn Grambo measured against the two
// handhelds' screens (streamdog-hunting/astro-usb, docs/protocol.md and the
// test reports it links), and to nothing more:
//
//   - Status codes are per model. Astro 320 (product 1339) sends 1..4,
//     Alpha 100 (product 1562) sends 33..36. Any other pair of model and
//     code is Unknown with the raw byte kept. ANT's status names and flag
//     masks are not reused: they differ (ANT Sitting is 0, USB Sitting is
//     1 or 33), and copying them gives wrong values.
//   - A new source time is not a new fix. A collar that has lost GPS keeps
//     sending the same coordinates with an advancing time and a status of
//     "moving". `gps_lost` (byte 17, bit 0x20) says so, for the two models
//     where loss and recovery were confirmed, and the position is still
//     returned: whether it is a fix is the caller's decision.
//   - The battery is a raw level, not a percentage. Two of four bars on the
//     screen read 40 on the Astro and 2 on the Alpha. The scale is unknown.
//   - Bytes 22..23 are the collar's configured radio id, which is editable
//     and not unique, and byte 30 and the bytes from 40 on are not an
//     identity. None of them is a key. They are returned as evidence only.
//   - A packet shorter than its fixed part, or whose frame is invalid, is
//     refused, not guessed at.
//
// Platform-agnostic, no allocation, no I/O.
//

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace antz {

    // Packet layers of the Garmin USB frame header.
    constexpr uint8_t GARMIN_USB_LAYER_PROTOCOL    = 0;
    constexpr uint8_t GARMIN_USB_LAYER_APPLICATION = 20;

    // Packet ids this decoder reads.
    constexpr uint16_t GARMIN_USB_PID_SESSION_STARTED = 6;
    constexpr uint16_t GARMIN_USB_PID_PVT_DATA        = 51;
    constexpr uint16_t GARMIN_USB_PID_PRODUCT_DATA    = 255;
    constexpr uint16_t GARMIN_USB_PID_DOG_DATA        = 3078;

    // Products whose dog packets are confirmed against their screens.
    constexpr uint16_t GARMIN_USB_PRODUCT_ASTRO_320 = 1339;
    constexpr uint16_t GARMIN_USB_PRODUCT_ALPHA_100 = 1562;

    // Garmin time counts seconds from 1989-12-31 00:00:00 UTC.
    constexpr int64_t GARMIN_USB_EPOCH_UNIX_S = 631065600;

    // Longest name a dog packet can hold: it starts at byte 31 of 100.
    constexpr size_t GARMIN_USB_DOG_NAME_MAX = 69;

    // Frame: 12 byte header (layer u8, 3 reserved, pid u16, 2 reserved,
    // size u32, little endian) followed by `size` bytes. The largest packet
    // the framer will buffer. A larger one is reported without its payload
    // and its bytes are skipped.
    constexpr size_t GARMIN_USB_FRAME_HEADER = 12;
    constexpr size_t GARMIN_USB_FRAMER_BUFFER = 256;
    // The largest size the header may claim; more is a corrupt stream.
    constexpr uint32_t GARMIN_USB_MAX_PACKET_SIZE = 65536;

    struct antz_garmin_usb_header_t {
        uint8_t  layer;
        uint16_t pid;
        uint32_t size;
    };

    struct antz_garmin_usb_packet_t {
        uint8_t        layer;
        uint16_t       pid;
        uint32_t       size;
        // The payload, valid during the callback only. Null when the
        // payload was larger than the framer's buffer and was skipped.
        const uint8_t* data;
    };

    typedef void (*antz_garmin_usb_packet_cb)(const antz_garmin_usb_packet_t* packet, void* ctx);

    // A byte stream in, whole packets out. Holds partial input between
    // calls, so a read that ends mid packet is completed by the next one.
    struct antz_garmin_usb_framer_t {
        uint8_t  buf[GARMIN_USB_FRAME_HEADER + GARMIN_USB_FRAMER_BUFFER];
        uint32_t have;      // bytes of the current frame held in buf
        uint32_t skip;      // payload bytes of an oversize packet still to drop
        uint8_t  failed;    // set by an invalid header; sticky until init
    };

    void garmin_usb_framer_init(antz_garmin_usb_framer_t* framer);

    // Feeds `len` bytes and calls `cb` once per whole packet, in order.
    // Returns 0, or nonzero once a header is invalid (a layer other than 0
    // or 20, or a size above GARMIN_USB_MAX_PACKET_SIZE). The framer then
    // refuses all further input until it is re-initialised: after a corrupt
    // header there is no way to find the next frame.
    int garmin_usb_framer_feed(antz_garmin_usb_framer_t* framer, const uint8_t* chunk, size_t len,
                               antz_garmin_usb_packet_cb cb, void* ctx);

    // The 12 byte header at the start of `raw`. Nonzero on null or short
    // input, or an invalid layer or size.
    int garmin_usb_decode_header(const uint8_t* raw, size_t len, antz_garmin_usb_header_t* out);

    // Session_Started: the handheld's unit id, which identifies the handheld
    // and no collar.
    int garmin_usb_decode_session_started(const uint8_t* data, size_t len, uint32_t* unit_id);

    struct antz_garmin_usb_product_t {
        uint16_t product_id;
        int16_t  software_version_x100;   // 460 is 4.60
    };

    // Product_Data: product id and software version. The description
    // strings are not read.
    int garmin_usb_decode_product(const uint8_t* data, size_t len, antz_garmin_usb_product_t* out);

    struct antz_garmin_usb_pvt_t {
        double latitude_deg;
        double longitude_deg;
        double altitude_m;        // above mean sea level
        uint16_t fix;             // the handheld's own fix type, as sent
        float  epe_m;
    };

    // Pvt_Data (pid 51): the handheld's own position.
    int garmin_usb_decode_pvt(const uint8_t* data, size_t len, antz_garmin_usb_pvt_t* out);

    enum antz_garmin_usb_dog_status_t {
        GARMIN_USB_DOG_STATUS_UNKNOWN = 0,
        GARMIN_USB_DOG_STATUS_SITTING,
        GARMIN_USB_DOG_STATUS_MOVING,
        GARMIN_USB_DOG_STATUS_TREED,
        GARMIN_USB_DOG_STATUS_POINTED,    // «Stand (on point)»
    };

    struct antz_garmin_usb_dog_t {
        double   latitude_deg;
        double   longitude_deg;
        float    altitude_m;              // probable, not confirmed
        uint32_t garmin_time_s;           // as sent
        int64_t  unix_time_s;             // garmin_time_s on the unix epoch

        uint8_t  status_raw;              // byte 16, always kept
        antz_garmin_usb_dog_status_t status;
        uint8_t  flags;                   // byte 17, always kept

        // True only where loss and recovery were confirmed (both models
        // above). False for any other product, whatever the bit says.
        uint8_t  gps_lost_known;
        uint8_t  gps_lost;

        uint8_t  gps_bars;                // byte 26, model dependent scale
        uint8_t  gps_bars_max;            // 5 on the Astro 320, 3 on the Alpha 100; 0 when unknown
        uint8_t  comm_bars;               // byte 27, 0..5
        uint8_t  battery_level_raw;       // byte 25, not a percentage
        uint8_t  track_color_code;        // byte 24, only for the two models; 0 otherwise
        uint8_t  track_color_known;

        // Evidence only, never a key.
        uint8_t  id_bytes[2];             // bytes 22..23, raw
        uint8_t  configured_id_known;     // Astro 320 only
        uint16_t configured_id;

        char     name[GARMIN_USB_DOG_NAME_MAX + 1];   // byte 31 to the first NUL, NUL terminated
    };

    // Dog_Data (pid 3078). `product_id` is the handheld's, from Product_Data,
    // or 0 when not known yet. Nonzero on null input or fewer than 40 bytes.
    int garmin_usb_decode_dog(const uint8_t* data, size_t len, uint16_t product_id,
                              antz_garmin_usb_dog_t* out);

    // semicircles / 2^31 * 180
    double garmin_usb_semicircles_to_degrees(int32_t semicircles);
}
