#include "common_decoder.h"

#include "data/antz_bytes.h"

namespace antz {

    namespace {
        // A field a device declines to supply is all-ones, per D00001198.
        constexpr uint8_t  INVALID_U8  = 0xFF;
        constexpr uint32_t INVALID_U32 = 0xFFFFFFFFu;
    }

    int common_decode_manufacturer(const uint8_t* raw, const uint8_t len,
                                   antz_common_manufacturer_t* out) {
        if (!raw || !out || len < 8) return -1;
        if (raw[0] != ANTZ_COMMON_PAGE_MANUFACTURER) return -1;

        // Bytes 1-2 are reserved and transmitted as 0xFF. Not read: a device
        // that puts something there is not describing itself, and a decoder
        // that surfaced it would invite a consumer to depend on it.
        out->hw_revision     = raw[3];
        out->manufacturer_id = bytes_to_uint16(&raw[4]);
        out->model_number    = bytes_to_uint16(&raw[6]);
        return 0;
    }

    int common_decode_product(const uint8_t* raw, const uint8_t len,
                              antz_common_product_t* out) {
        if (!raw || !out || len < 8) return -1;
        if (raw[0] != ANTZ_COMMON_PAGE_PRODUCT) return -1;

        out->sw_revision_supplemental = raw[2];
        out->sw_revision_supplemental_valid = raw[2] != INVALID_U8;
        out->sw_revision_main = raw[3];

        const uint32_t serial = bytes_to_uint32(&raw[4]);
        out->serial = serial;
        // Zero is a real serial and all-ones is the refusal. Testing the
        // value instead of the flag would collapse every device that
        // declines into serial 4294967295, which is worse than none at all:
        // they would look like one device.
        out->serial_valid = serial != INVALID_U32;
        return 0;
    }

} // namespace antz
