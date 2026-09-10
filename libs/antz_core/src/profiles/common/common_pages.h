//
// ANT+ Common Data Pages (D00001198) — the two that carry identity.
//
// Common to every ANT+ device profile rather than to one, which is why they
// sit beside the profiles instead of inside `tracker/`. A heart-rate strap
// and a Garmin Alpha answer Page 80 and Page 81 the same way.
//
// Both are required transmissions and both are requestable, so a display can
// ask for them rather than waiting out a broadcast rotation. That matters for
// identity: the pair is what names a device to something outside ANT+, and
// the Streamdog platform's ADR 0070 uses `"<manufacturer_id>:<serial>"` from
// exactly these two pages as a handheld's foreign identity. The 32-bit serial
// survives battery changes and re-pairings where the u16 ANT device number
// does not.
//
// Held to packages/platform-messages/spec/wire/ant/vectors/ in the
// streamdog-platform repository.
//

#pragma once

#include <stdint.h>

typedef enum {
    ANTZ_COMMON_PAGE_MANUFACTURER = 0x50, // Page 80, manufacturer identification
    ANTZ_COMMON_PAGE_PRODUCT      = 0x51, // Page 81, product information
} antz_common_page_e;

// Page 80. The registry triple off the air: ANT+ manufacturer id, the
// maker's own model number, and a hardware revision.
typedef struct {
    uint8_t  hw_revision;
    uint16_t manufacturer_id;
    uint16_t model_number;
} antz_common_manufacturer_t;

// Page 81.
//
// `serial_valid` is false for the all-ones sentinel, which means the device
// declines to give one. Zero is not that sentinel and is not a refusal, so a
// caller must read the flag rather than test the value — a serial read as 0
// would make every such device the same device.
//
// `sw_revision_supplemental_valid` is false for 0xFF, which is the same
// arrangement one field down: a device with no supplemental revision reports
// only the main one, and 255 is not a revision number.
typedef struct {
    uint8_t  sw_revision_main;
    uint8_t  sw_revision_supplemental;
    bool     sw_revision_supplemental_valid;
    uint32_t serial;
    bool     serial_valid;
} antz_common_product_t;
