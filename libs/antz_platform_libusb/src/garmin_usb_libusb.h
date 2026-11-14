//
// The libusb transport for a Garmin Astro / Alpha handheld, USB id 091e:0003.
//
// Gives the Garmin USB session (profiles/garmin_usb/garmin_usb_session.h) its
// two functions, write a packet and read an endpoint, over libusb. The libusb
// calls themselves sit behind a small table (`antz_libusb_ops_t`), so the logic
// here, which device to open, which endpoints to use, what to undo when a step
// fails, and how a write and a read behave, is tested with a fake table and no
// handheld. `garmin_usb_libusb_ops.cpp` binds the table to the real libusb 1.0.
//
// The sequence and the details are from the tested Linux logger in
// streamdog-hunting/astro-usb (astro_usb_linux.py):
//
//   - Exactly one Garmin device must be present. None, or more than one, is
//     an error: the host cannot tell which handheld the hunter meant.
//   - The interface is 0, alternate setting 0. Its endpoints are one interrupt
//     in, one bulk in and one bulk out, found by type and direction, never by a
//     fixed address.
//   - A kernel driver active on the interface is detached first and attached
//     again on close, so the handheld is left as it was found.
//   - A write sends the whole packet on the bulk out endpoint. When the packet
//     fills a whole number of the endpoint's max packets, a zero length packet
//     follows, or the handheld waits for more.
//   - A read from the interrupt endpoint asks for at most one max packet. A
//     read from the bulk endpoint asks for what the caller has room for.
//   - A read that times out is not an error: it returns 0, which the session
//     reads as «nothing came». Any other failure returns a negative number.
//   - A step that fails undoes the steps before it, in reverse. After a failed
//     open nothing is claimed, detached or open.
//
// No operating system and no libusb header here. Platform-independent logic.
//

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "profiles/garmin_usb/garmin_usb_session.h"

namespace antz {

    constexpr uint16_t GARMIN_USB_VENDOR_ID  = 0x091E;
    constexpr uint16_t GARMIN_USB_PRODUCT_ID = 0x0003;
    constexpr int      GARMIN_USB_INTERFACE  = 0;

    // What an ops call returns, besides a byte count or zero for success.
    constexpr int ANTZ_LIBUSB_TIMEOUT = -2;   // the transfer timed out
    constexpr int ANTZ_LIBUSB_ERROR   = -1;   // anything else that went wrong

    // The timeout of a write, in milliseconds. A read takes the session's.
    constexpr uint32_t GARMIN_USB_WRITE_TIMEOUT_MS = 1500;

    struct antz_libusb_endpoint_t {
        uint8_t  address;       // the endpoint address, direction bit included
        uint16_t max_packet;    // wMaxPacketSize
    };

    // The libusb calls the transport makes, each returning >= 0 on success.
    // Transfers return the bytes moved, or ANTZ_LIBUSB_TIMEOUT or a negative
    // ANTZ_LIBUSB_ERROR.
    struct antz_libusb_ops_t {
        // How many devices with this vendor and product id are present, or
        // negative if they could not be listed.
        int (*count_devices)(void* ctx, uint16_t vendor_id, uint16_t product_id);
        // Opens the one such device.
        int (*open)(void* ctx, uint16_t vendor_id, uint16_t product_id);
        // The three endpoints of interface 0, alternate setting 0. Nonzero if
        // one of them is missing.
        int (*endpoints)(void* ctx, antz_libusb_endpoint_t* interrupt_in,
                         antz_libusb_endpoint_t* bulk_in, antz_libusb_endpoint_t* bulk_out);
        // 1 if a kernel driver holds the interface, 0 if not, negative on error.
        int (*kernel_driver_active)(void* ctx, int interface_number);
        int (*detach_kernel_driver)(void* ctx, int interface_number);
        int (*attach_kernel_driver)(void* ctx, int interface_number);
        int (*claim_interface)(void* ctx, int interface_number);
        int (*release_interface)(void* ctx, int interface_number);
        int (*interrupt_in)(void* ctx, uint8_t endpoint, uint8_t* buf, size_t len, uint32_t timeout_ms);
        int (*bulk_in)(void* ctx, uint8_t endpoint, uint8_t* buf, size_t len, uint32_t timeout_ms);
        int (*bulk_out)(void* ctx, uint8_t endpoint, const uint8_t* buf, size_t len, uint32_t timeout_ms);
        void (*close)(void* ctx);
        void* ctx;
    };

    enum antz_garmin_usb_libusb_status_t {
        GARMIN_USB_LIBUSB_OK = 0,
        GARMIN_USB_LIBUSB_NO_DEVICE,        // none present
        GARMIN_USB_LIBUSB_MORE_THAN_ONE,    // two or more present
        GARMIN_USB_LIBUSB_LIST_FAILED,      // the devices could not be listed
        GARMIN_USB_LIBUSB_OPEN_FAILED,
        GARMIN_USB_LIBUSB_NO_ENDPOINTS,     // one of the three endpoints is missing
        GARMIN_USB_LIBUSB_DETACH_FAILED,
        GARMIN_USB_LIBUSB_CLAIM_FAILED,
        GARMIN_USB_LIBUSB_BAD_ARGUMENT,     // a null pointer or an ops table with a null call
    };

    struct antz_garmin_usb_libusb_t {
        antz_libusb_ops_t      ops;
        antz_libusb_endpoint_t interrupt_in;
        antz_libusb_endpoint_t bulk_in;
        antz_libusb_endpoint_t bulk_out;
        uint8_t opened;       // the device is open
        uint8_t detached;     // this transport detached a kernel driver and must attach it again
        uint8_t claimed;      // the interface is claimed
    };

    // Finds the one handheld, opens it and claims interface 0. On any failure
    // it undoes what it did and returns why; the struct is then closed.
    antz_garmin_usb_libusb_status_t garmin_usb_libusb_open(antz_garmin_usb_libusb_t* transport,
                                                            const antz_libusb_ops_t* ops);

    // Releases the interface, attaches the kernel driver again if this
    // transport detached it, and closes the device, in that order. Safe to
    // call twice and on a transport that never opened.
    void garmin_usb_libusb_close(antz_garmin_usb_libusb_t* transport);

    // The session's transport over this one. Valid while `transport` is.
    antz_garmin_usb_transport_t garmin_usb_libusb_transport(antz_garmin_usb_libusb_t* transport);

    // A short sentence for a status, for a host's error message. Never null.
    const char* garmin_usb_libusb_status_text(antz_garmin_usb_libusb_status_t status);

    // The ops table over the real libusb 1.0, in garmin_usb_libusb_ops.cpp, built
    // only where libusb is found. The returned table owns a libusb context and
    // one device handle; close() of the table releases both.
    // Returns false if libusb cannot be initialised.
    bool garmin_usb_libusb_real_ops(antz_libusb_ops_t* out);
}
