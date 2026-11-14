//
// A Garmin handheld's USB session, over a transport the host provides.
//
// What it does: sends Start_Session, waits for Session_Started, asks for the
// product and starts the position stream, then reads the two endpoints and
// hands every decoded handheld, position and dog packet to the host. What it
// does not do: open a port, claim an interface or know libusb, grmnusb or an
// operating system. The host gives it a transport of two functions, one that
// writes a whole packet and one that reads an endpoint, and that is the
// whole of the cable as far as this file is concerned. A fake transport
// tests it with no handheld, which is what the tests do.
//
// The sequence and the two endpoints are from the tested Linux logger in
// streamdog-hunting/astro-usb (astro_usb_linux.py):
//
//   - The handheld announces bulk data on the interrupt endpoint with a
//     protocol-layer Data_Available packet. The session then reads the bulk
//     endpoint, which carries its own framing, until it stops giving data
//     at a packet boundary, and returns to the interrupt endpoint. The two
//     endpoints each have their own frame reader, so a packet cut in two on
//     one of them is not mixed with the other's bytes.
//   - A bulk read that returns nothing while a packet is half read is an
//     error, not an end of data: «Bulk transfer ended in a partial Garmin
//     packet». A read that returns nothing at a packet boundary returns to
//     the interrupt endpoint.
//   - The session keeps no clock. How long without a packet is too long is
//     the host's: the logger gave up after 15 seconds, and a host that polls
//     with a timeout counts the idle polls itself.
//
// Platform-agnostic, no allocation, no I/O.
//

#pragma once

#include "garmin_usb_decoder.h"

namespace antz {

    // Packet ids and commands the session sends, from the Garmin Device
    // Interface (the USB protocol layer and the application layer).
    constexpr uint16_t GARMIN_USB_PID_DATA_AVAILABLE = 2;
    constexpr uint16_t GARMIN_USB_PID_START_SESSION  = 5;
    constexpr uint16_t GARMIN_USB_PID_COMMAND_DATA   = 10;
    constexpr uint16_t GARMIN_USB_PID_PRODUCT_RQST   = 254;
    constexpr uint16_t GARMIN_USB_CMND_START_PVT     = 49;
    constexpr uint16_t GARMIN_USB_CMND_STOP_PVT      = 50;

    enum antz_garmin_usb_endpoint_t {
        GARMIN_USB_ENDPOINT_INTERRUPT = 0,   // notifications, small reads
        GARMIN_USB_ENDPOINT_BULK      = 1,   // the data a notification announced
    };

    struct antz_garmin_usb_transport_t {
        // Writes one whole packet, header and payload. Returns 0 on success.
        // The host adds the zero length packet a USB write needs when the
        // packet fills a whole number of the endpoint's max packets.
        int (*write)(void* ctx, const uint8_t* data, size_t len);

        // Reads from an endpoint into `buf` (capacity `cap`), waiting at most
        // `timeout_ms`. Returns the bytes read; 0 when nothing came (a
        // timeout, or a zero length packet); a negative number on an error
        // that ends the session (the cable was pulled).
        int (*read)(void* ctx, antz_garmin_usb_endpoint_t endpoint, uint8_t* buf, size_t cap,
                    uint32_t timeout_ms);

        void* ctx;
    };

    struct antz_garmin_usb_events_t {
        // Each may be null. The pointer is valid during the call only.
        void (*on_session)(void* ctx, uint32_t unit_id);
        void (*on_product)(void* ctx, const antz_garmin_usb_product_t* product);
        void (*on_pvt)(void* ctx, const antz_garmin_usb_pvt_t* pvt);
        void (*on_dog)(void* ctx, const antz_garmin_usb_dog_t* dog);
        void* ctx;
    };

    enum antz_garmin_usb_poll_t {
        GARMIN_USB_POLL_OK = 0,              // something was read and handled
        GARMIN_USB_POLL_IDLE,                // nothing arrived within the timeout
        GARMIN_USB_POLL_TRANSPORT_ERROR,     // a read failed: the session is over
        GARMIN_USB_POLL_FRAME_ERROR,         // an invalid header: the session is over
        GARMIN_USB_POLL_PARTIAL_PACKET,      // bulk ended inside a packet: the session is over
    };

    struct antz_garmin_usb_session_t {
        antz_garmin_usb_transport_t transport;
        antz_garmin_usb_events_t    events;
        antz_garmin_usb_framer_t    interrupt_frames;
        antz_garmin_usb_framer_t    bulk_frames;
        uint8_t  reading_bulk;
        uint8_t  started;        // Start_Session was sent
        uint8_t  streaming;      // Session_Started arrived and the product and position requests were sent
        uint8_t  failed;         // a poll returned an error; no more input is read
        uint16_t product_id;     // from Product_Data, 0 until it arrives
    };

    // Returns nonzero when `transport` has no write or no read.
    int garmin_usb_session_init(antz_garmin_usb_session_t* session,
                                const antz_garmin_usb_transport_t* transport,
                                const antz_garmin_usb_events_t* events);

    // Sends Start_Session. Returns nonzero if the write fails or the session
    // was already started.
    int garmin_usb_session_start(antz_garmin_usb_session_t* session);

    // One read of the endpoint the session is on, then handle what came.
    // Returns what happened; after any error the session reads no more.
    antz_garmin_usb_poll_t garmin_usb_session_poll(antz_garmin_usb_session_t* session,
                                                   uint32_t timeout_ms);

    // Sends Stop_Pvt_Data if the position stream was started. Returns nonzero
    // if the write fails. Safe to call at any time, and when nothing was started.
    int garmin_usb_session_stop(antz_garmin_usb_session_t* session);
}
