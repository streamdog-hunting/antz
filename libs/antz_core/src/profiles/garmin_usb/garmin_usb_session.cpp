#include "garmin_usb_session.h"

#include <string.h>

namespace antz {

    namespace {

        // Largest read from either endpoint. The logger read 64 bytes from the
        // interrupt endpoint and 4096 from the bulk one. The session offers
        // the same room on both and the transport reads what its endpoint
        // gives, so a transport clamps an interrupt read to the endpoint's
        // max packet itself. The frame reader joins what the reads cut apart.
        constexpr size_t READ_CAP = 512;

        void put16(uint8_t* p, const uint16_t v) {
            p[0] = static_cast<uint8_t>(v & 0xFF);
            p[1] = static_cast<uint8_t>(v >> 8);
        }

        void put32(uint8_t* p, const uint32_t v) {
            for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>((v >> (8 * i)) & 0xFF);
        }

        // header (layer, 3 reserved, pid, 2 reserved, size) + payload.
        // Returns the packet length.
        size_t build(uint8_t* out, const uint8_t layer, const uint16_t pid,
                     const uint8_t* payload, const uint32_t size) {
            memset(out, 0, GARMIN_USB_FRAME_HEADER);
            out[0] = layer;
            put16(out + 4, pid);
            put32(out + 8, size);
            if (size) memcpy(out + GARMIN_USB_FRAME_HEADER, payload, size);
            return GARMIN_USB_FRAME_HEADER + size;
        }

        int send(antz_garmin_usb_session_t* s, const uint8_t layer, const uint16_t pid,
                 const uint8_t* payload, const uint32_t size) {
            uint8_t packet[GARMIN_USB_FRAME_HEADER + 2];
            const size_t n = build(packet, layer, pid, payload, size);
            return s->transport.write(s->transport.ctx, packet, n);
        }

        int send_command(antz_garmin_usb_session_t* s, const uint16_t command) {
            uint8_t payload[2];
            put16(payload, command);
            return send(s, GARMIN_USB_LAYER_APPLICATION, GARMIN_USB_PID_COMMAND_DATA, payload, 2);
        }

        void on_packet(const antz_garmin_usb_packet_t* pk, void* ctx) {
            antz_garmin_usb_session_t* s = static_cast<antz_garmin_usb_session_t*>(ctx);
            // A packet too large to buffer carries nothing this session reads.
            if (!pk->data) return;

            if (pk->layer == GARMIN_USB_LAYER_PROTOCOL) {
                if (pk->pid == GARMIN_USB_PID_DATA_AVAILABLE) {
                    // The next read is of the bulk endpoint.
                    s->reading_bulk = 1;
                } else if (pk->pid == GARMIN_USB_PID_SESSION_STARTED) {
                    uint32_t unit_id;
                    if (s->streaming || garmin_usb_decode_session_started(pk->data, pk->size, &unit_id) != 0)
                        return;
                    s->streaming = 1;
                    if (s->events.on_session) s->events.on_session(s->events.ctx, unit_id);
                    if (send(s, GARMIN_USB_LAYER_APPLICATION, GARMIN_USB_PID_PRODUCT_RQST, nullptr, 0) != 0 ||
                        send_command(s, GARMIN_USB_CMND_START_PVT) != 0) {
                        s->write_failed = 1;
                    }
                }
                return;
            }

            if (pk->layer != GARMIN_USB_LAYER_APPLICATION) return;

            if (pk->pid == GARMIN_USB_PID_PRODUCT_DATA) {
                antz_garmin_usb_product_t product;
                if (garmin_usb_decode_product(pk->data, pk->size, &product) != 0) return;
                s->product_id = product.product_id;
                if (s->events.on_product) s->events.on_product(s->events.ctx, &product);
            } else if (pk->pid == GARMIN_USB_PID_PVT_DATA) {
                antz_garmin_usb_pvt_t pvt;
                if (garmin_usb_decode_pvt(pk->data, pk->size, &pvt) != 0) return;
                if (s->events.on_pvt) s->events.on_pvt(s->events.ctx, &pvt);
            } else if (pk->pid == GARMIN_USB_PID_DOG_DATA) {
                antz_garmin_usb_dog_t dog;
                if (garmin_usb_decode_dog(pk->data, pk->size, s->product_id, &dog) != 0) return;
                if (s->events.on_dog) s->events.on_dog(s->events.ctx, &dog);
            }
        }

        antz_garmin_usb_poll_t fail(antz_garmin_usb_session_t* s, const antz_garmin_usb_poll_t why) {
            s->failed = 1;
            s->error = static_cast<uint8_t>(why);
            return why;
        }
    }

    int garmin_usb_session_init(antz_garmin_usb_session_t* session,
                                const antz_garmin_usb_transport_t* transport,
                                const antz_garmin_usb_events_t* events) {
        if (!session || !transport || !transport->write || !transport->read) return 1;
        memset(session, 0, sizeof *session);
        session->transport = *transport;
        if (events) session->events = *events;
        garmin_usb_framer_init(&session->interrupt_frames);
        garmin_usb_framer_init(&session->bulk_frames);
        return 0;
    }

    int garmin_usb_session_start(antz_garmin_usb_session_t* session) {
        if (!session || session->started) return 1;
        if (send(session, GARMIN_USB_LAYER_PROTOCOL, GARMIN_USB_PID_START_SESSION, nullptr, 0) != 0) return 1;
        session->started = 1;
        return 0;
    }

    antz_garmin_usb_poll_t garmin_usb_session_poll(antz_garmin_usb_session_t* session,
                                                   const uint32_t timeout_ms) {
        if (!session) return GARMIN_USB_POLL_TRANSPORT_ERROR;
        if (session->failed) return static_cast<antz_garmin_usb_poll_t>(session->error);

        const bool bulk = session->reading_bulk != 0;
        uint8_t buf[READ_CAP];
        const int n = session->transport.read(session->transport.ctx,
                                              bulk ? GARMIN_USB_ENDPOINT_BULK : GARMIN_USB_ENDPOINT_INTERRUPT,
                                              buf, READ_CAP, timeout_ms);
        if (n < 0) return fail(session, GARMIN_USB_POLL_TRANSPORT_ERROR);

        if (n == 0) {
            if (!bulk) return GARMIN_USB_POLL_IDLE;
            // Nothing more on bulk. At a packet boundary that is the end of
            // the burst; inside a packet it is a transfer that stopped short.
            if (session->bulk_frames.have > 0 || session->bulk_frames.skip > 0)
                return fail(session, GARMIN_USB_POLL_PARTIAL_PACKET);
            session->reading_bulk = 0;
            return GARMIN_USB_POLL_IDLE;
        }

        antz_garmin_usb_framer_t* frames = bulk ? &session->bulk_frames : &session->interrupt_frames;
        const int framed = garmin_usb_framer_feed(frames, buf, static_cast<size_t>(n), on_packet, session);
        if (framed != 0) return fail(session, GARMIN_USB_POLL_FRAME_ERROR);
        // A write the handshake needed failed: the cable is gone.
        if (session->write_failed) return fail(session, GARMIN_USB_POLL_TRANSPORT_ERROR);
        return GARMIN_USB_POLL_OK;
    }

    int garmin_usb_session_stop(antz_garmin_usb_session_t* session) {
        if (!session || !session->streaming) return 0;
        session->streaming = 0;
        return send_command(session, GARMIN_USB_CMND_STOP_PVT);
    }
}
