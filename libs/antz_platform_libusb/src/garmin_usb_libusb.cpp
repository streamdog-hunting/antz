#include "garmin_usb_libusb.h"

#include <string.h>

namespace antz {

    namespace {

        bool ops_complete(const antz_libusb_ops_t* o) {
            return o && o->count_devices && o->open && o->endpoints && o->kernel_driver_active &&
                   o->detach_kernel_driver && o->attach_kernel_driver && o->claim_interface &&
                   o->release_interface && o->interrupt_in && o->bulk_in && o->bulk_out && o->close;
        }

        // A transfer result as the session reads it: bytes, 0 for a timeout,
        // negative for anything else.
        int read_result(const int r) {
            if (r == ANTZ_LIBUSB_TIMEOUT) return 0;
            if (r < 0) return -1;
            return r;
        }

        int transport_write(void* ctx, const uint8_t* data, const size_t len) {
            antz_garmin_usb_libusb_t* t = static_cast<antz_garmin_usb_libusb_t*>(ctx);
            if (!t || !data || !t->opened) return 1;

            const int sent = t->ops.bulk_out(t->ops.ctx, t->bulk_out.address, data, len,
                                             GARMIN_USB_WRITE_TIMEOUT_MS);
            if (sent < 0 || static_cast<size_t>(sent) != len) return 1;

            // A packet that fills a whole number of max packets is not known to
            // have ended until a zero length packet follows.
            if (len > 0 && t->bulk_out.max_packet > 0 && len % t->bulk_out.max_packet == 0) {
                if (t->ops.bulk_out(t->ops.ctx, t->bulk_out.address, nullptr, 0,
                                    GARMIN_USB_WRITE_TIMEOUT_MS) < 0)
                    return 1;
            }
            return 0;
        }

        int transport_read(void* ctx, const antz_garmin_usb_endpoint_t endpoint, uint8_t* buf,
                           const size_t cap, const uint32_t timeout_ms) {
            antz_garmin_usb_libusb_t* t = static_cast<antz_garmin_usb_libusb_t*>(ctx);
            if (!t || !buf || !t->opened) return -1;

            if (endpoint == GARMIN_USB_ENDPOINT_INTERRUPT) {
                // One max packet at most, as the logger read.
                size_t want = cap;
                if (t->interrupt_in.max_packet > 0 && want > t->interrupt_in.max_packet)
                    want = t->interrupt_in.max_packet;
                return read_result(t->ops.interrupt_in(t->ops.ctx, t->interrupt_in.address, buf, want,
                                                       timeout_ms));
            }
            return read_result(t->ops.bulk_in(t->ops.ctx, t->bulk_in.address, buf, cap, timeout_ms));
        }
    }

    antz_garmin_usb_libusb_status_t garmin_usb_libusb_open(antz_garmin_usb_libusb_t* transport,
                                                            const antz_libusb_ops_t* ops) {
        if (!transport) return GARMIN_USB_LIBUSB_BAD_ARGUMENT;
        memset(transport, 0, sizeof *transport);
        if (!ops_complete(ops)) return GARMIN_USB_LIBUSB_BAD_ARGUMENT;
        transport->ops = *ops;
        const antz_libusb_ops_t& o = transport->ops;

        const int count = o.count_devices(o.ctx, GARMIN_USB_VENDOR_ID, GARMIN_USB_PRODUCT_ID);
        if (count < 0) return GARMIN_USB_LIBUSB_LIST_FAILED;
        if (count == 0) return GARMIN_USB_LIBUSB_NO_DEVICE;
        if (count > 1) return GARMIN_USB_LIBUSB_MORE_THAN_ONE;

        if (o.open(o.ctx, GARMIN_USB_VENDOR_ID, GARMIN_USB_PRODUCT_ID) < 0)
            return GARMIN_USB_LIBUSB_OPEN_FAILED;
        transport->opened = 1;

        if (o.endpoints(o.ctx, &transport->interrupt_in, &transport->bulk_in, &transport->bulk_out) != 0) {
            garmin_usb_libusb_close(transport);
            return GARMIN_USB_LIBUSB_NO_ENDPOINTS;
        }

        // A kernel driver holding the interface is detached now and given back
        // on close, so the handheld is left as it was found.
        const int active = o.kernel_driver_active(o.ctx, GARMIN_USB_INTERFACE);
        if (active > 0) {
            if (o.detach_kernel_driver(o.ctx, GARMIN_USB_INTERFACE) < 0) {
                garmin_usb_libusb_close(transport);
                return GARMIN_USB_LIBUSB_DETACH_FAILED;
            }
            transport->detached = 1;
        }

        if (o.claim_interface(o.ctx, GARMIN_USB_INTERFACE) < 0) {
            garmin_usb_libusb_close(transport);
            return GARMIN_USB_LIBUSB_CLAIM_FAILED;
        }
        transport->claimed = 1;
        return GARMIN_USB_LIBUSB_OK;
    }

    void garmin_usb_libusb_close(antz_garmin_usb_libusb_t* transport) {
        if (!transport || !transport->opened) return;
        const antz_libusb_ops_t& o = transport->ops;
        if (transport->claimed) {
            o.release_interface(o.ctx, GARMIN_USB_INTERFACE);
            transport->claimed = 0;
        }
        if (transport->detached) {
            o.attach_kernel_driver(o.ctx, GARMIN_USB_INTERFACE);
            transport->detached = 0;
        }
        o.close(o.ctx);
        transport->opened = 0;
    }

    antz_garmin_usb_transport_t garmin_usb_libusb_transport(antz_garmin_usb_libusb_t* transport) {
        antz_garmin_usb_transport_t t;
        t.write = transport_write;
        t.read = transport_read;
        t.ctx = transport;
        return t;
    }

    const char* garmin_usb_libusb_status_text(const antz_garmin_usb_libusb_status_t status) {
        switch (status) {
            case GARMIN_USB_LIBUSB_OK:             return "ok";
            case GARMIN_USB_LIBUSB_NO_DEVICE:      return "no Garmin handheld found on USB";
            case GARMIN_USB_LIBUSB_MORE_THAN_ONE:  return "more than one Garmin handheld on USB; connect one";
            case GARMIN_USB_LIBUSB_LIST_FAILED:    return "could not list the USB devices";
            case GARMIN_USB_LIBUSB_OPEN_FAILED:    return "could not open the Garmin handheld (permissions?)";
            case GARMIN_USB_LIBUSB_NO_ENDPOINTS:   return "the Garmin handheld lacks the interrupt and bulk endpoints";
            case GARMIN_USB_LIBUSB_DETACH_FAILED:  return "could not detach the kernel driver from the handheld";
            case GARMIN_USB_LIBUSB_CLAIM_FAILED:   return "could not claim the handheld's USB interface";
            case GARMIN_USB_LIBUSB_BAD_ARGUMENT:   return "bad argument to the USB transport";
        }
        return "unknown USB transport status";
    }
}
