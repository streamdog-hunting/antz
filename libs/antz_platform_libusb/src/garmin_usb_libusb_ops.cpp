// The ops table over libusb 1.0. See garmin_usb_libusb.h.
#include "garmin_usb_libusb.h"

#include <libusb.h>

#include <new>

namespace antz {

    namespace {

        struct Real {
            libusb_context*       context = nullptr;
            libusb_device_handle* handle = nullptr;
        };

        Real* R(void* ctx) { return static_cast<Real*>(ctx); }

        // libusb's result as the ops table reports it: a byte count or zero, the
        // timeout code, or a general error.
        int mapped(const int rc) {
            if (rc >= 0) return rc;
            return rc == LIBUSB_ERROR_TIMEOUT ? ANTZ_LIBUSB_TIMEOUT : ANTZ_LIBUSB_ERROR;
        }

        int count_devices(void* ctx, const uint16_t vendor_id, const uint16_t product_id) {
            libusb_device** list = nullptr;
            const ssize_t n = libusb_get_device_list(R(ctx)->context, &list);
            if (n < 0) return -1;
            int found = 0;
            for (ssize_t i = 0; i < n; ++i) {
                libusb_device_descriptor d;
                if (libusb_get_device_descriptor(list[i], &d) == 0 &&
                    d.idVendor == vendor_id && d.idProduct == product_id)
                    ++found;
            }
            libusb_free_device_list(list, 1);
            return found;
        }

        int open_device(void* ctx, const uint16_t vendor_id, const uint16_t product_id) {
            Real* r = R(ctx);
            r->handle = libusb_open_device_with_vid_pid(r->context, vendor_id, product_id);
            return r->handle ? 0 : -1;
        }

        int endpoints(void* ctx, antz_libusb_endpoint_t* interrupt_in, antz_libusb_endpoint_t* bulk_in,
                      antz_libusb_endpoint_t* bulk_out) {
            Real* r = R(ctx);
            if (!r->handle) return 1;
            libusb_device* dev = libusb_get_device(r->handle);
            libusb_config_descriptor* config = nullptr;
            if (libusb_get_active_config_descriptor(dev, &config) != 0 || !config) return 1;

            bool have_int = false, have_in = false, have_out = false;
            if (config->bNumInterfaces > GARMIN_USB_INTERFACE &&
                config->interface[GARMIN_USB_INTERFACE].num_altsetting > 0) {
                const libusb_interface_descriptor& alt = config->interface[GARMIN_USB_INTERFACE].altsetting[0];
                for (uint8_t i = 0; i < alt.bNumEndpoints; ++i) {
                    const libusb_endpoint_descriptor& e = alt.endpoint[i];
                    const bool in = (e.bEndpointAddress & LIBUSB_ENDPOINT_IN) != 0;
                    const uint8_t type = e.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK;
                    const antz_libusb_endpoint_t d = {e.bEndpointAddress, e.wMaxPacketSize};
                    if (type == LIBUSB_TRANSFER_TYPE_INTERRUPT && in && !have_int) { *interrupt_in = d; have_int = true; }
                    else if (type == LIBUSB_TRANSFER_TYPE_BULK && in && !have_in)  { *bulk_in = d; have_in = true; }
                    else if (type == LIBUSB_TRANSFER_TYPE_BULK && !in && !have_out) { *bulk_out = d; have_out = true; }
                }
            }
            libusb_free_config_descriptor(config);
            return (have_int && have_in && have_out) ? 0 : 1;
        }

        int kernel_driver_active(void* ctx, const int n) {
            const int rc = libusb_kernel_driver_active(R(ctx)->handle, n);
            // Where the platform has no kernel driver to detach, none is active.
            return rc == LIBUSB_ERROR_NOT_SUPPORTED ? 0 : rc;
        }
        int detach_kernel_driver(void* ctx, const int n) { return libusb_detach_kernel_driver(R(ctx)->handle, n); }
        int attach_kernel_driver(void* ctx, const int n) { return libusb_attach_kernel_driver(R(ctx)->handle, n); }
        int claim_interface(void* ctx, const int n) { return libusb_claim_interface(R(ctx)->handle, n); }
        int release_interface(void* ctx, const int n) { return libusb_release_interface(R(ctx)->handle, n); }

        int interrupt_in(void* ctx, const uint8_t ep, uint8_t* buf, const size_t len, const uint32_t timeout_ms) {
            int got = 0;
            const int rc = libusb_interrupt_transfer(R(ctx)->handle, ep, buf, static_cast<int>(len), &got, timeout_ms);
            // A timeout may still have moved bytes; hand them over.
            if (rc == LIBUSB_ERROR_TIMEOUT && got > 0) return got;
            return rc == 0 ? got : mapped(rc);
        }
        int bulk_in(void* ctx, const uint8_t ep, uint8_t* buf, const size_t len, const uint32_t timeout_ms) {
            int got = 0;
            const int rc = libusb_bulk_transfer(R(ctx)->handle, ep, buf, static_cast<int>(len), &got, timeout_ms);
            if (rc == LIBUSB_ERROR_TIMEOUT && got > 0) return got;
            return rc == 0 ? got : mapped(rc);
        }
        int bulk_out(void* ctx, const uint8_t ep, const uint8_t* buf, const size_t len, const uint32_t timeout_ms) {
            int sent = 0;
            const int rc = libusb_bulk_transfer(R(ctx)->handle, ep, const_cast<uint8_t*>(buf),
                                                static_cast<int>(len), &sent, timeout_ms);
            return rc == 0 ? sent : mapped(rc);
        }

        void close_all(void* ctx) {
            Real* r = R(ctx);
            if (r->handle) libusb_close(r->handle);
            if (r->context) libusb_exit(r->context);
            delete r;
        }
    }

    bool garmin_usb_libusb_real_ops(antz_libusb_ops_t* out) {
        if (!out) return false;
        Real* r = new (std::nothrow) Real();
        if (!r) return false;
        if (libusb_init(&r->context) != 0) {
            delete r;
            return false;
        }
        out->count_devices = count_devices;
        out->open = open_device;
        out->endpoints = endpoints;
        out->kernel_driver_active = kernel_driver_active;
        out->detach_kernel_driver = detach_kernel_driver;
        out->attach_kernel_driver = attach_kernel_driver;
        out->claim_interface = claim_interface;
        out->release_interface = release_interface;
        out->interrupt_in = interrupt_in;
        out->bulk_in = bulk_in;
        out->bulk_out = bulk_out;
        out->close = close_all;
        out->ctx = r;
        return true;
    }
}
