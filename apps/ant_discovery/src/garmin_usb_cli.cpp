#include "garmin_usb_cli.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>

#ifdef ANTZ_HAVE_GARMIN_USB
#include "garmin_usb_libusb.h"
#endif

namespace ant::garmin_usb {

    namespace {
        std::atomic<bool> g_stop{false};

        void onSignal(int) { g_stop.store(true); }
    }

#ifdef ANTZ_HAVE_GARMIN_USB

    int runCli(const Options& options) {
        antz::antz_libusb_ops_t ops;
        if (!antz::garmin_usb_libusb_real_ops(&ops)) {
            std::cerr << "could not initialise libusb" << std::endl;
            return 1;
        }

        antz::antz_garmin_usb_libusb_t usb;
        const antz::antz_garmin_usb_libusb_status_t status = antz::garmin_usb_libusb_open(&usb, &ops);
        if (status != antz::GARMIN_USB_LIBUSB_OK) {
            std::cerr << antz::garmin_usb_libusb_status_text(status) << std::endl;
            // The ops table owns libusb's context; a failed open leaves it to us.
            ops.close(ops.ctx);
            return 1;
        }

        g_stop.store(false);
        std::signal(SIGINT, onSignal);
        std::signal(SIGTERM, onSignal);

        const auto began = std::chrono::steady_clock::now();
        const RunResult result = run(antz::garmin_usb_libusb_transport(&usb), options, std::cout, std::cerr, g_stop,
                                     [began]() {
                                         return static_cast<uint64_t>(
                                             std::chrono::duration_cast<std::chrono::milliseconds>(
                                                 std::chrono::steady_clock::now() - began).count());
                                     });

        // Releases the interface, gives the kernel driver back and closes the
        // device, then frees libusb's context.
        antz::garmin_usb_libusb_close(&usb);
        return exitCode(result);
    }

#else

    int runCli(const Options&) {
        std::cerr << "this build has no libusb: --garmin-usb is not available" << std::endl;
        return 1;
    }

#endif
}
