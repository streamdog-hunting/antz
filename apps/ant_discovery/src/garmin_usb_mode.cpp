#include "garmin_usb_mode.h"

#include <ostream>
#include <set>
#include <string>

namespace ant::garmin_usb {

    namespace {

        struct Ctx {
            const Options* options;
            std::ostream* out;
            std::ostream* err;
            uint32_t unit_id = 0;
            std::set<std::string> headers_written;

            void line(const char* page, const std::string& text) {
                if (options->format == Format::Csv && headers_written.insert(page).second)
                    *out << csvHeader(page) << '\n';
                *out << text << '\n';
            }
        };

        void on_session(void* c, const uint32_t unit_id) {
            Ctx* x = static_cast<Ctx*>(c);
            x->unit_id = unit_id;
            if (x->options->verbose) *x->err << "session started, handheld unit " << unit_id << '\n';
        }

        void on_product(void* c, const antz::antz_garmin_usb_product_t* p) {
            Ctx* x = static_cast<Ctx*>(c);
            x->line("handheld", formatHandheld(x->options->format, x->unit_id, *p));
        }

        void on_pvt(void* c, const antz::antz_garmin_usb_pvt_t* p) {
            Ctx* x = static_cast<Ctx*>(c);
            x->line("handheld_position", formatHandheldPosition(x->options->format, x->unit_id, *p));
        }

        void on_dog(void* c, const antz::antz_garmin_usb_dog_t* d) {
            Ctx* x = static_cast<Ctx*>(c);
            x->line("dog", formatDog(x->options->format, x->unit_id, *d));
        }
    }

    int exitCode(const RunResult result) {
        return result == RunResult::Stopped ? 0 : 1;
    }

    RunResult run(const antz::antz_garmin_usb_transport_t& transport, const Options& options,
                  std::ostream& out, std::ostream& err, const std::atomic<bool>& stop,
                  const std::function<uint64_t()>& now_ms) {
        using namespace antz;

        Ctx ctx;
        ctx.options = &options;
        ctx.out = &out;
        ctx.err = &err;

        antz_garmin_usb_events_t events = {on_session, on_product, on_pvt, on_dog, &ctx};
        antz_garmin_usb_session_t session;
        if (garmin_usb_session_init(&session, &transport, &events) != 0) {
            err << "the USB transport is incomplete\n";
            return RunResult::StartFailed;
        }

        if (stop.load()) return RunResult::Stopped;

        if (garmin_usb_session_start(&session) != 0) {
            err << "could not send Start_Session to the handheld\n";
            return RunResult::StartFailed;
        }

        const uint64_t began = now_ms();
        uint64_t last_packet = began;
        const uint64_t idle_limit = static_cast<uint64_t>(options.idle_timeout_s) * 1000u;
        const uint64_t time_limit = static_cast<uint64_t>(options.seconds * 1000.0);

        for (;;) {
            if (stop.load() || (time_limit > 0 && now_ms() - began >= time_limit)) {
                garmin_usb_session_stop(&session);
                return RunResult::Stopped;
            }

            switch (garmin_usb_session_poll(&session, options.poll_ms)) {
                case GARMIN_USB_POLL_OK:
                    last_packet = now_ms();
                    break;
                case GARMIN_USB_POLL_IDLE:
                    break;
                case GARMIN_USB_POLL_TRANSPORT_ERROR:
                    err << "the USB link to the handheld failed (a read or write error); is the cable connected?\n";
                    return RunResult::TransportError;
                case GARMIN_USB_POLL_FRAME_ERROR:
                    err << "the handheld sent an invalid packet header; the stream cannot be trusted\n";
                    return RunResult::FrameError;
                case GARMIN_USB_POLL_PARTIAL_PACKET:
                    err << "a USB transfer from the handheld ended inside a packet\n";
                    return RunResult::PartialPacket;
            }

            if (idle_limit > 0 && now_ms() - last_packet >= idle_limit) {
                err << "no packet from the handheld for " << options.idle_timeout_s << " seconds\n";
                return RunResult::IdleTimeout;
            }
        }
    }
}
