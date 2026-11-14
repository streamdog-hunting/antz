//
// ant_discovery's Garmin USB mode: read a handheld over its USB cable and print
// what it sends, in the formats the ANT mode prints.
//
// `run` opens nothing. It takes a transport, runs the Garmin USB session over
// it (antz_core's garmin_usb_session.h), and writes one line per handheld,
// position and dog packet to `out`, until it is told to stop or the cable goes
// quiet. main.cpp opens the libusb transport and calls it with `--garmin-usb`,
// and the tests call it with a fake transport and the replay recordings.
//
// Behaviour, from the tested Linux logger in streamdog-hunting/astro-usb:
//   - Writes the CSV header row once per page before its first row, in CSV mode.
//   - Polls the session with `poll_ms`. A poll that read nothing counts as idle.
//     No packet for `idle_timeout_s` seconds ends the run with IdleTimeout (the
//     logger used 15 seconds). A packet resets the idle count.
//   - A stop request (SIGINT, SIGTERM) ends the run after the current poll,
//     sends Stop_Pvt_Data if the position stream started, and returns Stopped.
//   - A transport, frame or partial-packet error from the session ends the
//     run with the matching result and a one line message on `err`.
//   - `out` gets data only: handheld, position and dog lines. Messages and
//     verbose notes go to `err`.
//   - The mode makes no identity decision. It prints each collar as the handheld
//     sent it, as evidence; telling collars apart is the SDK's job.
//
#pragma once

#include "garmin_usb_format.h"
#include "profiles/garmin_usb/garmin_usb_session.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <iosfwd>

namespace ant::garmin_usb {

    struct Options {
        Format   format = Format::Text;
        bool     verbose = false;
        uint32_t poll_ms = 500;
        uint32_t idle_timeout_s = 15;
        // Stop after this many seconds; 0 means run until stopped.
        double   seconds = 0;
    };

    enum class RunResult {
        Stopped,          // a stop was requested, or `seconds` passed
        StartFailed,      // Start_Session could not be written
        IdleTimeout,      // no packet for idle_timeout_s
        TransportError,   // a read or a handshake write failed
        FrameError,       // an invalid frame header
        PartialPacket,    // a bulk transfer ended inside a packet
    };

    // The process exit code for a result: 0 for Stopped, nonzero otherwise.
    int exitCode(RunResult result);

    // `now_ms` is a monotonic clock in milliseconds (std::chrono::steady_clock
    // in main.cpp, a counter the fake transport advances in the tests).
    RunResult run(const antz::antz_garmin_usb_transport_t& transport, const Options& options,
                  std::ostream& out, std::ostream& err, const std::atomic<bool>& stop,
                  const std::function<uint64_t()>& now_ms);
}
