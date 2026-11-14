//
// The command line side of the Garmin USB mode: open the handheld over libusb,
// run the mode, close, and turn what happened into an exit code.
//
// Kept out of main.cpp, which includes the ANT SDK, so the USB mode does not
// depend on it. `--garmin-usb` in main.cpp calls runCli before anything of the
// ANT mode starts.
//
#pragma once

#include "garmin_usb_mode.h"

namespace ant::garmin_usb {

    // Opens the one Garmin handheld on USB, runs the mode until SIGINT, SIGTERM,
    // `options.seconds` or an error, and returns the process exit code:
    // 0 for a requested stop, 1 for anything else, with the cause on stderr.
    // Built without libusb it prints that and returns 1.
    int runCli(const Options& options);
}
