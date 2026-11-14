# Changelog

All notable changes to `antz_core`, the platform-independent ANT+ and Garmin
decoding library in `libs/antz_core`. Versions are tagged `antz-core-v<version>`.
A breaking change to a public header bumps the minor version while the major
version is 0.

## [0.1.0]

The first tagged version. Before it, consumers pinned a commit.

### Added

- ANT+ Asset Tracker decoding and encoding (Asset Location pages 1 and 2,
  identification pages 1 and 2, the status byte), with the handheld's own
  position (pages 0x04 and 0x05).
- ANT+ Heart Rate Monitor decoding.
- ANT+ Common Data Pages 70 (encode), 80, 81 and 82.
- Garmin Astro and Alpha handhelds over USB: a frame reader, decoders for
  Session_Started, Product_Data, Pvt_Data and the dog packet (pid 3078), and a
  session over a transport of two functions (write a packet, read an endpoint),
  with no port, no libusb and no operating system in it. Status codes are per
  model, GPS loss is byte 17 bit 0x20 for the two confirmed models, the battery
  stays a raw level, and bytes 22 to 23, 30 and 40 onward are evidence and never
  a key (Streamdog ADR 0162).
