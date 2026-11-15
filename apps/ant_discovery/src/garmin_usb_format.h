//
// Output lines for what a Garmin handheld sends over USB.
//
// Pure string building over the decoder's plain structs, kept apart from the
// I/O for the reason ant_format.h gives: a malformed line that reached a
// consumer unnoticed is how those bugs are found late. Nothing here touches
// std::cout or a global, and it depends on neither the ANT SDK nor libusb, so
// tests/garmin_usb_mode_test.cpp runs anywhere.
//
// One line per event, in the formats the ANT mode already offers (text, JSON
// and CSV, selected with -f). A field the handheld model does not confirm is
// null in JSON, empty in CSV and «?» in text; it is never a guessed value.
//
//   handheld          the product (id and software version) once it is known
//   handheld_position the handheld's own position (Pvt_Data)
//   dog               a collar's packet: position, status, GPS and radio bars, raw battery
//
// JSON keys, CSV columns and text wording are fixed here and are what the tests
// pin:
//
//   {"page":"handheld","unit_id":3886356464,"product_id":1339,"software":"4.60"}
//   {"page":"handheld_position","unit_id":..,"latitude":..,"longitude":..,"altitude":..,"fix":3}
//   {"page":"dog","unit_id":..,"name":"..","latitude":..,"longitude":..,"time":"2026-11-10T12:00:00Z",
//    "status":"moving","status_raw":2,"gps_lost":false,"gps_bars":5,"comm_bars":5,
//    "battery_raw":40,"flags":0,"track_color":12,"configured_id":11}
//
// CSV is one row per event, columns in this order, a header row per page from
// csvHeader(page), cells quoted as RFC 4180 where they hold a comma, quote or
// line break. A null above is an empty cell:
//
//   handheld           page,unit_id,product_id,software
//   handheld_position  page,unit_id,latitude,longitude,altitude,fix
//   dog                page,unit_id,name,latitude,longitude,time,status,status_raw,gps_lost,
//                      gps_bars,comm_bars,battery_raw,flags,track_color,configured_id
//
// Text is one line per event; an unknown gps_lost is «?»:
//
//   handheld unit 3886356464 product 1339 software 4.60
//   handheld_position unit 3886356464 59.900000,10.700000 alt 141.0 fix 3
//   dog Ragnar unit 3886356464 59.900000,10.700000 2026-11-10T12:00:00Z moving (2) gps_lost no gps 5 comm 5 battery_raw 40
//
// The functions return a line without its trailing newline; the run loop adds it.
// `software` is the version in hundredths with two decimals («4.60»). Altitude
// has one decimal. A name is JSON-escaped (quote, backslash, \n \r \t \b \f as
// themselves, other control characters as \u00xx).
//
// Coordinates have six decimals. `status` is sitting, moving, treed, pointed or
// unknown. `gps_lost` is true or false where loss is confirmed for the model
// and null where it is not; `track_color` and `configured_id` are null where
// the model does not confirm them. `battery_raw` is the raw level, never a
// percentage. A position is always printed as sent: a lost fix is marked by
// `gps_lost`, not hidden.
//
#pragma once

#include "profiles/garmin_usb/garmin_usb_decoder.h"

#include <cstdint>
#include <string>

namespace ant::garmin_usb {

    enum class Format { Text, Json, Csv };

    // The CSV header row for each page, in the order the rows are written.
    std::string csvHeader(const char* page);

    std::string statusName(antz::antz_garmin_usb_dog_status_t status);

    std::string formatHandheld(Format fmt, uint32_t unit_id, const antz::antz_garmin_usb_product_t& product);

    std::string formatHandheldPosition(Format fmt, uint32_t unit_id, const antz::antz_garmin_usb_pvt_t& pvt);

    std::string formatDog(Format fmt, uint32_t unit_id, const antz::antz_garmin_usb_dog_t& dog);
}
