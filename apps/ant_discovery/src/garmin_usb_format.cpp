#include "garmin_usb_format.h"

#include <cmath>
#include <cstdio>
#include <ctime>
#include <sstream>

namespace ant::garmin_usb {

    namespace {

        std::string fixed(const double v, const int decimals) {
            char buf[64];
            std::snprintf(buf, sizeof buf, "%.*f", decimals, v);
            return buf;
        }

        std::string coordinate(const double degrees) { return fixed(degrees, 6); }

        std::string jsonEscape(const std::string& in) {
            std::string out;
            for (const char c : in) {
                switch (c) {
                    case '"':  out += "\\\""; break;
                    case '\\': out += "\\\\"; break;
                    case '\n': out += "\\n"; break;
                    case '\r': out += "\\r"; break;
                    case '\t': out += "\\t"; break;
                    case '\b': out += "\\b"; break;
                    case '\f': out += "\\f"; break;
                    default:
                        if (static_cast<unsigned char>(c) <= 0x1F) {
                            char buf[8];
                            std::snprintf(buf, sizeof buf, "\\u%04x", static_cast<unsigned>(static_cast<unsigned char>(c)));
                            out += buf;
                        } else {
                            out += c;
                        }
                }
            }
            return out;
        }

        // RFC 4180: a cell with a comma, a quote or a line break is quoted, and
        // a quote inside it is doubled.
        std::string csvCell(const std::string& in) {
            if (in.find_first_of(",\"\r\n") == std::string::npos) return in;
            std::string out = "\"";
            for (const char c : in) {
                if (c == '"') out += '"';
                out += c;
            }
            return out + "\"";
        }

        std::string isoTime(const int64_t unix_s) {
            const std::time_t t = static_cast<std::time_t>(unix_s);
            std::tm tm{};
            gmtime_r(&t, &tm);
            char buf[32];
            std::strftime(buf, sizeof buf, "%Y-%m-%dT%H:%M:%SZ", &tm);
            return buf;
        }

        // 460 is «4.60»; a value below 1.00 keeps its leading zero.
        std::string software(const int16_t x100) {
            const int v = x100;
            const int a = v < 0 ? -v : v;
            char buf[32];
            std::snprintf(buf, sizeof buf, "%s%d.%02d", v < 0 ? "-" : "", a / 100, a % 100);
            return buf;
        }

        std::string tri(const bool known, const bool value, const char* t, const char* f, const char* unknown) {
            return known ? (value ? t : f) : unknown;
        }
    }

    std::string statusName(const antz::antz_garmin_usb_dog_status_t status) {
        switch (status) {
            case antz::GARMIN_USB_DOG_STATUS_SITTING: return "sitting";
            case antz::GARMIN_USB_DOG_STATUS_MOVING:  return "moving";
            case antz::GARMIN_USB_DOG_STATUS_TREED:   return "treed";
            case antz::GARMIN_USB_DOG_STATUS_POINTED: return "pointed";
            case antz::GARMIN_USB_DOG_STATUS_UNKNOWN: break;
        }
        return "unknown";
    }

    std::string csvHeader(const char* page) {
        const std::string p = page ? page : "";
        if (p == "handheld") return "page,unit_id,product_id,software";
        if (p == "handheld_position") return "page,unit_id,latitude,longitude,altitude,fix";
        if (p == "dog")
            return "page,unit_id,name,latitude,longitude,time,status,status_raw,gps_lost,gps_bars,comm_bars,"
                   "battery_raw,flags,track_color,configured_id";
        return "page";
    }

    std::string formatHandheld(const Format fmt, const uint32_t unit_id, const antz::antz_garmin_usb_product_t& product) {
        std::ostringstream o;
        switch (fmt) {
            case Format::Json:
                o << R"({"page":"handheld","unit_id":)" << unit_id << R"(,"product_id":)" << product.product_id
                  << R"(,"software":")" << software(product.software_version_x100) << "\"}";
                break;
            case Format::Csv:
                o << "handheld," << unit_id << ',' << product.product_id << ',' << software(product.software_version_x100);
                break;
            case Format::Text:
                o << "handheld unit " << unit_id << " product " << product.product_id << " software "
                  << software(product.software_version_x100);
                break;
        }
        return o.str();
    }

    std::string formatHandheldPosition(const Format fmt, const uint32_t unit_id, const antz::antz_garmin_usb_pvt_t& pvt) {
        std::ostringstream o;
        switch (fmt) {
            case Format::Json:
                o << R"({"page":"handheld_position","unit_id":)" << unit_id << R"(,"latitude":)" << coordinate(pvt.latitude_deg)
                  << R"(,"longitude":)" << coordinate(pvt.longitude_deg) << R"(,"altitude":)" << fixed(pvt.altitude_m, 1)
                  << R"(,"fix":)" << pvt.fix << '}';
                break;
            case Format::Csv:
                o << "handheld_position," << unit_id << ',' << coordinate(pvt.latitude_deg) << ',' << coordinate(pvt.longitude_deg)
                  << ',' << fixed(pvt.altitude_m, 1) << ',' << pvt.fix;
                break;
            case Format::Text:
                o << "handheld_position unit " << unit_id << ' ' << coordinate(pvt.latitude_deg) << ','
                  << coordinate(pvt.longitude_deg) << " alt " << fixed(pvt.altitude_m, 1) << " fix " << pvt.fix;
                break;
        }
        return o.str();
    }

    std::string formatDog(const Format fmt, const uint32_t unit_id, const antz::antz_garmin_usb_dog_t& d) {
        const std::string lat = coordinate(d.latitude_deg), lon = coordinate(d.longitude_deg);
        const std::string when = isoTime(d.unix_time_s);
        const std::string status = statusName(d.status);
        const bool lost_known = d.gps_lost_known != 0, lost = d.gps_lost != 0;
        std::ostringstream o;
        switch (fmt) {
            case Format::Json:
                o << R"({"page":"dog","unit_id":)" << unit_id << R"(,"name":")" << jsonEscape(d.name) << '"'
                  << R"(,"latitude":)" << lat << R"(,"longitude":)" << lon << R"(,"time":")" << when << '"'
                  << R"(,"status":")" << status << R"(","status_raw":)" << static_cast<unsigned>(d.status_raw)
                  << R"(,"gps_lost":)" << tri(lost_known, lost, "true", "false", "null")
                  << R"(,"gps_bars":)" << static_cast<unsigned>(d.gps_bars)
                  << R"(,"comm_bars":)" << static_cast<unsigned>(d.comm_bars)
                  << R"(,"battery_raw":)" << static_cast<unsigned>(d.battery_level_raw)
                  << R"(,"flags":)" << static_cast<unsigned>(d.flags) << R"(,"track_color":)";
                if (d.track_color_known) o << static_cast<unsigned>(d.track_color_code); else o << "null";
                o << R"(,"configured_id":)";
                if (d.configured_id_known) o << d.configured_id; else o << "null";
                o << '}';
                break;
            case Format::Csv:
                o << "dog," << unit_id << ',' << csvCell(d.name) << ',' << lat << ',' << lon << ',' << when << ',' << status
                  << ',' << static_cast<unsigned>(d.status_raw) << ',' << tri(lost_known, lost, "true", "false", "")
                  << ',' << static_cast<unsigned>(d.gps_bars) << ',' << static_cast<unsigned>(d.comm_bars)
                  << ',' << static_cast<unsigned>(d.battery_level_raw) << ',' << static_cast<unsigned>(d.flags) << ',';
                if (d.track_color_known) o << static_cast<unsigned>(d.track_color_code);
                o << ',';
                if (d.configured_id_known) o << d.configured_id;
                break;
            case Format::Text:
                o << "dog " << d.name << " unit " << unit_id << ' ' << lat << ',' << lon << ' ' << when << ' ' << status
                  << " (" << static_cast<unsigned>(d.status_raw) << ") gps_lost " << tri(lost_known, lost, "yes", "no", "?")
                  << " gps " << static_cast<unsigned>(d.gps_bars) << " comm " << static_cast<unsigned>(d.comm_bars)
                  << " battery_raw " << static_cast<unsigned>(d.battery_level_raw);
                break;
        }
        return o.str();
    }
}
