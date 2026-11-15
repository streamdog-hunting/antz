//
// Tests for the Garmin USB decoder: framing, header, session, product, the
// handheld's position and the dog packet.
//
// Links nothing but antz_core, so it runs with no dongle and no handheld.
// The packets are built from the byte layout in streamdog-hunting/astro-usb
// docs/protocol.md, and every expected value is the one that handheld's
// screen confirmed there, not what an implementation printed.
//
// The private recordings (raw positions, kept out of git) can be replayed
// through the same checks: set ANTZ_GARMIN_USB_REPLAY to a JSON lines file
// of {"type":..,"pid":..,"hex":".."} records, and the `replay` group decodes
// every dog packet in it and checks what holds for any real packet.
//
#include "profiles/garmin_usb/garmin_usb_decoder.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

using namespace antz;

static int checks = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); return 1; } } while (0)

static bool near(const double a, const double b, const double eps = 1e-6) {
    return std::fabs(a - b) <= eps;
}

// ─── Builders: the byte layouts, written out once ──────────────────────────
static void put_u16(std::vector<uint8_t>& v, size_t at, uint16_t x) {
    v[at] = x & 0xFF; v[at + 1] = x >> 8;
}
static void put_u32(std::vector<uint8_t>& v, size_t at, uint32_t x) {
    for (int i = 0; i < 4; ++i) v[at + i] = (x >> (8 * i)) & 0xFF;
}
static void put_f32(std::vector<uint8_t>& v, size_t at, float x) {
    uint32_t b; std::memcpy(&b, &x, 4); put_u32(v, at, b);
}
static void put_f64(std::vector<uint8_t>& v, size_t at, double x) {
    uint64_t b; std::memcpy(&b, &x, 8);
    for (int i = 0; i < 8; ++i) v[at + i] = (b >> (8 * i)) & 0xFF;
}
static int32_t semicircles(const double deg) {
    return static_cast<int32_t>(std::lround(deg * 2147483648.0 / 180.0));
}

// A 100 byte dog packet, protocol.md §pid 3078.
static std::vector<uint8_t> dog_packet(double lat, double lon, uint32_t garmin_time,
                                       uint8_t status, uint8_t flags, const char* name) {
    std::vector<uint8_t> p(100, 0);
    put_u32(p, 0, static_cast<uint32_t>(semicircles(lat)));
    put_u32(p, 4, static_cast<uint32_t>(semicircles(lon)));
    put_f32(p, 8, 312.5f);
    put_u32(p, 12, garmin_time);
    p[16] = status;
    p[17] = flags;
    put_u16(p, 22, 11);          // configured id
    p[24] = 12;                  // track colour, blue
    p[25] = 40;                  // battery, two of four bars on an Astro
    p[26] = 5;                   // GPS bars
    p[27] = 5;                   // comm bars
    p[30] = 207;                 // unknown, not an identity
    const size_t n = std::strlen(name);
    std::memcpy(&p[31], name, n);
    p[31 + n] = 0;
    for (size_t i = 32 + n; i < p.size(); ++i) p[i] = 'Z';   // tail after the name
    return p;
}

// A frame: header plus payload.
static std::vector<uint8_t> frame(uint8_t layer, uint16_t pid, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> f(12 + payload.size(), 0);
    f[0] = layer;
    put_u16(f, 4, pid);
    put_u32(f, 8, static_cast<uint32_t>(payload.size()));
    // &f[12] of an empty payload is one past the end: not a valid index.
    if (!payload.empty()) std::memcpy(f.data() + 12, payload.data(), payload.size());
    return f;
}

// ─── Header ─────────────────────────────────────────────────────────────────
static int header() {
    auto f = frame(20, 3078, std::vector<uint8_t>(100, 0));
    antz_garmin_usb_header_t h{};
    CHECK(garmin_usb_decode_header(f.data(), f.size(), &h) == 0);
    CHECK(h.layer == 20 && h.pid == 3078 && h.size == 100);

    // The protocol layer is the other valid one.
    auto p = frame(0, 6, std::vector<uint8_t>(4, 0));
    CHECK(garmin_usb_decode_header(p.data(), p.size(), &h) == 0);
    CHECK(h.layer == 0 && h.pid == 6 && h.size == 4);

    // Refused: short, null, an unknown layer, a size beyond the limit.
    CHECK(garmin_usb_decode_header(f.data(), 11, &h) != 0);
    CHECK(garmin_usb_decode_header(nullptr, 12, &h) != 0);
    CHECK(garmin_usb_decode_header(f.data(), f.size(), nullptr) != 0);
    auto bad_layer = f; bad_layer[0] = 7;
    CHECK(garmin_usb_decode_header(bad_layer.data(), bad_layer.size(), &h) != 0);
    auto too_big = f; put_u32(too_big, 8, GARMIN_USB_MAX_PACKET_SIZE + 1);
    CHECK(garmin_usb_decode_header(too_big.data(), too_big.size(), &h) != 0);
    auto at_limit = f; put_u32(at_limit, 8, GARMIN_USB_MAX_PACKET_SIZE);
    CHECK(garmin_usb_decode_header(at_limit.data(), at_limit.size(), &h) == 0);
    return 0;
}

// ─── Session and product ────────────────────────────────────────────────────
static int session_and_product() {
    // Session_Started carries the handheld's unit id, u32 little endian.
    // 3886356464 is the Astro 320's, from astro-usb docs/protocol.md.
    std::vector<uint8_t> s(4, 0);
    put_u32(s, 0, 3886356464u);
    uint32_t unit = 0;
    CHECK(garmin_usb_decode_session_started(s.data(), s.size(), &unit) == 0);
    CHECK(unit == 3886356464u);
    CHECK(garmin_usb_decode_session_started(s.data(), 3, &unit) != 0);
    CHECK(garmin_usb_decode_session_started(nullptr, 4, &unit) != 0);

    // Product_Data: product id u16, software version s16 in hundredths,
    // then descriptions that are not read.
    std::vector<uint8_t> p(4 + 32, 0);
    put_u16(p, 0, 1339);
    put_u16(p, 2, 460);
    std::memcpy(&p[4], "Astro 320 Software Version 4.60", 31);
    antz_garmin_usb_product_t prod{};
    CHECK(garmin_usb_decode_product(p.data(), p.size(), &prod) == 0);
    CHECK(prod.product_id == 1339 && prod.software_version_x100 == 460);
    put_u16(p, 0, 1562); put_u16(p, 2, 860);
    CHECK(garmin_usb_decode_product(p.data(), p.size(), &prod) == 0);
    CHECK(prod.product_id == GARMIN_USB_PRODUCT_ALPHA_100 && prod.software_version_x100 == 860);
    CHECK(garmin_usb_decode_product(p.data(), 3, &prod) != 0);
    return 0;
}

// ─── The handheld's own position, Pvt_Data (D800) ───────────────────────────
static int pvt() {
    // <ffffHdddffffhI : alt epe eph epv fix tow lat lon east north up msl leap days
    // at byte         0   4   8   12  16  18  26  34  42   46    50 54  58   60
    std::vector<uint8_t> d(64, 0);
    put_f32(d, 0, 100.0f);                          // alt, above the ellipsoid
    put_f32(d, 4, 3.5f);                            // epe
    put_u16(d, 16, 3);                              // fix
    put_f64(d, 18, 123456.0);                       // time of week
    put_f64(d, 26, 59.9 * M_PI / 180.0);            // lat, radians
    put_f64(d, 34, 10.7 * M_PI / 180.0);            // lon, radians
    put_f32(d, 54, 41.0f);                          // height of mean sea level
    antz_garmin_usb_pvt_t out{};
    CHECK(garmin_usb_decode_pvt(d.data(), d.size(), &out) == 0);
    CHECK(near(out.latitude_deg, 59.9) && near(out.longitude_deg, 10.7));
    CHECK(near(out.altitude_m, 141.0, 1e-3));       // alt + msl
    CHECK(out.fix == 3);
    CHECK(garmin_usb_decode_pvt(d.data(), 63, &out) != 0);
    CHECK(garmin_usb_decode_pvt(nullptr, 64, &out) != 0);
    return 0;
}

// ─── The dog packet: fields, protocol.md §pid 3078 ──────────────────────────
static int dog_fields() {
    // 2026-11-10 12:00:00 UTC on the Garmin epoch.
    const int64_t unix_s = 1794312000;
    const uint32_t garmin_t = static_cast<uint32_t>(unix_s - GARMIN_USB_EPOCH_UNIX_S);
    auto p = dog_packet(59.9139, 10.7522, garmin_t, 2, 0, "Ragnar");
    antz_garmin_usb_dog_t d{};
    CHECK(garmin_usb_decode_dog(p.data(), p.size(), GARMIN_USB_PRODUCT_ASTRO_320, &d) == 0);
    CHECK(near(d.latitude_deg, 59.9139, 1e-6) && near(d.longitude_deg, 10.7522, 1e-6));
    CHECK(near(d.altitude_m, 312.5f, 1e-3));
    CHECK(d.garmin_time_s == garmin_t && d.unix_time_s == unix_s);
    CHECK(d.status_raw == 2 && d.status == GARMIN_USB_DOG_STATUS_MOVING);
    CHECK(d.flags == 0 && d.gps_lost_known == 1 && d.gps_lost == 0);
    CHECK(d.gps_bars == 5 && d.gps_bars_max == 5 && d.comm_bars == 5);
    CHECK(d.battery_level_raw == 40);
    CHECK(d.track_color_known == 1 && d.track_color_code == 12);
    CHECK(d.configured_id_known == 1 && d.configured_id == 11);
    CHECK(d.id_bytes[0] == 11 && d.id_bytes[1] == 0);
    CHECK(std::strcmp(d.name, "Ragnar") == 0);

    // The same collar read as an Alpha 100 carries no configured id (its
    // bytes 22..23 are not that field there), a GPS maximum of 3, and the
    // raw byte pair kept as evidence.
    p[22] = 1; p[23] = 2;
    CHECK(garmin_usb_decode_dog(p.data(), p.size(), GARMIN_USB_PRODUCT_ALPHA_100, &d) == 0);
    CHECK(d.configured_id_known == 0);
    CHECK(d.id_bytes[0] == 1 && d.id_bytes[1] == 2);
    CHECK(d.gps_bars_max == 3 && d.track_color_known == 1);

    // Southern and western hemispheres: semicircles are signed.
    auto s = dog_packet(-33.8688, -70.6693, garmin_t, 1, 0, "Sur");
    CHECK(garmin_usb_decode_dog(s.data(), s.size(), GARMIN_USB_PRODUCT_ASTRO_320, &d) == 0);
    CHECK(near(d.latitude_deg, -33.8688, 1e-6) && near(d.longitude_deg, -70.6693, 1e-6));
    return 0;
}

// ─── Status codes are per model, never ANT's ───────────────────────────────
static int dog_status() {
    struct Case { uint16_t product; uint8_t raw; antz_garmin_usb_dog_status_t want; };
    const Case cases[] = {
        {1339, 1, GARMIN_USB_DOG_STATUS_SITTING}, {1339, 2, GARMIN_USB_DOG_STATUS_MOVING},
        {1339, 3, GARMIN_USB_DOG_STATUS_TREED},   {1339, 4, GARMIN_USB_DOG_STATUS_POINTED},
        {1562, 33, GARMIN_USB_DOG_STATUS_SITTING}, {1562, 34, GARMIN_USB_DOG_STATUS_MOVING},
        {1562, 35, GARMIN_USB_DOG_STATUS_TREED},   {1562, 36, GARMIN_USB_DOG_STATUS_POINTED},
        // Everything else is unknown, including the other model's codes, the
        // codes ANT would read as Sitting or Moving, and an unidentified handheld.
        {1339, 0, GARMIN_USB_DOG_STATUS_UNKNOWN},  {1339, 5, GARMIN_USB_DOG_STATUS_UNKNOWN},
        {1339, 33, GARMIN_USB_DOG_STATUS_UNKNOWN}, {1562, 1, GARMIN_USB_DOG_STATUS_UNKNOWN},
        {1562, 65, GARMIN_USB_DOG_STATUS_UNKNOWN}, {0, 1, GARMIN_USB_DOG_STATUS_UNKNOWN},
        {1339, 0xFF, GARMIN_USB_DOG_STATUS_UNKNOWN},
    };
    for (const Case& c : cases) {
        auto p = dog_packet(59.0, 10.0, 1000, c.raw, 0, "X");
        antz_garmin_usb_dog_t d{};
        CHECK(garmin_usb_decode_dog(p.data(), p.size(), c.product, &d) == 0);
        CHECK(d.status == c.want);
        CHECK(d.status_raw == c.raw);     // kept whatever the name
    }
    return 0;
}

// ─── GPS loss: the flag, and a position that stays put ──────────────────────
static int dog_gps_loss() {
    // Confirmed on both models: byte 17 bit 0x20 is GPS lost. The position
    // and status are returned as sent; the caller decides what a fix is.
    for (uint16_t product : {GARMIN_USB_PRODUCT_ASTRO_320, GARMIN_USB_PRODUCT_ALPHA_100}) {
        auto p = dog_packet(60.0, 11.0, 5000, product == 1339 ? 2 : 34, 0x20, "Ragnar");
        p[26] = 0;
        antz_garmin_usb_dog_t d{};
        CHECK(garmin_usb_decode_dog(p.data(), p.size(), product, &d) == 0);
        CHECK(d.gps_lost_known == 1 && d.gps_lost == 1);
        CHECK(d.status == GARMIN_USB_DOG_STATUS_MOVING);    // status is not proof of a fix
        CHECK(d.gps_bars == 0 && d.comm_bars == 5);         // radio is fine, GPS is not
        CHECK(near(d.latitude_deg, 60.0, 1e-6));            // the last coordinates, still sent
        CHECK(d.flags == 0x20);
    }
    // Bit 0x10 is only likely low battery; it is not GPS loss.
    auto p = dog_packet(60.0, 11.0, 5000, 2, 0x10, "Ragnar");
    antz_garmin_usb_dog_t d{};
    CHECK(garmin_usb_decode_dog(p.data(), p.size(), GARMIN_USB_PRODUCT_ASTRO_320, &d) == 0);
    CHECK(d.gps_lost == 0 && d.flags == 0x10);
    // For an unidentified model the bit is not interpreted at all.
    auto q = dog_packet(60.0, 11.0, 5000, 2, 0x20, "Ragnar");
    CHECK(garmin_usb_decode_dog(q.data(), q.size(), 0, &d) == 0);
    CHECK(d.gps_lost_known == 0 && d.gps_lost == 0 && d.flags == 0x20);
    CHECK(d.gps_bars_max == 0 && d.track_color_known == 0);
    return 0;
}

// ─── Names, and the bytes that are not an identity ─────────────────────────
static int dog_name_and_evidence() {
    // The name ends at its own NUL whatever length it has, and the tail
    // after it is not read.
    for (const char* name : {"A", "Ragnar", "RagnarXX", "RagnarXX123456"}) {
        auto p = dog_packet(59.0, 10.0, 1000, 1, 0, name);
        antz_garmin_usb_dog_t d{};
        CHECK(garmin_usb_decode_dog(p.data(), p.size(), GARMIN_USB_PRODUCT_ASTRO_320, &d) == 0);
        CHECK(std::strcmp(d.name, name) == 0);
    }
    // A name with no terminator runs to the end of the packet and no further.
    std::vector<uint8_t> p(100, 'N');
    for (size_t i = 0; i < 31; ++i) p[i] = 0;
    antz_garmin_usb_dog_t d{};
    CHECK(garmin_usb_decode_dog(p.data(), p.size(), GARMIN_USB_PRODUCT_ASTRO_320, &d) == 0);
    CHECK(std::strlen(d.name) == 69 && d.name[69] == '\0');
    // An empty name is empty, not garbage.
    auto e = dog_packet(59.0, 10.0, 1000, 1, 0, "");
    CHECK(garmin_usb_decode_dog(e.data(), e.size(), GARMIN_USB_PRODUCT_ASTRO_320, &d) == 0);
    CHECK(d.name[0] == '\0');
    // Bytes 30 and 40.. change nothing the decoder returns: they are not keys.
    auto a = dog_packet(59.0, 10.0, 1000, 1, 0, "Ragnar");
    auto b = a;
    b[30] = 211; b[40] = 99; b[88] = 0xAA; b[92] = 0xBB;
    antz_garmin_usb_dog_t da{}, db{};
    CHECK(garmin_usb_decode_dog(a.data(), a.size(), 1339, &da) == 0);
    CHECK(garmin_usb_decode_dog(b.data(), b.size(), 1339, &db) == 0);
    CHECK(std::memcmp(&da, &db, sizeof da) == 0);
    return 0;
}

// ─── A packet that is too short is refused ─────────────────────────────────
static int dog_refused() {
    auto p = dog_packet(59.0, 10.0, 1000, 1, 0, "Ragnar");
    antz_garmin_usb_dog_t d{};
    CHECK(garmin_usb_decode_dog(p.data(), 40, 1339, &d) == 0);      // the fixed part and a name
    CHECK(garmin_usb_decode_dog(p.data(), 39, 1339, &d) != 0);
    CHECK(garmin_usb_decode_dog(p.data(), 0, 1339, &d) != 0);
    CHECK(garmin_usb_decode_dog(nullptr, 100, 1339, &d) != 0);
    CHECK(garmin_usb_decode_dog(p.data(), 100, 1339, nullptr) != 0);
    return 0;
}

// ─── Framing ────────────────────────────────────────────────────────────────
struct Seen {
    std::vector<antz_garmin_usb_header_t> headers;
    std::vector<std::vector<uint8_t>> payloads;
    std::vector<bool> had_data;
};
static void on_packet(const antz_garmin_usb_packet_t* pk, void* ctx) {
    Seen* s = static_cast<Seen*>(ctx);
    s->headers.push_back({pk->layer, pk->pid, pk->size});
    s->had_data.push_back(pk->data != nullptr);
    s->payloads.emplace_back(pk->data ? std::vector<uint8_t>(pk->data, pk->data + pk->size)
                                      : std::vector<uint8_t>());
}

static int framer() {
    // One packet, fed whole.
    antz_garmin_usb_framer_t fr;
    garmin_usb_framer_init(&fr);
    Seen seen;
    auto dog = dog_packet(59.0, 10.0, 1000, 1, 0, "Ragnar");
    auto f = frame(20, 3078, dog);
    CHECK(garmin_usb_framer_feed(&fr, f.data(), f.size(), on_packet, &seen) == 0);
    CHECK(seen.headers.size() == 1 && seen.headers[0].pid == 3078 && seen.headers[0].size == 100);
    CHECK(seen.payloads[0] == dog);

    // The same bytes fed one at a time give the same packet.
    garmin_usb_framer_init(&fr);
    Seen one;
    for (uint8_t b : f) CHECK(garmin_usb_framer_feed(&fr, &b, 1, on_packet, &one) == 0);
    CHECK(one.headers.size() == 1 && one.payloads[0] == dog);

    // A read that ends mid header, and one that ends mid payload.
    garmin_usb_framer_init(&fr);
    Seen split;
    CHECK(garmin_usb_framer_feed(&fr, f.data(), 7, on_packet, &split) == 0);
    CHECK(split.headers.empty());
    CHECK(garmin_usb_framer_feed(&fr, f.data() + 7, 60, on_packet, &split) == 0);
    CHECK(split.headers.empty());
    CHECK(garmin_usb_framer_feed(&fr, f.data() + 67, f.size() - 67, on_packet, &split) == 0);
    CHECK(split.headers.size() == 1 && split.payloads[0] == dog);

    // Two packets glued in one read, and a third starting at its end.
    garmin_usb_framer_init(&fr);
    Seen glued;
    std::vector<uint8_t> two = f;
    auto pv = frame(20, 51, std::vector<uint8_t>(64, 1));
    two.insert(two.end(), pv.begin(), pv.end());
    auto third = frame(0, 6, std::vector<uint8_t>(4, 9));
    two.insert(two.end(), third.begin(), third.begin() + 5);
    CHECK(garmin_usb_framer_feed(&fr, two.data(), two.size(), on_packet, &glued) == 0);
    CHECK(glued.headers.size() == 2 && glued.headers[1].pid == 51 && glued.headers[1].size == 64);
    CHECK(garmin_usb_framer_feed(&fr, third.data() + 5, third.size() - 5, on_packet, &glued) == 0);
    CHECK(glued.headers.size() == 3 && glued.headers[2].layer == 0 && glued.headers[2].pid == 6);

    // A packet with no payload is a packet.
    garmin_usb_framer_init(&fr);
    Seen empty;
    auto none = frame(0, 2, {});
    CHECK(garmin_usb_framer_feed(&fr, none.data(), none.size(), on_packet, &empty) == 0);
    CHECK(empty.headers.size() == 1 && empty.headers[0].size == 0 && empty.had_data[0]);
    return 0;
}

static int framer_oversize_and_corrupt() {
    // A payload larger than the buffer is reported without it, its bytes are
    // skipped across reads, and the next packet still arrives intact.
    antz_garmin_usb_framer_t fr;
    garmin_usb_framer_init(&fr);
    Seen seen;
    auto big = frame(20, 248, std::vector<uint8_t>(GARMIN_USB_FRAMER_BUFFER + 100, 0x55));
    auto dog = frame(20, 3078, dog_packet(59.0, 10.0, 1000, 1, 0, "Ragnar"));
    std::vector<uint8_t> all = big;
    all.insert(all.end(), dog.begin(), dog.end());
    for (size_t at = 0; at < all.size(); at += 37) {
        const size_t n = std::min<size_t>(37, all.size() - at);
        CHECK(garmin_usb_framer_feed(&fr, all.data() + at, n, on_packet, &seen) == 0);
    }
    CHECK(seen.headers.size() == 2);
    CHECK(seen.headers[0].pid == 248 && seen.headers[0].size == GARMIN_USB_FRAMER_BUFFER + 100);
    CHECK(!seen.had_data[0]);
    CHECK(seen.headers[1].pid == 3078 && seen.had_data[1] && seen.payloads[1].size() == 100);

    // A packet of exactly the buffer's size is buffered, not skipped.
    garmin_usb_framer_init(&fr);
    Seen exact;
    auto fit = frame(20, 248, std::vector<uint8_t>(GARMIN_USB_FRAMER_BUFFER, 0x33));
    CHECK(garmin_usb_framer_feed(&fr, fit.data(), fit.size(), on_packet, &exact) == 0);
    CHECK(exact.headers.size() == 1 && exact.had_data[0]);

    // An invalid layer stops the framer for good: nothing after a corrupt
    // header can be trusted to be a frame.
    garmin_usb_framer_init(&fr);
    Seen bad;
    auto corrupt = frame(7, 3078, std::vector<uint8_t>(10, 0));
    CHECK(garmin_usb_framer_feed(&fr, corrupt.data(), corrupt.size(), on_packet, &bad) != 0);
    CHECK(bad.headers.empty());
    CHECK(garmin_usb_framer_feed(&fr, dog.data(), dog.size(), on_packet, &bad) != 0);
    CHECK(bad.headers.empty());
    // Re-initialised, it reads again.
    garmin_usb_framer_init(&fr);
    CHECK(garmin_usb_framer_feed(&fr, dog.data(), dog.size(), on_packet, &bad) == 0);
    CHECK(bad.headers.size() == 1);

    // A size above the limit is corrupt too.
    garmin_usb_framer_init(&fr);
    auto huge = frame(20, 3078, std::vector<uint8_t>(4, 0));
    put_u32(huge, 8, GARMIN_USB_MAX_PACKET_SIZE + 1);
    CHECK(garmin_usb_framer_feed(&fr, huge.data(), huge.size(), on_packet, &bad) != 0);

    // Null input is refused and changes nothing.
    garmin_usb_framer_init(&fr);
    CHECK(garmin_usb_framer_feed(&fr, nullptr, 5, on_packet, &bad) != 0);
    CHECK(garmin_usb_framer_feed(nullptr, dog.data(), dog.size(), on_packet, &bad) != 0);
    return 0;
}

// ─── Replay of real recordings (private, set ANTZ_GARMIN_USB_REPLAY) ───────
static int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}
static bool json_field(const std::string& line, const char* key, std::string* out) {
    const std::string k = std::string("\"") + key + "\"";
    size_t at = line.find(k);
    if (at == std::string::npos) return false;
    at = line.find(':', at);
    if (at == std::string::npos) return false;
    ++at;
    while (at < line.size() && line[at] == ' ') ++at;
    size_t end;
    if (line[at] == '"') { ++at; end = line.find('"', at); }
    else { end = line.find_first_of(",}", at); }
    if (end == std::string::npos) return false;
    *out = line.substr(at, end - at);
    return true;
}

static int replay() {
    const char* path = std::getenv("ANTZ_GARMIN_USB_REPLAY");
    if (!path) { std::printf("replay: ANTZ_GARMIN_USB_REPLAY not set, skipped\n"); return 0; }
    std::ifstream in(path);
    CHECK(in.good());
    const uint16_t product = static_cast<uint16_t>(
        std::getenv("ANTZ_GARMIN_USB_PRODUCT") ? std::atoi(std::getenv("ANTZ_GARMIN_USB_PRODUCT")) : 0);
    std::string line;
    int dogs = 0;
    while (std::getline(in, line)) {
        std::string pid, hex;
        if (!json_field(line, "pid", &pid) || !json_field(line, "hex", &hex)) continue;
        if (std::atoi(pid.c_str()) != GARMIN_USB_PID_DOG_DATA) continue;
        std::vector<uint8_t> bytes;
        for (size_t i = 0; i + 1 < hex.size(); i += 2)
            bytes.push_back(static_cast<uint8_t>(hex_nibble(hex[i]) * 16 + hex_nibble(hex[i + 1])));
        antz_garmin_usb_dog_t d{};
        CHECK(garmin_usb_decode_dog(bytes.data(), bytes.size(), product, &d) == 0);
        ++dogs;
        // What holds for any real packet from these handhelds.
        CHECK(d.latitude_deg >= -90.0 && d.latitude_deg <= 90.0);
        CHECK(d.longitude_deg >= -180.0 && d.longitude_deg <= 180.0);
        CHECK(d.unix_time_s >= GARMIN_USB_EPOCH_UNIX_S);
        CHECK(d.comm_bars <= 5);
        CHECK(std::strlen(d.name) <= GARMIN_USB_DOG_NAME_MAX);
    }
    CHECK(dogs > 0);
    std::printf("replay: %d dog packets decoded\n", dogs);
    return 0;
}

int main(int argc, char** argv) {
    struct Group { const char* name; int (*fn)(); };
    const Group groups[] = {
        {"header", header}, {"session_and_product", session_and_product}, {"pvt", pvt},
        {"dog.fields", dog_fields}, {"dog.status", dog_status}, {"dog.gps_loss", dog_gps_loss},
        {"dog.name_and_evidence", dog_name_and_evidence}, {"dog.refused", dog_refused},
        {"framer", framer}, {"framer.oversize_and_corrupt", framer_oversize_and_corrupt},
        {"replay", replay},
    };
    for (const Group& g : groups) {
        if (argc > 1 && std::strcmp(argv[1], g.name) != 0) continue;
        if (g.fn() != 0) return 1;
    }
    std::printf("ok (%d checks)\n", checks);
    return 0;
}
