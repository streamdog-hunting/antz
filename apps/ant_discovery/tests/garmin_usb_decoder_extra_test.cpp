//
// Extra decoder cases, written by the test author from docs/protocol.md and
// the rules stated in garmin_usb_decoder.h. They complement
// garmin_usb_test.cpp and do not change it.
//
#include "profiles/garmin_usb/garmin_usb_decoder.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace antz;

static int checks = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); return 1; } } while (0)

static bool near(const double a, const double b, const double eps = 1e-6) {
    return std::fabs(a - b) <= eps;
}

typedef std::vector<uint8_t> Bytes;
static void put_u16(Bytes& v, size_t at, uint16_t x) { v[at] = x & 0xFF; v[at + 1] = x >> 8; }
static void put_u32(Bytes& v, size_t at, uint32_t x) {
    for (int i = 0; i < 4; ++i) v[at + i] = (x >> (8 * i)) & 0xFF;
}
static void put_f32(Bytes& v, size_t at, float x) {
    uint32_t b; std::memcpy(&b, &x, 4); put_u32(v, at, b);
}
static void put_f64(Bytes& v, size_t at, double x) {
    uint64_t b; std::memcpy(&b, &x, 8);
    for (int i = 0; i < 8; ++i) v[at + i] = (b >> (8 * i)) & 0xFF;
}
static int32_t semicircles(const double deg) {
    return static_cast<int32_t>(std::lround(deg * 2147483648.0 / 180.0));
}
static Bytes dog_packet(double lat, double lon, uint32_t garmin_time,
                        uint8_t status, uint8_t flags, const char* name) {
    Bytes p(100, 0);
    put_u32(p, 0, static_cast<uint32_t>(semicircles(lat)));
    put_u32(p, 4, static_cast<uint32_t>(semicircles(lon)));
    put_f32(p, 8, 312.5f);
    put_u32(p, 12, garmin_time);
    p[16] = status;
    p[17] = flags;
    put_u16(p, 22, 11);
    p[24] = 12;
    p[25] = 40;
    p[26] = 5;
    p[27] = 5;
    p[30] = 207;
    const size_t n = std::strlen(name);
    std::memcpy(&p[31], name, n);
    p[31 + n] = 0;
    for (size_t i = 32 + n; i < p.size(); ++i) p[i] = 'Z';
    return p;
}
static Bytes frame(uint8_t layer, uint16_t pid, const Bytes& payload) {
    Bytes f(12 + payload.size(), 0);
    f[0] = layer;
    put_u16(f, 4, pid);
    put_u32(f, 8, static_cast<uint32_t>(payload.size()));
    if (!payload.empty()) std::memcpy(&f[12], payload.data(), payload.size());
    return f;
}

struct Seen {
    std::vector<antz_garmin_usb_header_t> headers;
    std::vector<bool> had_data;
};
static void on_packet(const antz_garmin_usb_packet_t* pk, void* ctx) {
    Seen* s = static_cast<Seen*>(ctx);
    s->headers.push_back({pk->layer, pk->pid, pk->size});
    s->had_data.push_back(pk->data != nullptr);
}

// ─── Header ─────────────────────────────────────────────────────────────────
static int header_layers_and_reserved() {
    antz_garmin_usb_header_t h{};
    // Only layers 0 and 20 are valid.
    for (int layer : {1, 2, 19, 21, 100, 255}) {
        auto f = frame(static_cast<uint8_t>(layer), 3078, Bytes(4, 0));
        CHECK(garmin_usb_decode_header(f.data(), f.size(), &h) != 0);
    }
    // The reserved bytes (1..3 and 6..7) are not read: nonzero is still valid.
    auto f = frame(20, 3078, Bytes(4, 0));
    f[1] = f[2] = f[3] = 0xFF; f[6] = f[7] = 0xFF;
    CHECK(garmin_usb_decode_header(f.data(), f.size(), &h) == 0);
    CHECK(h.layer == 20 && h.pid == 3078 && h.size == 4);
    // A pid uses both bytes (0x0C06) and a zero size is fine; high size bytes count.
    auto z = frame(0, 0xFFFF, Bytes());
    CHECK(garmin_usb_decode_header(z.data(), z.size(), &h) == 0);
    CHECK(h.pid == 0xFFFF && h.size == 0);
    put_u32(z, 8, 0x00010000);
    CHECK(garmin_usb_decode_header(z.data(), z.size(), &h) == 0 && h.size == 65536);
    put_u32(z, 8, 0xFFFFFFFFu);
    CHECK(garmin_usb_decode_header(z.data(), z.size(), &h) != 0);
    // Exactly 12 bytes is enough.
    CHECK(garmin_usb_decode_header(f.data(), 12, &h) == 0);
    return 0;
}

// ─── Small packets ──────────────────────────────────────────────────────────
static int small_packets() {
    uint32_t unit = 1;
    Bytes s(4, 0xFF);
    CHECK(garmin_usb_decode_session_started(s.data(), 4, &unit) == 0 && unit == 0xFFFFFFFFu);
    CHECK(garmin_usb_decode_session_started(s.data(), 4, nullptr) != 0);
    // Extra bytes after the unit id are not read.
    Bytes more = {1, 0, 0, 0, 9, 9, 9, 9};
    CHECK(garmin_usb_decode_session_started(more.data(), more.size(), &unit) == 0 && unit == 1);

    // The software version is signed.
    Bytes p(4, 0);
    put_u16(p, 0, 1339); put_u16(p, 2, 0xFFFF);
    antz_garmin_usb_product_t prod{};
    CHECK(garmin_usb_decode_product(p.data(), 4, &prod) == 0);
    CHECK(prod.product_id == 1339 && prod.software_version_x100 == -1);
    CHECK(garmin_usb_decode_product(p.data(), 4, nullptr) != 0);
    CHECK(garmin_usb_decode_product(nullptr, 4, &prod) != 0);
    return 0;
}

// ─── Pvt ────────────────────────────────────────────────────────────────────
static int pvt_hemispheres_and_geoid() {
    Bytes d(64, 0);
    put_f32(d, 0, 10.0f);
    put_f64(d, 26, -33.8688 * M_PI / 180.0);
    put_f64(d, 34, -70.6693 * M_PI / 180.0);
    put_f32(d, 54, -20.0f);                      // the geoid can be below the ellipsoid
    put_f32(d, 4, 0.0f);
    antz_garmin_usb_pvt_t out{};
    CHECK(garmin_usb_decode_pvt(d.data(), d.size(), &out) == 0);
    CHECK(near(out.latitude_deg, -33.8688) && near(out.longitude_deg, -70.6693));
    CHECK(near(out.altitude_m, -10.0, 1e-3));    // alt + msl
    // More than 64 bytes is fine; the fixed part is what is read.
    Bytes big(80, 0xEE);
    std::memcpy(big.data(), d.data(), 64);
    CHECK(garmin_usb_decode_pvt(big.data(), big.size(), &out) == 0);
    CHECK(near(out.latitude_deg, -33.8688));
    CHECK(garmin_usb_decode_pvt(d.data(), 64, nullptr) != 0);
    return 0;
}

// ─── Semicircles and time ───────────────────────────────────────────────────
static int semicircle_extremes() {
    CHECK(garmin_usb_semicircles_to_degrees(0) == 0.0);
    CHECK(near(garmin_usb_semicircles_to_degrees(INT32_MIN), -180.0, 1e-9));
    CHECK(near(garmin_usb_semicircles_to_degrees(0x40000000), 90.0, 1e-9));
    CHECK(near(garmin_usb_semicircles_to_degrees(-0x40000000), -90.0, 1e-9));
    CHECK(garmin_usb_semicircles_to_degrees(INT32_MAX) < 180.0);
    CHECK(near(garmin_usb_semicircles_to_degrees(INT32_MAX), 180.0, 1e-6));
    return 0;
}

static int dog_time() {
    antz_garmin_usb_dog_t d{};
    // Garmin time 0 is 1989-12-31 00:00:00 UTC.
    auto p = dog_packet(59.0, 10.0, 0, 1, 0, "T");
    CHECK(garmin_usb_decode_dog(p.data(), p.size(), 1339, &d) == 0);
    CHECK(d.garmin_time_s == 0 && d.unix_time_s == 631065600);
    // The top of the u32 does not wrap on the 64 bit unix time.
    auto q = dog_packet(59.0, 10.0, 0xFFFFFFFFu, 1, 0, "T");
    CHECK(garmin_usb_decode_dog(q.data(), q.size(), 1339, &d) == 0);
    CHECK(d.garmin_time_s == 0xFFFFFFFFu);
    CHECK(d.unix_time_s == 631065600LL + 4294967295LL);
    // One second on is one second on.
    auto r = dog_packet(59.0, 10.0, 86400, 1, 0, "T");
    CHECK(garmin_usb_decode_dog(r.data(), r.size(), 1339, &d) == 0);
    CHECK(d.unix_time_s == 631065600LL + 86400);
    return 0;
}

// ─── GPS loss is bit 0x20 and only bit 0x20 ─────────────────────────────────
static int dog_gps_loss_bit_only() {
    struct Case { uint8_t flags; uint8_t lost; };
    const Case cases[] = {{0x00, 0}, {0x20, 1}, {0x30, 1}, {0xFF, 1}, {0xDF, 0},
                          {0x10, 0}, {0x01, 0}, {0x40, 0}, {0x80, 0}, {0x2F, 1}};
    for (uint16_t product : {GARMIN_USB_PRODUCT_ASTRO_320, GARMIN_USB_PRODUCT_ALPHA_100}) {
        for (const Case& c : cases) {
            auto p = dog_packet(60.0, 11.0, 5000, product == 1339 ? 2 : 34, c.flags, "K");
            antz_garmin_usb_dog_t d{};
            CHECK(garmin_usb_decode_dog(p.data(), p.size(), product, &d) == 0);
            CHECK(d.gps_lost_known == 1 && d.gps_lost == c.lost);
            CHECK(d.flags == c.flags);
        }
    }
    // Any other handheld: the bit is never read, whichever product it is.
    for (uint16_t product : {uint16_t(0), uint16_t(1338), uint16_t(1340), uint16_t(1561), uint16_t(1563), uint16_t(0xFFFF)}) {
        auto p = dog_packet(60.0, 11.0, 5000, 2, 0xFF, "K");
        antz_garmin_usb_dog_t d{};
        CHECK(garmin_usb_decode_dog(p.data(), p.size(), product, &d) == 0);
        CHECK(d.gps_lost_known == 0 && d.gps_lost == 0 && d.flags == 0xFF);
    }
    return 0;
}

// ─── Fields that only the two models own, and the evidence that is always kept
static int dog_model_gating() {
    for (uint16_t product : {uint16_t(0), uint16_t(1340), uint16_t(1563)}) {
        auto p = dog_packet(59.0, 10.0, 1000, 2, 0, "K");
        antz_garmin_usb_dog_t d{};
        CHECK(garmin_usb_decode_dog(p.data(), p.size(), product, &d) == 0);
        CHECK(d.gps_bars_max == 0);
        CHECK(d.track_color_known == 0 && d.track_color_code == 0);   // 0 otherwise, byte 24 is 12
        CHECK(d.configured_id_known == 0 && d.configured_id == 0);
        CHECK(d.status == GARMIN_USB_DOG_STATUS_UNKNOWN && d.status_raw == 2);
        // Raw level, GPS and comm bars and the id bytes are still returned.
        CHECK(d.battery_level_raw == 40 && d.gps_bars == 5 && d.comm_bars == 5);
        CHECK(d.id_bytes[0] == 11 && d.id_bytes[1] == 0);
    }
    // Alpha: no configured id even if byte 23 is zero, colour known, bars max 3.
    auto a = dog_packet(59.0, 10.0, 1000, 33, 0, "K");
    a[26] = 3;
    antz_garmin_usb_dog_t d{};
    CHECK(garmin_usb_decode_dog(a.data(), a.size(), 1562, &d) == 0);
    CHECK(d.configured_id_known == 0 && d.configured_id == 0);
    CHECK(d.track_color_known == 1 && d.track_color_code == 12);
    CHECK(d.gps_bars == 3 && d.gps_bars_max == 3);
    // Astro: the configured id is the whole u16 at 22..23.
    auto s = dog_packet(59.0, 10.0, 1000, 1, 0, "K");
    s[22] = 0x34; s[23] = 0x12;
    CHECK(garmin_usb_decode_dog(s.data(), s.size(), 1339, &d) == 0);
    CHECK(d.configured_id_known == 1 && d.configured_id == 0x1234);
    CHECK(d.id_bytes[0] == 0x34 && d.id_bytes[1] == 0x12);
    return 0;
}

// ─── Altitude is probable, not required; battery is raw ─────────────────────
static int dog_battery_and_comm_raw() {
    antz_garmin_usb_dog_t d{};
    for (int raw : {0, 1, 2, 20, 40, 60, 255}) {
        auto p = dog_packet(59.0, 10.0, 1000, 1, 0, "K");
        p[25] = static_cast<uint8_t>(raw);
        CHECK(garmin_usb_decode_dog(p.data(), p.size(), 1339, &d) == 0);
        CHECK(d.battery_level_raw == raw);
    }
    return 0;
}

// ─── Names ──────────────────────────────────────────────────────────────────
static int dog_name_short_packet_and_bytes() {
    antz_garmin_usb_dog_t d{};
    // A packet of 45 bytes with no NUL: the name stops where the packet stops.
    Bytes p(45, 'N');
    for (size_t i = 0; i < 31; ++i) p[i] = 0;
    CHECK(garmin_usb_decode_dog(p.data(), p.size(), 1339, &d) == 0);
    CHECK(std::strlen(d.name) == 14);
    // Fixed part only (40 bytes): the name has 9 bytes.
    Bytes q(40, 'M');
    for (size_t i = 0; i < 31; ++i) q[i] = 0;
    CHECK(garmin_usb_decode_dog(q.data(), q.size(), 1339, &d) == 0);
    CHECK(std::strlen(d.name) == 9);
    // A NUL at byte 31 is an empty name even when text follows.
    Bytes e(100, 'X');
    for (size_t i = 0; i < 31; ++i) e[i] = 0;
    e[31] = 0;
    CHECK(garmin_usb_decode_dog(e.data(), e.size(), 1339, &d) == 0);
    CHECK(d.name[0] == '\0');
    // Bytes above 0x7F (UTF-8 for Norwegian) are kept as they are.
    auto n = dog_packet(59.0, 10.0, 1000, 1, 0, "F\xC3\xB8y");
    CHECK(garmin_usb_decode_dog(n.data(), n.size(), 1339, &d) == 0);
    CHECK(std::strcmp(d.name, "F\xC3\xB8y") == 0);
    // Bytes 18..21, 28..29 and the area from 40 are not read at all.
    auto a = dog_packet(59.0, 10.0, 1000, 1, 0, "Khione");
    auto b = a;
    b[18] = b[19] = b[20] = b[21] = 0xAB; b[28] = b[29] = 0xCD;
    antz_garmin_usb_dog_t da{}, db{};
    CHECK(garmin_usb_decode_dog(a.data(), a.size(), 1562, &da) == 0);
    CHECK(garmin_usb_decode_dog(b.data(), b.size(), 1562, &db) == 0);
    CHECK(std::memcmp(&da, &db, sizeof da) == 0);
    return 0;
}

// ─── A refused packet leaves the output alone ───────────────────────────────
static int dog_refused_keeps_output() {
    auto p = dog_packet(59.0, 10.0, 1000, 1, 0, "Keep");
    antz_garmin_usb_dog_t d{};
    CHECK(garmin_usb_decode_dog(p.data(), p.size(), 1339, &d) == 0);
    antz_garmin_usb_dog_t before = d;
    CHECK(garmin_usb_decode_dog(p.data(), 39, 1339, &d) != 0);
    CHECK(std::memcmp(&before, &d, sizeof d) == 0);
    return 0;
}

// ─── Framer ─────────────────────────────────────────────────────────────────
static int framer_edge_cases() {
    antz_garmin_usb_framer_t fr;
    // Whole packets before a corrupt header in the same read still arrive,
    // then the framer fails and stays failed.
    garmin_usb_framer_init(&fr);
    Seen seen;
    Bytes stream = frame(20, 51, Bytes(64, 1));
    Bytes bad = frame(9, 1, Bytes(4, 0));
    stream.insert(stream.end(), bad.begin(), bad.end());
    CHECK(garmin_usb_framer_feed(&fr, stream.data(), stream.size(), on_packet, &seen) != 0);
    CHECK(seen.headers.size() == 1 && seen.headers[0].pid == 51);
    Bytes ok = frame(20, 51, Bytes(64, 1));
    CHECK(garmin_usb_framer_feed(&fr, ok.data(), ok.size(), on_packet, &seen) != 0);
    CHECK(seen.headers.size() == 1);

    // A corrupt header split across reads is found when it completes.
    garmin_usb_framer_init(&fr);
    Seen split;
    CHECK(garmin_usb_framer_feed(&fr, bad.data(), 6, on_packet, &split) == 0);
    CHECK(garmin_usb_framer_feed(&fr, bad.data() + 6, bad.size() - 6, on_packet, &split) != 0);
    CHECK(split.headers.empty());

    // An oversize packet at the limit of what the header may claim is
    // skipped, however many reads it takes, and the next packet arrives.
    garmin_usb_framer_init(&fr);
    Seen big;
    Bytes hdr = frame(20, 248, Bytes());
    put_u32(hdr, 8, GARMIN_USB_MAX_PACKET_SIZE);
    CHECK(garmin_usb_framer_feed(&fr, hdr.data(), hdr.size(), on_packet, &big) == 0);
    CHECK(big.headers.size() == 1 && big.headers[0].size == GARMIN_USB_MAX_PACKET_SIZE && !big.had_data[0]);
    Bytes filler(4096, 0x77);
    for (size_t sent = 0; sent < GARMIN_USB_MAX_PACKET_SIZE; sent += filler.size())
        CHECK(garmin_usb_framer_feed(&fr, filler.data(), filler.size(), on_packet, &big) == 0);
    CHECK(big.headers.size() == 1);
    Bytes next = frame(20, 3078, dog_packet(59.0, 10.0, 1000, 1, 0, "N"));
    CHECK(garmin_usb_framer_feed(&fr, next.data(), next.size(), on_packet, &big) == 0);
    CHECK(big.headers.size() == 2 && big.headers[1].pid == 3078 && big.had_data[1]);

    // One byte over the buffer is oversize; the buffer's size is not.
    garmin_usb_framer_init(&fr);
    Seen one;
    Bytes over = frame(20, 248, Bytes(GARMIN_USB_FRAMER_BUFFER + 1, 1));
    CHECK(garmin_usb_framer_feed(&fr, over.data(), over.size(), on_packet, &one) == 0);
    CHECK(one.headers.size() == 1 && !one.had_data[0]);

    // An empty feed is not an error and delivers nothing.
    garmin_usb_framer_init(&fr);
    Seen none;
    CHECK(garmin_usb_framer_feed(&fr, ok.data(), 0, on_packet, &none) == 0);
    CHECK(none.headers.empty());
    // A null callback is refused.
    CHECK(garmin_usb_framer_feed(&fr, ok.data(), ok.size(), nullptr, &none) != 0);
    return 0;
}

static int framer_every_split_point() {
    // A packet pair cut at every possible byte gives the same two packets.
    Bytes a = frame(20, 3078, dog_packet(59.0, 10.0, 1000, 1, 0, "Khione"));
    Bytes b = frame(0, 6, Bytes(4, 9));
    Bytes all = a; all.insert(all.end(), b.begin(), b.end());
    for (size_t cut = 1; cut < all.size(); ++cut) {
        antz_garmin_usb_framer_t fr;
        garmin_usb_framer_init(&fr);
        Seen seen;
        CHECK(garmin_usb_framer_feed(&fr, all.data(), cut, on_packet, &seen) == 0);
        CHECK(garmin_usb_framer_feed(&fr, all.data() + cut, all.size() - cut, on_packet, &seen) == 0);
        CHECK(seen.headers.size() == 2);
        CHECK(seen.headers[0].pid == 3078 && seen.headers[0].size == 100);
        CHECK(seen.headers[1].pid == 6 && seen.headers[1].layer == 0);
    }
    return 0;
}

int main(int argc, char** argv) {
    struct Group { const char* name; int (*fn)(); };
    const Group groups[] = {
        {"header", header_layers_and_reserved}, {"small_packets", small_packets},
        {"pvt", pvt_hemispheres_and_geoid}, {"semicircles", semicircle_extremes},
        {"dog.time", dog_time}, {"dog.gps_loss_bit", dog_gps_loss_bit_only},
        {"dog.model_gating", dog_model_gating}, {"dog.battery_raw", dog_battery_and_comm_raw},
        {"dog.name", dog_name_short_packet_and_bytes}, {"dog.refused", dog_refused_keeps_output},
        {"framer.edges", framer_edge_cases}, {"framer.split_points", framer_every_split_point},
    };
    for (const Group& g : groups) {
        if (argc > 1 && std::strcmp(argv[1], g.name) != 0) continue;
        if (g.fn() != 0) return 1;
    }
    std::printf("ok (%d checks)\n", checks);
    return 0;
}
