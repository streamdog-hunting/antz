//
// Tests for the Garmin USB session (garmin_usb_session.h), against a fake
// transport: scripted reads per endpoint, recorded writes, injectable
// failures. No handheld, no port.
//
// Expected values come from the documented protocol (astro-usb
// docs/protocol.md and the header comments), not from an implementation:
//   Start_Session   layer 0,  pid 5,   size 0
//   Product_Rqst    layer 20, pid 254, size 0
//   Command_Data    layer 20, pid 10,  payload u16 LE (49 start, 50 stop)
//   Session_Started layer 0,  pid 6,   unit id u32
//   Data_Available  layer 0,  pid 2,   on the interrupt endpoint
//
#include "profiles/garmin_usb/garmin_usb_session.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <vector>

using namespace antz;

static int checks = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); return 1; } } while (0)

static bool near(const double a, const double b, const double eps = 1e-6) {
    return std::fabs(a - b) <= eps;
}

// ─── Builders (as garmin_usb_test.cpp) ──────────────────────────────────────
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

static Bytes cat(const Bytes& a, const Bytes& b) {
    Bytes r = a; r.insert(r.end(), b.begin(), b.end()); return r;
}

static Bytes session_started(uint32_t unit) {
    Bytes p(4, 0); put_u32(p, 0, unit); return frame(0, 6, p);
}
static Bytes data_available() { return frame(0, 2, Bytes()); }
static Bytes product_data(uint16_t id, uint16_t ver) {
    Bytes p(4 + 20, 0);
    put_u16(p, 0, id); put_u16(p, 2, ver);
    std::memcpy(&p[4], "Handheld", 8);
    return frame(20, 255, p);
}
static Bytes dog_frame(uint8_t status, const char* name, double lat = 59.9139, double lon = 10.7522,
                       uint32_t t = 1000) {
    return frame(20, 3078, dog_packet(lat, lon, t, status, 0, name));
}
static Bytes pvt_frame(double lat, double lon) {
    Bytes d(64, 0);
    put_f32(d, 0, 100.0f);
    put_f32(d, 4, 3.5f);
    put_u16(d, 16, 3);
    put_f64(d, 26, lat * M_PI / 180.0);
    put_f64(d, 34, lon * M_PI / 180.0);
    put_f32(d, 54, 41.0f);
    return frame(20, 51, d);
}

// ─── The fake transport ─────────────────────────────────────────────────────
struct ReadCall { int endpoint; uint32_t timeout_ms; size_t cap; };

struct Fake {
    struct Item { Bytes bytes; int err = 0; };
    std::deque<Item> script[2];
    Bytes pending[2];                         // what did not fit the caller's buffer
    std::vector<Bytes> writes;
    std::vector<ReadCall> reads;
    int fail_write_from = -1;                 // writes with index >= this fail
    int write_calls = 0;

    void push(int ep, const Bytes& b) { Item i; i.bytes = b; script[ep].push_back(i); }
    // The same bytes in reads of at most `n`.
    void push_chunked(int ep, const Bytes& b, size_t n) {
        for (size_t at = 0; at < b.size(); at += n)
            push(ep, Bytes(b.begin() + at, b.begin() + std::min(b.size(), at + n)));
    }
    void push_error(int ep) { Item i; i.err = -1; script[ep].push_back(i); }
    void push_nothing(int ep) { push(ep, Bytes()); }

    size_t reads_on(int ep) const {
        size_t n = 0; for (const auto& r : reads) if (r.endpoint == ep) ++n; return n;
    }
};

static int fake_write(void* ctx, const uint8_t* data, size_t len) {
    Fake* f = static_cast<Fake*>(ctx);
    const int idx = f->write_calls++;
    if (f->fail_write_from >= 0 && idx >= f->fail_write_from) return -1;
    f->writes.emplace_back(data, data + len);
    return 0;
}

static int fake_read(void* ctx, antz_garmin_usb_endpoint_t ep, uint8_t* buf, size_t cap, uint32_t timeout_ms) {
    Fake* f = static_cast<Fake*>(ctx);
    f->reads.push_back({static_cast<int>(ep), timeout_ms, cap});
    Bytes& pend = f->pending[ep];
    if (pend.empty()) {
        if (f->script[ep].empty()) return 0;
        Fake::Item it = f->script[ep].front();
        f->script[ep].pop_front();
        if (it.err) return it.err;
        pend = it.bytes;
        if (pend.empty()) return 0;
    }
    const size_t n = std::min(cap, pend.size());
    std::memcpy(buf, pend.data(), n);
    pend.erase(pend.begin(), pend.begin() + n);
    return static_cast<int>(n);
}

// ─── Event log ──────────────────────────────────────────────────────────────
struct Log {
    std::vector<uint32_t> sessions;
    std::vector<antz_garmin_usb_product_t> products;
    std::vector<antz_garmin_usb_pvt_t> pvts;
    std::vector<antz_garmin_usb_dog_t> dogs;
};
static void on_session(void* c, uint32_t u) { static_cast<Log*>(c)->sessions.push_back(u); }
static void on_product(void* c, const antz_garmin_usb_product_t* p) { static_cast<Log*>(c)->products.push_back(*p); }
static void on_pvt(void* c, const antz_garmin_usb_pvt_t* p) { static_cast<Log*>(c)->pvts.push_back(*p); }
static void on_dog(void* c, const antz_garmin_usb_dog_t* d) { static_cast<Log*>(c)->dogs.push_back(*d); }

struct Rig {
    Fake fake;
    Log log;
    antz_garmin_usb_session_t s;
    int init_rc = -1;

    Rig() {
        std::memset(&s, 0xAA, sizeof s);      // init must not depend on prior contents
        antz_garmin_usb_transport_t t = {fake_write, fake_read, &fake};
        antz_garmin_usb_events_t e = {on_session, on_product, on_pvt, on_dog, &log};
        init_rc = garmin_usb_session_init(&s, &t, &e);
    }
    antz_garmin_usb_poll_t poll(uint32_t timeout_ms = 10) { return garmin_usb_session_poll(&s, timeout_ms); }
};

static bool is_packet(const Bytes& w, uint8_t layer, uint16_t pid, const Bytes& payload) {
    return w == frame(layer, pid, payload);
}
static Bytes u16le(uint16_t v) { Bytes b(2, 0); put_u16(b, 0, v); return b; }

// Start, then deliver Session_Started so the stream is running.
static bool open_stream(Rig& r, uint32_t unit = 3886356464u) {
    if (r.init_rc != 0) return false;
    if (garmin_usb_session_start(&r.s) != 0) return false;
    r.fake.push(GARMIN_USB_ENDPOINT_INTERRUPT, session_started(unit));
    return r.poll() == GARMIN_USB_POLL_OK && r.fake.writes.size() == 3;
}

// ─── Start_Session ──────────────────────────────────────────────────────────
static int start() {
    Rig r;
    CHECK(r.init_rc == 0);
    CHECK(r.fake.writes.empty());             // init writes nothing
    CHECK(r.fake.reads.empty());
    CHECK(garmin_usb_session_start(&r.s) == 0);
    CHECK(r.fake.writes.size() == 1);
    const Bytes want = {0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0};
    CHECK(r.fake.writes[0] == want);
    CHECK(r.fake.writes[0].size() == GARMIN_USB_FRAME_HEADER);
    // A second start is refused and writes nothing.
    CHECK(garmin_usb_session_start(&r.s) != 0);
    CHECK(r.fake.writes.size() == 1);
    return 0;
}

static int start_write_fails() {
    Rig r;
    r.fake.fail_write_from = 0;
    CHECK(garmin_usb_session_start(&r.s) != 0);
    CHECK(r.fake.writes.empty());
    // Nothing was started, so a stop has nothing to stop.
    r.fake.fail_write_from = -1;
    CHECK(garmin_usb_session_stop(&r.s) == 0);
    CHECK(r.fake.writes.empty());
    return 0;
}

// ─── init ───────────────────────────────────────────────────────────────────
static int init_refuses() {
    antz_garmin_usb_session_t s;
    Fake f;
    antz_garmin_usb_events_t e = {nullptr, nullptr, nullptr, nullptr, nullptr};
    antz_garmin_usb_transport_t no_write = {nullptr, fake_read, &f};
    antz_garmin_usb_transport_t no_read = {fake_write, nullptr, &f};
    antz_garmin_usb_transport_t none = {nullptr, nullptr, &f};
    antz_garmin_usb_transport_t ok = {fake_write, fake_read, &f};
    CHECK(garmin_usb_session_init(&s, &no_write, &e) != 0);
    CHECK(garmin_usb_session_init(&s, &no_read, &e) != 0);
    CHECK(garmin_usb_session_init(&s, &none, &e) != 0);
    CHECK(garmin_usb_session_init(&s, &ok, &e) == 0);
    return 0;
}

// ─── Session_Started: product and position requests ─────────────────────────
static int handshake() {
    Rig r;
    CHECK(garmin_usb_session_start(&r.s) == 0);
    CHECK(r.fake.writes.size() == 1);
    r.fake.push(GARMIN_USB_ENDPOINT_INTERRUPT, session_started(3886356464u));
    CHECK(r.poll() == GARMIN_USB_POLL_OK);
    CHECK(r.log.sessions.size() == 1 && r.log.sessions[0] == 3886356464u);
    CHECK(r.fake.writes.size() == 3);
    CHECK(is_packet(r.fake.writes[1], 20, 254, Bytes()));            // Product_Rqst, empty
    CHECK(r.fake.writes[1].size() == 12);
    CHECK(is_packet(r.fake.writes[2], 20, 10, u16le(49)));           // Command_Data, Start_Pvt_Data
    CHECK(r.fake.writes[2].size() == 14);
    return 0;
}

static int handshake_once() {
    Rig r;
    CHECK(open_stream(r));
    // A repeated Session_Started, whole or twice in one read, changes nothing.
    r.fake.push(GARMIN_USB_ENDPOINT_INTERRUPT, session_started(3886356464u));
    r.fake.push(GARMIN_USB_ENDPOINT_INTERRUPT, cat(session_started(3886356464u), session_started(3886356464u)));
    CHECK(r.poll() == GARMIN_USB_POLL_OK);
    CHECK(r.poll() == GARMIN_USB_POLL_OK);
    CHECK(r.fake.writes.size() == 3);
    CHECK(r.log.sessions.size() == 1);
    return 0;
}

static int handshake_short_session_started() {
    // A Session_Started without its unit id is not a session.
    Rig r;
    CHECK(garmin_usb_session_start(&r.s) == 0);
    r.fake.push(GARMIN_USB_ENDPOINT_INTERRUPT, frame(0, 6, Bytes(3, 1)));
    r.poll();
    CHECK(r.log.sessions.empty());
    CHECK(r.fake.writes.size() == 1);
    // A proper one afterwards still works.
    r.fake.push(GARMIN_USB_ENDPOINT_INTERRUPT, session_started(77));
    CHECK(r.poll() == GARMIN_USB_POLL_OK);
    CHECK(r.log.sessions.size() == 1 && r.log.sessions[0] == 77);
    CHECK(r.fake.writes.size() == 3);
    return 0;
}

static int poll_passes_timeout() {
    Rig r;
    CHECK(garmin_usb_session_start(&r.s) == 0);
    CHECK(r.poll(1234) == GARMIN_USB_POLL_IDLE);
    CHECK(r.fake.reads.size() == 1);
    CHECK(r.fake.reads[0].endpoint == GARMIN_USB_ENDPOINT_INTERRUPT);
    CHECK(r.fake.reads[0].timeout_ms == 1234);
    CHECK(r.fake.reads[0].cap > 0);
    // Idle is not an error: the next poll reads again.
    CHECK(r.poll(5) == GARMIN_USB_POLL_IDLE);
    CHECK(r.fake.reads.size() == 2 && r.fake.reads[1].timeout_ms == 5);
    return 0;
}

// ─── Product_Data and the dogs ──────────────────────────────────────────────
static int product_and_dogs() {
    // Astro 320.
    {
        Rig r;
        CHECK(open_stream(r));
        r.fake.push(GARMIN_USB_ENDPOINT_INTERRUPT, product_data(1339, 460));
        CHECK(r.poll() == GARMIN_USB_POLL_OK);
        CHECK(r.log.products.size() == 1);
        CHECK(r.log.products[0].product_id == 1339 && r.log.products[0].software_version_x100 == 460);
        r.fake.push(GARMIN_USB_ENDPOINT_INTERRUPT, dog_frame(3, "Ragnar"));
        CHECK(r.poll() == GARMIN_USB_POLL_OK);
        CHECK(r.log.dogs.size() == 1);
        const auto& d = r.log.dogs[0];
        CHECK(d.status_raw == 3 && d.status == GARMIN_USB_DOG_STATUS_TREED);
        CHECK(std::strcmp(d.name, "Ragnar") == 0);
        CHECK(near(d.latitude_deg, 59.9139) && near(d.longitude_deg, 10.7522));
        CHECK(d.gps_bars_max == 5 && d.configured_id_known == 1 && d.configured_id == 11);
        CHECK(d.gps_lost_known == 1);
    }
    // Alpha 100: its codes are 33..36, and the Astro's are unknown on it.
    {
        Rig r;
        CHECK(open_stream(r));
        r.fake.push(GARMIN_USB_ENDPOINT_INTERRUPT, product_data(1562, 860));
        CHECK(r.poll() == GARMIN_USB_POLL_OK);
        CHECK(r.log.products.size() == 1 && r.log.products[0].product_id == 1562);
        CHECK(r.log.products[0].software_version_x100 == 860);
        r.fake.push(GARMIN_USB_ENDPOINT_INTERRUPT, dog_frame(36, "Hund 1"));
        r.fake.push(GARMIN_USB_ENDPOINT_INTERRUPT, dog_frame(2, "Hund 6"));
        CHECK(r.poll() == GARMIN_USB_POLL_OK);
        CHECK(r.poll() == GARMIN_USB_POLL_OK);
        CHECK(r.log.dogs.size() == 2);
        CHECK(r.log.dogs[0].status == GARMIN_USB_DOG_STATUS_POINTED && r.log.dogs[0].status_raw == 36);
        CHECK(r.log.dogs[1].status == GARMIN_USB_DOG_STATUS_UNKNOWN && r.log.dogs[1].status_raw == 2);
        CHECK(r.log.dogs[0].gps_bars_max == 3 && r.log.dogs[0].configured_id_known == 0);
    }
    return 0;
}

static int dog_before_product() {
    // Product_Data has not arrived: the dog is decoded for product 0, so the
    // status is Unknown with the raw byte kept and no model-only field.
    Rig r;
    CHECK(open_stream(r));
    r.fake.push(GARMIN_USB_ENDPOINT_INTERRUPT, dog_frame(2, "Early"));
    CHECK(r.poll() == GARMIN_USB_POLL_OK);
    CHECK(r.log.dogs.size() == 1);
    CHECK(r.log.dogs[0].status == GARMIN_USB_DOG_STATUS_UNKNOWN && r.log.dogs[0].status_raw == 2);
    CHECK(r.log.dogs[0].gps_lost_known == 0 && r.log.dogs[0].track_color_known == 0);
    CHECK(std::strcmp(r.log.dogs[0].name, "Early") == 0);
    // Once the product arrives the same dog decodes with it.
    r.fake.push(GARMIN_USB_ENDPOINT_INTERRUPT, product_data(1339, 460));
    r.fake.push(GARMIN_USB_ENDPOINT_INTERRUPT, dog_frame(2, "Early"));
    CHECK(r.poll() == GARMIN_USB_POLL_OK);
    CHECK(r.poll() == GARMIN_USB_POLL_OK);
    CHECK(r.log.dogs.size() == 2 && r.log.dogs[1].status == GARMIN_USB_DOG_STATUS_MOVING);
    return 0;
}

static int short_dog_is_ignored() {
    // The decoder refuses a dog packet under 40 bytes; the session drops it
    // and carries on.
    Rig r;
    CHECK(open_stream(r));
    r.fake.push(GARMIN_USB_ENDPOINT_INTERRUPT, frame(20, 3078, Bytes(39, 1)));
    r.fake.push(GARMIN_USB_ENDPOINT_INTERRUPT, dog_frame(1, "After"));
    CHECK(r.poll() == GARMIN_USB_POLL_OK);
    CHECK(r.log.dogs.empty());
    CHECK(r.poll() == GARMIN_USB_POLL_OK);
    CHECK(r.log.dogs.size() == 1 && std::strcmp(r.log.dogs[0].name, "After") == 0);
    return 0;
}

static int pvt_delivered() {
    Rig r;
    CHECK(open_stream(r));
    r.fake.push(GARMIN_USB_ENDPOINT_INTERRUPT, pvt_frame(59.9, 10.7));
    CHECK(r.poll() == GARMIN_USB_POLL_OK);
    CHECK(r.log.pvts.size() == 1);
    CHECK(near(r.log.pvts[0].latitude_deg, 59.9) && near(r.log.pvts[0].longitude_deg, 10.7));
    CHECK(near(r.log.pvts[0].altitude_m, 141.0, 1e-3) && r.log.pvts[0].fix == 3);
    return 0;
}

static int unknown_pids_ignored() {
    Rig r;
    CHECK(open_stream(r));
    const Bytes satellite = frame(20, 114, Bytes(84, 7));
    const Bytes ext_big = frame(20, 248, Bytes(GARMIN_USB_FRAMER_BUFFER + 144, 0x55));   // oversize: payload skipped
    const Bytes proto_array = frame(20, 253, Bytes(9, 3));
    r.fake.push(GARMIN_USB_ENDPOINT_INTERRUPT, satellite);
    r.fake.push_chunked(GARMIN_USB_ENDPOINT_INTERRUPT, ext_big, 64);
    r.fake.push(GARMIN_USB_ENDPOINT_INTERRUPT, cat(proto_array, dog_frame(1, "Still")));
    for (int i = 0; i < 1 + 7 + 1; ++i) CHECK(r.poll() == GARMIN_USB_POLL_OK);
    CHECK(r.log.dogs.size() == 1 && std::strcmp(r.log.dogs[0].name, "Still") == 0);
    CHECK(r.log.products.empty() && r.log.pvts.empty());
    CHECK(r.log.sessions.size() == 1);
    CHECK(r.fake.writes.size() == 3);         // nothing answered
    return 0;
}

static int null_events_are_fine() {
    Fake f;
    antz_garmin_usb_session_t s;
    antz_garmin_usb_transport_t t = {fake_write, fake_read, &f};
    antz_garmin_usb_events_t e = {nullptr, nullptr, nullptr, nullptr, nullptr};
    CHECK(garmin_usb_session_init(&s, &t, &e) == 0);
    CHECK(garmin_usb_session_start(&s) == 0);
    f.push(GARMIN_USB_ENDPOINT_INTERRUPT, session_started(5));
    f.push(GARMIN_USB_ENDPOINT_INTERRUPT, product_data(1339, 460));
    f.push(GARMIN_USB_ENDPOINT_INTERRUPT, pvt_frame(59.9, 10.7));
    f.push(GARMIN_USB_ENDPOINT_INTERRUPT, dog_frame(1, "X"));
    for (int i = 0; i < 4; ++i) CHECK(garmin_usb_session_poll(&s, 10) == GARMIN_USB_POLL_OK);
    CHECK(f.writes.size() == 3);              // the stream still started
    return 0;
}

// ─── Interrupt and bulk ─────────────────────────────────────────────────────
static int endpoints_switch() {
    Rig r;
    CHECK(open_stream(r));
    const int INT = GARMIN_USB_ENDPOINT_INTERRUPT, BULK = GARMIN_USB_ENDPOINT_BULK;
    r.fake.reads.clear();
    r.fake.push(INT, data_available());
    r.fake.push(BULK, dog_frame(1, "One"));
    r.fake.push(BULK, dog_frame(2, "Two"));
    r.fake.push_nothing(BULK);                            // nothing at a packet boundary: bulk is done
    r.fake.push(INT, dog_frame(3, "Three"));              // waits on the interrupt endpoint

    CHECK(r.poll() == GARMIN_USB_POLL_OK);                // Data_Available
    CHECK(r.fake.reads.size() == 1 && r.fake.reads[0].endpoint == INT);
    CHECK(r.poll() == GARMIN_USB_POLL_OK);                // bulk, One
    CHECK(r.poll() == GARMIN_USB_POLL_OK);                // bulk, Two
    CHECK(r.log.dogs.size() == 2);
    const auto end_of_bulk = r.poll();                    // bulk, nothing
    CHECK(end_of_bulk == GARMIN_USB_POLL_OK || end_of_bulk == GARMIN_USB_POLL_IDLE);   // not an error
    CHECK(r.poll() == GARMIN_USB_POLL_OK);                // back on the interrupt endpoint
    CHECK(r.fake.reads.size() == 5);
    const int want[] = {INT, BULK, BULK, BULK, INT};
    for (int i = 0; i < 5; ++i) CHECK(r.fake.reads[i].endpoint == want[i]);
    CHECK(r.log.dogs.size() == 3 && std::strcmp(r.log.dogs[2].name, "Three") == 0);
    CHECK(r.fake.script[INT].empty() && r.fake.script[BULK].empty());
    // The bulk reads were all asked with the caller's timeout and a real buffer.
    for (const auto& c : r.fake.reads) CHECK(c.cap > 0 && c.timeout_ms == 10);
    return 0;
}

static int bulk_does_not_return_early() {
    // Interrupt data that arrives while bulk is being read is left alone
    // until bulk gives nothing; several announcements in a row are fine.
    Rig r;
    CHECK(open_stream(r));
    const int INT = GARMIN_USB_ENDPOINT_INTERRUPT, BULK = GARMIN_USB_ENDPOINT_BULK;
    r.fake.reads.clear();
    r.fake.push(INT, data_available());
    for (int i = 0; i < 6; ++i) r.fake.push(BULK, dog_frame(1, "D"));
    r.fake.push_nothing(BULK);
    r.fake.push(INT, data_available());
    r.fake.push(BULK, dog_frame(2, "E"));
    r.fake.push_nothing(BULK);
    for (int i = 0; i < 1 + 6 + 1 + 1 + 1 + 1; ++i) {
        const auto rc = r.poll();
        CHECK(rc == GARMIN_USB_POLL_OK || rc == GARMIN_USB_POLL_IDLE);
    }
    CHECK(r.log.dogs.size() == 7);
    CHECK(r.fake.reads_on(INT) == 2 && r.fake.reads_on(BULK) == 9);
    return 0;
}

static int bulk_idle_on_interrupt_stays() {
    // A timeout on the interrupt endpoint (no announcement) is IDLE and
    // never moves to bulk.
    Rig r;
    CHECK(open_stream(r));
    r.fake.reads.clear();
    CHECK(r.poll() == GARMIN_USB_POLL_IDLE);
    CHECK(r.poll() == GARMIN_USB_POLL_IDLE);
    CHECK(r.fake.reads_on(GARMIN_USB_ENDPOINT_BULK) == 0 && r.fake.reads_on(GARMIN_USB_ENDPOINT_INTERRUPT) == 2);
    return 0;
}

static int bulk_data_available_is_not_a_switch() {
    // Data_Available is announced on the interrupt endpoint; the same
    // packet inside a bulk stream does not matter.
    Rig r;
    CHECK(open_stream(r));
    const int INT = GARMIN_USB_ENDPOINT_INTERRUPT, BULK = GARMIN_USB_ENDPOINT_BULK;
    r.fake.reads.clear();
    r.fake.push(INT, data_available());
    r.fake.push(BULK, cat(data_available(), dog_frame(1, "In bulk")));
    r.fake.push_nothing(BULK);
    for (int i = 0; i < 4; ++i) r.poll();
    CHECK(r.log.dogs.size() == 1);
    CHECK(r.fake.reads.size() == 4 && r.fake.reads[3].endpoint == INT);
    return 0;
}

// ─── Reads that cut packets ─────────────────────────────────────────────────
static int split_on_interrupt() {
    Rig r;
    CHECK(open_stream(r));
    const int INT = GARMIN_USB_ENDPOINT_INTERRUPT;
    const Bytes stream = cat(product_data(1339, 460), cat(dog_frame(2, "Ragnar"), pvt_frame(59.9, 10.7)));
    r.fake.push_chunked(INT, stream, 64);                 // 64 byte reads
    const size_t n64 = (stream.size() + 63) / 64;
    for (size_t i = 0; i < n64; ++i) {
        const auto rc = r.poll();
        CHECK(rc == GARMIN_USB_POLL_OK);
    }
    CHECK(r.log.products.size() == 1 && r.log.dogs.size() == 1 && r.log.pvts.size() == 1);
    CHECK(r.log.dogs[0].status == GARMIN_USB_DOG_STATUS_MOVING);
    // One byte at a time, through a header and a payload.
    Rig q;
    CHECK(open_stream(q));
    const Bytes dog = dog_frame(1, "Bytewise");
    q.fake.push_chunked(INT, dog, 1);
    for (size_t i = 0; i + 1 < dog.size(); ++i) {
        CHECK(q.poll() == GARMIN_USB_POLL_OK);
        CHECK(q.log.dogs.empty());
    }
    CHECK(q.poll() == GARMIN_USB_POLL_OK);
    CHECK(q.log.dogs.size() == 1 && std::strcmp(q.log.dogs[0].name, "Bytewise") == 0);
    return 0;
}

static int split_on_bulk() {
    Rig r;
    CHECK(open_stream(r));
    const int INT = GARMIN_USB_ENDPOINT_INTERRUPT, BULK = GARMIN_USB_ENDPOINT_BULK;
    const Bytes stream = cat(product_data(1562, 860),
                         cat(dog_frame(33, "Hund 1"), cat(dog_frame(35, "Hund 6"), dog_frame(36, "Hund 3"))));
    r.fake.push(INT, data_available());
    r.fake.push_chunked(BULK, stream, 64);
    r.fake.push_nothing(BULK);
    const size_t polls = 1 + (stream.size() + 63) / 64 + 1;
    for (size_t i = 0; i < polls; ++i) {
        const auto rc = r.poll();
        CHECK(rc == GARMIN_USB_POLL_OK || rc == GARMIN_USB_POLL_IDLE);
    }
    CHECK(r.log.products.size() == 1 && r.log.dogs.size() == 3);
    CHECK(r.log.dogs[0].status == GARMIN_USB_DOG_STATUS_SITTING);
    CHECK(r.log.dogs[1].status == GARMIN_USB_DOG_STATUS_TREED);
    CHECK(r.log.dogs[2].status == GARMIN_USB_DOG_STATUS_POINTED);
    CHECK(std::strcmp(r.log.dogs[2].name, "Hund 3") == 0);
    // And a zero length read between the pieces of a packet on the
    // interrupt endpoint is only a timeout there.
    Rig q;
    CHECK(open_stream(q));
    const Bytes dog = dog_frame(1, "Half");
    q.fake.push(INT, Bytes(dog.begin(), dog.begin() + 5));
    q.fake.push_nothing(INT);
    q.fake.push(INT, Bytes(dog.begin() + 5, dog.end()));
    CHECK(q.poll() == GARMIN_USB_POLL_OK);
    CHECK(q.poll() == GARMIN_USB_POLL_IDLE);
    CHECK(q.poll() == GARMIN_USB_POLL_OK);
    CHECK(q.log.dogs.size() == 1);
    return 0;
}

static int independent_frame_readers() {
    // The interrupt endpoint is left in the middle of a Session_Started
    // packet when the handheld announces bulk data; bulk carries a dog cut
    // in two. Neither reader may see the other's bytes. The rest of the
    // Session_Started arrives on interrupt after bulk is done.
    Rig r;
    CHECK(garmin_usb_session_start(&r.s) == 0);
    const int INT = GARMIN_USB_ENDPOINT_INTERRUPT, BULK = GARMIN_USB_ENDPOINT_BULK;
    const Bytes ss = session_started(4242);
    const Bytes dog = dog_frame(2, "Mixed");
    r.fake.push(INT, cat(data_available(), Bytes(ss.begin(), ss.begin() + 5)));
    r.fake.push(BULK, Bytes(dog.begin(), dog.begin() + 50));
    r.fake.push(BULK, Bytes(dog.begin() + 50, dog.end()));
    r.fake.push_nothing(BULK);
    r.fake.push(INT, Bytes(ss.begin() + 5, ss.end()));

    CHECK(r.poll() == GARMIN_USB_POLL_OK);
    CHECK(r.poll() == GARMIN_USB_POLL_OK);
    CHECK(r.log.dogs.empty());
    CHECK(r.poll() == GARMIN_USB_POLL_OK);
    CHECK(r.log.dogs.size() == 1 && std::strcmp(r.log.dogs[0].name, "Mixed") == 0);
    r.poll();                                             // bulk ends
    CHECK(r.log.sessions.empty());
    CHECK(r.poll() == GARMIN_USB_POLL_OK);                // the rest of Session_Started
    CHECK(r.log.sessions.size() == 1 && r.log.sessions[0] == 4242);
    CHECK(r.fake.writes.size() == 3);
    return 0;
}

// ─── Errors ─────────────────────────────────────────────────────────────────
static int partial_packet_on_bulk() {
    const int INT = GARMIN_USB_ENDPOINT_INTERRUPT, BULK = GARMIN_USB_ENDPOINT_BULK;
    const Bytes dog = dog_frame(1, "Cut");
    // Cut inside the payload, and cut inside the header.
    for (size_t cut : {size_t(50), size_t(5)}) {
        Rig r;
        CHECK(open_stream(r));
        r.fake.push(INT, data_available());
        r.fake.push(BULK, Bytes(dog.begin(), dog.begin() + cut));
        r.fake.push_nothing(BULK);
        r.fake.push(BULK, Bytes(dog.begin() + cut, dog.end()));   // would have completed it
        CHECK(r.poll() == GARMIN_USB_POLL_OK);
        CHECK(r.poll() == GARMIN_USB_POLL_OK);
        CHECK(r.poll() == GARMIN_USB_POLL_PARTIAL_PACKET);
        CHECK(r.log.dogs.empty());
        const size_t reads = r.fake.reads.size();
        r.poll(); r.poll();
        CHECK(r.fake.reads.size() == reads);                      // over: no more reads
    }
    return 0;
}

static int transport_error() {
    const int INT = GARMIN_USB_ENDPOINT_INTERRUPT, BULK = GARMIN_USB_ENDPOINT_BULK;
    {   // on interrupt
        Rig r;
        CHECK(open_stream(r));
        r.fake.push_error(INT);
        r.fake.push(INT, dog_frame(1, "Never"));
        CHECK(r.poll() == GARMIN_USB_POLL_TRANSPORT_ERROR);
        const size_t reads = r.fake.reads.size();
        r.poll(); r.poll();
        CHECK(r.fake.reads.size() == reads);
        CHECK(r.log.dogs.empty());
    }
    {   // on bulk, at a boundary and inside a packet
        for (int inside = 0; inside < 2; ++inside) {
            Rig r;
            CHECK(open_stream(r));
            r.fake.push(INT, data_available());
            if (inside) r.fake.push(BULK, Bytes(30, 0));          // not a whole header; the error wins
            r.fake.push_error(BULK);
            CHECK(r.poll() == GARMIN_USB_POLL_OK);
            if (inside) CHECK(r.poll() == GARMIN_USB_POLL_OK);
            CHECK(r.poll() == GARMIN_USB_POLL_TRANSPORT_ERROR);
            const size_t reads = r.fake.reads.size();
            r.poll();
            CHECK(r.fake.reads.size() == reads);
        }
    }
    return 0;
}

static int frame_error() {
    const int INT = GARMIN_USB_ENDPOINT_INTERRUPT, BULK = GARMIN_USB_ENDPOINT_BULK;
    {   // an invalid layer on interrupt
        Rig r;
        CHECK(open_stream(r));
        r.fake.push(INT, frame(7, 3078, Bytes(10, 0)));
        r.fake.push(INT, dog_frame(1, "Never"));
        CHECK(r.poll() == GARMIN_USB_POLL_FRAME_ERROR);
        const size_t reads = r.fake.reads.size();
        r.poll(); r.poll();
        CHECK(r.fake.reads.size() == reads);
        CHECK(r.log.dogs.empty());
    }
    {   // the same on bulk
        Rig r;
        CHECK(open_stream(r));
        r.fake.push(INT, data_available());
        r.fake.push(BULK, frame(7, 3078, Bytes(10, 0)));
        r.fake.push(BULK, dog_frame(1, "Never"));
        CHECK(r.poll() == GARMIN_USB_POLL_OK);
        CHECK(r.poll() == GARMIN_USB_POLL_FRAME_ERROR);
        const size_t reads = r.fake.reads.size();
        r.poll(); r.poll();
        CHECK(r.fake.reads.size() == reads);
        CHECK(r.log.dogs.empty());
    }
    {   // a size above the limit
        Rig r;
        CHECK(open_stream(r));
        Bytes huge = frame(20, 3078, Bytes(4, 0));
        put_u32(huge, 8, GARMIN_USB_MAX_PACKET_SIZE + 1);
        r.fake.push(INT, huge);
        CHECK(r.poll() == GARMIN_USB_POLL_FRAME_ERROR);
    }
    return 0;
}

// ─── Stop ───────────────────────────────────────────────────────────────────
static int stop() {
    {   // a running stream: Command_Data 50
        Rig r;
        CHECK(open_stream(r));
        CHECK(garmin_usb_session_stop(&r.s) == 0);
        CHECK(r.fake.writes.size() == 4);
        CHECK(is_packet(r.fake.writes[3], 20, 10, u16le(50)));
        CHECK(r.fake.writes[3].size() == 14);
    }
    {   // never started: nothing is written
        Rig r;
        CHECK(garmin_usb_session_stop(&r.s) == 0);
        CHECK(r.fake.writes.empty());
    }
    {   // Start_Session sent, Session_Started not yet: no stream to stop
        Rig r;
        CHECK(garmin_usb_session_start(&r.s) == 0);
        CHECK(garmin_usb_session_stop(&r.s) == 0);
        CHECK(r.fake.writes.size() == 1);
    }
    {   // the write fails: nonzero
        Rig r;
        CHECK(open_stream(r));
        r.fake.fail_write_from = 3;
        CHECK(garmin_usb_session_stop(&r.s) != 0);
    }
    return 0;
}

int main(int argc, char** argv) {
    struct Group { const char* name; int (*fn)(); };
    const Group groups[] = {
        {"start", start}, {"start_write_fails", start_write_fails}, {"init_refuses", init_refuses},
        {"handshake", handshake}, {"handshake_once", handshake_once},
        {"handshake_short", handshake_short_session_started},
        {"poll_timeout", poll_passes_timeout},
        {"product_and_dogs", product_and_dogs}, {"dog_before_product", dog_before_product},
        {"short_dog_ignored", short_dog_is_ignored}, {"pvt", pvt_delivered},
        {"unknown_pids", unknown_pids_ignored}, {"null_events", null_events_are_fine},
        {"endpoints_switch", endpoints_switch}, {"bulk_continues", bulk_does_not_return_early},
        {"interrupt_idle", bulk_idle_on_interrupt_stays},
        {"bulk_data_available", bulk_data_available_is_not_a_switch},
        {"split.interrupt", split_on_interrupt}, {"split.bulk", split_on_bulk},
        {"independent_readers", independent_frame_readers},
        {"error.partial_packet", partial_packet_on_bulk}, {"error.transport", transport_error},
        {"error.frame", frame_error}, {"stop", stop},
    };
    for (const Group& g : groups) {
        if (argc > 1 && std::strcmp(argv[1], g.name) != 0) continue;
        if (g.fn() != 0) return 1;
    }
    std::printf("ok (%d checks)\n", checks);
    return 0;
}
