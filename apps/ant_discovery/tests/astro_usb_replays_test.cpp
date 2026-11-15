//
// Replays of real Astro 320 and Alpha 100 USB sessions (and one paired
// USB + ANT capture), from Arnfinn Grambo's astro-usb replays/ directory.
// The fixtures are in this repository, in tests/data/astro-usb (a copy of
// astro-usb's replays/ directory, unchanged). The tests read that folder
// through a compile definition; ANTZ_ASTRO_USB_REPLAYS may name another
// directory instead. A missing or empty data directory is a failure, not a skip.
//
// Expectations come from replays/README.md and the documented protocol
// (docs/protocol.md), and where the README is silent from the facts the
// README states about each file (names, which collar stops, which flag is
// set). The positions in the files are synthetic (South Pacific); a test
// below makes sure no real position can slip into a fixture.
//
// What the paired group does and does not show (ADR 0162's cross-transport
// evidence): the ANT location pages 1 and 2 of asset index 0 and 1 decode to
// exactly the USB Dog_Data positions of Thora and Ragnar, and the ANT device
// number equals the low 16 bits of the USB unit id. BUT the builder replaced
// both: it mapped the original ANT coordinates onto the USB ones by matching
// the original values (and stops when a match is ambiguous), and it WROTE the
// device number as the low 16 bits of the fictional USB unit id. So the
// position equality shows the original matching held (each ANT asset index
// follows one USB collar name), while the device number equality is by
// construction of the fixture and is NOT independent evidence that a real
// Astro's USB unit id and ANT device number agree. The capture has no ANT
// Common product or serial pages (no pages 80/81), so it cannot show that the
// USB unit id equals an ANT serial number or that the number is a stable
// identity. The situation byte of the ANT pages does agree with the USB
// status of the same collar (Thora treed, Ragnar sitting).
//
#include "profiles/garmin_usb/garmin_usb_session.h"
#include "profiles/tracker/tracker_decoder.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <map>
#include <string>
#include <vector>

using namespace antz;

static int checks = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); return 1; } } while (0)

typedef std::vector<uint8_t> Bytes;

// ─── Loading ────────────────────────────────────────────────────────────────
static int nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}
static bool jfield(const std::string& line, const char* key, std::string* out) {
    const std::string k = std::string("\"") + key + "\"";
    size_t at = line.find(k);
    if (at == std::string::npos) return false;
    at = line.find(':', at);
    if (at == std::string::npos) return false;
    ++at;
    while (at < line.size() && line[at] == ' ') ++at;
    size_t end;
    if (line[at] == '"') { ++at; end = line.find('"', at); } else end = line.find_first_of(",}", at);
    if (end == std::string::npos) return false;
    *out = line.substr(at, end - at);
    return true;
}

struct Rec { double t; int layer; int pid; Bytes payload; };       // USB: layer = "type"
struct AntRec { double t; int mid; Bytes payload; };

#ifndef ANTZ_ASTRO_USB_REPLAYS_DEFAULT_DIR
#error "ANTZ_ASTRO_USB_REPLAYS_DEFAULT_DIR must name tests/data/astro-usb"
#endif
static const char* dir() {
    const char* e = std::getenv("ANTZ_ASTRO_USB_REPLAYS");
    return (e && *e) ? e : ANTZ_ASTRO_USB_REPLAYS_DEFAULT_DIR;
}

static bool load_usb(const char* name, std::vector<Rec>* out) {
    std::ifstream in(std::string(dir()) + "/" + name + ".jsonl");
    if (!in.good()) return false;
    std::string line;
    while (std::getline(in, line)) {
        std::string t, ty, pid, hex;
        if (!jfield(line, "t", &t) || !jfield(line, "type", &ty) || !jfield(line, "pid", &pid) ||
            !jfield(line, "hex", &hex)) return false;
        Rec r; r.t = std::atof(t.c_str()); r.layer = std::atoi(ty.c_str()); r.pid = std::atoi(pid.c_str());
        for (size_t i = 0; i + 1 < hex.size(); i += 2)
            r.payload.push_back(static_cast<uint8_t>(nibble(hex[i]) * 16 + nibble(hex[i + 1])));
        out->push_back(r);
    }
    return !out->empty();
}
static bool load_ant(const char* name, std::vector<AntRec>* out) {
    std::ifstream in(std::string(dir()) + "/" + name + ".jsonl");
    if (!in.good()) return false;
    std::string line;
    while (std::getline(in, line)) {
        std::string t, mid, hex;
        if (!jfield(line, "t", &t) || !jfield(line, "mid", &mid) || !jfield(line, "hex", &hex)) return false;
        AntRec r; r.t = std::atof(t.c_str()); r.mid = std::atoi(mid.c_str());
        for (size_t i = 0; i + 1 < hex.size(); i += 2)
            r.payload.push_back(static_cast<uint8_t>(nibble(hex[i]) * 16 + nibble(hex[i + 1])));
        out->push_back(r);
    }
    return !out->empty();
}

// A file decoded record by record, directly with the decoder.
struct Dog { double wall; antz_garmin_usb_dog_t d; };
struct Pvt { double wall; antz_garmin_usb_pvt_t p; };
struct Parsed {
    std::vector<Rec> recs;
    std::vector<uint32_t> units;
    std::vector<antz_garmin_usb_product_t> products;
    std::vector<Pvt> pvts;
    std::vector<Dog> dogs;
    bool ok = false;
};
static bool parse(const char* name, Parsed* p) {
    if (!load_usb(name, &p->recs)) return false;
    uint16_t product = 0;
    for (const Rec& r : p->recs) {
        if (r.layer == 0 && r.pid == GARMIN_USB_PID_SESSION_STARTED) {
            uint32_t u; if (garmin_usb_decode_session_started(r.payload.data(), r.payload.size(), &u)) return false;
            p->units.push_back(u);
        } else if (r.layer == 20 && r.pid == GARMIN_USB_PID_PRODUCT_DATA) {
            antz_garmin_usb_product_t pr; if (garmin_usb_decode_product(r.payload.data(), r.payload.size(), &pr)) return false;
            product = pr.product_id; p->products.push_back(pr);
        } else if (r.layer == 20 && r.pid == GARMIN_USB_PID_PVT_DATA) {
            Pvt v; v.wall = r.t;
            if (garmin_usb_decode_pvt(r.payload.data(), r.payload.size(), &v.p)) return false;
            p->pvts.push_back(v);
        } else if (r.layer == 20 && r.pid == GARMIN_USB_PID_DOG_DATA) {
            Dog d; d.wall = r.t;
            if (garmin_usb_decode_dog(r.payload.data(), r.payload.size(), product, &d.d)) return false;
            p->dogs.push_back(d);
        } else {
            return false;                          // the README says only these four packets are included
        }
    }
    p->ok = true;
    return true;
}
static std::vector<Dog> of(const Parsed& p, const char* name) {
    std::vector<Dog> v;
    for (const Dog& d : p.dogs) if (std::strcmp(d.d.name, name) == 0) v.push_back(d);
    return v;
}

static const char* ASTRO[] = {"astro-normal", "astro-rename-before", "astro-rename-after", "astro-id-before",
                              "astro-id-after", "astro-collar-off", "astro-gps-loss", "astro-gps-recovery",
                              "paired-astro-usb"};
static const char* ALPHA[] = {"alpha-normal", "alpha-after-reboot", "alpha-after-fix", "alpha-rename",
                              "alpha-collar-off", "alpha-gps-loss", "alpha-gps-recovery"};
static std::vector<std::string> all_usb() {
    std::vector<std::string> v;
    for (const char* n : ASTRO) v.push_back(n);
    for (const char* n : ALPHA) v.push_back(n);
    return v;
}
static bool is_alpha(const std::string& n) { return n.compare(0, 6, "alpha-") == 0; }

// ─── Session over a fake transport (as garmin_usb_replay_test.cpp) ─────────
static void put_u16(Bytes& v, size_t at, uint16_t x) { v[at] = x & 0xFF; v[at + 1] = x >> 8; }
static void put_u32(Bytes& v, size_t at, uint32_t x) { for (int i = 0; i < 4; ++i) v[at + i] = (x >> (8 * i)) & 0xFF; }
static Bytes frame(int layer, int pid, const Bytes& payload) {
    Bytes f(12 + payload.size(), 0);
    f[0] = static_cast<uint8_t>(layer); put_u16(f, 4, static_cast<uint16_t>(pid)); put_u32(f, 8, static_cast<uint32_t>(payload.size()));
    if (!payload.empty()) std::memcpy(&f[12], payload.data(), payload.size());
    return f;
}
struct Fake {
    std::deque<Bytes> script[2];
    Bytes pending[2];
    std::vector<Bytes> writes;
    void push(int ep, const Bytes& b) { script[ep].push_back(b); }
    void push_chunked(int ep, const Bytes& b, size_t n) {
        for (size_t at = 0; at < b.size(); at += n) push(ep, Bytes(b.begin() + at, b.begin() + std::min(b.size(), at + n)));
    }
    bool drained() const { return script[0].empty() && script[1].empty() && pending[0].empty() && pending[1].empty(); }
};
static int fake_write(void* c, const uint8_t* d, size_t n) { static_cast<Fake*>(c)->writes.emplace_back(d, d + n); return 0; }
static int fake_read(void* c, antz_garmin_usb_endpoint_t ep, uint8_t* buf, size_t cap, uint32_t) {
    Fake* f = static_cast<Fake*>(c);
    Bytes& pend = f->pending[ep];
    if (pend.empty()) {
        if (f->script[ep].empty()) return 0;
        pend = f->script[ep].front(); f->script[ep].pop_front();
        if (pend.empty()) return 0;
    }
    const size_t n = std::min(cap, pend.size());
    std::memcpy(buf, pend.data(), n);
    pend.erase(pend.begin(), pend.begin() + n);
    return static_cast<int>(n);
}
struct Events {
    std::vector<uint32_t> units;
    std::vector<antz_garmin_usb_product_t> products;
    std::vector<antz_garmin_usb_pvt_t> pvts;
    std::vector<antz_garmin_usb_dog_t> dogs;
};
static void ev_s(void* c, uint32_t u) { static_cast<Events*>(c)->units.push_back(u); }
static void ev_p(void* c, const antz_garmin_usb_product_t* p) { static_cast<Events*>(c)->products.push_back(*p); }
static void ev_v(void* c, const antz_garmin_usb_pvt_t* p) { static_cast<Events*>(c)->pvts.push_back(*p); }
static void ev_d(void* c, const antz_garmin_usb_dog_t* d) { static_cast<Events*>(c)->dogs.push_back(*d); }

static bool through_session(const std::vector<Rec>& recs, Events* ev, int* errors) {
    Fake fake;
    antz_garmin_usb_session_t s;
    antz_garmin_usb_transport_t t = {fake_write, fake_read, &fake};
    antz_garmin_usb_events_t e = {ev_s, ev_p, ev_v, ev_d, ev};
    if (garmin_usb_session_init(&s, &t, &e) != 0 || garmin_usb_session_start(&s) != 0) return false;
    size_t n = 0;
    for (const Rec& r : recs) {
        const Bytes f = frame(r.layer, r.pid, r.payload);
        const size_t piece = (n++ % 2 == 0) ? 64 : f.size();
        if (r.layer == 20 && (r.pid == GARMIN_USB_PID_DOG_DATA || r.pid == GARMIN_USB_PID_PVT_DATA)) {
            fake.push(0, frame(0, 2, Bytes()));                  // Data_Available
            fake.push_chunked(1, f, piece);
            fake.push(1, Bytes());
        } else {
            fake.push_chunked(0, f, piece);                      // Session_Started, Product_Data
        }
    }
    *errors = 0;
    for (int guard = 0; guard < 1000000 && !fake.drained(); ++guard) {
        const auto rc = garmin_usb_session_poll(&s, 10);
        if (rc != GARMIN_USB_POLL_OK && rc != GARMIN_USB_POLL_IDLE) { ++*errors; break; }
    }
    return fake.drained();
}
static bool same_dog(const antz_garmin_usb_dog_t& a, const antz_garmin_usb_dog_t& b) {
    return a.latitude_deg == b.latitude_deg && a.longitude_deg == b.longitude_deg && a.altitude_m == b.altitude_m &&
           a.garmin_time_s == b.garmin_time_s && a.unix_time_s == b.unix_time_s && a.status_raw == b.status_raw &&
           a.status == b.status && a.flags == b.flags && a.gps_lost_known == b.gps_lost_known && a.gps_lost == b.gps_lost &&
           a.gps_bars == b.gps_bars && a.gps_bars_max == b.gps_bars_max && a.comm_bars == b.comm_bars &&
           a.battery_level_raw == b.battery_level_raw && a.track_color_code == b.track_color_code &&
           a.track_color_known == b.track_color_known && a.id_bytes[0] == b.id_bytes[0] && a.id_bytes[1] == b.id_bytes[1] &&
           a.configured_id_known == b.configured_id_known && a.configured_id == b.configured_id &&
           std::strcmp(a.name, b.name) == 0;
}

// ─── (1) every record decodes; the session gives the same events ───────────
static int decodes() {
    for (const auto& n : all_usb()) {
        Parsed p;
        CHECK(parse(n.c_str(), &p));                                // every record decodes
        CHECK(p.units.size() == 1 && p.products.size() == 1);
        CHECK(!p.pvts.empty());
        // Record sizes as the builder documents them.
        for (const Rec& r : p.recs) {
            if (r.pid == 6) CHECK(r.payload.size() == 4);
            if (r.pid == 51) CHECK(r.payload.size() == 64);
            if (r.pid == 3078) CHECK(r.payload.size() == 100);
        }
        // Time never runs backwards.
        for (size_t i = 1; i < p.recs.size(); ++i) CHECK(p.recs[i].t >= p.recs[i - 1].t);
    }
    return 0;
}

static int session_matches_decoding() {
    for (const auto& n : all_usb()) {
        Parsed p;
        CHECK(parse(n.c_str(), &p));
        Events ev; int errors = 0;
        CHECK(through_session(p.recs, &ev, &errors));
        CHECK(errors == 0);
        CHECK(ev.units == p.units);
        CHECK(ev.products.size() == p.products.size());
        for (size_t i = 0; i < p.products.size(); ++i) {
            CHECK(ev.products[i].product_id == p.products[i].product_id);
            CHECK(ev.products[i].software_version_x100 == p.products[i].software_version_x100);
        }
        CHECK(ev.pvts.size() == p.pvts.size());
        for (size_t i = 0; i < p.pvts.size(); ++i) {
            CHECK(ev.pvts[i].latitude_deg == p.pvts[i].p.latitude_deg && ev.pvts[i].longitude_deg == p.pvts[i].p.longitude_deg);
            CHECK(ev.pvts[i].altitude_m == p.pvts[i].p.altitude_m && ev.pvts[i].fix == p.pvts[i].p.fix);
        }
        CHECK(ev.dogs.size() == p.dogs.size());                      // in order, record for record
        for (size_t i = 0; i < p.dogs.size(); ++i) CHECK(same_dog(ev.dogs[i], p.dogs[i].d));
    }
    return 0;
}

// ─── (2) product and unit id ───────────────────────────────────────────────
static int products_and_units() {
    for (const auto& n : all_usb()) {
        Parsed p; CHECK(parse(n.c_str(), &p));
        CHECK(p.products.size() == 1 && p.units.size() == 1);
        if (is_alpha(n)) {
            CHECK(p.products[0].product_id == GARMIN_USB_PRODUCT_ALPHA_100 && p.products[0].product_id == 1562);
            CHECK(p.products[0].software_version_x100 == 860);      // Alpha 100 EMEA/APAC SW 8.60 (protocol.md)
            CHECK(p.units[0] == 0x10020001u);
        } else {
            CHECK(p.products[0].product_id == GARMIN_USB_PRODUCT_ASTRO_320 && p.products[0].product_id == 1339);
            CHECK(p.products[0].software_version_x100 == 460);      // Astro 320 SW 4.60 (protocol.md)
            CHECK(p.units[0] == 0x100117F0u);
        }
        // Session_Started comes first, then Product_Data, before any dog.
        CHECK(p.recs[0].pid == 6 && p.recs[1].pid == 255);
    }
    // One consistent unit id per handheld across all of its files.
    for (const char* n : ASTRO) { Parsed p; CHECK(parse(n, &p)); CHECK(p.units[0] == 0x100117F0u); }
    for (const char* n : ALPHA) { Parsed p; CHECK(parse(n, &p)); CHECK(p.units[0] == 0x10020001u); }
    return 0;
}

// ─── (3) privacy: nothing outside the South Pacific box ────────────────────
// The README says every known coordinate was replaced by a synthetic point in
// the South Pacific. In the data they all lie within latitude -35.000 to
// -34.989 and longitude -130.000 to -129.987. The bounds here are that area
// with 0.1 degree of margin: latitude -35.1..-34.9, longitude -130.1..-129.9.
static bool in_box(double lat, double lon) {
    return lat >= -35.1 && lat <= -34.9 && lon >= -130.1 && lon <= -129.9;
}
static int privacy_box() {
    int points = 0;
    for (const auto& n : all_usb()) {
        Parsed p; CHECK(parse(n.c_str(), &p));
        for (const Dog& d : p.dogs) { CHECK(in_box(d.d.latitude_deg, d.d.longitude_deg)); ++points; }
        for (const Pvt& v : p.pvts) { CHECK(in_box(v.p.latitude_deg, v.p.longitude_deg)); ++points; }
    }
    CHECK(points > 1000);
    // The names the data holds are exactly this set, and every dog packet's name field is one of them
    // (the made-up names of the README: Ragnar, RagnarXX and Thora on the Astro, Hund 1 and Hund 6 on the
    // Alpha, and Ragnar again where an Alpha collar was renamed).
    const char* allowed[] = {"Ragnar", "RagnarXX", "Thora", "Hund 1", "Hund 6"};
    std::map<std::string, int> seen;
    for (const auto& n : all_usb()) {
        Parsed p; CHECK(parse(n.c_str(), &p));
        for (const Dog& d : p.dogs) {
            bool known = false;
            for (const char* a : allowed) if (std::strcmp(d.d.name, a) == 0) known = true;
            CHECK(known);
            ++seen[d.d.name];
        }
    }
    CHECK(seen.size() == 5);
    for (const char* a : allowed) CHECK(seen[a] > 0);
    // The ANT side too.
    std::vector<AntRec> ant; CHECK(load_ant("paired-astro-ant", &ant));
    for (const AntRec& r : ant) {
        if (r.mid != 78) continue;
        CHECK(r.payload.size() == 14);
        antz_tracker_location2_t l2;
        if (r.payload[1] == 2) {
            CHECK(tracker_decode_location2(&r.payload[1], 8, &l2) == 0);
            const double lon = tracker_semicircles_to_degrees(l2.longitude_semicircles);
            CHECK(lon >= -130.1 && lon <= -129.9);
        }
    }
    return 0;
}

// ─── (4) statuses ───────────────────────────────────────────────────────────
// The data holds astro codes 1 (sitting), 2 (moving), 3 (treed) and alpha 33,
// 34, 35; code 4 / 36 (pointed) never occurs in these files, and no other code
// occurs, so Unknown never appears.
static int statuses() {
    for (const auto& n : all_usb()) {
        Parsed p; CHECK(parse(n.c_str(), &p));
        for (const Dog& d : p.dogs) {
            const int base = is_alpha(n) ? 32 : 0;
            CHECK(d.d.status_raw >= base + 1 && d.d.status_raw <= base + 4);
            CHECK(d.d.status != GARMIN_USB_DOG_STATUS_UNKNOWN);
            const antz_garmin_usb_dog_status_t want[] = {GARMIN_USB_DOG_STATUS_SITTING, GARMIN_USB_DOG_STATUS_MOVING,
                                                         GARMIN_USB_DOG_STATUS_TREED, GARMIN_USB_DOG_STATUS_POINTED};
            CHECK(d.d.status == want[d.d.status_raw - base - 1]);
        }
    }
    // Named cases from the files: sitting Ragnar and treed Thora in the normal
    // Astro session; Hund 1 sitting, Hund 6 treed on the Alpha.
    Parsed a; CHECK(parse("astro-normal", &a));
    for (const Dog& d : of(a, "Ragnar")) CHECK(d.d.status == GARMIN_USB_DOG_STATUS_SITTING && d.d.status_raw == 1);
    for (const Dog& d : of(a, "Thora")) CHECK(d.d.status == GARMIN_USB_DOG_STATUS_TREED && d.d.status_raw == 3);
    Parsed b; CHECK(parse("alpha-normal", &b));
    for (const Dog& d : of(b, "Hund 1")) CHECK(d.d.status == GARMIN_USB_DOG_STATUS_SITTING && d.d.status_raw == 33);
    for (const Dog& d : of(b, "Hund 6")) CHECK(d.d.status == GARMIN_USB_DOG_STATUS_TREED && d.d.status_raw == 35);
    // The collar with GPS lost is "moving" (README: status stays).
    Parsed g; CHECK(parse("astro-gps-loss", &g));
    for (const Dog& d : of(g, "Ragnar")) CHECK(d.d.status == GARMIN_USB_DOG_STATUS_MOVING && d.d.status_raw == 2);
    Parsed h; CHECK(parse("alpha-gps-loss", &h));
    for (const Dog& d : of(h, "Ragnar")) CHECK(d.d.status == GARMIN_USB_DOG_STATUS_MOVING && d.d.status_raw == 34);
    return 0;
}

// ─── (5) names and evidence ────────────────────────────────────────────────
static size_t count_name(const Parsed& p, const char* n) { return of(p, n).size(); }
static int names_astro() {
    Parsed n; CHECK(parse("astro-normal", &n));
    CHECK(n.dogs.size() == 36 && count_name(n, "Ragnar") == 18 && count_name(n, "Thora") == 18);   // two collars
    for (const Dog& d : n.dogs) {
        CHECK(d.d.configured_id_known == 1 && d.d.gps_bars_max == 5 && d.d.track_color_known == 1);
        CHECK(d.d.comm_bars <= 5 && d.d.gps_bars <= 5);
    }
    for (const Dog& d : of(n, "Ragnar")) CHECK(d.d.configured_id == 11 && d.d.track_color_code == 12);
    for (const Dog& d : of(n, "Thora")) CHECK(d.d.configured_id == 6 && d.d.track_color_code == 9);

    // Rename: Ragnar, then RagnarXX in the later session.
    Parsed rb; CHECK(parse("astro-rename-before", &rb));
    CHECK(count_name(rb, "Ragnar") == 24 && count_name(rb, "RagnarXX") == 0);
    Parsed ra; CHECK(parse("astro-rename-after", &ra));
    CHECK(count_name(ra, "RagnarXX") == 12 && count_name(ra, "Ragnar") == 0);
    // Only the name changed: same radio id, colour and status class for that collar.
    const Dog a = of(rb, "Ragnar")[0], b = of(ra, "RagnarXX")[0];
    CHECK(a.d.configured_id == 11 && b.d.configured_id == 11);
    CHECK(a.d.track_color_code == b.d.track_color_code && a.d.id_bytes[0] == b.d.id_bytes[0] && a.d.id_bytes[1] == b.d.id_bytes[1]);

    // Radio id: 11 before, 12 after; name and colour unchanged.
    Parsed ib; CHECK(parse("astro-id-before", &ib));
    Parsed ia; CHECK(parse("astro-id-after", &ia));
    CHECK(count_name(ib, "Ragnar") == 6 && count_name(ia, "Ragnar") == 6);
    for (const Dog& d : of(ib, "Ragnar")) CHECK(d.d.configured_id == 11 && d.d.id_bytes[0] == 11 && d.d.id_bytes[1] == 0);
    for (const Dog& d : of(ia, "Ragnar")) CHECK(d.d.configured_id == 12 && d.d.id_bytes[0] == 12 && d.d.id_bytes[1] == 0);
    CHECK(of(ib, "Ragnar")[0].d.track_color_code == of(ia, "Ragnar")[0].d.track_color_code);
    CHECK(of(ib, "Ragnar")[0].d.track_color_code == 12);
    // The other collar is untouched by the change.
    for (const Dog& d : of(ib, "Thora")) CHECK(d.d.configured_id == 6);
    for (const Dog& d : of(ia, "Thora")) CHECK(d.d.configured_id == 6);
    return 0;
}

static int names_alpha() {
    Parsed n; CHECK(parse("alpha-normal", &n));
    CHECK(n.dogs.size() == 36 && count_name(n, "Hund 1") == 18 && count_name(n, "Hund 6") == 18);
    Parsed r; CHECK(parse("alpha-rename", &r));
    CHECK(count_name(r, "Ragnar") == 12 && count_name(r, "Hund 6") == 12 && count_name(r, "Hund 1") == 0);   // Hund 1 became Ragnar
    for (const auto& f : {std::string("alpha-normal"), std::string("alpha-rename"), std::string("alpha-after-fix"),
                          std::string("alpha-collar-off"), std::string("alpha-gps-loss"), std::string("alpha-gps-recovery")}) {
        Parsed p; CHECK(parse(f.c_str(), &p));
        CHECK(!p.dogs.empty());
        for (const Dog& d : p.dogs) {
            CHECK(d.d.configured_id_known == 0 && d.d.configured_id == 0);   // no configured id on an Alpha
            CHECK(d.d.gps_bars_max == 3 && d.d.track_color_known == 1);
            // The raw id bytes are kept as evidence: (1,2) for the first collar, (6,1) for the second.
            const bool first = std::strcmp(d.d.name, "Hund 6") != 0;
            CHECK(d.d.id_bytes[0] == (first ? 1 : 6) && d.d.id_bytes[1] == (first ? 2 : 1));
        }
    }
    return 0;
}

// ─── (6) collar off: one collar stops, the other goes on ──────────────────
static int collar_off() {
    struct Case { const char* file; const char* gone; size_t gone_n; const char* stays; size_t stays_n; };
    const Case cases[] = {{"astro-collar-off", "Ragnar", 3, "Thora", 24}, {"alpha-collar-off", "Ragnar", 4, "Hund 6", 24}};
    for (const Case& c : cases) {
        Parsed p; CHECK(parse(c.file, &p));
        const auto gone = of(p, c.gone), stays = of(p, c.stays);
        CHECK(gone.size() == c.gone_n && stays.size() == c.stays_n);
        CHECK(p.dogs.size() == c.gone_n + c.stays_n);
        // The gap is in the data: the other collar's last packet is far later than the silent one's last.
        CHECK(stays.back().wall - gone.back().wall > 90.0);
        CHECK(gone.back().wall < 20.0);
        // The silent collar's source times were regular up to its last packet, and the other
        // collar's continue without a break: every step 5 s.
        for (size_t i = 1; i < gone.size(); ++i) CHECK(gone[i].d.garmin_time_s - gone[i - 1].d.garmin_time_s == 5);
        for (size_t i = 1; i < stays.size(); ++i) CHECK(stays[i].d.garmin_time_s - stays[i - 1].d.garmin_time_s == 5);
        // The handheld kept sending its own position throughout.
        CHECK(p.pvts.size() >= 119 && p.pvts.back().wall > stays.back().wall - 6.0);
        // Through the session: no error, the absent collar produces nothing more and nothing is reported removed
        // (the session has no such event; every packet of both collars arrives exactly once).
        Events ev; int errors = 0;
        CHECK(through_session(p.recs, &ev, &errors) && errors == 0);
        size_t g = 0, s = 0;
        for (const auto& d : ev.dogs) { if (!std::strcmp(d.name, c.gone)) ++g; else if (!std::strcmp(d.name, c.stays)) ++s; }
        CHECK(g == c.gone_n && s == c.stays_n);
    }
    return 0;
}

// ─── (7) GPS loss and recovery ──────────────────────────────────────────────
static int gps_loss_and_recovery() {
    for (const char* f : {"astro-gps-loss", "alpha-gps-loss"}) {
        Parsed p; CHECK(parse(f, &p));
        const auto k = of(p, "Ragnar");
        CHECK(k.size() == 12);
        for (size_t i = 0; i < k.size(); ++i) {
            CHECK((k[i].d.flags & 0x20) == 0x20);                    // byte 17 bit 0x20
            CHECK(k[i].d.gps_lost_known == 1 && k[i].d.gps_lost == 1);
            CHECK(k[i].d.gps_bars == 0);
            CHECK(k[i].d.latitude_deg == k[0].d.latitude_deg && k[i].d.longitude_deg == k[0].d.longitude_deg);   // stale point
            if (i) CHECK(k[i].d.garmin_time_s - k[i - 1].d.garmin_time_s == 5);                                  // time advances
            CHECK(k[i].d.comm_bars == 5);                                                                       // the radio is fine
        }
        // The other collar is not affected.
        const char* other = std::strcmp(f, "astro-gps-loss") == 0 ? "Thora" : "Hund 6";
        for (const Dog& d : of(p, other)) CHECK((d.d.flags & 0x20) == 0 && d.d.gps_lost == 0);
    }
    for (const char* f : {"astro-gps-recovery", "alpha-gps-recovery"}) {
        Parsed p; CHECK(parse(f, &p));
        const auto k = of(p, "Ragnar");
        CHECK(k.size() == 6);
        for (size_t i = 0; i < k.size(); ++i) {
            CHECK((k[i].d.flags & 0x20) == 0);                       // flag clear
            CHECK(k[i].d.gps_lost_known == 1 && k[i].d.gps_lost == 0);
            CHECK(k[i].d.gps_bars > 0);
            if (i) CHECK(k[i].d.garmin_time_s - k[i - 1].d.garmin_time_s == 5);
        }
        // The handheld's own position moves in these sessions (the Pvt track steps along).
        CHECK(p.pvts.front().p.latitude_deg != p.pvts.back().p.latitude_deg);
        // NOT tested: that Ragnar's position moves again after recovery. In both recovery files the
        // collar's coordinates are one repeated point (README: repeated original points remain
        // repeated), so the data cannot show movement.
    }
    return 0;
}

// ─── (8) alpha after reboot: Pvt, no dogs ──────────────────────────────────
static int alpha_after_reboot() {
    Parsed r; CHECK(parse("alpha-after-reboot", &r));
    CHECK(r.dogs.empty() && r.pvts.size() == 60);
    CHECK(r.products[0].product_id == 1562);
    for (const Pvt& v : r.pvts) CHECK(v.p.fix != 3);                  // no 3D fix yet (README: first session after reboot)
    // The later session with a 3D fix has both collars.
    Parsed f; CHECK(parse("alpha-after-fix", &f));
    CHECK(f.pvts.size() == 120 && f.dogs.size() == 48);
    for (const Pvt& v : f.pvts) CHECK(v.p.fix == 3);
    CHECK(count_name(f, "Ragnar") == 24 && count_name(f, "Hund 6") == 24);
    // Through the session, too.
    Events ev; int errors = 0;
    CHECK(through_session(r.recs, &ev, &errors) && errors == 0 && ev.dogs.empty() && ev.pvts.size() == 60);
    return 0;
}

// ─── (9) cadence ────────────────────────────────────────────────────────────
static int cadence() {
    for (const auto& n : all_usb()) {
        Parsed p; CHECK(parse(n.c_str(), &p));
        std::map<std::string, std::vector<Dog>> by;
        for (const Dog& d : p.dogs) by[d.d.name].push_back(d);
        for (const auto& kv : by) {
            const auto& v = kv.second;
            for (size_t i = 1; i < v.size(); ++i) {
                CHECK(v[i].d.garmin_time_s - v[i - 1].d.garmin_time_s == 5);   // every five seconds per collar
                CHECK(v[i].d.unix_time_s == GARMIN_USB_EPOCH_UNIX_S + v[i].d.garmin_time_s);
            }
        }
    }
    // The normal files: the two collars are offset, each arriving about every five seconds on the wire.
    for (const char* f : {"astro-normal", "alpha-normal"}) {
        Parsed p; CHECK(parse(f, &p));
        std::map<std::string, std::vector<Dog>> by;
        for (const Dog& d : p.dogs) by[d.d.name].push_back(d);
        CHECK(by.size() == 2);
        for (const auto& kv : by) {
            CHECK(kv.second.size() == 18);
            for (size_t i = 1; i < kv.second.size(); ++i) {
                const double dt = kv.second[i].wall - kv.second[i - 1].wall;
                CHECK(dt > 3.5 && dt < 6.5);
            }
        }
        // 90 seconds of 1 Hz Pvt.
        CHECK(p.pvts.size() == 90);
    }
    return 0;
}

// ─── (10) paired USB + ANT ──────────────────────────────────────────────────
static int paired() {
    Parsed u; CHECK(parse("paired-astro-usb", &u));
    std::vector<AntRec> ant; CHECK(load_ant("paired-astro-ant", &ant));
    // USB: Ragnar and Thora, each at one fixed fictional point.
    const auto ragnar = of(u, "Ragnar"), thora = of(u, "Thora");
    CHECK(ragnar.size() == 12 && thora.size() == 12);
    for (const Dog& d : ragnar) CHECK(d.d.latitude_deg == ragnar[0].d.latitude_deg && d.d.longitude_deg == ragnar[0].d.longitude_deg);
    for (const Dog& d : thora) CHECK(d.d.latitude_deg == thora[0].d.latitude_deg && d.d.longitude_deg == thora[0].d.longitude_deg);
    CHECK(ragnar[0].d.latitude_deg != thora[0].d.latitude_deg);

    const uint16_t unit_low = static_cast<uint16_t>(u.units[0] & 0xFFFF);
    CHECK(unit_low == 0x17F0);

    antz_tracker_location1_t l1[32]; antz_tracker_location2_t l2[32];
    bool have1[32] = {false}, have2[32] = {false};
    int assembled[2] = {0, 0}, broadcasts = 0, pages1 = 0, pages2 = 0;
    double first_bcast = 1e9, last_bcast = 0;
    for (const AntRec& r : ant) {
        if (r.mid != 78) continue;                                  // control messages: 64 and 111
        CHECK(r.payload.size() == 14);
        ++broadcasts;
        // Extended data: flag byte 0x80 (device number follows), device number, device type, transmission type.
        CHECK(r.payload[9] == 0x80);
        const uint16_t devnum = static_cast<uint16_t>(r.payload[10] | (r.payload[11] << 8));
        CHECK(devnum == unit_low);                                  // by construction of the fixture, see the header comment
        CHECK(r.payload[0] == 0);                                   // channel 0
        first_bcast = std::min(first_bcast, r.t); last_bcast = std::max(last_bcast, r.t);

        const uint8_t* page = &r.payload[1];                         // the 8 byte page, page number first
        if (page[0] == 1) {
            antz_tracker_location1_t t;
            CHECK(tracker_decode_location1(page, 8, &t) == 0);
            CHECK(t.asset_index < 2);
            l1[t.asset_index] = t; have1[t.asset_index] = true; ++pages1;
            // The situation agrees with the USB status of the same collar: index 0 Thora treed, index 1 Ragnar sitting.
            CHECK(t.status.situation_valid);
            CHECK(t.status.situation == (t.asset_index == 0 ? ANTZ_TRACKER_SITUATION_TREED : ANTZ_TRACKER_SITUATION_SITTING));
            CHECK(!t.status.gps_lost && !t.status.remove);          // nothing is reported removed
        } else if (page[0] == 2) {
            antz_tracker_location2_t t;
            CHECK(tracker_decode_location2(page, 8, &t) == 0);
            CHECK(t.asset_index < 2);
            l2[t.asset_index] = t; have2[t.asset_index] = true; ++pages2;
        } else {
            CHECK(false);                                           // only pages 1 and 2 are in the capture
        }
        // Whenever both halves of an index are known, the position equals the USB dog position.
        for (int i = 0; i < 2; ++i) {
            if (!have1[i] || !have2[i]) continue;
            const int32_t lat = tracker_latitude(l1[i].latitude_bits_0_15, l2[i].latitude_bits_16_31);
            const double dlat = tracker_semicircles_to_degrees(lat);
            const double dlon = tracker_semicircles_to_degrees(l2[i].longitude_semicircles);
            const Dog& want = i == 0 ? thora[0] : ragnar[0];         // ANT index 0 = Thora, 1 = Ragnar (builder)
            CHECK(std::fabs(dlat - want.d.latitude_deg) < 1e-9 && std::fabs(dlon - want.d.longitude_deg) < 1e-9);
            CHECK(in_box(dlat, dlon));
            ++assembled[i];
        }
    }
    CHECK(broadcasts == 655 && pages1 > 100 && pages2 > 100);
    CHECK(assembled[0] > 100 && assembled[1] > 100);
    // The two captures share a time origin and overlap: ANT broadcasts span about a minute inside the USB session.
    CHECK(first_bcast >= u.recs.front().t - 2.0 && last_bcast <= u.recs.back().t + 2.0);
    return 0;
}

// ─── The data is all there ─────────────────────────────────────────────────
// The 17 files the README table names (a name in backticks at the start of a
// row) exist and are not empty. A missing directory fails here.
static int data_complete() {
    std::ifstream readme(std::string(dir()) + "/README.md");
    CHECK(readme.good());
    std::vector<std::string> names;
    std::string line;
    while (std::getline(readme, line)) {
        if (line.compare(0, 3, "| `") != 0) continue;
        const size_t end = line.find('`', 3);
        if (end == std::string::npos) continue;
        names.push_back(line.substr(3, end - 3));
    }
    CHECK(names.size() == 17);
    for (const auto& n : names) {
        std::ifstream f(std::string(dir()) + "/" + n + ".jsonl", std::ios::binary | std::ios::ate);
        CHECK(f.good());
        CHECK(f.tellg() > 0);
    }
    return 0;
}

int main(int argc, char** argv) {
    struct Group { const char* name; int (*fn)(); };
    const Group groups[] = {
        {"data_complete", data_complete}, {"decodes", decodes}, {"session_matches_decoding", session_matches_decoding},
        {"products_and_units", products_and_units}, {"privacy_box", privacy_box}, {"statuses", statuses},
        {"names_astro", names_astro}, {"names_alpha", names_alpha}, {"collar_off", collar_off},
        {"gps_loss_and_recovery", gps_loss_and_recovery}, {"alpha_after_reboot", alpha_after_reboot},
        {"cadence", cadence}, {"paired", paired},
    };
    for (const Group& g : groups) {
        if (argc > 1 && std::strcmp(argv[1], g.name) != 0) continue;
        if (g.fn() != 0) return 1;
    }
    std::printf("ok (%d checks)\n", checks);
    return 0;
}
