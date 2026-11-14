//
// Replays the generated sequences in tests/data through the session, over a
// fake transport. Dog packets go over bulk (announced on interrupt), the
// handshake and the rest over interrupt, some reads cut into 64 byte pieces.
// Expectations are the generator's documented intent (data/README.md), not
// what the decoder printed. A Session_Started record starts a new session.
//
#include "profiles/garmin_usb/garmin_usb_session.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <string>
#include <vector>

#ifndef ANTZ_REPLAY_DATA_DIR
#error "ANTZ_REPLAY_DATA_DIR must name tests/data"
#endif

using namespace antz;

static int checks = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); return 1; } } while (0)

typedef std::vector<uint8_t> Bytes;

// ─── Reading a replay file ─────────────────────────────────────────────────
struct Record { int layer; int pid; Bytes payload; };

static int nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}
static bool field(const std::string& line, const char* key, std::string* out) {
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
static bool load(const char* name, std::vector<Record>* out) {
    std::ifstream in(std::string(ANTZ_REPLAY_DATA_DIR) + "/" + name);
    if (!in.good()) return false;
    std::string line;
    while (std::getline(in, line)) {
        std::string ty, pid, hex;
        if (!field(line, "type", &ty) || !field(line, "pid", &pid) || !field(line, "hex", &hex)) continue;
        Record r;
        r.layer = std::atoi(ty.c_str());
        r.pid = std::atoi(pid.c_str());
        for (size_t i = 0; i + 1 < hex.size(); i += 2)
            r.payload.push_back(static_cast<uint8_t>(nibble(hex[i]) * 16 + nibble(hex[i + 1])));
        out->push_back(r);
    }
    return !out->empty();
}

static void put_u16(Bytes& v, size_t at, uint16_t x) { v[at] = x & 0xFF; v[at + 1] = x >> 8; }
static void put_u32(Bytes& v, size_t at, uint32_t x) { for (int i = 0; i < 4; ++i) v[at + i] = (x >> (8 * i)) & 0xFF; }
static Bytes frame(const Record& r) {
    Bytes f(12 + r.payload.size(), 0);
    f[0] = static_cast<uint8_t>(r.layer);
    put_u16(f, 4, static_cast<uint16_t>(r.pid));
    put_u32(f, 8, static_cast<uint32_t>(r.payload.size()));
    if (!r.payload.empty()) std::memcpy(&f[12], r.payload.data(), r.payload.size());
    return f;
}

// ─── Fake transport ─────────────────────────────────────────────────────────
struct Fake {
    std::deque<Bytes> script[2];
    Bytes pending[2];
    std::vector<Bytes> writes;
    void push(int ep, const Bytes& b) { script[ep].push_back(b); }
    void push_chunked(int ep, const Bytes& b, size_t n) {
        for (size_t at = 0; at < b.size(); at += n)
            push(ep, Bytes(b.begin() + at, b.begin() + std::min(b.size(), at + n)));
    }
    bool drained() const {
        return script[0].empty() && script[1].empty() && pending[0].empty() && pending[1].empty();
    }
};
static int fake_write(void* ctx, const uint8_t* d, size_t n) {
    static_cast<Fake*>(ctx)->writes.emplace_back(d, d + n);
    return 0;
}
static int fake_read(void* ctx, antz_garmin_usb_endpoint_t ep, uint8_t* buf, size_t cap, uint32_t) {
    Fake* f = static_cast<Fake*>(ctx);
    Bytes& pend = f->pending[ep];
    if (pend.empty()) {
        if (f->script[ep].empty()) return 0;
        pend = f->script[ep].front();
        f->script[ep].pop_front();
        if (pend.empty()) return 0;
    }
    const size_t n = std::min(cap, pend.size());
    std::memcpy(buf, pend.data(), n);
    pend.erase(pend.begin(), pend.begin() + n);
    return static_cast<int>(n);
}

struct Result {
    std::vector<uint32_t> units;
    std::vector<antz_garmin_usb_product_t> products;
    std::vector<antz_garmin_usb_pvt_t> pvts;
    std::vector<antz_garmin_usb_dog_t> dogs;
    std::vector<int> dog_session;         // which session each dog came in
    int sessions = 0;
    int errors = 0;                        // polls that were not OK or IDLE
};
struct Sink { Result* r; };
static void ev_session(void* c, uint32_t u) { static_cast<Sink*>(c)->r->units.push_back(u); }
static void ev_product(void* c, const antz_garmin_usb_product_t* p) { static_cast<Sink*>(c)->r->products.push_back(*p); }
static void ev_pvt(void* c, const antz_garmin_usb_pvt_t* p) { static_cast<Sink*>(c)->r->pvts.push_back(*p); }
static void ev_dog(void* c, const antz_garmin_usb_dog_t* d) {
    Result* r = static_cast<Sink*>(c)->r;
    r->dogs.push_back(*d);
    r->dog_session.push_back(r->sessions);
}

// Play one session's records.
static bool play_segment(const std::vector<Record>& recs, Result* res, bool chunk_all) {
    Fake fake;
    Sink sink = {res};
    antz_garmin_usb_session_t s;
    antz_garmin_usb_transport_t t = {fake_write, fake_read, &fake};
    antz_garmin_usb_events_t e = {ev_session, ev_product, ev_pvt, ev_dog, &sink};
    if (garmin_usb_session_init(&s, &t, &e) != 0 || garmin_usb_session_start(&s) != 0) return false;
    ++res->sessions;
    size_t n = 0;
    for (const Record& r : recs) {
        const Bytes f = frame(r);
        const size_t piece = (chunk_all || (n++ % 2 == 0)) ? 64 : f.size();
        if (r.layer == 20 && r.pid == GARMIN_USB_PID_DOG_DATA) {
            fake.push(GARMIN_USB_ENDPOINT_INTERRUPT, frame({0, 2, Bytes()}));   // Data_Available
            fake.push_chunked(GARMIN_USB_ENDPOINT_BULK, f, piece);
            fake.push(GARMIN_USB_ENDPOINT_BULK, Bytes());                       // nothing at a boundary
        } else {
            fake.push_chunked(GARMIN_USB_ENDPOINT_INTERRUPT, f, piece);
        }
    }
    for (int guard = 0; guard < 100000 && !fake.drained(); ++guard) {
        const auto rc = garmin_usb_session_poll(&s, 10);
        if (rc != GARMIN_USB_POLL_OK && rc != GARMIN_USB_POLL_IDLE) { ++res->errors; break; }
    }
    // The handshake was answered once per session: Start, Product_Rqst, Start_Pvt.
    if (fake.writes.size() != 3) ++res->errors;
    return fake.drained();
}

static bool play(const char* file, Result* res, bool chunk_all = false) {
    std::vector<Record> recs;
    if (!load(file, &recs)) return false;
    std::vector<Record> seg;
    for (const Record& r : recs) {
        if (r.layer == 0 && r.pid == GARMIN_USB_PID_SESSION_STARTED && !seg.empty()) {
            if (!play_segment(seg, res, chunk_all)) return false;
            seg.clear();
        }
        seg.push_back(r);
    }
    return play_segment(seg, res, chunk_all);
}

// ─── Helpers ────────────────────────────────────────────────────────────────
static const uint32_t T0 = 1163246400u;
static std::vector<antz_garmin_usb_dog_t> by_name(const Result& r, const char* name) {
    std::vector<antz_garmin_usb_dog_t> v;
    for (const auto& d : r.dogs) if (std::strcmp(d.name, name) == 0) v.push_back(d);
    return v;
}
static bool in_area(const antz_garmin_usb_dog_t& d) {
    return std::fabs(d.latitude_deg - 60.0) < 0.01 && std::fabs(d.longitude_deg - 10.0) < 0.01;
}
static bool same_but_position_and_name(const antz_garmin_usb_dog_t& a, const antz_garmin_usb_dog_t& b) {
    return a.status == b.status && a.status_raw == b.status_raw && a.flags == b.flags &&
           a.gps_lost == b.gps_lost && a.gps_bars == b.gps_bars && a.comm_bars == b.comm_bars &&
           a.battery_level_raw == b.battery_level_raw && a.track_color_code == b.track_color_code &&
           a.id_bytes[0] == b.id_bytes[0] && a.id_bytes[1] == b.id_bytes[1] &&
           a.configured_id == b.configured_id;
}
// Source times of one collar must be Garmin T0 + 5 r (+2 for Luna) at the listed rounds.
static bool times_are(const std::vector<antz_garmin_usb_dog_t>& v, int offset, std::vector<int> rounds) {
    if (v.size() != rounds.size()) return false;
    for (size_t i = 0; i < v.size(); ++i) {
        if (v[i].garmin_time_s != T0 + 5u * rounds[i] + offset) return false;
        if (v[i].unix_time_s != GARMIN_USB_EPOCH_UNIX_S + v[i].garmin_time_s) return false;
    }
    return true;
}
static bool clean_astro_start(const Result& r, size_t sessions) {
    return r.errors == 0 && r.sessions == static_cast<int>(sessions) && r.units.size() == sessions &&
           r.products.size() == sessions;
}

// ─── Groups ─────────────────────────────────────────────────────────────────
static int normal() {
    for (int chunk_all = 0; chunk_all < 2; ++chunk_all) {
        Result r;
        CHECK(play("replay_normal_astro.jsonl", &r, chunk_all != 0));
        CHECK(clean_astro_start(r, 1));
        CHECK(r.units[0] == 1234567890u);
        CHECK(r.products[0].product_id == 1339 && r.products[0].software_version_x100 == 460);
        CHECK(r.pvts.size() == 1);
        CHECK(std::fabs(r.pvts[0].latitude_deg - 60.0) < 1e-6 && std::fabs(r.pvts[0].longitude_deg - 10.0) < 1e-6);
        CHECK(r.dogs.size() == 12);
        auto rex = by_name(r, "Rex"), luna = by_name(r, "Luna");
        CHECK(times_are(rex, 0, {0, 1, 2, 3, 4, 5}));
        CHECK(times_are(luna, 2, {0, 1, 2, 3, 4, 5}));
        for (size_t i = 0; i < r.dogs.size(); ++i) {                // arrival order: Rex, Luna, Rex, ...
            CHECK(std::strcmp(r.dogs[i].name, i % 2 == 0 ? "Rex" : "Luna") == 0);
            CHECK(in_area(r.dogs[i]));
            CHECK(r.dogs[i].status == GARMIN_USB_DOG_STATUS_MOVING && r.dogs[i].status_raw == 2);
            CHECK(r.dogs[i].gps_lost_known == 1 && r.dogs[i].gps_lost == 0);
            CHECK(r.dogs[i].comm_bars == 5 && r.dogs[i].gps_bars == 5 && r.dogs[i].gps_bars_max == 5);
            CHECK(r.dogs[i].configured_id_known == 1);
        }
        CHECK(rex[0].configured_id == 11 && luna[0].configured_id == 6);
        CHECK(rex[0].track_color_code == 12 && luna[0].track_color_code == 9);
        // The collars move: consecutive fixes of one collar differ.
        CHECK(rex[0].latitude_deg != rex[1].latitude_deg || rex[0].longitude_deg != rex[1].longitude_deg);
    }
    return 0;
}

static int reconnect() {
    Result r;
    CHECK(play("replay_reconnect.jsonl", &r));
    CHECK(clean_astro_start(r, 2));
    CHECK(r.units[0] == 1234567890u && r.units[1] == 1234567890u);     // the same handheld
    CHECK(r.products[0].product_id == 1339 && r.products[1].product_id == 1339);
    CHECK(r.dogs.size() == 12);
    CHECK(times_are(by_name(r, "Rex"), 0, {0, 1, 2, 3, 4, 5}));        // no gap, no repeat
    CHECK(times_are(by_name(r, "Luna"), 2, {0, 1, 2, 3, 4, 5}));
    int in_second = 0;
    for (size_t i = 0; i < r.dogs.size(); ++i) {
        if (r.dog_session[i] == 2) ++in_second;
        CHECK(r.dogs[i].status == GARMIN_USB_DOG_STATUS_MOVING);       // decoded with the product in both
    }
    CHECK(in_second == 6);
    // The collars are the same ones: same ids and colours before and after.
    auto rex = by_name(r, "Rex");
    CHECK(same_but_position_and_name(rex[0], rex[5]));
    return 0;
}

static int reboot() {
    Result r;
    CHECK(play("replay_reboot.jsonl", &r));
    CHECK(clean_astro_start(r, 2));
    CHECK(r.dogs.size() == 12);
    CHECK(times_are(by_name(r, "Rex"), 0, {0, 1, 2, 8, 9, 10}));
    CHECK(times_are(by_name(r, "Luna"), 2, {0, 1, 2, 8, 9, 10}));
    for (const auto& d : r.dogs) CHECK(d.status == GARMIN_USB_DOG_STATUS_MOVING && d.gps_lost_known == 1);
    // Dogs resume in the second session.
    int second = 0; for (int s : r.dog_session) if (s == 2) ++second;
    CHECK(second == 6);
    return 0;
}

static int rename() {
    Result r;
    CHECK(play("replay_rename.jsonl", &r));
    CHECK(clean_astro_start(r, 1));
    CHECK(r.dogs.size() == 12);
    auto rex = by_name(r, "Rex"), rexi = by_name(r, "Rexi"), luna = by_name(r, "Luna");
    CHECK(rex.size() == 3 && rexi.size() == 3 && luna.size() == 6);
    // The rename is the only change: the same collar, times continuous.
    std::vector<antz_garmin_usb_dog_t> one = rex;
    one.insert(one.end(), rexi.begin(), rexi.end());
    CHECK(times_are(one, 0, {0, 1, 2, 3, 4, 5}));
    for (const auto& d : one) CHECK(same_but_position_and_name(one[0], d));
    CHECK(one[0].configured_id == 11 && one[5].configured_id == 11);
    CHECK(times_are(luna, 2, {0, 1, 2, 3, 4, 5}));
    for (const auto& d : luna) CHECK(same_but_position_and_name(luna[0], d));
    return 0;
}

static int radio_id() {
    Result r;
    CHECK(play("replay_radio_id.jsonl", &r));
    CHECK(clean_astro_start(r, 1));
    auto rex = by_name(r, "Rex"), luna = by_name(r, "Luna");
    CHECK(rex.size() == 6 && luna.size() == 6 && r.dogs.size() == 12);
    const uint8_t want[6] = {11, 11, 12, 12, 11, 11};
    for (int i = 0; i < 6; ++i) {
        CHECK(rex[i].id_bytes[0] == want[i] && rex[i].id_bytes[1] == 0);
        CHECK(rex[i].configured_id == want[i]);
        // Nothing else follows the id: name, colour, status, flags, bars, battery.
        CHECK(std::strcmp(rex[i].name, "Rex") == 0);
        CHECK(rex[i].track_color_code == 12 && rex[i].status == GARMIN_USB_DOG_STATUS_MOVING);
        CHECK(rex[i].flags == 0 && rex[i].gps_lost == 0 && rex[i].battery_level_raw == rex[0].battery_level_raw);
        CHECK(rex[i].gps_bars == 5 && rex[i].comm_bars == 5);
        CHECK(luna[i].id_bytes[0] == 6 && luna[i].configured_id == 6);
    }
    CHECK(times_are(rex, 0, {0, 1, 2, 3, 4, 5}));
    return 0;
}

static int power_cycle() {
    Result r;
    CHECK(play("replay_power_cycle.jsonl", &r));
    CHECK(clean_astro_start(r, 1));
    auto rex = by_name(r, "Rex"), luna = by_name(r, "Luna");
    CHECK(rex.size() == 7 && luna.size() == 4 && r.dogs.size() == 11);
    CHECK(times_are(rex, 0, {0, 1, 2, 3, 4, 5, 6}));                    // untouched by the other's silence
    CHECK(times_are(luna, 2, {0, 1, 5, 6}));                            // a 20 s gap, then back
    for (const auto& d : luna) CHECK(same_but_position_and_name(luna[0], d) && in_area(d));
    // Silence is not a removal: the session saw no error and nothing else changed.
    CHECK(luna[2].garmin_time_s - luna[1].garmin_time_s == 20);
    return 0;
}

static int gps_loss() {
    Result r;
    CHECK(play("replay_gps_loss.jsonl", &r));
    CHECK(clean_astro_start(r, 1));
    auto rex = by_name(r, "Rex"), luna = by_name(r, "Luna");
    CHECK(rex.size() == 6 && luna.size() == 6);
    CHECK(times_are(rex, 0, {0, 1, 2, 3, 4, 5}));                       // time keeps advancing
    for (int i = 0; i < 6; ++i) {
        const bool lost = (i == 2 || i == 3);
        CHECK(rex[i].gps_lost_known == 1 && rex[i].gps_lost == (lost ? 1 : 0));
        CHECK(rex[i].flags == (lost ? 0x20 : 0));
        CHECK(rex[i].gps_bars == (lost ? 0 : 5));
        CHECK(rex[i].comm_bars == 5);                                    // the radio is fine
        CHECK(rex[i].status == GARMIN_USB_DOG_STATUS_MOVING);            // still "moving"
        CHECK(luna[i].gps_lost == 0 && luna[i].flags == 0);              // the other collar is fine
    }
    // The coordinates stood still from the last good fix through the loss.
    for (int i : {2, 3}) {
        CHECK(rex[i].latitude_deg == rex[1].latitude_deg);
        CHECK(rex[i].longitude_deg == rex[1].longitude_deg);
    }
    // After recovery the position moves again.
    CHECK(rex[4].latitude_deg != rex[3].latitude_deg || rex[4].longitude_deg != rex[3].longitude_deg);
    return 0;
}

static int alpha() {
    Result r;
    CHECK(play("replay_alpha.jsonl", &r));
    CHECK(clean_astro_start(r, 1));
    CHECK(r.products[0].product_id == 1562 && r.products[0].software_version_x100 == 860);
    auto rex = by_name(r, "Rex"), luna = by_name(r, "Luna");
    CHECK(rex.size() == 6 && luna.size() == 6);
    const antz_garmin_usb_dog_status_t cycle[4] = {GARMIN_USB_DOG_STATUS_SITTING, GARMIN_USB_DOG_STATUS_MOVING,
                                                   GARMIN_USB_DOG_STATUS_TREED, GARMIN_USB_DOG_STATUS_POINTED};
    for (int i = 0; i < 6; ++i) {
        CHECK(rex[i].status_raw == 33 + (i % 4) && rex[i].status == cycle[i % 4]);
        CHECK(luna[i].status_raw == 33 + ((i + 1) % 4) && luna[i].status == cycle[(i + 1) % 4]);
        CHECK(rex[i].gps_bars_max == 3 && rex[i].gps_bars == 3);
        CHECK(rex[i].configured_id_known == 0);                          // Alpha bytes 22..23 are raw evidence
        CHECK(rex[i].id_bytes[0] == 1 && rex[i].id_bytes[1] == 2);
        CHECK(luna[i].id_bytes[0] == 6 && luna[i].id_bytes[1] == 1);
        CHECK(rex[i].track_color_known == 1 && rex[i].track_color_code == 12);
    }
    CHECK(times_are(rex, 0, {0, 1, 2, 3, 4, 5}));
    return 0;
}

int main(int argc, char** argv) {
    struct Group { const char* name; int (*fn)(); };
    const Group groups[] = {
        {"normal", normal}, {"reconnect", reconnect}, {"reboot", reboot}, {"rename", rename},
        {"radio_id", radio_id}, {"power_cycle", power_cycle}, {"gps_loss", gps_loss}, {"alpha", alpha},
    };
    for (const Group& g : groups) {
        if (argc > 1 && std::strcmp(argv[1], g.name) != 0) continue;
        if (g.fn() != 0) return 1;
    }
    std::printf("ok (%d checks)\n", checks);
    return 0;
}
