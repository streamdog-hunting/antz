//
// Tests for ant_discovery's Garmin USB mode: the format functions
// (garmin_usb_format.h) and the run loop (garmin_usb_mode.h) over a fake
// session transport, replaying the generated sequences in tests/data.
//
#include "garmin_usb_mode.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

#ifndef ANTZ_REPLAY_DATA_DIR
#error "ANTZ_REPLAY_DATA_DIR must name tests/data"
#endif

using namespace antz;
using namespace ant::garmin_usb;

static int checks = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); return 1; } } while (0)

typedef std::vector<uint8_t> Bytes;

// ─── Small string helpers ───────────────────────────────────────────────────
static std::string chomp(std::string s) {
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
    return s;
}
static std::vector<std::string> lines_of(const std::string& s) {
    std::vector<std::string> v; std::string cur;
    for (char c : s) { if (c == '\n') { v.push_back(cur); cur.clear(); } else cur += c; }
    if (!cur.empty()) v.push_back(cur);
    return v;
}
static bool has(const std::string& s, const std::string& part) { return s.find(part) != std::string::npos; }
static std::string num_after(const std::string& s, const char* key) {   // the token after `"key":`
    const std::string k = std::string("\"") + key + "\":";
    size_t at = s.find(k);
    if (at == std::string::npos) return "";
    at += k.size();
    size_t end = s.find_first_of(",}", at);
    return s.substr(at, end - at);
}
static std::vector<std::string> csv_cells(const std::string& line) {      // no quoted commas in these rows
    std::vector<std::string> v; std::string cur;
    for (char c : line) { if (c == ',') { v.push_back(cur); cur.clear(); } else cur += c; }
    v.push_back(cur);
    return v;
}

// ─── Structs for the format tests ──────────────────────────────────────────
static antz_garmin_usb_dog_t astro_dog() {
    antz_garmin_usb_dog_t d;
    std::memset(&d, 0, sizeof d);
    d.latitude_deg = 59.9; d.longitude_deg = 10.7; d.altitude_m = 300.0f;
    d.garmin_time_s = 1794312000u - 631065600u;
    d.unix_time_s = 1794312000;
    d.status_raw = 2; d.status = GARMIN_USB_DOG_STATUS_MOVING; d.flags = 0;
    d.gps_lost_known = 1; d.gps_lost = 0;
    d.gps_bars = 5; d.gps_bars_max = 5; d.comm_bars = 5; d.battery_level_raw = 40;
    d.track_color_known = 1; d.track_color_code = 12;
    d.configured_id_known = 1; d.configured_id = 11;
    d.id_bytes[0] = 11;
    std::strcpy(d.name, "Khione");
    return d;
}

// ─── Format ─────────────────────────────────────────────────────────────────
static int status_names() {
    CHECK(statusName(GARMIN_USB_DOG_STATUS_SITTING) == "sitting");
    CHECK(statusName(GARMIN_USB_DOG_STATUS_MOVING) == "moving");
    CHECK(statusName(GARMIN_USB_DOG_STATUS_TREED) == "treed");
    CHECK(statusName(GARMIN_USB_DOG_STATUS_POINTED) == "pointed");
    CHECK(statusName(GARMIN_USB_DOG_STATUS_UNKNOWN) == "unknown");
    CHECK(statusName(static_cast<antz_garmin_usb_dog_status_t>(99)) == "unknown");
    return 0;
}

static int json_handheld() {
    antz_garmin_usb_product_t p = {1339, 460};
    CHECK(chomp(formatHandheld(Format::Json, 3886356464u, p)) ==
          R"({"page":"handheld","unit_id":3886356464,"product_id":1339,"software":"4.60"})");
    antz_garmin_usb_product_t a = {1562, 860};
    CHECK(chomp(formatHandheld(Format::Json, 7, a)) ==
          R"({"page":"handheld","unit_id":7,"product_id":1562,"software":"8.60"})");
    return 0;
}

static int json_position() {
    antz_garmin_usb_pvt_t v;
    std::memset(&v, 0, sizeof v);
    v.latitude_deg = 59.9; v.longitude_deg = 10.7; v.altitude_m = 141.0; v.fix = 3;
    const std::string s = chomp(formatHandheldPosition(Format::Json, 3886356464u, v));
    const std::string head = R"({"page":"handheld_position","unit_id":3886356464,"latitude":59.900000,"longitude":10.700000,"altitude":)";
    CHECK(s.compare(0, head.size(), head) == 0);
    CHECK(std::fabs(std::atof(num_after(s, "altitude").c_str()) - 141.0) < 1e-3);
    const std::string tail = R"(,"fix":3})";
    CHECK(s.size() > tail.size() && s.compare(s.size() - tail.size(), tail.size(), tail) == 0);
    // Southern and western hemispheres.
    v.latitude_deg = -33.8688; v.longitude_deg = -70.6693;
    const std::string w = formatHandheldPosition(Format::Json, 1, v);
    CHECK(has(w, R"("latitude":-33.868800,"longitude":-70.669300)"));
    return 0;
}

static int json_dog() {
    const antz_garmin_usb_dog_t d = astro_dog();
    CHECK(chomp(formatDog(Format::Json, 3886356464u, d)) ==
          R"({"page":"dog","unit_id":3886356464,"name":"Khione","latitude":59.900000,"longitude":10.700000,)"
          R"("time":"2026-11-10T12:00:00Z","status":"moving","status_raw":2,"gps_lost":false,"gps_bars":5,)"
          R"("comm_bars":5,"battery_raw":40,"flags":0,"track_color":12,"configured_id":11})");

    // Lost GPS is true, the position is still printed as sent.
    antz_garmin_usb_dog_t l = d; l.gps_lost = 1; l.flags = 0x20; l.gps_bars = 0;
    const std::string ls = formatDog(Format::Json, 1, l);
    CHECK(has(ls, R"("gps_lost":true)") && has(ls, R"("flags":32)") && has(ls, R"("gps_bars":0)"));
    CHECK(has(ls, R"("latitude":59.900000,"longitude":10.700000)"));

    // A model that confirms nothing: null, never a guess; the raw status stays.
    antz_garmin_usb_dog_t u = d;
    u.gps_lost_known = 0; u.gps_lost = 0; u.track_color_known = 0; u.track_color_code = 0;
    u.configured_id_known = 0; u.configured_id = 0; u.status = GARMIN_USB_DOG_STATUS_UNKNOWN; u.status_raw = 77;
    const std::string us = formatDog(Format::Json, 1, u);
    CHECK(has(us, R"("status":"unknown","status_raw":77)"));
    CHECK(has(us, R"("gps_lost":null)") && has(us, R"("track_color":null)") && has(us, R"("configured_id":null)"));
    CHECK(has(us, R"("battery_raw":40)"));

    // Southern and western position, six decimals.
    antz_garmin_usb_dog_t s = d; s.latitude_deg = -33.8688; s.longitude_deg = -70.6693;
    CHECK(has(formatDog(Format::Json, 1, s), R"("latitude":-33.868800,"longitude":-70.669300)"));
    antz_garmin_usb_dog_t r = d; r.latitude_deg = 59.1234567; r.longitude_deg = 10.0;
    CHECK(has(formatDog(Format::Json, 1, r), R"("latitude":59.123457,"longitude":10.000000)"));

    // Each status by name.
    struct C { antz_garmin_usb_dog_status_t s; const char* n; };
    for (const C& c : {C{GARMIN_USB_DOG_STATUS_SITTING, "sitting"}, C{GARMIN_USB_DOG_STATUS_TREED, "treed"},
                       C{GARMIN_USB_DOG_STATUS_POINTED, "pointed"}}) {
        antz_garmin_usb_dog_t x = d; x.status = c.s;
        CHECK(has(formatDog(Format::Json, 1, x), std::string("\"status\":\"") + c.n + "\""));
    }
    // The time follows the unix time, UTC.
    antz_garmin_usb_dog_t t0 = d; t0.unix_time_s = 631065600;
    CHECK(has(formatDog(Format::Json, 1, t0), R"("time":"1989-12-31T00:00:00Z")"));
    return 0;
}

static int json_escaping() {
    antz_garmin_usb_dog_t d = astro_dog();
    std::strcpy(d.name, "a\"b\\c");
    CHECK(has(formatDog(Format::Json, 1, d), R"("name":"a\"b\\c")"));
    std::memset(d.name, 0, sizeof d.name);
    std::strcpy(d.name, "x\x01y");
    CHECK(has(formatDog(Format::Json, 1, d), "\"name\":\"x\\u0001y\""));
    std::memset(d.name, 0, sizeof d.name);
    std::strcpy(d.name, "n\nt\t");
    const std::string s = formatDog(Format::Json, 1, d);
    CHECK(has(s, "\"name\":\"n\\nt\\t\"") || has(s, "\"name\":\"n\\u000at\\u0009\""));
    CHECK(chomp(s).find('\n') == std::string::npos);               // still one line
    CHECK(chomp(s).find('\t') == std::string::npos);
    // UTF-8 (Norwegian) passes through as it is.
    std::memset(d.name, 0, sizeof d.name);
    std::strcpy(d.name, "F\xC3\xB8y");
    CHECK(has(formatDog(Format::Json, 1, d), "\"name\":\"F\xC3\xB8y\""));
    std::memset(d.name, 0, sizeof d.name);
    CHECK(has(formatDog(Format::Json, 1, d), R"("name":"")"));
    return 0;
}

static int csv_format() {
    const char* pages[] = {"handheld", "handheld_position", "dog"};
    for (const char* p : pages) {
        const std::string h = csvHeader(p);
        CHECK(!h.empty());
        CHECK(chomp(h).find('\n') == std::string::npos);
    }
    antz_garmin_usb_product_t prod = {1339, 460};
    antz_garmin_usb_pvt_t pvt; std::memset(&pvt, 0, sizeof pvt);
    pvt.latitude_deg = 59.9; pvt.longitude_deg = 10.7; pvt.altitude_m = 141; pvt.fix = 3;
    const antz_garmin_usb_dog_t d = astro_dog();
    // A row has as many cells as its header.
    CHECK(csv_cells(chomp(csvHeader("handheld"))).size() == csv_cells(chomp(formatHandheld(Format::Csv, 5, prod))).size());
    CHECK(csv_cells(chomp(csvHeader("handheld_position"))).size() == csv_cells(chomp(formatHandheldPosition(Format::Csv, 5, pvt))).size());
    CHECK(csv_cells(chomp(csvHeader("dog"))).size() == csv_cells(chomp(formatDog(Format::Csv, 5, d))).size());
    // The columns are named as the JSON keys are.
    const std::string dh = csvHeader("dog");
    for (const char* k : {"unit_id", "name", "latitude", "longitude", "status", "gps_lost", "gps_bars", "comm_bars",
                          "battery_raw", "track_color", "configured_id"})
        CHECK(has(dh, k));
    CHECK(has(csvHeader("handheld"), "product_id") && has(csvHeader("handheld_position"), "altitude"));
    const std::string row = chomp(formatDog(Format::Csv, 3886356464u, d));
    CHECK(has(row, "3886356464") && has(row, "Khione") && has(row, "59.900000,10.700000") && has(row, "moving"));
    CHECK(has(row, "2026-11-10T12:00:00Z"));
    CHECK(row.find('{') == std::string::npos);
    // Unknown fields are empty cells, and only those three.
    antz_garmin_usb_dog_t u = d;
    u.gps_lost_known = 0; u.gps_lost = 0; u.track_color_known = 0; u.track_color_code = 0;
    u.configured_id_known = 0; u.configured_id = 0;
    int empty = 0;
    for (const auto& c : csv_cells(chomp(formatDog(Format::Csv, 5, u)))) if (c.empty()) ++empty;
    CHECK(empty == 3);
    empty = 0;
    for (const auto& c : csv_cells(row)) if (c.empty()) ++empty;
    CHECK(empty == 0);
    // A name with a comma or a quote is quoted (RFC 4180) and stays one row.
    antz_garmin_usb_dog_t q = d; std::strcpy(q.name, "a,b");
    CHECK(has(formatDog(Format::Csv, 1, q), "\"a,b\""));
    std::memset(q.name, 0, sizeof q.name); std::strcpy(q.name, "a\"b");
    CHECK(has(formatDog(Format::Csv, 1, q), "\"a\"\"b\""));
    return 0;
}

static int text_format() {
    antz_garmin_usb_dog_t d = astro_dog();
    const std::string s = chomp(formatDog(Format::Text, 3886356464u, d));
    CHECK(!s.empty() && s.find('\n') == std::string::npos);
    CHECK(has(s, "Khione") && has(s, "moving") && has(s, "59.900000") && has(s, "10.700000"));
    CHECK(s.find('?') == std::string::npos);                        // everything is known here
    d.gps_lost_known = 0; d.track_color_known = 0; d.configured_id_known = 0;
    d.status = GARMIN_USB_DOG_STATUS_UNKNOWN;
    const std::string u = chomp(formatDog(Format::Text, 1, d));
    CHECK(has(u, "?"));                                              // unknown is «?», never a guess
    CHECK(u.find("null") == std::string::npos);
    antz_garmin_usb_product_t p = {1339, 460};
    const std::string h = chomp(formatHandheld(Format::Text, 3886356464u, p));
    CHECK(has(h, "1339") && has(h, "4.60") && h.find('\n') == std::string::npos);
    return 0;
}

// ─── The fake session transport ────────────────────────────────────────────
struct Record { int layer; int pid; Bytes payload; };
static int nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}
static bool jfield(const std::string& line, const char* key, std::string* out) {
    const std::string k = std::string("\"") + key + "\"";
    size_t at = line.find(k);
    if (at == std::string::npos) return false;
    at = line.find(':', at); ++at;
    while (at < line.size() && line[at] == ' ') ++at;
    size_t end;
    if (line[at] == '"') { ++at; end = line.find('"', at); } else end = line.find_first_of(",}", at);
    *out = line.substr(at, end - at);
    return true;
}
static bool load(const char* name, std::vector<Record>* out) {
    std::ifstream in(std::string(ANTZ_REPLAY_DATA_DIR) + "/" + name);
    std::string line;
    while (std::getline(in, line)) {
        std::string ty, pid, hex;
        if (!jfield(line, "type", &ty) || !jfield(line, "pid", &pid) || !jfield(line, "hex", &hex)) continue;
        Record r; r.layer = std::atoi(ty.c_str()); r.pid = std::atoi(pid.c_str());
        for (size_t i = 0; i + 1 < hex.size(); i += 2) r.payload.push_back(static_cast<uint8_t>(nibble(hex[i]) * 16 + nibble(hex[i + 1])));
        out->push_back(r);
    }
    return !out->empty();
}
static void put_u16(Bytes& v, size_t at, uint16_t x) { v[at] = x & 0xFF; v[at + 1] = x >> 8; }
static void put_u32(Bytes& v, size_t at, uint32_t x) { for (int i = 0; i < 4; ++i) v[at + i] = (x >> (8 * i)) & 0xFF; }
static Bytes frame(int layer, int pid, const Bytes& payload) {
    Bytes f(12 + payload.size(), 0);
    f[0] = static_cast<uint8_t>(layer); put_u16(f, 4, static_cast<uint16_t>(pid)); put_u32(f, 8, static_cast<uint32_t>(payload.size()));
    if (!payload.empty()) std::memcpy(&f[12], payload.data(), payload.size());
    return f;
}

struct Fake {
    std::deque<Bytes> script[2];                 // an empty Bytes is a timeout
    std::deque<int> errors[2];                   // not used directly; see err_at
    Bytes pending[2];
    std::vector<Bytes> writes;
    uint64_t clock = 0;
    int reads = 0;
    int fail_write_from = -1;
    int err_after_reads = -1;                    // reads after this many return -1
    std::atomic<bool>* stop = nullptr;
    int stop_after_reads = -1;

    void push(int ep, const Bytes& b) { script[ep].push_back(b); }
    void push_chunked(int ep, const Bytes& b, size_t n) {
        for (size_t at = 0; at < b.size(); at += n) push(ep, Bytes(b.begin() + at, b.begin() + std::min(b.size(), at + n)));
    }
    void idle(int ep, int n) { for (int i = 0; i < n; ++i) push(ep, Bytes()); }
    // The records of a replay: dogs over bulk (announced), the rest over interrupt.
    bool load_replay(const char* name) {
        std::vector<Record> recs;
        if (!load(name, &recs)) return false;
        size_t n = 0;
        for (const Record& r : recs) {
            const Bytes f = frame(r.layer, r.pid, r.payload);
            const size_t piece = (n++ % 2 == 0) ? 64 : f.size();
            if (r.layer == 20 && r.pid == 3078) {
                push(1 - 1, frame(0, 2, Bytes()));
                push_chunked(1, f, piece);
                push(1, Bytes());
            } else {
                push_chunked(0, f, piece);
            }
        }
        return true;
    }
};
static int fake_write(void* c, const uint8_t* d, size_t n) {
    Fake* f = static_cast<Fake*>(c);
    const int idx = static_cast<int>(f->writes.size());
    if (f->fail_write_from >= 0 && idx >= f->fail_write_from) return -1;
    f->writes.emplace_back(d, d + n);
    return 0;
}
static int fake_read(void* c, antz_garmin_usb_endpoint_t ep, uint8_t* buf, size_t cap, uint32_t timeout_ms) {
    Fake* f = static_cast<Fake*>(c);
    ++f->reads;
    if (f->stop && f->stop_after_reads >= 0 && f->reads >= f->stop_after_reads) f->stop->store(true);
    if (f->err_after_reads >= 0 && f->reads > f->err_after_reads) return -1;
    Bytes& pend = f->pending[ep];
    if (pend.empty()) {
        if (f->script[ep].empty()) { f->clock += timeout_ms; return 0; }
        pend = f->script[ep].front(); f->script[ep].pop_front();
        if (pend.empty()) { f->clock += timeout_ms; return 0; }
    }
    const size_t n = std::min(cap, pend.size());
    std::memcpy(buf, pend.data(), n);
    pend.erase(pend.begin(), pend.begin() + n);
    f->clock += 1;
    return static_cast<int>(n);
}

struct Outcome {
    RunResult result;
    std::string out, err;
    uint64_t elapsed_ms;
};
static Outcome run_with(Fake& f, Options o, std::atomic<bool>* stop = nullptr) {
    std::atomic<bool> none{false};
    if (!stop) stop = &none;
    f.stop = stop;
    antz_garmin_usb_transport_t t = {fake_write, fake_read, &f};
    std::ostringstream out, err;
    const uint64_t start = f.clock;
    Outcome r;
    r.result = run(t, o, out, err, *stop, [&f]() { return f.clock; });
    r.out = out.str(); r.err = err.str(); r.elapsed_ms = f.clock - start;
    return r;
}
static Options opts(Format fmt, double seconds = 0, uint32_t idle_s = 15) {
    Options o; o.format = fmt; o.poll_ms = 500; o.idle_timeout_s = idle_s; o.seconds = seconds; return o;
}
static bool is_stop_cmd(const Bytes& w) {
    Bytes p(2, 0); put_u16(p, 0, 50);
    return w == frame(20, 10, p);
}
static std::vector<std::string> pages_of(const std::string& out) {
    std::vector<std::string> v;
    for (const auto& l : lines_of(out)) {
        const size_t at = l.find("\"page\":\"");
        if (at == std::string::npos) continue;
        const size_t s = at + 8;
        v.push_back(l.substr(s, l.find('"', s) - s));
    }
    return v;
}

// ─── Exact CSV and text, from the comment in garmin_usb_format.h ───────────
static int exact_csv_and_text() {
    CHECK(csvHeader("handheld") == "page,unit_id,product_id,software");
    CHECK(csvHeader("handheld_position") == "page,unit_id,latitude,longitude,altitude,fix");
    CHECK(csvHeader("dog") == "page,unit_id,name,latitude,longitude,time,status,status_raw,gps_lost,"
                              "gps_bars,comm_bars,battery_raw,flags,track_color,configured_id");

    antz_garmin_usb_product_t prod = {1339, 460};
    antz_garmin_usb_pvt_t pvt; std::memset(&pvt, 0, sizeof pvt);
    pvt.latitude_deg = 59.9; pvt.longitude_deg = 10.7; pvt.altitude_m = 141.0; pvt.fix = 3;
    const antz_garmin_usb_dog_t d = astro_dog();
    CHECK(formatHandheld(Format::Csv, 3886356464u, prod) == "handheld,3886356464,1339,4.60");
    CHECK(formatHandheldPosition(Format::Csv, 3886356464u, pvt) == "handheld_position,3886356464,59.900000,10.700000,141.0,3");
    CHECK(formatDog(Format::Csv, 3886356464u, d) ==
          "dog,3886356464,Khione,59.900000,10.700000,2026-11-10T12:00:00Z,moving,2,false,5,5,40,0,12,11");

    // Unknowns are empty cells: gps_lost, track_color, configured_id; the raw status stays.
    antz_garmin_usb_dog_t u = d;
    u.gps_lost_known = 0; u.gps_lost = 0; u.track_color_known = 0; u.track_color_code = 0;
    u.configured_id_known = 0; u.configured_id = 0; u.status = GARMIN_USB_DOG_STATUS_UNKNOWN; u.status_raw = 77;
    CHECK(formatDog(Format::Csv, 1, u) == "dog,1,Khione,59.900000,10.700000,2026-11-10T12:00:00Z,unknown,77,,5,5,40,0,,");

    // Lost GPS, southern and western position.
    antz_garmin_usb_dog_t l = d; l.gps_lost = 1; l.flags = 0x20; l.gps_bars = 0; l.latitude_deg = -33.8688; l.longitude_deg = -70.6693;
    CHECK(formatDog(Format::Csv, 1, l) == "dog,1,Khione,-33.868800,-70.669300,2026-11-10T12:00:00Z,moving,2,true,0,5,40,32,12,11");

    // Quoting, RFC 4180.
    antz_garmin_usb_dog_t q = d; std::strcpy(q.name, "a,b");
    CHECK(formatDog(Format::Csv, 1, q) == "dog,1,\"a,b\",59.900000,10.700000,2026-11-10T12:00:00Z,moving,2,false,5,5,40,0,12,11");
    std::memset(q.name, 0, sizeof q.name); std::strcpy(q.name, "say \"hi\"");
    CHECK(formatDog(Format::Csv, 1, q) == "dog,1,\"say \"\"hi\"\"\",59.900000,10.700000,2026-11-10T12:00:00Z,moving,2,false,5,5,40,0,12,11");
    std::memset(q.name, 0, sizeof q.name); std::strcpy(q.name, "two\nlines");
    CHECK(formatDog(Format::Csv, 1, q).find("\"two\nlines\"") != std::string::npos);

    // Text.
    CHECK(formatHandheld(Format::Text, 3886356464u, prod) == "handheld unit 3886356464 product 1339 software 4.60");
    CHECK(formatHandheldPosition(Format::Text, 3886356464u, pvt) == "handheld_position unit 3886356464 59.900000,10.700000 alt 141.0 fix 3");
    CHECK(formatDog(Format::Text, 3886356464u, d) ==
          "dog Khione unit 3886356464 59.900000,10.700000 2026-11-10T12:00:00Z moving (2) gps_lost no gps 5 comm 5 battery_raw 40");
    CHECK(formatDog(Format::Text, 1, u) ==
          "dog Khione unit 1 59.900000,10.700000 2026-11-10T12:00:00Z unknown (77) gps_lost ? gps 5 comm 5 battery_raw 40");
    // «yes» for a lost fix is the counterpart of «no» in the comment (inferred).
    CHECK(has(formatDog(Format::Text, 1, l), "moving (2) gps_lost yes gps 0 comm 5"));
    return 0;
}

// ─── Run: a normal replay ──────────────────────────────────────────────────
static int run_normal_json() {
    Fake f; CHECK(f.load_replay("replay_normal_astro.jsonl"));
    Outcome r = run_with(f, opts(Format::Json));
    CHECK(r.result == RunResult::IdleTimeout);                       // the replay ends in silence
    const auto lines = lines_of(r.out);
    CHECK(lines.size() == 14);                                       // handheld, position, 12 dogs
    const auto pages = pages_of(r.out);
    CHECK(pages.size() == 14 && pages[0] == "handheld" && pages[1] == "handheld_position");
    for (size_t i = 2; i < 14; ++i) CHECK(pages[i] == "dog");
    CHECK(has(lines[0], R"("unit_id":1234567890,"product_id":1339,"software":"4.60")"));
    for (size_t i = 2; i < 14; ++i) {
        CHECK(has(lines[i], R"("unit_id":1234567890)"));
        CHECK(has(lines[i], std::string("\"name\":\"") + (i % 2 == 0 ? "Rex" : "Luna") + "\""));   // in arrival order
        CHECK(has(lines[i], R"("status":"moving","status_raw":2,"gps_lost":false)"));
        CHECK(has(lines[i], R"("comm_bars":5)"));
    }
    CHECK(has(lines[2], R"("track_color":12,"configured_id":11)") && has(lines[3], R"("track_color":9,"configured_id":6)"));
    // Data only on out, one line each; messages on err and never JSON.
    for (const auto& l : lines) CHECK(l.size() > 2 && l[0] == '{' && l.back() == '}');
    CHECK(!r.err.empty() && !has(r.err, "\"page\"") && !has(r.err, "Rex"));
    return 0;
}

static int run_formats() {
    for (Format fmt : {Format::Text, Format::Csv}) {
        Fake f; CHECK(f.load_replay("replay_normal_astro.jsonl"));
        Outcome r = run_with(f, opts(fmt));
        CHECK(r.result == RunResult::IdleTimeout);
        const auto lines = lines_of(r.out);
        if (fmt == Format::Text) {
            CHECK(lines.size() == 14);
            CHECK(has(lines[2], "Rex") && has(lines[3], "Luna"));
            CHECK(!has(r.out, "\"page\""));
        } else {
            // Header once per page, before its first row: 3 headers + 14 rows.
            CHECK(lines.size() == 17);
            const std::string hh = chomp(csvHeader("handheld")), hp = chomp(csvHeader("handheld_position")),
                              hd = chomp(csvHeader("dog"));
            CHECK(lines[0] == hh && lines[2] == hp && lines[4] == hd);
            int dogs = 0, hdrs = 0;
            for (const auto& l : lines) { if (l == hh || l == hp || l == hd) ++hdrs; }
            CHECK(hdrs == 3);
            for (size_t i = 5; i < lines.size(); ++i) {
                CHECK(csv_cells(lines[i]).size() == csv_cells(hd).size());
                CHECK(has(lines[i], i % 2 == 1 ? "Rex" : "Luna"));
                ++dogs;
            }
            CHECK(dogs == 12);
            CHECK(has(lines[1], "1339") && has(lines[3], "60.000000"));
        }
    }
    return 0;
}

static int run_csv_header_once_per_page() {
    // A reconnect replay carries two handshakes in one run: the rows repeat, the headers do not.
    Fake f; CHECK(f.load_replay("replay_reconnect.jsonl"));
    Outcome r = run_with(f, opts(Format::Csv));
    CHECK(r.result == RunResult::IdleTimeout);
    const auto lines = lines_of(r.out);
    const std::string hh = chomp(csvHeader("handheld")), hp = chomp(csvHeader("handheld_position")),
                      hd = chomp(csvHeader("dog"));
    int a = 0, b = 0, c = 0, rows = 0;
    for (const auto& l : lines) {
        if (l == hh) ++a; else if (l == hp) ++b; else if (l == hd) ++c; else ++rows;
    }
    CHECK(a == 1 && b == 1 && c == 1);
    int dog_rows = 0;
    for (const auto& l : lines) if (has(l, "Rex") || has(l, "Luna")) ++dog_rows;
    CHECK(dog_rows == 12);
    CHECK(rows >= 12 + 1 + 1);                                       // dogs, at least a handheld and a position row
    // Each header sits before the first row of its page: the dog header before any dog row.
    size_t hd_at = 0, first_dog = 0;
    for (size_t i = 0; i < lines.size(); ++i) { if (lines[i] == hd) hd_at = i; if (!first_dog && (has(lines[i], "Rex"))) first_dog = i; }
    CHECK(hd_at < first_dog);
    return 0;
}

static int run_verbose_keeps_out_clean() {
    Fake f1; CHECK(f1.load_replay("replay_normal_astro.jsonl"));
    Fake f2; CHECK(f2.load_replay("replay_normal_astro.jsonl"));
    Options v = opts(Format::Json); v.verbose = true;
    Outcome a = run_with(f1, opts(Format::Json)), b = run_with(f2, v);
    CHECK(a.out == b.out);
    CHECK(b.err.size() >= a.err.size());
    return 0;
}

static int run_sequences() {
    {   // GPS loss: Rex rounds 2 and 3 are lost, frozen, time advancing.
        Fake f; CHECK(f.load_replay("replay_gps_loss.jsonl"));
        Outcome r = run_with(f, opts(Format::Json));
        CHECK(r.result == RunResult::IdleTimeout);
        std::vector<std::string> rex;
        for (const auto& l : lines_of(r.out)) if (has(l, R"("name":"Rex")")) rex.push_back(l);
        CHECK(rex.size() == 6);
        for (int i = 0; i < 6; ++i) CHECK(has(rex[i], (i == 2 || i == 3) ? R"("gps_lost":true)" : R"("gps_lost":false)"));
        CHECK(num_after(rex[2], "latitude") == num_after(rex[1], "latitude"));
        CHECK(num_after(rex[3], "longitude") == num_after(rex[1], "longitude"));
        CHECK(num_after(rex[4], "latitude") != num_after(rex[3], "latitude"));
        CHECK(has(rex[2], R"("status":"moving")") && has(rex[3], R"("status":"moving")"));
        std::string t2 = rex[2].substr(rex[2].find("\"time\""));
        std::string t3 = rex[3].substr(rex[3].find("\"time\""));
        CHECK(t2.substr(0, 28) != t3.substr(0, 28));                // time advances
    }
    {   // Rename: only the name changes.
        Fake f; CHECK(f.load_replay("replay_rename.jsonl"));
        Outcome r = run_with(f, opts(Format::Json));
        int rex = 0, rexi = 0, luna = 0;
        for (const auto& l : lines_of(r.out)) {
            if (has(l, R"("name":"Rex")")) ++rex;
            if (has(l, R"("name":"Rexi")")) ++rexi;
            if (has(l, R"("name":"Luna")")) ++luna;
        }
        CHECK(rex == 3 && rexi == 3 && luna == 6);
    }
    {   // Power cycle: the other collar's lines are all there; nothing says «removed».
        Fake f; CHECK(f.load_replay("replay_power_cycle.jsonl"));
        Outcome r = run_with(f, opts(Format::Json));
        int rex = 0, luna = 0;
        for (const auto& l : lines_of(r.out)) {
            if (has(l, R"("name":"Rex")")) ++rex;
            if (has(l, R"("name":"Luna")")) ++luna;
            CHECK(!has(l, "removed"));
        }
        CHECK(rex == 7 && luna == 4);
        CHECK(!has(r.err, "removed"));
    }
    {   // Alpha: the product and the status names.
        Fake f; CHECK(f.load_replay("replay_alpha.jsonl"));
        Outcome r = run_with(f, opts(Format::Json));
        const auto lines = lines_of(r.out);
        CHECK(has(lines[0], R"("product_id":1562,"software":"8.60")"));
        CHECK(has(lines[2], R"("name":"Rex")") && has(lines[2], R"("status":"sitting","status_raw":33)"));
        CHECK(has(lines[2], R"("configured_id":null)") && has(lines[2], R"("track_color":12)") && has(lines[2], R"("gps_lost":false)"));
        CHECK(has(lines[3], R"("status":"moving","status_raw":34)"));
    }
    return 0;
}

// ─── Run: time, stop, limits ───────────────────────────────────────────────
static int run_idle_timeout() {
    // Nothing at all: the idle count starts with the run.
    Fake f;
    Outcome r = run_with(f, opts(Format::Json, 0, 3));
    CHECK(r.result == RunResult::IdleTimeout);
    CHECK(r.elapsed_ms >= 3000 && r.elapsed_ms <= 3600);
    CHECK(r.out.empty());
    CHECK(!r.err.empty());
    // A packet resets the count: silence is counted from the last packet.
    Fake g;
    Bytes unit(4, 0); put_u32(unit, 0, 5);
    g.push(0, frame(0, 6, unit));
    const uint64_t t0 = g.clock;
    Outcome q = run_with(g, opts(Format::Json, 0, 3));
    CHECK(q.result == RunResult::IdleTimeout);
    CHECK(q.elapsed_ms - 0 >= 3000 && g.clock - t0 <= 3700);
    // A pvt every 2.5 s for ten rounds never idles out at 3 s.
    Fake h;
    Bytes pvt(64, 0);
    for (int i = 0; i < 10; ++i) { h.push(0, frame(20, 51, pvt)); h.idle(0, 5); }
    Outcome k = run_with(h, opts(Format::Json, 0, 3));
    CHECK(k.result == RunResult::IdleTimeout);
    // The last packet is read 9 rounds (2501 ms each: a 1 ms read and five 500 ms timeouts) after
    // the first; then 3000 ms of silence is needed, noticed at a poll (500 ms steps).
    CHECK(k.elapsed_ms >= 9 * 2501 + 1 + 3000 && k.elapsed_ms <= 9 * 2501 + 1 + 3000 + 500);
    int positions = 0;
    for (const auto& p : pages_of(k.out)) if (p == "handheld_position") ++positions;
    CHECK(positions == 10);
    return 0;
}

static int run_stop_request() {
    // Streaming, then a stop: Stop_Pvt_Data is sent and the result is Stopped.
    Fake f; CHECK(f.load_replay("replay_normal_astro.jsonl"));
    f.idle(0, 40);
    std::atomic<bool> stop{false};
    f.stop_after_reads = 8;
    Outcome r = run_with(f, opts(Format::Json), &stop);
    CHECK(r.result == RunResult::Stopped);
    CHECK(f.writes.size() == 4 && is_stop_cmd(f.writes[3]));
    // It stopped promptly: the rest of the replay was not read.
    CHECK(!f.script[0].empty() || !f.script[1].empty());
    CHECK(exitCode(r.result) == 0);
    return 0;
}

static int run_stop_before_stream() {
    // Asked to stop before Session_Started: no Stop_Pvt_Data, result Stopped.
    Fake f;
    std::atomic<bool> stop{true};
    Outcome r = run_with(f, opts(Format::Json), &stop);
    CHECK(r.result == RunResult::Stopped);
    for (const auto& w : f.writes) CHECK(!is_stop_cmd(w));
    CHECK(f.writes.size() <= 1);
    return 0;
}

static int run_seconds_limit() {
    {   // nothing arrives: the limit ends the run, as Stopped and before idle
        Fake f;
        Outcome r = run_with(f, opts(Format::Json, 2, 15));
        CHECK(r.result == RunResult::Stopped);
        CHECK(r.elapsed_ms >= 2000 && r.elapsed_ms <= 2700);
        CHECK(f.writes.size() == 1);                                 // only Start_Session: no stream, no stop
    }
    {   // streaming: the limit sends Stop_Pvt_Data
        Fake f; CHECK(f.load_replay("replay_normal_astro.jsonl"));
        f.idle(0, 100);
        Outcome r = run_with(f, opts(Format::Json, 5, 15));
        CHECK(r.result == RunResult::Stopped);
        CHECK(f.writes.size() == 4 && is_stop_cmd(f.writes[3]));
        CHECK(r.elapsed_ms >= 5000 && r.elapsed_ms <= 5700);
    }
    return 0;
}

// ─── Run: errors ───────────────────────────────────────────────────────────
static bool one_line(const std::string& s) {
    const auto l = lines_of(s);
    return l.size() == 1 && !l[0].empty();
}
static int run_errors() {
    {   // a read fails
        Fake f; f.err_after_reads = 2;
        Bytes unit(4, 0); put_u32(unit, 0, 5);
        f.push(0, frame(0, 6, unit));
        Outcome r = run_with(f, opts(Format::Json));
        CHECK(r.result == RunResult::TransportError);
        CHECK(one_line(r.err) && r.out.find("\"page\"") == std::string::npos);
        CHECK(exitCode(r.result) != 0);
    }
    {   // an invalid frame header
        Fake f;
        f.push(0, frame(7, 3078, Bytes(10, 0)));
        Outcome r = run_with(f, opts(Format::Json));
        CHECK(r.result == RunResult::FrameError);
        CHECK(one_line(r.err));
        CHECK(exitCode(r.result) != 0);
    }
    {   // bulk ends inside a packet
        Fake f;
        const Bytes dog = frame(20, 3078, Bytes(100, 0));
        f.push(0, frame(0, 2, Bytes()));
        f.push(1, Bytes(dog.begin(), dog.begin() + 50));
        f.push(1, Bytes());
        Outcome r = run_with(f, opts(Format::Json));
        CHECK(r.result == RunResult::PartialPacket);
        CHECK(one_line(r.err) && r.out.empty());
        CHECK(exitCode(r.result) != 0);
    }
    {   // Start_Session cannot be written
        Fake f; f.fail_write_from = 0;
        Outcome r = run_with(f, opts(Format::Json));
        CHECK(r.result == RunResult::StartFailed);
        CHECK(one_line(r.err) && r.out.empty());
        CHECK(f.reads == 0);
        CHECK(exitCode(r.result) != 0);
    }
    {   // the handshake answer cannot be written: the cable is gone
        Fake f; f.fail_write_from = 1;
        Bytes unit(4, 0); put_u32(unit, 0, 5);
        f.push(0, frame(0, 6, unit));
        Outcome r = run_with(f, opts(Format::Json));
        CHECK(r.result == RunResult::TransportError);
        CHECK(one_line(r.err));
    }
    // The messages differ between causes.
    return 0;
}

static int exit_codes() {
    CHECK(exitCode(RunResult::Stopped) == 0);
    for (RunResult r : {RunResult::StartFailed, RunResult::IdleTimeout, RunResult::TransportError,
                        RunResult::FrameError, RunResult::PartialPacket})
        CHECK(exitCode(r) != 0);
    return 0;
}

int main(int argc, char** argv) {
    struct Group { const char* name; int (*fn)(); };
    const Group groups[] = {
        {"status_names", status_names}, {"json_handheld", json_handheld}, {"json_position", json_position},
        {"json_dog", json_dog}, {"json_escaping", json_escaping}, {"csv_format", csv_format},
        {"text_format", text_format}, {"exact_csv_and_text", exact_csv_and_text}, {"run_normal", run_normal_json}, {"run_formats", run_formats},
        {"run_csv_header", run_csv_header_once_per_page}, {"run_verbose", run_verbose_keeps_out_clean},
        {"run_sequences", run_sequences}, {"run_idle", run_idle_timeout}, {"run_stop", run_stop_request},
        {"run_stop_early", run_stop_before_stream}, {"run_seconds", run_seconds_limit},
        {"run_errors", run_errors}, {"exit_codes", exit_codes},
    };
    for (const Group& g : groups) {
        if (argc > 1 && std::strcmp(argv[1], g.name) != 0) continue;
        if (g.fn() != 0) return 1;
    }
    std::printf("ok (%d checks)\n", checks);
    return 0;
}
