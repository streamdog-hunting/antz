//
// Tests for the libusb transport (garmin_usb_libusb.h) over a fake ops table:
// every call recorded in order, failures injectable per call, device counts,
// endpoints and transfer results scripted. No libusb, no handheld.
//
#include "garmin_usb_libusb.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <deque>
#include <map>
#include <string>
#include <vector>

using namespace antz;

static int checks = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); return 1; } } while (0)

typedef std::vector<uint8_t> Bytes;

// ─── Fake ops ───────────────────────────────────────────────────────────────
struct Fake {
    std::vector<std::string> calls;                 // names, in order
    std::map<std::string, int> fail;                // call name -> value it returns instead
    int devices = 1;
    int driver_active = 0;
    antz_libusb_endpoint_t ep_int = {0x83, 64};
    antz_libusb_endpoint_t ep_bin = {0x84, 512};
    antz_libusb_endpoint_t ep_bout = {0x05, 512};

    // what the transfers saw
    struct Xfer { std::string call; uint8_t endpoint; size_t len; uint32_t timeout; Bytes data; };
    std::vector<Xfer> xfers;
    uint16_t seen_vid = 0, seen_pid = 0;
    std::vector<int> seen_interfaces;

    // scripted results for reads: bytes, or a code (timeout / error)
    struct Item { Bytes bytes; int code = 0; bool is_code = false; };
    std::deque<Item> int_in, bulk_in;
    int short_write_by = 0;                         // bulk_out returns len - this

    void push(std::deque<Item>& q, const Bytes& b) { Item i; i.bytes = b; q.push_back(i); }
    void push_code(std::deque<Item>& q, int c) { Item i; i.code = c; i.is_code = true; q.push_back(i); }

    int count(const char* name) const {
        int n = 0; for (const auto& c : calls) if (c == name) ++n; return n;
    }
    int first(const char* name) const {
        for (size_t i = 0; i < calls.size(); ++i) if (calls[i] == name) return static_cast<int>(i);
        return -1;
    }
    std::string seq() const {
        std::string s; for (const auto& c : calls) { if (!s.empty()) s += ","; s += c; } return s;
    }
    bool failing(const char* n, int* out) { auto it = fail.find(n); if (it == fail.end()) return false; *out = it->second; return true; }
};
static Fake& F(void* c) { return *static_cast<Fake*>(c); }

static int f_count(void* c, uint16_t v, uint16_t p) {
    Fake& f = F(c); f.calls.push_back("count_devices"); f.seen_vid = v; f.seen_pid = p;
    int r; if (f.failing("count_devices", &r)) return r; return f.devices;
}
static int f_open(void* c, uint16_t v, uint16_t p) {
    Fake& f = F(c); f.calls.push_back("open");
    if (v != f.seen_vid || p != f.seen_pid) f.calls.push_back("open_wrong_ids");
    int r; if (f.failing("open", &r)) return r; return 0;
}
static int f_endpoints(void* c, antz_libusb_endpoint_t* i, antz_libusb_endpoint_t* bi, antz_libusb_endpoint_t* bo) {
    Fake& f = F(c); f.calls.push_back("endpoints");
    int r; if (f.failing("endpoints", &r)) return r;
    *i = f.ep_int; *bi = f.ep_bin; *bo = f.ep_bout; return 0;
}
static int f_active(void* c, int n) {
    Fake& f = F(c); f.calls.push_back("kernel_driver_active"); f.seen_interfaces.push_back(n);
    int r; if (f.failing("kernel_driver_active", &r)) return r; return f.driver_active;
}
static int f_detach(void* c, int n) {
    Fake& f = F(c); f.calls.push_back("detach"); f.seen_interfaces.push_back(n);
    int r; if (f.failing("detach", &r)) return r; return 0;
}
static int f_attach(void* c, int n) {
    Fake& f = F(c); f.calls.push_back("attach"); f.seen_interfaces.push_back(n);
    int r; if (f.failing("attach", &r)) return r; return 0;
}
static int f_claim(void* c, int n) {
    Fake& f = F(c); f.calls.push_back("claim"); f.seen_interfaces.push_back(n);
    int r; if (f.failing("claim", &r)) return r; return 0;
}
static int f_release(void* c, int n) {
    Fake& f = F(c); f.calls.push_back("release"); f.seen_interfaces.push_back(n);
    int r; if (f.failing("release", &r)) return r; return 0;
}
static int do_in(Fake& f, const char* name, std::deque<Fake::Item>& q, uint8_t ep, uint8_t* buf, size_t len, uint32_t to) {
    f.xfers.push_back({name, ep, len, to, Bytes()});
    int r; if (f.failing(name, &r)) return r;
    if (q.empty()) return ANTZ_LIBUSB_TIMEOUT;
    Fake::Item it = q.front(); q.pop_front();
    if (it.is_code) return it.code;
    const size_t n = std::min(len, it.bytes.size());
    std::memcpy(buf, it.bytes.data(), n);
    return static_cast<int>(n);
}
static int f_int_in(void* c, uint8_t ep, uint8_t* b, size_t l, uint32_t t) { return do_in(F(c), "interrupt_in", F(c).int_in, ep, b, l, t); }
static int f_bulk_in(void* c, uint8_t ep, uint8_t* b, size_t l, uint32_t t) { return do_in(F(c), "bulk_in", F(c).bulk_in, ep, b, l, t); }
static int f_bulk_out(void* c, uint8_t ep, const uint8_t* b, size_t l, uint32_t t) {
    Fake& f = F(c);
    f.xfers.push_back({"bulk_out", ep, l, t, Bytes(b ? b : (const uint8_t*)"", (b ? b : (const uint8_t*)"") + l)});
    int r; if (f.failing("bulk_out", &r)) return r;
    return static_cast<int>(l) - f.short_write_by;
}
static void f_close(void* c) { F(c).calls.push_back("close"); }

static antz_libusb_ops_t ops_for(Fake* f) {
    antz_libusb_ops_t o;
    std::memset(&o, 0, sizeof o);
    o.count_devices = f_count; o.open = f_open; o.endpoints = f_endpoints;
    o.kernel_driver_active = f_active; o.detach_kernel_driver = f_detach; o.attach_kernel_driver = f_attach;
    o.claim_interface = f_claim; o.release_interface = f_release;
    o.interrupt_in = f_int_in; o.bulk_in = f_bulk_in; o.bulk_out = f_bulk_out;
    o.close = f_close; o.ctx = f;
    return o;
}

static bool nothing_held(const antz_garmin_usb_libusb_t& t) { return !t.opened && !t.detached && !t.claimed; }

// ─── Frame builders for the end-to-end group ───────────────────────────────
static void put_u16(Bytes& v, size_t at, uint16_t x) { v[at] = x & 0xFF; v[at + 1] = x >> 8; }
static void put_u32(Bytes& v, size_t at, uint32_t x) { for (int i = 0; i < 4; ++i) v[at + i] = (x >> (8 * i)) & 0xFF; }
static Bytes frame(uint8_t layer, uint16_t pid, const Bytes& payload) {
    Bytes f(12 + payload.size(), 0);
    f[0] = layer; put_u16(f, 4, pid); put_u32(f, 8, static_cast<uint32_t>(payload.size()));
    if (!payload.empty()) std::memcpy(&f[12], payload.data(), payload.size());
    return f;
}
static Bytes cat(Bytes a, const Bytes& b) { a.insert(a.end(), b.begin(), b.end()); return a; }

// ─── Choosing the device ────────────────────────────────────────────────────
static int one_device() {
    Fake f; antz_libusb_ops_t o = ops_for(&f);
    antz_garmin_usb_libusb_t t;
    CHECK(garmin_usb_libusb_open(&t, &o) == GARMIN_USB_LIBUSB_OK);
    CHECK(f.seen_vid == 0x091E && f.seen_pid == 0x0003);
    CHECK(f.count("open_wrong_ids") == 0);
    CHECK(t.opened && t.claimed && !t.detached);
    CHECK(t.interrupt_in.address == 0x83 && t.interrupt_in.max_packet == 64);
    CHECK(t.bulk_in.address == 0x84 && t.bulk_out.address == 0x05 && t.bulk_out.max_packet == 512);
    for (int n : f.seen_interfaces) CHECK(n == GARMIN_USB_INTERFACE && n == 0);
    return 0;
}

static int device_count() {
    struct Case { int devices; antz_garmin_usb_libusb_status_t want; };
    const Case cases[] = {{0, GARMIN_USB_LIBUSB_NO_DEVICE}, {2, GARMIN_USB_LIBUSB_MORE_THAN_ONE},
                          {5, GARMIN_USB_LIBUSB_MORE_THAN_ONE}, {-1, GARMIN_USB_LIBUSB_LIST_FAILED}};
    for (const Case& c : cases) {
        Fake f; f.devices = c.devices; antz_libusb_ops_t o = ops_for(&f);
        antz_garmin_usb_libusb_t t;
        CHECK(garmin_usb_libusb_open(&t, &o) == c.want);
        CHECK(f.seq() == "count_devices");            // nothing was opened, so nothing to undo
        CHECK(nothing_held(t));
    }
    // A list call that fails outright, whatever the number it returns.
    Fake f; f.fail["count_devices"] = -7; antz_libusb_ops_t o = ops_for(&f);
    antz_garmin_usb_libusb_t t;
    CHECK(garmin_usb_libusb_open(&t, &o) == GARMIN_USB_LIBUSB_LIST_FAILED);
    return 0;
}

// ─── Open order and undo ────────────────────────────────────────────────────
static int open_order() {
    Fake f; antz_libusb_ops_t o = ops_for(&f);
    antz_garmin_usb_libusb_t t;
    CHECK(garmin_usb_libusb_open(&t, &o) == GARMIN_USB_LIBUSB_OK);
    CHECK(f.first("count_devices") < f.first("open"));
    CHECK(f.first("open") < f.first("endpoints"));
    CHECK(f.first("endpoints") < f.first("kernel_driver_active"));
    CHECK(f.first("kernel_driver_active") < f.first("claim"));
    CHECK(f.count("detach") == 0 && f.count("attach") == 0);       // no driver held it
    CHECK(f.calls.back() == "claim");
    return 0;
}

static int open_fails_each_step() {
    {   // open
        Fake f; f.fail["open"] = -1; antz_libusb_ops_t o = ops_for(&f);
        antz_garmin_usb_libusb_t t;
        CHECK(garmin_usb_libusb_open(&t, &o) == GARMIN_USB_LIBUSB_OPEN_FAILED);
        CHECK(f.count("close") == 0 && f.count("claim") == 0 && f.count("detach") == 0 && f.count("release") == 0);
        CHECK(nothing_held(t));
    }
    {   // endpoints: the device is open and is closed again
        Fake f; f.fail["endpoints"] = 1; antz_libusb_ops_t o = ops_for(&f);
        antz_garmin_usb_libusb_t t;
        CHECK(garmin_usb_libusb_open(&t, &o) == GARMIN_USB_LIBUSB_NO_ENDPOINTS);
        CHECK(f.calls.back() == "close" && f.count("close") == 1);
        CHECK(f.count("detach") == 0 && f.count("claim") == 0 && f.count("release") == 0 && f.count("attach") == 0);
        CHECK(nothing_held(t));
    }
    {   // detach: nothing was detached, so no attach; the device is closed
        Fake f; f.driver_active = 1; f.fail["detach"] = -1; antz_libusb_ops_t o = ops_for(&f);
        antz_garmin_usb_libusb_t t;
        CHECK(garmin_usb_libusb_open(&t, &o) == GARMIN_USB_LIBUSB_DETACH_FAILED);
        CHECK(f.count("attach") == 0 && f.count("claim") == 0 && f.count("release") == 0);
        CHECK(f.calls.back() == "close" && f.count("close") == 1);
        CHECK(nothing_held(t));
    }
    {   // claim after a detach: attach again, then close, no release (never claimed)
        Fake f; f.driver_active = 1; f.fail["claim"] = -1; antz_libusb_ops_t o = ops_for(&f);
        antz_garmin_usb_libusb_t t;
        CHECK(garmin_usb_libusb_open(&t, &o) == GARMIN_USB_LIBUSB_CLAIM_FAILED);
        CHECK(f.count("release") == 0);
        CHECK(f.count("attach") == 1 && f.count("close") == 1);
        CHECK(f.first("detach") < f.first("claim") && f.first("claim") < f.first("attach"));
        CHECK(f.first("attach") < f.first("close") && f.calls.back() == "close");
        CHECK(nothing_held(t));
    }
    {   // claim with no driver detached: only the close
        Fake f; f.fail["claim"] = -1; antz_libusb_ops_t o = ops_for(&f);
        antz_garmin_usb_libusb_t t;
        CHECK(garmin_usb_libusb_open(&t, &o) == GARMIN_USB_LIBUSB_CLAIM_FAILED);
        CHECK(f.count("attach") == 0 && f.count("release") == 0 && f.count("close") == 1);
        CHECK(nothing_held(t));
    }
    return 0;
}

static int kernel_driver() {
    {   // active: detached once, attached again on close
        Fake f; f.driver_active = 1; antz_libusb_ops_t o = ops_for(&f);
        antz_garmin_usb_libusb_t t;
        CHECK(garmin_usb_libusb_open(&t, &o) == GARMIN_USB_LIBUSB_OK);
        CHECK(f.count("detach") == 1 && t.detached == 1);
        CHECK(f.first("detach") < f.first("claim"));
        garmin_usb_libusb_close(&t);
        CHECK(f.count("attach") == 1);
    }
    {   // not active: neither detach nor attach, ever
        Fake f; f.driver_active = 0; antz_libusb_ops_t o = ops_for(&f);
        antz_garmin_usb_libusb_t t;
        CHECK(garmin_usb_libusb_open(&t, &o) == GARMIN_USB_LIBUSB_OK);
        garmin_usb_libusb_close(&t);
        CHECK(f.count("detach") == 0 && f.count("attach") == 0);
    }
    return 0;
}

// ─── Close ──────────────────────────────────────────────────────────────────
static int close_order() {
    Fake f; f.driver_active = 1; antz_libusb_ops_t o = ops_for(&f);
    antz_garmin_usb_libusb_t t;
    CHECK(garmin_usb_libusb_open(&t, &o) == GARMIN_USB_LIBUSB_OK);
    f.calls.clear();
    garmin_usb_libusb_close(&t);
    CHECK(f.seq() == "release,attach,close");
    CHECK(nothing_held(t));
    // Twice: nothing more is called.
    f.calls.clear();
    garmin_usb_libusb_close(&t);
    CHECK(f.calls.empty());
    // Without a detach: release, close.
    Fake g; antz_libusb_ops_t o2 = ops_for(&g);
    antz_garmin_usb_libusb_t t2;
    CHECK(garmin_usb_libusb_open(&t2, &o2) == GARMIN_USB_LIBUSB_OK);
    g.calls.clear();
    garmin_usb_libusb_close(&t2);
    CHECK(g.seq() == "release,close");
    return 0;
}

static int close_never_opened() {
    antz_garmin_usb_libusb_t t;
    std::memset(&t, 0, sizeof t);
    garmin_usb_libusb_close(&t);                       // no ops at all: must not call through null
    CHECK(nothing_held(t));
    garmin_usb_libusb_close(nullptr);
    // After a refused open the struct is closed too.
    Fake f; f.devices = 0; antz_libusb_ops_t o = ops_for(&f);
    antz_garmin_usb_libusb_t u;
    CHECK(garmin_usb_libusb_open(&u, &o) == GARMIN_USB_LIBUSB_NO_DEVICE);
    f.calls.clear();
    garmin_usb_libusb_close(&u);
    CHECK(f.calls.empty());
    return 0;
}

static int bad_arguments() {
    Fake f; antz_libusb_ops_t o = ops_for(&f);
    antz_garmin_usb_libusb_t t;
    CHECK(garmin_usb_libusb_open(nullptr, &o) == GARMIN_USB_LIBUSB_BAD_ARGUMENT);
    CHECK(garmin_usb_libusb_open(&t, nullptr) == GARMIN_USB_LIBUSB_BAD_ARGUMENT);
    // Every call in the table is required.
    antz_libusb_ops_t base = ops_for(&f);
    for (int which = 0; which < 12; ++which) {
        antz_libusb_ops_t b = base;
        switch (which) {
            case 0: b.count_devices = nullptr; break;
            case 1: b.open = nullptr; break;
            case 2: b.endpoints = nullptr; break;
            case 3: b.kernel_driver_active = nullptr; break;
            case 4: b.detach_kernel_driver = nullptr; break;
            case 5: b.attach_kernel_driver = nullptr; break;
            case 6: b.claim_interface = nullptr; break;
            case 7: b.release_interface = nullptr; break;
            case 8: b.interrupt_in = nullptr; break;
            case 9: b.bulk_in = nullptr; break;
            case 10: b.bulk_out = nullptr; break;
            default: b.close = nullptr; break;
        }
        f.calls.clear();
        CHECK(garmin_usb_libusb_open(&t, &b) == GARMIN_USB_LIBUSB_BAD_ARGUMENT);
        CHECK(f.calls.empty());
        CHECK(nothing_held(t));
    }
    return 0;
}

// ─── Writing ────────────────────────────────────────────────────────────────
static int write_packets() {
    for (uint16_t maxp : {uint16_t(512), uint16_t(64)}) {
        Fake f; f.ep_bout.max_packet = maxp; antz_libusb_ops_t o = ops_for(&f);
        antz_garmin_usb_libusb_t t;
        CHECK(garmin_usb_libusb_open(&t, &o) == GARMIN_USB_LIBUSB_OK);
        antz_garmin_usb_transport_t tr = garmin_usb_libusb_transport(&t);
        CHECK(tr.write && tr.read);

        // Not a multiple: one transfer, the whole packet, on the bulk out endpoint.
        Bytes p(12, 0x11);
        f.xfers.clear();
        CHECK(tr.write(tr.ctx, p.data(), p.size()) == 0);
        CHECK(f.xfers.size() == 1 && f.xfers[0].call == "bulk_out");
        CHECK(f.xfers[0].endpoint == 0x05 && f.xfers[0].len == 12 && f.xfers[0].data == p);
        CHECK(f.xfers[0].timeout == GARMIN_USB_WRITE_TIMEOUT_MS && GARMIN_USB_WRITE_TIMEOUT_MS == 1500);

        // One byte over a multiple: none.
        Bytes q(maxp + 1, 0x22);
        f.xfers.clear();
        CHECK(tr.write(tr.ctx, q.data(), q.size()) == 0);
        CHECK(f.xfers.size() == 1 && f.xfers[0].len == q.size());

        // A whole number of max packets: the packet, then a zero length packet.
        for (size_t mult : {size_t(1), size_t(2)}) {
            Bytes m(maxp * mult, 0x33);
            f.xfers.clear();
            CHECK(tr.write(tr.ctx, m.data(), m.size()) == 0);
            CHECK(f.xfers.size() == 2);
            CHECK(f.xfers[0].len == m.size() && f.xfers[0].data == m);
            CHECK(f.xfers[1].call == "bulk_out" && f.xfers[1].len == 0 && f.xfers[1].endpoint == 0x05);
            CHECK(f.xfers[1].timeout == GARMIN_USB_WRITE_TIMEOUT_MS);
        }
        // The zero length packet is not sent when the packet itself failed.
        Bytes m(maxp, 0x44);
        f.xfers.clear();
        f.fail["bulk_out"] = ANTZ_LIBUSB_ERROR;
        CHECK(tr.write(tr.ctx, m.data(), m.size()) != 0);
        CHECK(f.xfers.size() == 1);
    }
    return 0;
}

static int write_failures() {
    Fake f; antz_libusb_ops_t o = ops_for(&f);
    antz_garmin_usb_libusb_t t;
    CHECK(garmin_usb_libusb_open(&t, &o) == GARMIN_USB_LIBUSB_OK);
    antz_garmin_usb_transport_t tr = garmin_usb_libusb_transport(&t);
    Bytes p(12, 1);
    f.short_write_by = 1;                               // the transfer moved 11 of 12
    CHECK(tr.write(tr.ctx, p.data(), p.size()) != 0);
    f.short_write_by = 0;
    f.fail["bulk_out"] = ANTZ_LIBUSB_TIMEOUT;
    CHECK(tr.write(tr.ctx, p.data(), p.size()) != 0);   // a write timeout is an error, not «nothing»
    f.fail["bulk_out"] = ANTZ_LIBUSB_ERROR;
    CHECK(tr.write(tr.ctx, p.data(), p.size()) != 0);
    // A short zero-length-packet-free packet that is moved whole is fine again.
    f.fail.clear();
    CHECK(tr.write(tr.ctx, p.data(), p.size()) == 0);
    return 0;
}

// ─── Reading ────────────────────────────────────────────────────────────────
static int read_interrupt() {
    Fake f; antz_libusb_ops_t o = ops_for(&f);
    antz_garmin_usb_libusb_t t;
    CHECK(garmin_usb_libusb_open(&t, &o) == GARMIN_USB_LIBUSB_OK);
    antz_garmin_usb_transport_t tr = garmin_usb_libusb_transport(&t);
    uint8_t buf[2048];

    f.push(f.int_in, Bytes(64, 7));
    f.xfers.clear();
    CHECK(tr.read(tr.ctx, GARMIN_USB_ENDPOINT_INTERRUPT, buf, 512, 77) == 64);
    CHECK(f.xfers.size() == 1 && f.xfers[0].call == "interrupt_in");
    CHECK(f.xfers[0].endpoint == 0x83 && f.xfers[0].len == 64 && f.xfers[0].timeout == 77);   // min(cap, max packet)
    CHECK(buf[0] == 7 && buf[63] == 7);

    f.push(f.int_in, Bytes(10, 8));
    f.xfers.clear();
    CHECK(tr.read(tr.ctx, GARMIN_USB_ENDPOINT_INTERRUPT, buf, 10, 5) == 10);
    CHECK(f.xfers[0].len == 10);                                                              // the caller's room is smaller

    f.push(f.int_in, Bytes(12, 9));                     // fewer than a max packet
    CHECK(tr.read(tr.ctx, GARMIN_USB_ENDPOINT_INTERRUPT, buf, 512, 5) == 12);
    return 0;
}

static int read_bulk() {
    Fake f; antz_libusb_ops_t o = ops_for(&f);
    antz_garmin_usb_libusb_t t;
    CHECK(garmin_usb_libusb_open(&t, &o) == GARMIN_USB_LIBUSB_OK);
    antz_garmin_usb_transport_t tr = garmin_usb_libusb_transport(&t);
    static uint8_t buf[4096];
    f.push(f.bulk_in, Bytes(300, 5));
    f.xfers.clear();
    CHECK(tr.read(tr.ctx, GARMIN_USB_ENDPOINT_BULK, buf, 2048, 123) == 300);
    CHECK(f.xfers.size() == 1 && f.xfers[0].call == "bulk_in");
    CHECK(f.xfers[0].endpoint == 0x84 && f.xfers[0].len == 2048 && f.xfers[0].timeout == 123);   // not clamped to a max packet
    CHECK(buf[0] == 5 && buf[299] == 5);
    // A zero length packet reads as 0.
    f.push(f.bulk_in, Bytes());
    CHECK(tr.read(tr.ctx, GARMIN_USB_ENDPOINT_BULK, buf, 512, 5) == 0);
    return 0;
}

static int read_timeout_and_errors() {
    Fake f; antz_libusb_ops_t o = ops_for(&f);
    antz_garmin_usb_libusb_t t;
    CHECK(garmin_usb_libusb_open(&t, &o) == GARMIN_USB_LIBUSB_OK);
    antz_garmin_usb_transport_t tr = garmin_usb_libusb_transport(&t);
    uint8_t buf[512];
    for (int ep : {GARMIN_USB_ENDPOINT_INTERRUPT, GARMIN_USB_ENDPOINT_BULK}) {
        auto& q = ep == GARMIN_USB_ENDPOINT_INTERRUPT ? f.int_in : f.bulk_in;
        const auto e = static_cast<antz_garmin_usb_endpoint_t>(ep);
        f.push_code(q, ANTZ_LIBUSB_TIMEOUT);
        CHECK(tr.read(tr.ctx, e, buf, 512, 5) == 0);            // not an error
        f.push_code(q, ANTZ_LIBUSB_ERROR);
        CHECK(tr.read(tr.ctx, e, buf, 512, 5) < 0);
        f.push_code(q, -4);                                      // any other failure
        CHECK(tr.read(tr.ctx, e, buf, 512, 5) < 0);
        f.push_code(q, -99);
        CHECK(tr.read(tr.ctx, e, buf, 512, 5) < 0);
        CHECK(tr.read(tr.ctx, e, buf, 512, 5) == 0);             // script empty: timeout
    }
    return 0;
}

// ─── The real session over the fake ops ─────────────────────────────────────
struct Log {
    std::vector<uint32_t> units;
    std::vector<antz_garmin_usb_product_t> products;
    std::vector<antz_garmin_usb_dog_t> dogs;
};
static void ev_s(void* c, uint32_t u) { static_cast<Log*>(c)->units.push_back(u); }
static void ev_p(void* c, const antz_garmin_usb_product_t* p) { static_cast<Log*>(c)->products.push_back(*p); }
static void ev_d(void* c, const antz_garmin_usb_dog_t* d) { static_cast<Log*>(c)->dogs.push_back(*d); }

static int end_to_end() {
    Fake f; antz_libusb_ops_t o = ops_for(&f);
    antz_garmin_usb_libusb_t t;
    CHECK(garmin_usb_libusb_open(&t, &o) == GARMIN_USB_LIBUSB_OK);
    antz_garmin_usb_transport_t tr = garmin_usb_libusb_transport(&t);
    Log log;
    antz_garmin_usb_events_t ev = {ev_s, ev_p, nullptr, ev_d, &log};
    antz_garmin_usb_session_t s;
    CHECK(garmin_usb_session_init(&s, &tr, &ev) == 0);
    CHECK(garmin_usb_session_start(&s) == 0);

    // Start_Session is on bulk out, 12 bytes, and not followed by a zero length packet.
    CHECK(f.xfers.size() == 1 && f.xfers[0].call == "bulk_out" && f.xfers[0].endpoint == 0x05);
    CHECK((f.xfers[0].data == Bytes{0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0}));

    // Session_Started and Product_Data arrive on interrupt.
    Bytes unit(4, 0); put_u32(unit, 0, 3886356464u);
    f.push(f.int_in, frame(0, 6, unit));
    CHECK(garmin_usb_session_poll(&s, 10) == GARMIN_USB_POLL_OK);
    CHECK(log.units.size() == 1 && log.units[0] == 3886356464u);
    CHECK(f.xfers.size() == 3);
    CHECK(f.xfers[1].data == frame(20, 254, Bytes()));
    Bytes cmd(2, 0); put_u16(cmd, 0, 49);
    CHECK(f.xfers[2].data == frame(20, 10, cmd));

    Bytes prod(24, 0); put_u16(prod, 0, 1339); put_u16(prod, 2, 460);
    f.push(f.int_in, frame(20, 255, prod));
    CHECK(garmin_usb_session_poll(&s, 10) == GARMIN_USB_POLL_OK);
    CHECK(log.products.size() == 1 && log.products[0].product_id == 1339);

    // A dog comes on bulk after Data_Available on interrupt, cut in two bulk reads;
    // the empty bulk read (a timeout) hands the session back to interrupt.
    Bytes dog(100, 0);
    dog[16] = 2; dog[31] = 'K'; dog[32] = 'i'; dog[33] = 'm';
    const Bytes dogf = frame(20, 3078, dog);
    f.push(f.int_in, frame(0, 2, Bytes()));
    f.push(f.bulk_in, Bytes(dogf.begin(), dogf.begin() + 60));
    f.push(f.bulk_in, Bytes(dogf.begin() + 60, dogf.end()));
    f.push_code(f.bulk_in, ANTZ_LIBUSB_TIMEOUT);
    f.xfers.clear();
    for (int i = 0; i < 4; ++i) {
        const auto rc = garmin_usb_session_poll(&s, 10);
        CHECK(rc == GARMIN_USB_POLL_OK || rc == GARMIN_USB_POLL_IDLE);
    }
    CHECK(log.dogs.size() == 1 && std::strcmp(log.dogs[0].name, "Kim") == 0);
    CHECK(log.dogs[0].status == GARMIN_USB_DOG_STATUS_MOVING);
    CHECK(f.xfers.size() == 4 && f.xfers[0].call == "interrupt_in" && f.xfers[1].call == "bulk_in" &&
          f.xfers[2].call == "bulk_in" && f.xfers[3].call == "bulk_in");
    // Idle on interrupt again.
    CHECK(garmin_usb_session_poll(&s, 10) == GARMIN_USB_POLL_IDLE);
    CHECK(f.xfers.back().call == "interrupt_in");

    // A stop is written on bulk out too.
    f.xfers.clear();
    CHECK(garmin_usb_session_stop(&s) == 0);
    Bytes stop(2, 0); put_u16(stop, 0, 50);
    CHECK(f.xfers.size() == 1 && f.xfers[0].data == frame(20, 10, stop));

    // A failing read ends the session as a transport error.
    f.push_code(f.int_in, ANTZ_LIBUSB_ERROR);
    CHECK(garmin_usb_session_poll(&s, 10) == GARMIN_USB_POLL_TRANSPORT_ERROR);
    return 0;
}

static int status_text() {
    for (int s = GARMIN_USB_LIBUSB_OK; s <= GARMIN_USB_LIBUSB_BAD_ARGUMENT; ++s) {
        const char* t = garmin_usb_libusb_status_text(static_cast<antz_garmin_usb_libusb_status_t>(s));
        CHECK(t != nullptr && t[0] != '\0');
    }
    CHECK(garmin_usb_libusb_status_text(static_cast<antz_garmin_usb_libusb_status_t>(99)) != nullptr);
    CHECK(garmin_usb_libusb_status_text(static_cast<antz_garmin_usb_libusb_status_t>(-1)) != nullptr);
    // The failures read differently from each other.
    CHECK(std::strcmp(garmin_usb_libusb_status_text(GARMIN_USB_LIBUSB_NO_DEVICE),
                      garmin_usb_libusb_status_text(GARMIN_USB_LIBUSB_MORE_THAN_ONE)) != 0);
    return 0;
}

int main(int argc, char** argv) {
    struct Group { const char* name; int (*fn)(); };
    const Group groups[] = {
        {"one_device", one_device}, {"device_count", device_count}, {"open_order", open_order},
        {"open_fails", open_fails_each_step}, {"kernel_driver", kernel_driver},
        {"close_order", close_order}, {"close_never_opened", close_never_opened},
        {"bad_arguments", bad_arguments}, {"write", write_packets}, {"write_failures", write_failures},
        {"read_interrupt", read_interrupt}, {"read_bulk", read_bulk},
        {"read_errors", read_timeout_and_errors}, {"end_to_end", end_to_end}, {"status_text", status_text},
    };
    for (const Group& g : groups) {
        if (argc > 1 && std::strcmp(argv[1], g.name) != 0) continue;
        if (g.fn() != 0) return 1;
    }
    std::printf("ok (%d checks)\n", checks);
    return 0;
}
