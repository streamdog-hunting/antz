#!/usr/bin/env python3
"""Writes the sanitised Garmin USB replay sets next to this script.

Run:  python3 -I generate_replays.py
Deterministic (fixed seed). Standard library only.

NOT RECORDINGS. No real capture is used. Every packet is built from the byte
layout in astro-usb docs/protocol.md. Coordinates are made up, in one fictional
area near 60.000 N, 10.000 E. Names are made up. The unit id is made up.

Format, one JSON object per line, as the astro-usb logger writes it:
  {"t": seconds since start, "type": layer, "pid": pid, "hex": payload hex}
A record of type 0, pid 6 (Session_Started) begins a new session.
"""
import json, os, random, struct

HERE = os.path.dirname(os.path.abspath(__file__))
UNIT = 1234567890            # made up
T0 = 1163246400              # garmin seconds, made up (2026-11-10 12:00 UTC)
ASTRO, ALPHA = 1339, 1562
BASE_LAT, BASE_LON = 60.000, 10.000


def semis(deg):
    return struct.pack('<i', round(deg * 2147483648.0 / 180.0))


def dog(lat, lon, t, status, flags, name, cid=(11, 0), colour=12, battery=40, gps=5, comm=5, b30=207, seed=0):
    p = bytearray(100)
    p[0:4] = semis(lat)
    p[4:8] = semis(lon)
    p[8:12] = struct.pack('<f', 300.0)
    p[12:16] = struct.pack('<I', t)
    p[16] = status
    p[17] = flags
    p[22], p[23] = cid
    p[24] = colour
    p[25] = battery
    p[26] = gps
    p[27] = comm
    p[30] = b30
    nb = name.encode()
    p[31:31 + len(nb)] = nb
    r = random.Random(seed)              # filler after the name: not a field
    for i in range(32 + len(nb), 100):
        p[i] = r.randrange(256)
    return bytes(p)


def session_started():
    return struct.pack('<I', UNIT)


def product(pid, ver, text):
    return struct.pack('<HH', pid, ver) + text.encode() + b'\0'


def pvt(lat, lon):
    d = bytearray(64)
    d[0:4] = struct.pack('<f', 360.0)
    d[4:8] = struct.pack('<f', 4.0)
    d[16:18] = struct.pack('<H', 3)
    d[26:34] = struct.pack('<d', lat * 3.141592653589793 / 180.0)
    d[34:42] = struct.pack('<d', lon * 3.141592653589793 / 180.0)
    d[54:58] = struct.pack('<f', 40.0)
    return bytes(d)


class Log:
    def __init__(self):
        self.lines = []

    def add(self, t, layer, pid, payload):
        self.lines.append(json.dumps({"t": round(t, 3), "type": layer, "pid": pid, "hex": payload.hex()}))

    def handshake(self, t, prod, ver, text):
        self.add(t, 0, 6, session_started())
        self.add(t + 0.1, 20, 255, product(prod, ver, text))
        self.add(t + 0.2, 20, 114, bytes(84))            # Satellite_Data, ignored
        self.add(t + 0.3, 20, 51, pvt(BASE_LAT, BASE_LON))

    def write(self, name):
        with open(os.path.join(HERE, name), 'w') as f:
            f.write("\n".join(self.lines) + "\n")


class Collars:
    """Two fictional collars on a hand-held; positions wander a little."""
    def __init__(self, seed, status=(2, 2), model=ASTRO):
        self.rng = random.Random(seed)
        self.model = model
        self.pos = [[BASE_LAT + 0.0010, BASE_LON + 0.0020], [BASE_LAT - 0.0015, BASE_LON + 0.0005]]
        self.names = ["Rex", "Luna"]
        self.status = list(status)
        self.cid = [(11, 0), (6, 0)] if model == ASTRO else [(1, 2), (6, 1)]
        self.colour = [12, 9]
        self.n = 0

    def packet(self, i, t, flags=0, freeze=False, **kw):
        if not freeze:
            self.pos[i][0] += self.rng.uniform(-0.0002, 0.0002)
            self.pos[i][1] += self.rng.uniform(-0.0002, 0.0002)
        self.n += 1
        args = dict(cid=self.cid[i], colour=self.colour[i], b30=(207, 209)[i], seed=self.n)
        args.update(kw)
        return dog(self.pos[i][0], self.pos[i][1], t, self.status[i], flags, self.names[i], **args)


def rounds(log, c, t_start, first, last, absent=(), loss=(), rename_from=None, cid_change=None):
    """Round r: collar 0 at T0+5r, collar 1 at T0+5r+2 (source time), logged
    2.5 s apart on the wall clock. absent: (collar, round) pairs not sent."""
    for r in range(first, last + 1):
        for i in (0, 1):
            if (i, r) in absent:
                continue
            if rename_from is not None and i == 0 and r >= rename_from:
                c.names[0] = "Rexi"
            kw = {}
            if cid_change and i == 0 and cid_change[0] <= r <= cid_change[1]:
                kw['cid'] = (12, 0)
            lost = (i, r) in loss
            flags = 0x20 if lost else 0
            kw['gps'] = 0 if lost else (5 if c.model == ASTRO else 3)
            pkt = c.packet(i, T0 + 5 * r + 2 * i, flags=flags, freeze=lost, **kw)
            log.add(t_start + 5 * r + 2.5 * i, 20, 3078, pkt)


def main():
    # 1. a normal session, two collars, six rounds
    L = Log(); L.handshake(0.0, ASTRO, 460, "Astro 320 Software Version 4.60")
    rounds(L, Collars(1), 1.0, 0, 5)
    L.write("replay_normal_astro.jsonl")

    # 2. reconnect: the same handshake again, same unit id and collars
    L = Log(); L.handshake(0.0, ASTRO, 460, "Astro 320 Software Version 4.60")
    c = Collars(2)
    rounds(L, c, 1.0, 0, 2)
    L.handshake(20.0, ASTRO, 460, "Astro 320 Software Version 4.60")
    rounds(L, c, 21.0 - 15, 3, 5)
    L.write("replay_reconnect.jsonl")

    # 3. handheld reboot: a gap, a new session, the product again, dogs resume
    L = Log(); L.handshake(0.0, ASTRO, 460, "Astro 320 Software Version 4.60")
    c = Collars(3)
    rounds(L, c, 1.0, 0, 2)
    L.handshake(95.0, ASTRO, 460, "Astro 320 Software Version 4.60")
    rounds(L, c, 96.0 - 5 * 8, 8, 10)         # source time keeps counting: rounds 8..10
    L.write("replay_reboot.jsonl")

    # 4. rename: collar 0 "Rex" becomes "Rexi" from round 3
    L = Log(); L.handshake(0.0, ASTRO, 460, "Astro 320 Software Version 4.60")
    rounds(L, Collars(4), 1.0, 0, 5, rename_from=3)
    L.write("replay_rename.jsonl")

    # 5. radio id on an Astro: 11 -> 12 in rounds 2..3 -> 11 again
    L = Log(); L.handshake(0.0, ASTRO, 460, "Astro 320 Software Version 4.60")
    rounds(L, Collars(5), 1.0, 0, 5, cid_change=(2, 3))
    L.write("replay_radio_id.jsonl")

    # 6. collar 1 powered off for rounds 2..4, on again at round 5
    L = Log(); L.handshake(0.0, ASTRO, 460, "Astro 320 Software Version 4.60")
    rounds(L, Collars(6), 1.0, 0, 6, absent={(1, r) for r in (2, 3, 4)})
    L.write("replay_power_cycle.jsonl")

    # 7. GPS loss on collar 0 in rounds 2..3: frozen coordinates, time moves on
    L = Log(); L.handshake(0.0, ASTRO, 460, "Astro 320 Software Version 4.60")
    rounds(L, Collars(7), 1.0, 0, 5, loss={(0, 2), (0, 3)})
    L.write("replay_gps_loss.jsonl")

    # 8. the same collars on an Alpha 100: statuses 33..36 cycling
    L = Log(); L.handshake(0.0, ALPHA, 860, "Alpha 100 EMEA/APAC Software Version 8.60")
    c = Collars(8, model=ALPHA)
    c.names = ["Rex", "Luna"]
    for r in range(6):
        c.status = [33 + (r % 4), 33 + ((r + 1) % 4)]
        rounds(L, c, 1.0, r, r)
    L.write("replay_alpha.jsonl")


if __name__ == '__main__':
    main()
