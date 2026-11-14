# Replay set (generated, not recorded)

These files are NOT recordings. No raw capture of a handheld is on the
machine they were made on (the real ones are private). `generate_replays.py`
builds every packet from the byte layout in astro-usb `docs/protocol.md`, with
made-up coordinates near 60.000 N, 10.000 E, made-up names (Rex, Luna, Rexi)
and a made-up unit id. Regenerate with `python3 -I generate_replays.py`; the
output is deterministic.

Format: one JSON object per line, as the astro-usb logger writes it:
`{"t": seconds, "type": layer, "pid": pid, "hex": payload hex}`. A record of
type 0, pid 6 (Session_Started) begins a new session.

Collar 0 (Rex) sends at source time T0+5r, collar 1 (Luna) at T0+5r+2, for round r.

| file | what it holds |
|---|---|
| replay_normal_astro | Astro 320, two collars, 6 rounds |
| replay_reconnect | the same handshake again after round 2, same unit id and collars |
| replay_reboot | a gap, a new session and product, rounds 8..10 resume |
| replay_rename | Rex becomes Rexi from round 3, nothing else changes |
| replay_radio_id | Rex's bytes 22..23 go 11 to 12 (rounds 2..3) and back |
| replay_power_cycle | Luna sends nothing in rounds 2..4, Rex unaffected |
| replay_gps_loss | Rex rounds 2..3: byte 17 bit 0x20, frozen position, time advances |
| replay_alpha | Alpha 100 (1562), statuses 33..36 cycling |
