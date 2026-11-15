# Garmin USB replays from astro-usb

Copied from `streamdog-hunting/astro-usb`, pull request 2
(`test(usb): add real Garmin replays with synthetic positions`), branch
`test/synthetic-garmin-usb-replays` at commit `0d4adf2a`, directory `replays/`.
Arnfinn Grambo made them from his private captures of an Astro 320 and an
Alpha 100. The tests that read them are `tests/astro_usb_replays_test.cpp`.
The script that makes them from the private captures stays in astro-usb
(`replays/build_replays.py`), because it needs files that are not published.

One change was made after copying: the real dogs' names were replaced by made-up
names of the same length, wherever their bytes occur (the name field and any text
left in the packet after it), so packet sizes and layouts do not change. The
Alpha's default labels "Hund 1" and "Hund 6" stay. The rest is as
he made it. What follows is his README for them, with the names in it changed the same way.

---

# Garmin USB replay fixtures

These fixtures accompany astro-usb issue #1. They retain real packet order,
timing, model metadata, dog names, flags and GPS/source-time fields. All known
latitude and longitude fields are replaced with deterministic synthetic points
in the South Pacific. D800 coordinates are in radians and Dog_Data coordinates
are in signed semicircles. Repeated original points remain repeated within
each session. USB unit IDs are fictional and consistent per handheld.

USB records contain `t` (elapsed seconds), `type`, `pid` and `hex`.
Only Session_Started, Product_Data, Pvt_Data and Dog_Data records are included.
Satellite, protocol-array and extended-product records are omitted. Source
timestamps inside payloads stay as recorded. Each file is a separate captured
session unless explicitly described as part of the simultaneous USB/ANT pair.

| File (all JSONL) | What the capture shows |
| --- | --- |
| `astro-normal` | 90-second Astro 320 session with two collars. |
| `astro-rename-before` | First 120 seconds of an earlier Astro session with Ragnar. |
| `astro-rename-after` | 60-second session after the rename to RagnarXX. |
| `astro-id-before` | Astro baseline with Ragnar at configured radio ID 11. |
| `astro-id-after` | New session with Ragnar at configured radio ID 12. |
| `astro-collar-off` | One collar stops reporting while the other continues. |
| `astro-gps-loss` | Ragnar keeps reporting a stale point with byte 17 bit 0x20 set. |
| `astro-gps-recovery` | Later session after GPS recovered. |
| `alpha-normal` | 90-second Alpha 100 session with two collars. |
| `alpha-after-reboot` | First Alpha session following reboot, PVT without dog packets. |
| `alpha-after-fix` | Later Alpha session with 3D fix and both collars, capped at 120 seconds. |
| `alpha-rename` | Alpha session after Hund 1 was renamed Ragnar. |
| `alpha-collar-off` | One collar stops reporting while the other continues. |
| `alpha-gps-loss` | Alpha session with Ragnar's GPS-loss flag set. |
| `alpha-gps-recovery` | Later Alpha session with the flag cleared. |
| `paired-astro-usb` | Astro USB side of the simultaneous capture, with one fictional fixed point per collar. |
| `paired-astro-ant` | Matching ANT side. Records contain `t`, `mid` and `hex`, using the same time origin as USB, with zero at the earliest ANT record. Location pages 1 and 2 carry the same fictional points as USB. The ANT device number is replaced consistently with the fictional USB unit ID's low 16 bits. |

Rename, ID change, reboot and GPS recovery are checkpoints from separate
sessions. There is no continuous cable-unplug/replug recording or captured
transition back on in the collar-off recording. The ANT stream contains
location pages 1 and 2, with no Common pages 80/81. No Alpha 10 recording
is available in this set.

Run `python replays/build_replays.py PATH_TO_PRIVATE_CAPTURES --check` to
verify exact regeneration. Keep the private captures outside this directory.
