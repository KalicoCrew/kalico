# Servo telemetry capture

> User and format reference for the 1 kHz servo capture pipeline. Companion to
> the design spec
> ([`2026-06-10-servo-telemetry-capture-design.md`](../superpowers/specs/2026-06-10-servo-telemetry-capture-design.md))
> and to [`ethercat-bench-bringup.md`](ethercat-bench-bringup.md) (getting the
> drive up in the first place).

## What it is

Every DC cycle (1 kHz) while a capture is active, the EtherCAT endpoint records
the drive's feedback (actual position, internally interpolated demand,
following error, torque, status/error words) together with the position the
host commanded that same cycle, into a crash-survivable file on the Pi. This
replaces the vendor's Windows scope tool: instead of eyeballing a live trace
over a USB cable, you run a scripted move set, get a file, and get numbers —
following-error peak/RMS, overshoot, settling time, torque saturation,
resonance spectra. Servo tuning becomes measurable instead of vibes-based.

## Commands

```
SERVO_CAPTURE_START [SERVO=<name>] [NAME=<tag>]
SERVO_CAPTURE_STOP
```

- Files land in `~/printer_data/logs/servo_captures/<tag>_<YYYYMMDD_HHMMSS>.scap`.
  The default tag is `capture`; `NAME=` must match `[A-Za-z0-9_-]+` — anything
  else is rejected before the capture starts.
- `SERVO=` names the `[ethercat_node]` to capture. With exactly one node
  configured it can be omitted; with several it is required. Comma lists
  (`SERVO=a,b`) are rejected: multi-servo capture requires all drives on one
  endpoint and is not implemented yet (the header's `drives` array anticipates
  it, but adding a second drive block grows the record — a wire change plus a
  format version bump).
- One capture at a time. A second `SERVO_CAPTURE_START` errors with the path
  of the active capture; `SERVO_CAPTURE_STOP` without one errors too.
- A healthy STOP prints the file path, sample count, and duration. If the
  capture failed (ring overflow or a writer error), the endpoint renames the
  file to `<name>.failed.scap` and STOP raises an error naming the endpoint
  code and the partial file. The analysis script refuses `.failed.scap` files
  — a silently gappy capture would poison every metric.

## The M400 footgun

`SERVO_CAPTURE_START` and `SERVO_CAPTURE_STOP` execute the instant the G-code
parser reaches them. Moves do not — they queue and play out later. Without a
barrier, STOP lands while the axis is still moving and the capture cuts off
mid-trajectory. **Always `M400` before `SERVO_CAPTURE_STOP`.** Reference
macro:

```ini
[gcode_macro SERVO_TUNE_X]
gcode:
    SERVO_CAPTURE_START NAME=xtune
    G91
    G1 X60 F12000
    G1 X-60 F12000
    G1 X20 F3000
    G1 X-20 F3000
    G90
    M400
    SERVO_CAPTURE_STOP
```

## Analysis

```
python3 scripts/servo_capture.py <file> [--fft] [--plot]
                                 [--settle-band N] [--torque-limit N]
```

The script segments the capture into move/dwell phases from the motion-active
flag and reports, per move and overall:

- **Following error peak and RMS** (counts and mm) — the headline number for
  position-loop gains: stiffer gains pull both down, until they buy you
  oscillation instead.
- **Resonance peaks** (`--fft`) — Welch PSD of the following error over the
  moving segments, top peaks in Hz. These are the notch-filter shopping list.
- **Overshoot and settling time** — error excursion after each move ends, and
  how long until the error stays inside `--settle-band` (counts, default 50)
  for 50 ms. The post-move window for one move is clamped at the next move's
  start, so back-to-back moves don't bleed into each other's settling numbers.
  Persistent overshoot / long settling means too little damping (or too much
  gain).
- **Torque saturation** — % of samples at/above `--torque-limit` (per-mille of
  rated, default 900). A drive at the torque ceiling isn't tracking, it's
  coasting; saturation tells you whether the feedrate/accel you're asking for
  has headroom at all.
- **Drive-vs-recomputed cross-check** — the drive's own 60F4h following error
  against `target_counts − position_actual` recomputed from the same record. A
  systematic delta is itself a finding: it exposes the drive's internal
  interpolation/reporting delay between accepting 607Ah and acting on it.

`--plot` opens a three-panel dashboard (demand vs actual vs host target,
following error, torque) with the moving segments shaded. All timing is
derived from `cycle_ns` in the file header — the script does not assume 1 kHz.

In plain English: the following error is the gap between where you told the
motor to be and where it actually is, sampled a thousand times a second. The
FFT of that gap is the machine telling you which frequencies it cannot follow
— those are the notch candidates. Overshoot and settling are how badly it
slings past the target and how long it wobbles before parking. Torque
saturation is the motor saying "I'm already pushing as hard as I can" — no
gain tuning fixes that, only asking for less.

## File format (`.scap`, version 1)

Line 1 is a newline-terminated JSON header; everything after it is raw
little-endian fixed-size records to EOF.

Header fields:

| field | meaning |
|-------|---------|
| `version` | format version, currently 1 |
| `cycle_ns` | DC cycle period in ns (sample interval) |
| `record_size` | bytes per record (31 in v1) |
| `started_utc` | wall-clock start, supplied by the host |
| `started_mono_ns` | endpoint monotonic clock at start |
| `drives` | `[{name, counts_per_mm}]` — one entry per captured drive |
| `channels` | `[{name, dtype, offset}]` — the full record layout |

v1 record layout (31 bytes):

| channel | dtype | offset |
|---------|-------|--------|
| cycle_index | u64 | 0 |
| flags | u8 | 8 |
| target_counts | i32 | 9 |
| position_demand | i32 | 13 |
| position_actual | i32 | 17 |
| following_error | i32 | 21 |
| torque_actual | i16 | 25 |
| statusword | u16 | 27 |
| error_code | u16 | 29 |

`flags` bit 0 is torque-enabled, bit 1 is motion-active. `torque_actual` is
per-mille of rated torque (6077h); the position channels are encoder counts.

Records are fixed-size and the writer fsyncs every second, so a file truncated
by endpoint death is valid up to the last whole record — the analysis script
drops a trailing partial record and parses the rest. That is the point: a
capture that survives the crash it was recording.

Future versions add channels by bumping `version` and extending the `channels`
descriptor; old files keep parsing. Any tool reading `.scap` files must derive
the record layout from the header, never hardcode offsets.

## Drive-side mapping

The A6-EC's fixed TxPDO 1B01h cannot carry 6062h, so bringup rewrites the
drive's one variable TxPDO, 1A00h (ceiling 10 objects / 40 bytes; we use all
10 objects, 32 bytes): the nine objects of 1B01h in the same order, plus 6062h
appended last. The drive does not persist PDO mapping across power cycles, so
`ec_rt_bringup` performs the remap over SDO at every bringup, in the manual's
documented order — configure the mapping group (1C13h) first, then the mapping
objects (1A00h).

6062h is the drive's internally interpolated position demand, in the same
reference units as 6064h (position actual) and 607Ah (host target) — exactly
the signal the drive's position loop chases, which makes it the right
reference trace for tuning.

Bringup fails loudly rather than running with a corrupt layout:

- **rc=-6** — an SDO write in the remap was refused; stderr names the failing
  object (`ec_rt: remap SDO write XXXXh:XXh failed`).
- **rc=-7** — after mapping, the drive's reported PDO sizes disagree with the
  host's structs; stderr prints both. Without this check a bad remap would
  silently corrupt every telemetry field.

## Endpoint capture error codes

| code | meaning |
|------|---------|
| -320 | capture already active |
| -321 | no capture active |
| -322 | file/writer error (open, write, fsync, or rename failed) |
| -323 | ring overflow — the RT loop outran the writer; capture is dead from that cycle |
| -324 | bad argument (drive name / timestamp not JSON-safe) |

Any nonzero stop result means the file was renamed `.failed.scap`; the sample
count still reports how many records made it to disk before the failure.
