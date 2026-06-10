# Servo Telemetry Capture — Design

Date: 2026-06-10
Branch: `servo-telemetry`

## Purpose

Replace the vendor's Windows scope tool with a measurable tuning workflow for
the A6-EC servo drives: capture per-DC-cycle (1 kHz) drive feedback plus the
host's commanded target into a crash-survivable file, and turn captures into
tuning metrics offline (following-error peak/RMS, overshoot, settling time,
torque saturation, resonance spectra for notch filters).

## Scope

Four deliverables:

1. **Drive PDO change** — TxPDO gains 6062h (position reference), via the
   variable mapping 1A00h.
2. **Capture engine** in `kalico-ethercat-rt` — SPSC ring filled by the RT
   loop, drained by a writer thread into a file under
   `~/printer_data/logs/servo_captures/`.
3. **Protocol + klippy surface** — `StartCapture`/`StopCapture` wire commands;
   `SERVO_CAPTURE_START`/`SERVO_CAPTURE_STOP` G-code commands.
4. **Offline analysis** — `scripts/servo_capture.py`.

Out of scope: live streaming/plotting, automatic gain tuning, capture of
Layer-4 feed-forward channels (the file format reserves room for them).

## 1. Drive: TxPDO via variable mapping 1A00h

The fixed TxPDO 1B01h (9 objects, 28 bytes) cannot be extended. The drive's
one variable TPDO, 1A00h, allows 10 objects / 40 bytes. We map the existing
nine objects **in the same order** plus 6062h appended last:

| # | object | size | field |
|---|--------|------|-------|
| 1 | 603Fh | u16 | error_code |
| 2 | 6041h | u16 | statusword |
| 3 | 6064h | i32 | position_actual |
| 4 | 6077h | i16 | torque_actual |
| 5 | 60F4h | i32 | following_error |
| 6 | 60B9h | u16 | tp_status |
| 7 | 60BAh | i32 | tp1_pos |
| 8 | 60BCh | i32 | tp2_pos |
| 9 | 60FDh | u32 | digital_inputs |
| 10 | 6062h | i32 | position_demand |

`in_t` in `bench/libecrt.c` gains a trailing `int32_t position_demand`; every
existing field keeps its offset. RxPDO stays fixed 1701h, untouched.

6062h is the drive's internally interpolated position reference in the same
reference units as 6064h/607Ah (60FCh is the encoder-unit twin via gear ratio
6091h), i.e. exactly the signal the drive's position loop chases — the right
reference for tuning. This exhausts the 10-object budget; velocity actual
(606Ch) does not fit and is instead derived offline by differentiating
position at 1 kHz.

### Bringup changes (`ec_rt_bringup`)

New SDO writes in the existing PRE-OP block (after `ec_config_init`, before
`ec_config_map`), required at every power-on because the drive does not
persist PDO mapping in EEPROM:

1. Write 0 to 1A00h:00 (clear mapping).
2. Write the ten mapping entries to 1A00h:01..0A (`603F0010h` …
   `60620020h`).
3. Write 10 to 1A00h:00.
4. Write 0 to 1C13h:00, write 0x1A00 to 1C13h:01, write 1 to 1C13h:00.

### Hardening

- After `ec_config_map`: assert `ec_slave[1].Ibytes == sizeof(in_t)` and
  `ec_slave[1].Obytes == sizeof(out_t)`; on mismatch fail bringup with a new
  distinct return code. (Today a mapping mismatch silently corrupts every
  field.)
- Replace the need for per-field FFI getters in the hot path with one
  snapshot call: `void ec_rt_get_telemetry(ec_telemetry_t *out)` copying the
  whole `in_t` once per cycle. Existing single-field getters stay for current
  callers.

### Risk

Whether this A6-EC firmware accepts the 1A00h remap exactly as documented
(SDO order quirks are common) is only verifiable on the bench. The `Ibytes`
assert turns a bad remap into a clean bringup error rather than data
corruption.

## 2. Endpoint: capture engine in `kalico-ethercat-rt`

New `capture` module owning:

- an **SPSC ring** of fixed-size records, capacity 4096 (≈ 4 s at 1 kHz);
  RT loop is the producer, writer thread the consumer;
- a **writer thread** (plain `std::thread`, not RT) that opens the capture
  file, writes the header, drains the ring, and fsyncs every second.

### Per-cycle record

While capture is active the RT loop pushes one record per DC cycle:

cycle-level fields:

| field | type | source |
|-------|------|--------|
| cycle_index | u64 | RT loop's DC cycle counter |
| flags | u8 | bit0 torque-enabled, bit1 motion-active (axis ring had a live piece this cycle) |

then one **drive block** per captured drive:

| field | type | source |
|-------|------|--------|
| target_counts | i32 | value the loop wrote to 607Ah this cycle |
| position_demand | i32 | 6062h |
| position_actual | i32 | 6064h |
| following_error | i32 | 60F4h |
| torque_actual | i16 | 6077h (per-mille of rated torque) |
| statusword | u16 | 6041h |
| error_code | u16 | 603Fh |

Multi-drive is structural from day one: the header declares the drive list and
the record carries one block per drive, all sampled in the same DC cycle —
time alignment by construction. Today the list has length 1 (single slave);
when CoreXY A/B share one bus/endpoint, both slaves' blocks land in the same
cycle-indexed record. Cross-coupling identification requires exactly this.

### Overflow is capture death, not sample loss

If the RT side finds the ring full, it sets a failed flag with the cycle index
where it died and stops pushing. The failure is reported in the
`StopCapture` response, and the writer renames the file to
`<name>.failed.scap` so the failure is visible on disk too (a footer would be
ambiguous with crash truncation; a rename is not). The analysis script
rejects `.failed.scap` files. A silently gappy capture poisons every FFT and fit
downstream. At ~30 bytes × 1 kHz the writer never legitimately falls behind,
so overflow means something is genuinely wrong — fail loudly.

### Error codes

`ERR_CAPTURE_ACTIVE` (-320), `ERR_CAPTURE_NOT_ACTIVE` (-321),
`ERR_CAPTURE_FILE` (-322), `ERR_CAPTURE_OVERFLOW` (-323),
`ERR_CAPTURE_BAD_ARG` (-324). All loud, all distinct. On any failure the
writer preserves the records-written count, so the stop response reports how
many records reached disk before the failure. Start validates `drive_name`
and `started_utc` for JSON safety (the header is emitted without an escaping
serializer) and returns `ERR_CAPTURE_BAD_ARG` before touching disk.

### Crash survivability

Records are fixed-size and the writer fsyncs periodically, so a capture
truncated by endpoint death is parseable up to the last flushed record. Files
live under `~/printer_data/logs/servo_captures/` (persistent storage), path
supplied by the host in `StartCapture`.

## 3. File format

Line 1: JSON header, newline-terminated. Then raw little-endian fixed-size
records to EOF.

Header fields:

- `version` (integer, starts at 1)
- `cycle_ns`
- `record_size` (bytes)
- `started_utc` (wall clock, supplied by host in `StartCapture`),
  `started_mono_ns` (endpoint monotonic)
- `drives`: `[{name, counts_per_mm}]`
- `channels`: `[{name, dtype, offset}]` — full record layout, including the
  cycle-level fields

The analysis script derives everything from the header: record layout from
`channels`, record count as `(filesize − header_len) / record_size`. It never
hardcodes offsets. Layer 4's velocity/torque feed-forward channels become a
version bump plus new channel descriptors; old captures keep parsing.

## 4. Protocol and klippy surface

### Wire commands (alongside `SetTorque`/`PushPieces` in `wire.rs`)

- `StartCapture { path, started_utc, drive_name } → StartCaptureResponse { result }`
- `StopCapture → StopCaptureResponse { result, samples, overflow_cycle }`

Error codes follow the existing negative-code convention: the `-32x` capture
family listed in §2. The wire carries a single `drive_name`; the file format
(header `drives` array) is multi-drive ready, and the wire message grows to a
drive list when a multi-slave endpoint exists.

Adding message kinds changes `SCHEMA_HASH` (generated from
`rust/kalico-protocol/schema_def.rs`), so rollout requires reflashing BOTH
MCUs together with the host rebuild — a stale MCU fails the schema handshake.

### Motion-bridge (PyO3)

`start_servo_capture(handle, path, started_utc, drive_name)` /
`stop_servo_capture(handle)` encode the wire messages over the existing
Unix-socket session, mirroring `set_torque()`.

### Klippy G-code commands

`SERVO_CAPTURE_START [SERVO=<name>] [NAME=<tag>]` and `SERVO_CAPTURE_STOP` —
two commands, not one with a subcommand, because klippy extended G-code
parameters must be `KEY=VALUE` and a bare `START` token does not parse. The
command object lives in `klippy/extras/servo_capture.py`, auto-loaded by
`ethercat_node` via `printer.load_object` (the endpoint handle stays in
`ethercat_node.py`; the capture commands look it up).

- `SERVO=` defaults to the only configured servo and is required with
  multiple `[ethercat_node]` sections. Comma lists (`SERVO=a,b`) error as
  not-implemented: multi-servo capture requires all drives on one endpoint.
- `NAME=` tags the filename:
  `servo_captures/<tag>_<YYYYMMDD_HHMMSS>.scap` (default tag `capture`),
  restricted to `[A-Za-z0-9_-]+`.
- STOP prints to the console: file path, sample count, duration, and overflow
  status (a failed capture prints as an error).

### Usage rhythm (documented, plus reference macro)

`SERVO_CAPTURE START` → test moves → `M400` → `SERVO_CAPTURE STOP`.
START/STOP execute when the G-code runs but queued moves execute later;
without `M400` the STOP lands while motion is still playing out. One line in
the docs and a reference tuning macro kill the footgun.

## 5. Offline analysis: `scripts/servo_capture.py`

numpy/matplotlib, following `graph_accelerometer.py` conventions. Refuses
captures marked failed/overflowed. Segments the capture into move/dwell
phases from the motion-active flag, then reports per move and overall:

- **Following error peak and RMS** — drive-reported 60F4h, cross-checked
  against recomputed `target_counts − position_actual`; a systematic mismatch
  between the two is itself a finding (exposes the drive's interpolation /
  reporting delay).
- **Overshoot and settling time** — error excursion after motion-active
  falls; time until error stays within `--settle-band` (counts). Each move's
  post window is clamped at the next move's start.
- **Torque saturation** — % of samples at/above `--torque-limit` (per-mille
  of rated).
- **`--fft`** — Welch PSD of following error over moving segments, top peaks
  annotated in Hz: the notch-filter shopping list.
- **`--plot`** — time-series dashboard (demand vs actual, error, torque)
  with move segments shaded.

Metrics print in both counts and mm (via `counts_per_mm` from the header).
All timing is fs-aware — the sample rate comes from the header's `cycle_ns`,
never an assumed 1 kHz.

## 6. Testing

- **Rust unit tests** (separate files from the tested code): SPSC ring
  semantics including the overflow-kills-capture path; writer header/record
  serialization; wire codec round-trips for the new messages.
- **Rust integration test** `capture_lifecycle.rs` (alongside
  `torque_lifecycle.rs`): start → records flow → stop with correct counts;
  double-start rejection; stop-without-start rejection; forced overflow
  reports failure.
- **Python**: analysis script tested against a synthesized capture file
  (known damped sine in the error channel → assert detected resonance peak
  frequency, settling time, RMS within tolerance).
- **Bench**: the 1A00h remap is verified on the Neptune drive; the `Ibytes`
  assert makes a bad remap fail at bringup.

## 7. Documentation

`docs/kalico-rewrite/servo-telemetry-capture.md`: command reference, file
format spec, the M400 footgun, a reference tuning macro, analysis script
usage. No code comments.
