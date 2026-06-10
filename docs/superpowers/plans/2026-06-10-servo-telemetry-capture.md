# Servo Telemetry Capture Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Per-DC-cycle (1 kHz) servo telemetry capture — drive feedback + commanded target — written to crash-survivable files under `~/printer_data/logs/servo_captures/`, started/stopped from G-code, analyzed offline into tuning metrics.

**Architecture:** The A6-EC drive's TxPDO is remapped from fixed 1B01h to variable 1A00h to add 6062h (position demand). The `kalico-ethercat-rt` endpoint gains a capture module (bounded queue filled by the RT loop, drained by a writer thread). Start/stop ride the existing Unix-socket protocol (`kalico-protocol` messages → `motion-bridge` PyO3 → klippy G-code). A numpy analysis script turns captures into metrics.

**Tech Stack:** C (SOEM/libecrt), Rust (kalico-protocol, kalico-ethercat-rt, motion-bridge), Python (klippy extras, numpy/matplotlib analysis).

**Spec:** `docs/superpowers/specs/2026-06-10-servo-telemetry-capture-design.md`

**Build/test commands:**
- Rust: `cargo nextest run -p <crate>` from `rust/` (never `cargo test`)
- Rust hw typecheck (no SOEM locally, so check only): `cargo check -p kalico-ethercat-rt --features hw`
- Python: `python -m pytest test/<file> -v` from repo root
- C (`bench/libecrt.c`) only compiles on the Pi (needs SOEM). No local build; bench validation is Task 14.

**Two facts discovered during planning that the spec text doesn't reflect (Task 12 aligns the spec):**
1. Klippy extended G-code params must be `KEY=VALUE`; a bare `START` word raises "Malformed command" (`klippy/gcode.py:377-389`). Commands are therefore `SERVO_CAPTURE_START` / `SERVO_CAPTURE_STOP`.
2. Adding message kinds to `rust/kalico-protocol/schema_def.rs` changes `SCHEMA_HASH`, which the host↔MCU identify path enforces (`rust/kalico-host-rt/src/host_io/kalico_native.rs:191`). After this lands, BOTH Trident MCUs (H7 + F446) must be reflashed together with the host rebuild. Wire commands here are host↔endpoint only, but the hash covers the whole schema table.

---

### Task 1: kalico-protocol — length-prefixed string codec helpers

**Files:**
- Modify: `rust/kalico-protocol/src/codec.rs`
- Test: `rust/kalico-protocol/src/messages/tests.rs` (round-trip tests land in Task 2; this task adds the helpers plus direct helper tests)

- [ ] **Step 1: Write failing tests**

Append to `rust/kalico-protocol/src/messages/tests.rs` (this file already exists; add at the end):

```rust
#[test]
fn put_get_str_round_trip() {
    use crate::codec::{get_str, put_str, Cursor};
    let mut buf = Vec::new();
    put_str(&mut buf, "servo_captures/x_20260610.scap");
    put_str(&mut buf, "");
    let mut c = Cursor::new(&buf);
    assert_eq!(get_str(&mut c).unwrap(), "servo_captures/x_20260610.scap");
    assert_eq!(get_str(&mut c).unwrap(), "");
}

#[test]
fn get_str_rejects_truncated_buffer() {
    use crate::codec::{get_str, Cursor};
    // length prefix claims 10 bytes, only 2 present
    let buf = [10u8, 0, b'a', b'b'];
    let mut c = Cursor::new(&buf);
    assert!(get_str(&mut c).is_err());
}

#[test]
fn get_str_rejects_invalid_utf8() {
    use crate::codec::{get_str, Cursor};
    let buf = [2u8, 0, 0xff, 0xfe];
    let mut c = Cursor::new(&buf);
    assert!(get_str(&mut c).is_err());
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run from `rust/`: `cargo nextest run -p kalico-protocol -E 'test(str)'`
Expected: compile error — `put_str`/`get_str` not found.

- [ ] **Step 3: Implement helpers**

In `rust/kalico-protocol/src/codec.rs`: add a `BadUtf8` variant to `DecodeError` (look at the existing enum at line 18 — variants include `UnexpectedEof`, `ArrayLengthExceedsBuffer`, `TrailingBytes`; follow its style). Then add next to the other `put_*`/`get_*` helpers:

```rust
/// u16-LE length prefix + UTF-8 bytes.
pub fn put_str(out: &mut Vec<u8>, s: &str) {
    let bytes = s.as_bytes();
    assert!(
        bytes.len() <= u16::MAX as usize,
        "wire string exceeds u16 length prefix"
    );
    put_u16(out, bytes.len() as u16);
    out.extend_from_slice(bytes);
}

pub fn get_str(c: &mut Cursor<'_>) -> Result<String, DecodeError> {
    let len = get_u16(c)? as usize;
    let mut bytes = vec![0u8; len];
    for b in &mut bytes {
        *b = get_u8(c)?;
    }
    String::from_utf8(bytes).map_err(|_| DecodeError::BadUtf8)
}
```

(If `Cursor` exposes a bulk-slice read used by `PushPieces` decode — see `messages.rs:183` which reads byte-by-byte — match whichever idiom `PushPieces` uses. Byte-by-byte via `get_u8` is the existing pattern.)

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo nextest run -p kalico-protocol -E 'test(str)'`
Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-protocol/src/codec.rs rust/kalico-protocol/src/messages/tests.rs
git commit -m "feat(protocol): length-prefixed string codec helpers"
```

---

### Task 2: kalico-protocol — StartCapture/StopCapture messages

**Files:**
- Modify: `rust/kalico-protocol/src/messages.rs`
- Modify: `rust/kalico-protocol/schema_def.rs`
- Test: `rust/kalico-protocol/src/messages/tests.rs`

- [ ] **Step 1: Write failing round-trip tests**

Append to `rust/kalico-protocol/src/messages/tests.rs`:

```rust
#[test]
fn start_capture_round_trip() {
    use crate::messages::StartCapture;
    let msg = StartCapture {
        path: "/home/pi/printer_data/logs/servo_captures/t.scap".into(),
        started_utc: "2026-06-10T12:00:00Z".into(),
        drive_name: "x".into(),
    };
    let buf = msg.encoded_to_vec();
    assert_eq!(StartCapture::decode(&buf).unwrap(), msg);
}

#[test]
fn stop_capture_response_round_trip() {
    use crate::messages::StopCaptureResponse;
    let msg = StopCaptureResponse {
        result: -323,
        samples: 12_345,
        overflow_cycle: u64::MAX,
    };
    let buf = msg.encoded_to_vec();
    assert_eq!(StopCaptureResponse::decode(&buf).unwrap(), msg);
}

#[test]
fn capture_message_kinds_round_trip_u16() {
    use crate::messages::MessageKind;
    for (kind, raw) in [
        (MessageKind::StartCapture, 0x0074u16),
        (MessageKind::StartCaptureResponse, 0x0075),
        (MessageKind::StopCapture, 0x0076),
        (MessageKind::StopCaptureResponse, 0x0077),
    ] {
        assert_eq!(kind.as_u16(), raw);
        assert_eq!(MessageKind::from_u16(raw), Some(kind));
    }
}
```

- [ ] **Step 2: Run to verify failure**

Run: `cargo nextest run -p kalico-protocol -E 'test(capture)'`
Expected: compile error — types not defined.

- [ ] **Step 3: Implement messages**

In `rust/kalico-protocol/src/messages.rs`:

Add to the `MessageKind` enum (after `Stop`/`StopResponse` = 0x0072/0x0073):

```rust
    StartCapture = 0x0074,
    StartCaptureResponse = 0x0075,
    StopCapture = 0x0076,
    StopCaptureResponse = 0x0077,
```

Add matching arms to `MessageKind::from_u16`:

```rust
            0x0074 => Self::StartCapture,
            0x0075 => Self::StartCaptureResponse,
            0x0076 => Self::StopCapture,
            0x0077 => Self::StopCaptureResponse,
```

Add structs + codecs near `SetTorque` (import `get_str`/`put_str` in the existing `use crate::codec::{...}` list at the top):

```rust
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StartCapture {
    pub path: String,
    pub started_utc: String,
    pub drive_name: String,
}

impl Encode for StartCapture {
    fn encode(&self, out: &mut Vec<u8>) {
        put_str(out, &self.path);
        put_str(out, &self.started_utc);
        put_str(out, &self.drive_name);
    }
}

impl Decode for StartCapture {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            path: get_str(c)?,
            started_utc: get_str(c)?,
            drive_name: get_str(c)?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StartCaptureResponse {
    pub result: i32,
}

impl Encode for StartCaptureResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
    }
}

impl Decode for StartCaptureResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StopCapture;

impl Encode for StopCapture {
    fn encode(&self, _out: &mut Vec<u8>) {}
}

impl Decode for StopCapture {
    fn decode_from(_c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self)
    }
}

/// `overflow_cycle == u64::MAX` means no overflow occurred.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StopCaptureResponse {
    pub result: i32,
    pub samples: u64,
    pub overflow_cycle: u64,
}

impl Encode for StopCaptureResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
        put_u64(out, self.samples);
        put_u64(out, self.overflow_cycle);
    }
}

impl Decode for StopCaptureResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
            samples: get_u64(c)?,
            overflow_cycle: get_u64(c)?,
        })
    }
}
```

In `rust/kalico-protocol/schema_def.rs`, append to `SCHEMA_MESSAGES` (after the 0x0073 entry, matching the existing style):

```rust
    SchemaMessage {
        type_tag: 0x0074,
        name: "StartCapture",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "path", ty: "string" },
            SchemaField { name: "started_utc", ty: "string" },
            SchemaField { name: "drive_name", ty: "string" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0075,
        name: "StartCaptureResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0076,
        name: "StopCapture",
        version: 1,
        channel: "control",
        fields: &[],
    },
    SchemaMessage {
        type_tag: 0x0077,
        name: "StopCaptureResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
            SchemaField { name: "samples", ty: "u64" },
            SchemaField { name: "overflow_cycle", ty: "u64" },
        ],
    },
```

- [ ] **Step 4: Run the whole crate's tests**

Run: `cargo nextest run -p kalico-protocol`
Expected: all pass (including any schema-table validation tests the crate has).

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-protocol/src/messages.rs rust/kalico-protocol/schema_def.rs rust/kalico-protocol/src/messages/tests.rs
git commit -m "feat(protocol): StartCapture/StopCapture wire messages"
```

---

### Task 3: libecrt — 1A00h TxPDO remap, position_demand, telemetry snapshot, size asserts

**Files:**
- Modify: `bench/libecrt.h`
- Modify: `bench/libecrt.c`

No local compile (SOEM lives on the Pi); this task is code-complete + careful read-through. Bench validation is Task 14.

- [ ] **Step 1: Update `bench/libecrt.h`**

Replace the bringup doc comment and add the telemetry type + accessors:

```c
/* go_realtime + ec_init + TxPDO remap (variable 1A00h) + CSP/DC config + map
 * + SAFE-OP + DC align + OP, then parks at CiA402 Ready-to-Switch-On (no
 * torque). 0 on success; -1 ec_init, -2 no slaves, -3 SAFE-OP, -4 OP,
 * -5 park timeout, -6 TxPDO remap SDO write failed, -7 mapped PDO sizes
 * disagree with out_t/in_t. */
int  ec_rt_bringup(const char *ifname, int64_t cycle_ns, int rt_cpu, int rt_prio);
```

After the existing getters, add:

```c
/* One-shot snapshot of the full TxPDO feedback plus the staged commanded
 * target (out_t), so a 1 kHz capture costs one FFI hop per cycle. */
typedef struct {
    uint16_t error_code;
    uint16_t statusword;
    int32_t  position_actual;
    int16_t  torque_actual;
    int32_t  following_error;
    int32_t  position_demand;
    int32_t  target_position;
} ec_telemetry_t;

void ec_rt_get_telemetry(ec_telemetry_t *out);
```

- [ ] **Step 2: Update `bench/libecrt.c`**

(a) Extend the layout comment block (lines 10-29) — the TxPDO is now **variable 1A00h (32 bytes)**, same nine objects as the old fixed 1B01h in the same order, plus `position_demand 6062 int32` appended; note the mapping is written via SDO at every bringup because the drive does not persist it. Update `in_t`:

```c
typedef struct {
    uint16_t error_code;
    uint16_t statusword;
    int32_t  position_actual;
    int16_t  torque_actual;
    int32_t  following_error;
    uint16_t tp_status;
    int32_t  tp1_pos;
    int32_t  tp2_pos;
    uint32_t digital_inputs;
    int32_t  position_demand;
} in_t;
```

(b) Add the remap function above `ec_rt_bringup`:

```c
/* The drive's fixed TxPDO 1B01h cannot carry 6062h; the variable TPDO 1A00h
 * (max 10 objects / 40 bytes) can. Mapping is RAM-only on the drive, so it
 * must be rewritten in PRE-OP at every bringup. Entry format per CoE:
 * index<<16 | subindex<<8 | bit-length. */
static int map_tx_pdo_1a00(void) {
    static const uint32_t entries[10] = {
        0x603F0010, /* error_code       u16 */
        0x60410010, /* statusword       u16 */
        0x60640020, /* position_actual  i32 */
        0x60770010, /* torque_actual    i16 */
        0x60F40020, /* following_error  i32 */
        0x60B90010, /* tp_status        u16 */
        0x60BA0020, /* tp1_pos          i32 */
        0x60BC0020, /* tp2_pos          i32 */
        0x60FD0020, /* digital_inputs   u32 */
        0x60620020, /* position_demand  i32 */
    };
    uint8_t  zero8  = 0;
    uint8_t  count  = 10;
    uint16_t assign = 0x1A00;
    uint8_t  one    = 1;
    if (ec_SDOwrite(1, 0x1A00, 0x00, FALSE, sizeof(zero8), &zero8, EC_TIMEOUTRXM) <= 0) return -1;
    for (int i = 0; i < 10; i++) {
        uint32_t v = entries[i];
        if (ec_SDOwrite(1, 0x1A00, (uint8_t)(i + 1), FALSE, sizeof(v), &v, EC_TIMEOUTRXM) <= 0) return -1;
    }
    if (ec_SDOwrite(1, 0x1A00, 0x00, FALSE, sizeof(count),  &count,  EC_TIMEOUTRXM) <= 0) return -1;
    if (ec_SDOwrite(1, 0x1C13, 0x00, FALSE, sizeof(zero8),  &zero8,  EC_TIMEOUTRXM) <= 0) return -1;
    if (ec_SDOwrite(1, 0x1C13, 0x01, FALSE, sizeof(assign), &assign, EC_TIMEOUTRXM) <= 0) return -1;
    if (ec_SDOwrite(1, 0x1C13, 0x00, FALSE, sizeof(one),    &one,    EC_TIMEOUTRXM) <= 0) return -1;
    return 0;
}
```

(c) In `ec_rt_bringup`, right after the `ec_config_init` check (line 111) and BEFORE the existing 0x6060/0x1C32 SDO block (PDO remap is only legal in PRE-OP):

```c
    if (map_tx_pdo_1a00() != 0) { ec_close(); return -6; }
```

(d) After `ec_config_map(&IOmap);` and before the SAFE-OP statecheck:

```c
    if (ec_slave[1].Obytes != sizeof(out_t) || ec_slave[1].Ibytes != sizeof(in_t)) {
        fprintf(stderr,
                "ec_rt: PDO size mismatch — mapped out=%u in=%u, expected out=%zu in=%zu\n",
                (unsigned)ec_slave[1].Obytes, (unsigned)ec_slave[1].Ibytes,
                sizeof(out_t), sizeof(in_t));
        ec_close();
        return -7;
    }
```

(e) Add the snapshot accessor next to the existing getters:

```c
void ec_rt_get_telemetry(ec_telemetry_t *out) {
    out->error_code      = g_in->error_code;
    out->statusword      = g_in->statusword;
    out->position_actual = g_in->position_actual;
    out->torque_actual   = g_in->torque_actual;
    out->following_error = g_in->following_error;
    out->position_demand = g_in->position_demand;
    out->target_position = g_out->target_position;
}
```

- [ ] **Step 3: Read-through check**

Verify: `in_t` is inside the `#pragma pack(push, 1)` block (it is — the new field must stay there); entry order matches `in_t` field order; 32 bytes total (2+2+4+2+4+2+4+4+4+4). `ec_telemetry_t` is NOT packed — it's host-side only, and the Rust mirror in Task 4 must match its natural layout.

- [ ] **Step 4: Commit**

```bash
git add bench/libecrt.h bench/libecrt.c
git commit -m "feat(ecrt): variable 1A00h TxPDO with position demand + telemetry snapshot"
```

---

### Task 4: kalico-ethercat-rt — FFI declaration for the telemetry snapshot

**Files:**
- Modify: `rust/kalico-ethercat-rt/src/ffi.rs`

- [ ] **Step 1: Add the mirror struct and extern**

In `rust/kalico-ethercat-rt/src/ffi.rs`, above the `extern "C"` block:

```rust
/// Mirror of `ec_telemetry_t` in bench/libecrt.h — natural (unpacked) C layout.
#[repr(C)]
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct EcTelemetry {
    pub error_code: u16,
    pub statusword: u16,
    pub position_actual: i32,
    pub torque_actual: i16,
    pub following_error: i32,
    pub position_demand: i32,
    pub target_position: i32,
}
```

Inside the `extern "C"` block:

```rust
    pub fn ec_rt_get_telemetry(out: *mut EcTelemetry);
```

- [ ] **Step 2: Typecheck the hw configuration**

Run from `rust/`: `cargo check -p kalico-ethercat-rt --features hw`
Expected: clean (check doesn't link SOEM, so this works without the Pi).

- [ ] **Step 3: Commit**

```bash
git add rust/kalico-ethercat-rt/src/ffi.rs
git commit -m "feat(ec-rt): EcTelemetry FFI snapshot binding"
```

---

### Task 5: kalico-ethercat-rt — capture module (record codec, header, ring + writer)

**Files:**
- Create: `rust/kalico-ethercat-rt/src/capture.rs`
- Create: `rust/kalico-ethercat-rt/src/capture/tests.rs`
- Modify: `rust/kalico-ethercat-rt/src/lib.rs` (add `pub mod capture;` next to the other ungated modules — capture has no FFI dependency)

- [ ] **Step 1: Write failing tests**

Create `rust/kalico-ethercat-rt/src/capture/tests.rs`:

```rust
use std::path::PathBuf;
use std::sync::mpsc::sync_channel;

use super::*;

fn sample(n: i32) -> DriveSample {
    DriveSample {
        target_counts: n,
        position_demand: n + 1,
        position_actual: n + 2,
        following_error: -3,
        torque_actual: 42,
        statusword: 0x0627,
        error_code: 0,
    }
}

fn record(cycle: u64) -> CaptureRecord {
    CaptureRecord {
        cycle_index: cycle,
        flags: FLAG_TORQUE_ENABLED | FLAG_MOTION_ACTIVE,
        drive: sample(1000),
    }
}

fn cfg(path: &PathBuf) -> CaptureConfig {
    CaptureConfig {
        path: path.to_str().unwrap().to_owned(),
        started_utc: "2026-06-10T12:00:00Z".to_owned(),
        drive_name: "x".to_owned(),
        cycle_ns: 1_000_000,
        counts_per_mm: 3276.8,
        started_mono_ns: 7,
    }
}

fn tmp_path(tag: &str) -> PathBuf {
    std::env::temp_dir().join(format!(
        "kalico-capture-{}-{}.scap",
        tag,
        std::process::id()
    ))
}

#[test]
fn record_encodes_to_fixed_little_endian_layout() {
    let r = CaptureRecord {
        cycle_index: 0x0102030405060708,
        flags: 0x03,
        drive: DriveSample {
            target_counts: -2,
            position_demand: 0x11223344,
            position_actual: -1,
            following_error: 5,
            torque_actual: -300,
            statusword: 0x0627,
            error_code: 0x7380,
        },
    };
    let b = encode_record(&r);
    assert_eq!(b.len(), RECORD_SIZE);
    assert_eq!(&b[0..8], &0x0102030405060708u64.to_le_bytes());
    assert_eq!(b[8], 0x03);
    assert_eq!(&b[9..13], &(-2i32).to_le_bytes());
    assert_eq!(&b[13..17], &0x11223344i32.to_le_bytes());
    assert_eq!(&b[17..21], &(-1i32).to_le_bytes());
    assert_eq!(&b[21..25], &5i32.to_le_bytes());
    assert_eq!(&b[25..27], &(-300i16).to_le_bytes());
    assert_eq!(&b[27..29], &0x0627u16.to_le_bytes());
    assert_eq!(&b[29..31], &0x7380u16.to_le_bytes());
}

#[test]
fn header_is_one_json_line_describing_the_record() {
    let path = tmp_path("hdr");
    let h = header_json(&cfg(&path));
    assert!(h.ends_with('\n'));
    assert_eq!(h.lines().count(), 1);
    for needle in [
        "\"version\":1",
        "\"cycle_ns\":1000000",
        "\"record_size\":31",
        "\"started_utc\":\"2026-06-10T12:00:00Z\"",
        "\"started_mono_ns\":7",
        "\"name\":\"x\"",
        "\"counts_per_mm\":3276.8",
        "{\"name\":\"following_error\",\"dtype\":\"i32\",\"offset\":21}",
        "{\"name\":\"error_code\",\"dtype\":\"u16\",\"offset\":29}",
    ] {
        assert!(h.contains(needle), "header missing {needle}: {h}");
    }
}

#[test]
fn lifecycle_start_push_stop_produces_parseable_file() {
    let path = tmp_path("happy");
    let _ = std::fs::remove_file(&path);
    let mut cap = Capture::new();
    assert!(!cap.is_active());
    assert_eq!(cap.start(cfg(&path)), 0);
    assert!(cap.is_active());
    for i in 0..50u64 {
        cap.push(record(i));
    }
    let out = cap.stop();
    assert_eq!(out.result, 0);
    assert_eq!(out.samples, 50);
    assert_eq!(out.overflow_cycle, None);
    assert!(!cap.is_active());

    let bytes = std::fs::read(&path).unwrap();
    let nl = bytes.iter().position(|&b| b == b'\n').unwrap();
    let header = std::str::from_utf8(&bytes[..nl]).unwrap();
    assert!(header.contains("\"version\":1"));
    let body = &bytes[nl + 1..];
    assert_eq!(body.len(), 50 * RECORD_SIZE);
    assert_eq!(&body[..RECORD_SIZE], &encode_record(&record(0)));
    std::fs::remove_file(&path).unwrap();
}

#[test]
fn double_start_rejected_and_capture_survives() {
    let path = tmp_path("dbl");
    let _ = std::fs::remove_file(&path);
    let mut cap = Capture::new();
    assert_eq!(cap.start(cfg(&path)), 0);
    assert_eq!(cap.start(cfg(&path)), ERR_CAPTURE_ACTIVE);
    assert!(cap.is_active());
    let out = cap.stop();
    assert_eq!(out.result, 0);
    std::fs::remove_file(&path).unwrap();
}

#[test]
fn stop_without_start_rejected() {
    let mut cap = Capture::new();
    let out = cap.stop();
    assert_eq!(out.result, ERR_CAPTURE_NOT_ACTIVE);
    assert_eq!(out.samples, 0);
}

#[test]
fn unwritable_path_fails_start() {
    let mut cap = Capture::new();
    let mut c = cfg(&PathBuf::from("/dev/null/nope/x.scap"));
    c.path = "/dev/null/nope/x.scap".to_owned();
    assert_eq!(cap.start(c), ERR_CAPTURE_FILE);
    assert!(!cap.is_active());
}

#[test]
fn quote_in_drive_name_rejected_before_touching_disk() {
    let path = tmp_path("badname");
    let mut cap = Capture::new();
    let mut c = cfg(&path);
    c.drive_name = "x\"evil".to_owned();
    assert_eq!(cap.start(c), ERR_CAPTURE_BAD_ARG);
    assert!(!path.exists());
}

#[test]
fn overflow_kills_capture_and_renames_file() {
    let path = tmp_path("ovf");
    let _ = std::fs::remove_file(&path);
    let failed = path.with_extension("failed.scap");
    let _ = std::fs::remove_file(&failed);

    let (gate_tx, gate_rx) = sync_channel::<()>(1);
    let mut cap = Capture::with_capacity(4);
    assert_eq!(cap.start_gated(cfg(&path), gate_rx), 0);
    // writer is blocked on the gate: pushes beyond capacity must overflow
    for i in 0..10u64 {
        cap.push(record(i));
    }
    gate_tx.send(()).unwrap();
    let out = cap.stop();
    assert_eq!(out.result, ERR_CAPTURE_OVERFLOW);
    assert_eq!(out.overflow_cycle, Some(4));
    assert_eq!(out.samples, 4);
    assert!(!path.exists(), "failed capture must not keep .scap name");
    assert!(failed.exists(), "failed capture must be renamed");
    std::fs::remove_file(&failed).unwrap();
}

#[test]
fn pushes_after_overflow_are_ignored() {
    let path = tmp_path("ovf2");
    let _ = std::fs::remove_file(&path);
    let (gate_tx, gate_rx) = sync_channel::<()>(1);
    let mut cap = Capture::with_capacity(2);
    assert_eq!(cap.start_gated(cfg(&path), gate_rx), 0);
    for i in 0..100u64 {
        cap.push(record(i));
    }
    gate_tx.send(()).unwrap();
    let out = cap.stop();
    assert_eq!(out.overflow_cycle, Some(2), "first refused cycle is recorded");
    let failed = path.with_extension("failed.scap");
    std::fs::remove_file(&failed).unwrap();
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo nextest run -p kalico-ethercat-rt -E 'test(capture)'`
Expected: compile error — module doesn't exist.

- [ ] **Step 3: Implement `rust/kalico-ethercat-rt/src/capture.rs`**

```rust
//! Per-DC-cycle telemetry capture: the RT loop pushes fixed-size records into
//! a bounded channel; a writer thread drains them to a header-prefixed file.
//! Ring overflow is capture death, not sample loss — a silently gappy capture
//! poisons every downstream FFT and fit.

use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::sync::mpsc::{sync_channel, Receiver, SyncSender, TrySendError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

pub const ERR_CAPTURE_ACTIVE: i32 = -320;
pub const ERR_CAPTURE_NOT_ACTIVE: i32 = -321;
pub const ERR_CAPTURE_FILE: i32 = -322;
pub const ERR_CAPTURE_OVERFLOW: i32 = -323;
pub const ERR_CAPTURE_BAD_ARG: i32 = -324;

pub const CAPTURE_RING_CAPACITY: usize = 4096;
pub const RECORD_SIZE: usize = 31;
pub const FLAG_TORQUE_ENABLED: u8 = 1 << 0;
pub const FLAG_MOTION_ACTIVE: u8 = 1 << 1;

const WRITER_SYNC_INTERVAL: Duration = Duration::from_secs(1);
const WRITER_RECV_TIMEOUT: Duration = Duration::from_millis(100);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DriveSample {
    pub target_counts: i32,
    pub position_demand: i32,
    pub position_actual: i32,
    pub following_error: i32,
    pub torque_actual: i16,
    pub statusword: u16,
    pub error_code: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CaptureRecord {
    pub cycle_index: u64,
    pub flags: u8,
    pub drive: DriveSample,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CaptureConfig {
    pub path: String,
    pub started_utc: String,
    pub drive_name: String,
    pub cycle_ns: i64,
    pub counts_per_mm: f64,
    pub started_mono_ns: u64,
}

#[derive(Debug, PartialEq, Eq)]
pub struct StopOutcome {
    pub result: i32,
    pub samples: u64,
    pub overflow_cycle: Option<u64>,
}

pub fn encode_record(r: &CaptureRecord) -> [u8; RECORD_SIZE] {
    let mut b = [0u8; RECORD_SIZE];
    b[0..8].copy_from_slice(&r.cycle_index.to_le_bytes());
    b[8] = r.flags;
    b[9..13].copy_from_slice(&r.drive.target_counts.to_le_bytes());
    b[13..17].copy_from_slice(&r.drive.position_demand.to_le_bytes());
    b[17..21].copy_from_slice(&r.drive.position_actual.to_le_bytes());
    b[21..25].copy_from_slice(&r.drive.following_error.to_le_bytes());
    b[25..27].copy_from_slice(&r.drive.torque_actual.to_le_bytes());
    b[27..29].copy_from_slice(&r.drive.statusword.to_le_bytes());
    b[29..31].copy_from_slice(&r.drive.error_code.to_le_bytes());
    b
}

fn json_string_safe(s: &str) -> bool {
    s.chars()
        .all(|c| (c.is_ascii_graphic() || c == ' ') && c != '"' && c != '\\')
}

pub fn header_json(cfg: &CaptureConfig) -> String {
    format!(
        concat!(
            "{{\"version\":1,\"cycle_ns\":{},\"record_size\":{},",
            "\"started_utc\":\"{}\",\"started_mono_ns\":{},",
            "\"drives\":[{{\"name\":\"{}\",\"counts_per_mm\":{}}}],",
            "\"channels\":[",
            "{{\"name\":\"cycle_index\",\"dtype\":\"u64\",\"offset\":0}},",
            "{{\"name\":\"flags\",\"dtype\":\"u8\",\"offset\":8}},",
            "{{\"name\":\"target_counts\",\"dtype\":\"i32\",\"offset\":9}},",
            "{{\"name\":\"position_demand\",\"dtype\":\"i32\",\"offset\":13}},",
            "{{\"name\":\"position_actual\",\"dtype\":\"i32\",\"offset\":17}},",
            "{{\"name\":\"following_error\",\"dtype\":\"i32\",\"offset\":21}},",
            "{{\"name\":\"torque_actual\",\"dtype\":\"i16\",\"offset\":25}},",
            "{{\"name\":\"statusword\",\"dtype\":\"u16\",\"offset\":27}},",
            "{{\"name\":\"error_code\",\"dtype\":\"u16\",\"offset\":29}}",
            "]}}\n",
        ),
        cfg.cycle_ns,
        RECORD_SIZE,
        cfg.started_utc,
        cfg.started_mono_ns,
        cfg.drive_name,
        cfg.counts_per_mm,
    )
}

struct ActiveCapture {
    tx: SyncSender<CaptureRecord>,
    writer: JoinHandle<Result<u64, String>>,
    path: PathBuf,
    failure: Option<(u64, i32)>,
}

pub struct Capture {
    capacity: usize,
    active: Option<ActiveCapture>,
}

impl Capture {
    pub fn new() -> Self {
        Self::with_capacity(CAPTURE_RING_CAPACITY)
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            capacity,
            active: None,
        }
    }

    pub fn is_active(&self) -> bool {
        self.active.is_some()
    }

    pub fn start(&mut self, cfg: CaptureConfig) -> i32 {
        self.start_inner(cfg, None)
    }

    #[cfg(test)]
    pub(crate) fn start_gated(&mut self, cfg: CaptureConfig, gate: Receiver<()>) -> i32 {
        self.start_inner(cfg, Some(gate))
    }

    fn start_inner(&mut self, cfg: CaptureConfig, gate: Option<Receiver<()>>) -> i32 {
        if self.active.is_some() {
            return ERR_CAPTURE_ACTIVE;
        }
        if !json_string_safe(&cfg.drive_name) || !json_string_safe(&cfg.started_utc) {
            return ERR_CAPTURE_BAD_ARG;
        }
        let path = PathBuf::from(&cfg.path);
        if let Some(parent) = path.parent() {
            if std::fs::create_dir_all(parent).is_err() {
                return ERR_CAPTURE_FILE;
            }
        }
        let file = match File::create(&path) {
            Ok(f) => f,
            Err(_) => return ERR_CAPTURE_FILE,
        };
        let header = header_json(&cfg);
        let (tx, rx) = sync_channel(self.capacity);
        let writer = std::thread::Builder::new()
            .name("capture-writer".into())
            .spawn(move || writer_thread(rx, file, header, gate))
            .expect("spawn capture writer thread");
        self.active = Some(ActiveCapture {
            tx,
            writer,
            path,
            failure: None,
        });
        0
    }

    pub fn push(&mut self, record: CaptureRecord) {
        let Some(active) = self.active.as_mut() else {
            return;
        };
        if active.failure.is_some() {
            return;
        }
        match active.tx.try_send(record) {
            Ok(()) => {}
            Err(TrySendError::Full(r)) => {
                active.failure = Some((r.cycle_index, ERR_CAPTURE_OVERFLOW));
            }
            Err(TrySendError::Disconnected(r)) => {
                active.failure = Some((r.cycle_index, ERR_CAPTURE_FILE));
            }
        }
    }

    pub fn stop(&mut self) -> StopOutcome {
        let Some(active) = self.active.take() else {
            return StopOutcome {
                result: ERR_CAPTURE_NOT_ACTIVE,
                samples: 0,
                overflow_cycle: None,
            };
        };
        drop(active.tx);
        let written = active.writer.join().expect("capture writer panicked");
        let (mut result, mut overflow_cycle) = (0i32, None);
        if let Some((cycle, code)) = active.failure {
            result = code;
            overflow_cycle = Some(cycle);
        }
        let samples = match written {
            Ok(n) => n,
            Err(_) if result == 0 => {
                result = ERR_CAPTURE_FILE;
                0
            }
            Err(_) => 0,
        };
        if result != 0 {
            let failed = active.path.with_extension("failed.scap");
            if std::fs::rename(&active.path, &failed).is_err() && result == ERR_CAPTURE_OVERFLOW {
                result = ERR_CAPTURE_FILE;
            }
        }
        StopOutcome {
            result,
            samples,
            overflow_cycle,
        }
    }
}

fn writer_thread(
    rx: Receiver<CaptureRecord>,
    file: File,
    header: String,
    gate: Option<Receiver<()>>,
) -> Result<u64, String> {
    let mut file = file;
    file.write_all(header.as_bytes())
        .map_err(|e| format!("capture header write: {e}"))?;
    if let Some(g) = gate {
        let _ = g.recv();
    }
    let mut written = 0u64;
    let mut last_sync = Instant::now();
    loop {
        match rx.recv_timeout(WRITER_RECV_TIMEOUT) {
            Ok(r) => {
                file.write_all(&encode_record(&r))
                    .map_err(|e| format!("capture record write: {e}"))?;
                written += 1;
            }
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {}
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => break,
        }
        if last_sync.elapsed() >= WRITER_SYNC_INTERVAL {
            file.sync_data()
                .map_err(|e| format!("capture fsync: {e}"))?;
            last_sync = Instant::now();
        }
    }
    file.sync_data()
        .map_err(|e| format!("capture final fsync: {e}"))?;
    Ok(written)
}

#[cfg(test)]
mod tests;
```

Implementation notes for the executor:
- No `BufWriter`: each record is one 31-byte `write_all`. At 1 kHz that's 1000 small writes/s on the *writer* thread (not the RT thread) — well within budget, and it keeps "flushed to the file" equal to "written", which the truncation-parseability story depends on. Do not add buffering.
- `sync_channel` is used as the SPSC ring: `try_send` on the RT side is non-blocking; capacity is the ring size.
- `overflow_cycle == Some(4)` in the gated test relies on records 0..3 filling capacity 4 and record 4 being the first refused — the writer is gate-blocked so it cannot drain. The header is written BEFORE the gate so file creation is still exercised.
- In the `unwritable_path_fails_start` test, `/dev/null/nope` — `create_dir_all` fails because `/dev/null` is not a directory.

- [ ] **Step 4: Run tests**

Run: `cargo nextest run -p kalico-ethercat-rt -E 'test(capture)'`
Expected: all 9 pass. Also run `cargo nextest run -p kalico-ethercat-rt` to confirm nothing else broke.

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-ethercat-rt/src/capture.rs rust/kalico-ethercat-rt/src/capture/tests.rs rust/kalico-ethercat-rt/src/lib.rs
git commit -m "feat(ec-rt): capture module — bounded ring, writer thread, fail-loud overflow"
```

---

### Task 6: kalico-ethercat-rt — wire decode + response frames for capture commands

**Files:**
- Modify: `rust/kalico-ethercat-rt/src/wire.rs`
- Test: `rust/kalico-ethercat-rt/src/wire/tests.rs`

- [ ] **Step 1: Write failing tests**

Append to `rust/kalico-ethercat-rt/src/wire/tests.rs` (read the file first and reuse its existing frame-building helpers if it has them; these tests are written against the public API only):

```rust
#[test]
fn decode_start_capture_command() {
    use kalico_native_transport::frame::CHANNEL_CONTROL;
    use kalico_protocol::codec::Encode as _;
    use kalico_protocol::messages::{MessageKind, StartCapture};

    let msg = StartCapture {
        path: "/tmp/t.scap".into(),
        started_utc: "2026-06-10T12:00:00Z".into(),
        drive_name: "x".into(),
    };
    let payload = crate::wire::frame_payload(MessageKind::StartCapture, 77, &msg.encoded_to_vec());
    match crate::wire::decode_command(CHANNEL_CONTROL, &payload).unwrap() {
        crate::wire::Command::StartCapture {
            correlation_id,
            msg: decoded,
        } => {
            assert_eq!(correlation_id, 77);
            assert_eq!(decoded, msg);
        }
        other => panic!("expected StartCapture, got {other:?}"),
    }
}

#[test]
fn decode_stop_capture_command() {
    use kalico_native_transport::frame::CHANNEL_CONTROL;
    use kalico_protocol::messages::MessageKind;

    let payload = crate::wire::frame_payload(MessageKind::StopCapture, 78, &[]);
    match crate::wire::decode_command(CHANNEL_CONTROL, &payload).unwrap() {
        crate::wire::Command::StopCapture { correlation_id } => assert_eq!(correlation_id, 78),
        other => panic!("expected StopCapture, got {other:?}"),
    }
}

#[test]
fn stop_capture_response_frame_round_trips() {
    use kalico_protocol::codec::Decode as _;
    use kalico_protocol::messages::StopCaptureResponse;

    let frame = crate::wire::stop_capture_response_frame(9, -323, 1234, 567);
    // frame = transport framing + 7-byte message header + body; decode the body
    // the same way the existing wire tests unwrap frames (reuse their helper).
    let payload = unwrap_frame_payload(&frame); // existing helper in this test file; if absent, decode via kalico_native_transport::frame
    let (_hdr, body) =
        kalico_native_transport::wire_helpers::decode_message_header(&payload).unwrap();
    let resp = StopCaptureResponse::decode(body).unwrap();
    assert_eq!(
        resp,
        StopCaptureResponse {
            result: -323,
            samples: 1234,
            overflow_cycle: 567
        }
    );
}
```

(If `wire/tests.rs` has no `unwrap_frame_payload` helper, write one using `kalico_native_transport::frame`'s decode API — mirror however the existing tests in that file unwrap `set_torque_response_frame`-style outputs.)

- [ ] **Step 2: Run to verify failure**

Run: `cargo nextest run -p kalico-ethercat-rt -E 'test(capture_command) or test(capture_response)'`
Expected: compile error — variants/functions missing.

- [ ] **Step 3: Implement in `rust/kalico-ethercat-rt/src/wire.rs`**

Extend the imports from `kalico_protocol::messages` with `StartCapture, StartCaptureResponse, StopCaptureResponse`.

Add to `enum Command` (before `Unknown`):

```rust
    StartCapture {
        correlation_id: u32,
        msg: StartCapture,
    },
    StopCapture {
        correlation_id: u32,
    },
```

Add arms to the `match MessageKind::from_u16(hdr.kind_raw)` in `decode_command` (before the `_ =>` arm):

```rust
        Some(MessageKind::StartCapture) => {
            let msg = StartCapture::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
            Ok(Command::StartCapture {
                correlation_id: cid,
                msg,
            })
        }
        Some(MessageKind::StopCapture) => Ok(Command::StopCapture {
            correlation_id: cid,
        }),
```

Add response builders next to `set_torque_response_frame`:

```rust
pub fn start_capture_response_frame(cid: u32, result: i32) -> Vec<u8> {
    let body = StartCaptureResponse { result }.encoded_to_vec();
    control_frame(MessageKind::StartCaptureResponse, cid, &body)
}

pub fn stop_capture_response_frame(
    cid: u32,
    result: i32,
    samples: u64,
    overflow_cycle: u64,
) -> Vec<u8> {
    let body = StopCaptureResponse {
        result,
        samples,
        overflow_cycle,
    }
    .encoded_to_vec();
    control_frame(MessageKind::StopCaptureResponse, cid, &body)
}
```

- [ ] **Step 4: Run tests**

Run: `cargo nextest run -p kalico-ethercat-rt`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-ethercat-rt/src/wire.rs rust/kalico-ethercat-rt/src/wire/tests.rs
git commit -m "feat(ec-rt): wire decode + response frames for capture commands"
```

---

### Task 7: endpoint binaries — capture wired into the DC loop and the stub

**Files:**
- Modify: `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs`
- Modify: `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt-stub.rs`

No new unit tests here (binaries); Task 8's integration test covers the stub path end-to-end, and `cargo check --features hw` covers the hw bin.

- [ ] **Step 1: Wire into the hw binary (`src/bin/kalico-ethercat-rt.rs`)**

Imports — extend the existing `use kalico_ethercat_rt::...` block:

```rust
use kalico_ethercat_rt::capture::{
    Capture, CaptureConfig, CaptureRecord, DriveSample, FLAG_MOTION_ACTIVE, FLAG_TORQUE_ENABLED,
};
use kalico_ethercat_rt::wire::{start_capture_response_frame, stop_capture_response_frame};
```

State, next to `let mut gate = TorqueGate::new();` (before the `'dc` loop):

```rust
    let mut capture = Capture::new();
    let mut cycle_index: u64 = 0;
```

Command arms, inside the `for cmd in server.poll_commands()` match (after the `SetTorque` arm):

```rust
                Command::StartCapture {
                    correlation_id,
                    msg,
                } => {
                    let rc = capture.start(CaptureConfig {
                        path: msg.path.clone(),
                        started_utc: msg.started_utc.clone(),
                        drive_name: msg.drive_name.clone(),
                        cycle_ns,
                        counts_per_mm,
                        started_mono_ns: monotonic_ns(),
                    });
                    eprintln!("ec-rt: StartCapture path={} rc={rc}", msg.path);
                    server.respond(&start_capture_response_frame(correlation_id, rc));
                }
                Command::StopCapture { correlation_id } => {
                    let out = capture.stop();
                    eprintln!(
                        "ec-rt: StopCapture result={} samples={} overflow={:?}",
                        out.result, out.samples, out.overflow_cycle
                    );
                    server.respond(&stop_capture_response_frame(
                        correlation_id,
                        out.result,
                        out.samples,
                        out.overflow_cycle.unwrap_or(u64::MAX),
                    ));
                }
```

Motion-active tracking — replace the existing sampling block:

```rust
        let mut motion_active = false;
        if gate.state() == TorqueState::Enabled {
            if let Some((pos_mm, _vel_mm_s)) = ring.sample(now) {
                let map = cmap.get_or_insert_with(|| {
                    let actual = unsafe { ffi::ec_rt_get_position_actual() };
                    CountMap::new(counts_per_mm, actual, f64::from(pos_mm))
                });
                let counts = map.target_counts(f64::from(pos_mm));
                unsafe { ffi::ec_rt_set_target_position(counts) };
                motion_active = true;
            } else {
                cmap = None;
            }
        }
```

Record push — immediately after `let wkc = unsafe { ffi::ec_rt_cycle(&mut toff) };`:

```rust
        cycle_index += 1;
        if capture.is_active() {
            let mut t = ffi::EcTelemetry::default();
            unsafe { ffi::ec_rt_get_telemetry(&mut t) };
            let mut flags = 0u8;
            if gate.state() == TorqueState::Enabled {
                flags |= FLAG_TORQUE_ENABLED;
            }
            if motion_active {
                flags |= FLAG_MOTION_ACTIVE;
            }
            capture.push(CaptureRecord {
                cycle_index,
                flags,
                drive: DriveSample {
                    target_counts: t.target_position,
                    position_demand: t.position_demand,
                    position_actual: t.position_actual,
                    following_error: t.following_error,
                    torque_actual: t.torque_actual,
                    statusword: t.statusword,
                    error_code: t.error_code,
                },
            });
        }
```

(The push sits after `ec_rt_cycle` so the record pairs this cycle's staged target with the feedback received in the same exchange.)

- [ ] **Step 2: Wire into the stub (`src/bin/kalico-ethercat-rt-stub.rs`)**

Same imports as the hw bin (capture types + the two frame fns). State before the `'session` loop:

```rust
    let mut capture = Capture::new();
    let mut cycle_index: u64 = 0;
```

Command arms — identical to the hw bin except the config uses stub constants:

```rust
                Command::StartCapture {
                    correlation_id,
                    msg,
                } => {
                    let rc = capture.start(CaptureConfig {
                        path: msg.path.clone(),
                        started_utc: msg.started_utc.clone(),
                        drive_name: msg.drive_name.clone(),
                        cycle_ns: 1_000_000,
                        counts_per_mm: 3276.8,
                        started_mono_ns: monotonic_ns(),
                    });
                    eprintln!("ec-rt-stub: StartCapture path={} rc={rc}", msg.path);
                    server.respond(&start_capture_response_frame(correlation_id, rc));
                }
                Command::StopCapture { correlation_id } => {
                    let out = capture.stop();
                    eprintln!(
                        "ec-rt-stub: StopCapture result={} samples={} overflow={:?}",
                        out.result, out.samples, out.overflow_cycle
                    );
                    server.respond(&stop_capture_response_frame(
                        correlation_id,
                        out.result,
                        out.samples,
                        out.overflow_cycle.unwrap_or(u64::MAX),
                    ));
                }
```

Synthetic telemetry — at the bottom of the `'session` loop body, just before the `sleep(Duration::from_millis(1));` (line ~241):

```rust
        cycle_index += 1;
        if capture.is_active() {
            let pos = i32::try_from((cycle_index % 100_000) * 10).unwrap_or(0);
            let mut flags = 0u8;
            if gate.state() == TorqueState::Enabled {
                flags |= FLAG_TORQUE_ENABLED;
            }
            if !ring.is_empty() {
                flags |= FLAG_MOTION_ACTIVE;
            }
            capture.push(CaptureRecord {
                cycle_index,
                flags,
                drive: DriveSample {
                    target_counts: pos,
                    position_demand: pos,
                    position_actual: pos - 3,
                    following_error: 3,
                    torque_actual: 100,
                    statusword: 0x0627,
                    error_code: 0,
                },
            });
        }
```

- [ ] **Step 3: Verify both configurations build**

Run from `rust/`:
- `cargo build -p kalico-ethercat-rt --bin kalico-ethercat-rt-stub`
- `cargo check -p kalico-ethercat-rt --features hw`
Expected: both clean.

- [ ] **Step 4: Commit**

```bash
git add rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt-stub.rs
git commit -m "feat(ec-rt): capture start/stop + per-cycle record push in endpoint and stub"
```

---

### Task 8: integration test — capture lifecycle against the stub

**Files:**
- Create: `rust/kalico-ethercat-rt/tests/capture_lifecycle.rs`

- [ ] **Step 1: Write the test**

Model the harness on `rust/kalico-ethercat-rt/tests/torque_lifecycle.rs` (spawn `CARGO_BIN_EXE_kalico-ethercat-rt-stub`, `ChildGuard`, `socket_path`, `wait_for_socket`, `do_handshake` — copy those helpers; they are test-file-local by convention in this crate).

```rust
use std::time::{Duration, Instant};

use kalico_host_rt::native_call::NativeCall;
use kalico_host_rt::unix_native_conn::UnixNativeConn;
use kalico_protocol::codec::{Decode, Encode};
use kalico_protocol::messages::{
    MessageKind, StartCapture, StartCaptureResponse, StopCapture, StopCaptureResponse,
};

// ... ChildGuard / socket_path / wait_for_socket / do_handshake copied from
// torque_lifecycle.rs (use the "cap" tag in socket_path) ...

const RECORD_SIZE: usize = 31;
const ERR_CAPTURE_ACTIVE: i32 = -320;
const ERR_CAPTURE_NOT_ACTIVE: i32 = -321;
const ERR_CAPTURE_FILE: i32 = -322;

fn start_capture(conn: &UnixNativeConn, path: &str) -> i32 {
    let body = StartCapture {
        path: path.to_owned(),
        started_utc: "2026-06-10T12:00:00Z".to_owned(),
        drive_name: "x".to_owned(),
    }
    .encoded_to_vec();
    let (kind, resp) = conn
        .kalico_call(MessageKind::StartCapture, body, Duration::from_secs(5))
        .expect("StartCapture call must succeed");
    assert_eq!(kind, MessageKind::StartCaptureResponse);
    StartCaptureResponse::decode(&resp).expect("decode").result
}

fn stop_capture(conn: &UnixNativeConn) -> StopCaptureResponse {
    let (kind, resp) = conn
        .kalico_call(
            MessageKind::StopCapture,
            StopCapture.encoded_to_vec(),
            Duration::from_secs(5),
        )
        .expect("StopCapture call must succeed");
    assert_eq!(kind, MessageKind::StopCaptureResponse);
    StopCaptureResponse::decode(&resp).expect("decode")
}

fn capture_file(tag: &str) -> String {
    format!(
        "{}/kalico-capture-it-{}-{}.scap",
        std::env::temp_dir().display(),
        tag,
        std::process::id()
    )
}

#[test]
fn capture_start_records_stop_produces_consistent_file() {
    // spawn stub + handshake (per torque_lifecycle.rs pattern)
    let path = capture_file("happy");
    let _ = std::fs::remove_file(&path);

    assert_eq!(start_capture(&conn, &path), 0);
    std::thread::sleep(Duration::from_millis(300));
    let resp = stop_capture(&conn);

    assert_eq!(resp.result, 0);
    assert!(
        resp.samples > 100,
        "stub pushes ~1/ms; got {}",
        resp.samples
    );
    assert_eq!(resp.overflow_cycle, u64::MAX);

    let bytes = std::fs::read(&path).expect("capture file must exist");
    let nl = bytes
        .iter()
        .position(|&b| b == b'\n')
        .expect("header newline");
    let header = std::str::from_utf8(&bytes[..nl]).expect("utf8 header");
    assert!(header.contains("\"version\":1"));
    assert!(header.contains("\"record_size\":31"));
    let body_len = bytes.len() - nl - 1;
    assert_eq!(body_len % RECORD_SIZE, 0, "fixed-size records");
    assert_eq!(body_len / RECORD_SIZE, resp.samples as usize);
    std::fs::remove_file(&path).unwrap();
}

#[test]
fn double_start_rejected_without_killing_first_capture() {
    let path = capture_file("dbl");
    let path2 = capture_file("dbl2");
    let _ = std::fs::remove_file(&path);

    assert_eq!(start_capture(&conn, &path), 0);
    assert_eq!(start_capture(&conn, &path2), ERR_CAPTURE_ACTIVE);
    let resp = stop_capture(&conn);
    assert_eq!(resp.result, 0);
    assert!(!std::path::Path::new(&path2).exists());
    std::fs::remove_file(&path).unwrap();
}

#[test]
fn stop_without_start_rejected() {
    let resp = stop_capture(&conn);
    assert_eq!(resp.result, ERR_CAPTURE_NOT_ACTIVE);
    assert_eq!(resp.samples, 0);
}

#[test]
fn unwritable_path_reports_file_error() {
    assert_eq!(
        start_capture(&conn, "/dev/null/nope/x.scap"),
        ERR_CAPTURE_FILE
    );
}
```

(Each test spawns its own stub on its own socket, exactly as `torque_lifecycle.rs` does — the `conn` in the snippets above comes from that per-test setup.)

- [ ] **Step 2: Run**

Run: `cargo nextest run -p kalico-ethercat-rt -E 'binary(capture_lifecycle)'`
Expected: 4 pass. If the stub doesn't push records (samples == 0), the stub's push is gated on something unexpected — fix the stub, not the test.

- [ ] **Step 3: Commit**

```bash
git add rust/kalico-ethercat-rt/tests/capture_lifecycle.rs
git commit -m "test(ec-rt): capture lifecycle integration tests against the stub"
```

---

### Task 9: motion-bridge — send helpers + PyO3 methods

**Files:**
- Create: `rust/motion-bridge/src/servo_capture.rs`
- Modify: `rust/motion-bridge/src/lib.rs` (add `pub mod servo_capture;` — wherever `pub mod servo_torque;` is declared)
- Modify: `rust/motion-bridge/src/bridge.rs`

- [ ] **Step 1: Create `rust/motion-bridge/src/servo_capture.rs`**

Mirror `servo_torque.rs`:

```rust
use std::time::Duration;

use kalico_host_rt::native_call::NativeCall as _;
use kalico_host_rt::unix_native_conn::UnixNativeConn;
use kalico_protocol::codec::{Decode as _, Encode as _};
use kalico_protocol::messages::{
    MessageKind, StartCapture, StartCaptureResponse, StopCapture, StopCaptureResponse,
};

/// Capture start/stop only touch the command path (no CiA402 ladder); a stop
/// additionally joins the writer thread, which flushes at most one fsync.
const CAPTURE_TIMEOUT: Duration = Duration::from_secs(5);

pub fn send_start_capture(
    conn: &UnixNativeConn,
    path: &str,
    started_utc: &str,
    drive_name: &str,
) -> Result<i32, String> {
    let body = StartCapture {
        path: path.to_owned(),
        started_utc: started_utc.to_owned(),
        drive_name: drive_name.to_owned(),
    }
    .encoded_to_vec();
    let (kind, resp) = conn
        .kalico_call(MessageKind::StartCapture, body, CAPTURE_TIMEOUT)
        .map_err(|e| format!("StartCapture transport: {e:?}"))?;
    if kind != MessageKind::StartCaptureResponse {
        return Err(format!(
            "StartCapture: unexpected response kind 0x{:04x}",
            kind.as_u16()
        ));
    }
    let r = StartCaptureResponse::decode(&resp)
        .map_err(|e| format!("StartCaptureResponse decode: {e:?}"))?;
    Ok(r.result)
}

pub fn send_stop_capture(conn: &UnixNativeConn) -> Result<StopCaptureResponse, String> {
    let (kind, resp) = conn
        .kalico_call(
            MessageKind::StopCapture,
            StopCapture.encoded_to_vec(),
            CAPTURE_TIMEOUT,
        )
        .map_err(|e| format!("StopCapture transport: {e:?}"))?;
    if kind != MessageKind::StopCaptureResponse {
        return Err(format!(
            "StopCapture: unexpected response kind 0x{:04x}",
            kind.as_u16()
        ));
    }
    StopCaptureResponse::decode(&resp).map_err(|e| format!("StopCaptureResponse decode: {e:?}"))
}
```

(Check whether `rust/motion-bridge/src/servo_torque/tests.rs` exists and what it covers; if it unit-tests `send_set_torque` against a fake conn, mirror those tests in `rust/motion-bridge/src/servo_capture/tests.rs`. If it only tests pure helpers, there is nothing pure here to test — the endpoint integration test covers the protocol — so skip the tests file.)

- [ ] **Step 2: Extract the endpoint-conn lookup helper and add PyO3 methods in `bridge.rs`**

`set_torque` (bridge.rs:803) inlines a conn lookup. Extract it (DRY — three users now). Add as a private method on the same impl block that holds `set_torque` (NOT inside `#[pymethods]` if helpers conventionally live elsewhere — match file convention):

```rust
    fn ethercat_conn(
        &self,
        mcu_handle: u32,
        ctx: &str,
    ) -> PyResult<Arc<UnixNativeConn>> {
        let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
        let mc = mcus.get(&mcu_handle).ok_or_else(|| {
            PyRuntimeError::new_err(format!("{ctx}: unknown mcu_handle {mcu_handle}"))
        })?;
        mc.endpoint_conn.clone().ok_or_else(|| {
            PyRuntimeError::new_err(format!(
                "{ctx}: mcu {mcu_handle} ({}) is not an EtherCAT endpoint",
                mc.label
            ))
        })
    }
```

Refactor `set_torque`'s `let conn = { ... }` block to `let conn = self.ethercat_conn(mcu_handle, "set_torque")?;` and confirm existing tests still pass.

Add to `#[pymethods]` (after `set_torque`):

```rust
    fn start_servo_capture(
        &self,
        mcu_handle: u32,
        path: String,
        started_utc: String,
        drive_name: String,
    ) -> PyResult<()> {
        let conn = self.ethercat_conn(mcu_handle, "start_servo_capture")?;
        tracing::info!(
            subsystem = "bridge",
            event = "servo_capture_start",
            mcu_handle,
            path,
            "servo capture start"
        );
        let result =
            crate::servo_capture::send_start_capture(&conn, &path, &started_utc, &drive_name)
                .map_err(PyRuntimeError::new_err)?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "servo capture start failed: endpoint result {result}"
            )));
        }
        Ok(())
    }

    /// Returns (result, samples, overflow_cycle or None). result != 0 means
    /// the capture failed (e.g. -323 ring overflow) and the file was renamed
    /// to .failed.scap — the caller phrases the user-facing error.
    fn stop_servo_capture(&self, mcu_handle: u32) -> PyResult<(i32, u64, Option<u64>)> {
        let conn = self.ethercat_conn(mcu_handle, "stop_servo_capture")?;
        let resp = crate::servo_capture::send_stop_capture(&conn)
            .map_err(PyRuntimeError::new_err)?;
        tracing::info!(
            subsystem = "bridge",
            event = "servo_capture_stop",
            mcu_handle,
            result = resp.result,
            samples = resp.samples,
            "servo capture stop"
        );
        let overflow = (resp.overflow_cycle != u64::MAX).then_some(resp.overflow_cycle);
        Ok((resp.result, resp.samples, overflow))
    }
```

- [ ] **Step 3: Build + test**

Run: `cargo nextest run -p motion-bridge` and `cargo build -p motion-bridge`
Expected: clean.

- [ ] **Step 4: Commit**

```bash
git add rust/motion-bridge/src/servo_capture.rs rust/motion-bridge/src/lib.rs rust/motion-bridge/src/bridge.rs
git commit -m "feat(bridge): start/stop servo capture PyO3 surface"
```

---

### Task 10: klippy — SERVO_CAPTURE_START / SERVO_CAPTURE_STOP

**Files:**
- Create: `klippy/extras/servo_capture.py`
- Modify: `klippy/extras/ethercat_node.py` (auto-load the command object)
- Test: `test/test_servo_capture_cmd.py`

- [ ] **Step 1: Write failing tests**

Create `test/test_servo_capture_cmd.py` (FakePrinter pattern from `test/test_servo_torque.py`):

```python
import pytest

from klippy.extras import servo_capture


class FakeGcode:
    def __init__(self):
        self.commands = {}

    def register_command(self, name, func, desc=None):
        assert name not in self.commands
        self.commands[name] = func


class FakeNode:
    def __init__(self, handle):
        self._h = handle

    def get_bridge_handle(self):
        return self._h


class FakeBridge:
    def __init__(self, stop_result=(0, 1234, None)):
        self.start_calls = []
        self.stop_calls = []
        self._stop_result = stop_result

    def start_servo_capture(self, handle, path, started_utc, drive_name):
        self.start_calls.append((handle, path, started_utc, drive_name))

    def stop_servo_capture(self, handle):
        self.stop_calls.append(handle)
        return self._stop_result


class FakePrinter:
    command_error = RuntimeError

    def __init__(self, objs):
        self._objs = objs

    def lookup_object(self, name):
        return self._objs[name]

    def lookup_objects(self, module=None):
        prefix = module + " "
        return [
            (name, obj)
            for name, obj in self._objs.items()
            if name == module or name.startswith(prefix)
        ]


class FakeConfig:
    def __init__(self, printer):
        self._printer = printer

    def get_printer(self):
        return self._printer


class FakeGcmd:
    error = RuntimeError

    def __init__(self, **params):
        self._params = params
        self.responses = []

    def get(self, name, default=None):
        return self._params.get(name, default)

    def respond_info(self, msg):
        self.responses.append(msg)


def make_capture(nodes=None, bridge=None):
    gcode = FakeGcode()
    objs = {"gcode": gcode, "motion_bridge": bridge or FakeBridge()}
    for name, handle in (nodes or {"x": 7}).items():
        objs["ethercat_node " + name] = FakeNode(handle)
    printer = FakePrinter(objs)
    sc = servo_capture.ServoCapture(FakeConfig(printer))
    return sc, gcode, objs["motion_bridge"]


def test_registers_both_commands():
    _, gcode, _ = make_capture()
    assert "SERVO_CAPTURE_START" in gcode.commands
    assert "SERVO_CAPTURE_STOP" in gcode.commands


def test_start_defaults_to_sole_servo_and_builds_path():
    sc, gcode, bridge = make_capture()
    gcmd = FakeGcmd(NAME="xtune")
    gcode.commands["SERVO_CAPTURE_START"](gcmd)
    assert len(bridge.start_calls) == 1
    handle, path, started_utc, drive_name = bridge.start_calls[0]
    assert handle == 7
    assert drive_name == "x"
    assert "/servo_captures/" in path
    assert path.endswith(".scap")
    assert "xtune_" in path
    assert started_utc.endswith("Z")
    assert any("started" in r for r in gcmd.responses)


def test_start_rejects_bad_name():
    sc, gcode, bridge = make_capture()
    with pytest.raises(RuntimeError):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd(NAME="../evil"))
    assert bridge.start_calls == []


def test_start_rejects_unknown_servo_and_comma_list():
    sc, gcode, bridge = make_capture()
    with pytest.raises(RuntimeError):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd(SERVO="nope"))
    with pytest.raises(RuntimeError):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd(SERVO="a,b"))
    assert bridge.start_calls == []


def test_double_start_rejected_in_klippy():
    sc, gcode, _ = make_capture()
    gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())
    with pytest.raises(RuntimeError):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())


def test_stop_without_start_rejected():
    sc, gcode, bridge = make_capture()
    with pytest.raises(RuntimeError):
        gcode.commands["SERVO_CAPTURE_STOP"](FakeGcmd())
    assert bridge.stop_calls == []


def test_stop_reports_samples():
    sc, gcode, bridge = make_capture()
    gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())
    gcmd = FakeGcmd()
    gcode.commands["SERVO_CAPTURE_STOP"](gcmd)
    assert bridge.stop_calls == [7]
    assert any("1234" in r for r in gcmd.responses)


def test_stop_overflow_raises_with_failed_filename():
    bridge = FakeBridge(stop_result=(-323, 999, 4096))
    sc, gcode, _ = make_capture(bridge=bridge)
    gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())
    with pytest.raises(RuntimeError, match="failed.scap"):
        gcode.commands["SERVO_CAPTURE_STOP"](FakeGcmd())
    # state cleared: a new capture can start
    gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())


def test_start_without_bridge_handle_fails_loudly():
    sc, gcode, bridge = make_capture(nodes={"x": None})
    with pytest.raises(RuntimeError, match="no bridge handle"):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())
    assert bridge.start_calls == []
```

- [ ] **Step 2: Run to verify failure**

Run: `python -m pytest test/test_servo_capture_cmd.py -v`
Expected: ImportError — module doesn't exist.

- [ ] **Step 3: Create `klippy/extras/servo_capture.py`**

```python
# Telemetry capture start/stop for EtherCAT servo drives.
import os
import re
import time

CAPTURE_DIR = "~/printer_data/logs/servo_captures"
NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")


class ServoCapture:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.capture_dir = os.path.expanduser(CAPTURE_DIR)
        self.active = None  # (servo_name, path)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SERVO_CAPTURE_START",
            self.cmd_SERVO_CAPTURE_START,
            desc=self.cmd_SERVO_CAPTURE_START_help,
        )
        gcode.register_command(
            "SERVO_CAPTURE_STOP",
            self.cmd_SERVO_CAPTURE_STOP,
            desc=self.cmd_SERVO_CAPTURE_STOP_help,
        )

    def _nodes(self):
        return {
            name.split()[-1]: obj
            for name, obj in self.printer.lookup_objects("ethercat_node")
        }

    def _resolve_node(self, gcmd):
        servo = gcmd.get("SERVO", None)
        nodes = self._nodes()
        if not nodes:
            raise gcmd.error("SERVO_CAPTURE: no [ethercat_node] configured")
        if servo is None:
            if len(nodes) != 1:
                raise gcmd.error(
                    "SERVO_CAPTURE: multiple servos configured (%s); "
                    "SERVO= is required" % (", ".join(sorted(nodes)),)
                )
            return next(iter(nodes.items()))
        if "," in servo:
            raise gcmd.error(
                "SERVO_CAPTURE: multi-servo capture requires all drives on "
                "one endpoint and is not implemented yet"
            )
        node = nodes.get(servo)
        if node is None:
            raise gcmd.error(
                "SERVO_CAPTURE: unknown servo %r (have: %s)"
                % (servo, ", ".join(sorted(nodes)))
            )
        return servo, node

    cmd_SERVO_CAPTURE_START_help = (
        "Start a servo telemetry capture (1 kHz). Wrap test moves and finish "
        "with M400 before SERVO_CAPTURE_STOP."
    )

    def cmd_SERVO_CAPTURE_START(self, gcmd):
        if self.active is not None:
            raise gcmd.error(
                "SERVO_CAPTURE: capture already active (%s)" % (self.active[1],)
            )
        tag = gcmd.get("NAME", "capture")
        if not NAME_RE.match(tag):
            raise gcmd.error(
                "SERVO_CAPTURE: NAME must match [A-Za-z0-9_-]+, got %r" % (tag,)
            )
        servo, node = self._resolve_node(gcmd)
        handle = node.get_bridge_handle()
        if handle is None:
            raise gcmd.error(
                "SERVO_CAPTURE: servo %r has no bridge handle (node not "
                "claimed)" % (servo,)
            )
        path = os.path.join(
            self.capture_dir,
            "%s_%s.scap" % (tag, time.strftime("%Y%m%d_%H%M%S")),
        )
        started_utc = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        bridge = self.printer.lookup_object("motion_bridge")
        bridge.start_servo_capture(handle, path, started_utc, servo)
        self.active = (servo, path)
        gcmd.respond_info("Servo capture started: %s" % (path,))

    cmd_SERVO_CAPTURE_STOP_help = "Stop the active servo telemetry capture."

    def cmd_SERVO_CAPTURE_STOP(self, gcmd):
        if self.active is None:
            raise gcmd.error("SERVO_CAPTURE: no capture active")
        servo, path = self.active
        self.active = None
        node = self._nodes().get(servo)
        if node is None or node.get_bridge_handle() is None:
            raise gcmd.error(
                "SERVO_CAPTURE: servo %r vanished mid-capture" % (servo,)
            )
        bridge = self.printer.lookup_object("motion_bridge")
        result, samples, overflow_cycle = bridge.stop_servo_capture(
            node.get_bridge_handle()
        )
        if result != 0:
            failed = os.path.splitext(path)[0] + ".failed.scap"
            raise gcmd.error(
                "Servo capture FAILED (endpoint code %d, overflow_cycle=%s); "
                "partial data in %s" % (result, overflow_cycle, failed)
            )
        gcmd.respond_info(
            "Servo capture stopped: %s\n"
            "samples=%d (%.2f s at the 1 kHz DC cycle)"
            % (path, samples, samples / 1000.0)
        )


def load_config(config):
    return ServoCapture(config)
```

- [ ] **Step 4: Auto-load from ethercat_node**

In `klippy/extras/ethercat_node.py`, at the end of `EtherCatNode.__init__` (after the `register_event_handler` line):

```python
        self.printer.load_object(config, "servo_capture")
```

(`load_object` is idempotent — with several `[ethercat_node]` sections only the first call instantiates `ServoCapture`, so command registration happens exactly once.)

- [ ] **Step 5: Run tests**

Run: `python -m pytest test/test_servo_capture_cmd.py test/test_servo_torque.py test/test_imports.py -v`
Expected: all pass.

- [ ] **Step 6: Commit**

```bash
git add klippy/extras/servo_capture.py klippy/extras/ethercat_node.py test/test_servo_capture_cmd.py
git commit -m "feat(klippy): SERVO_CAPTURE_START/STOP commands"
```

---

### Task 11: offline analysis — scripts/servo_capture.py

**Files:**
- Create: `scripts/servo_capture.py`
- Test: `test/test_servo_capture_analysis.py`

- [ ] **Step 1: Write failing tests**

Create `test/test_servo_capture_analysis.py`:

```python
import importlib.util
import json
import os
import struct
import sys

import numpy as np
import pytest

_SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "servo_capture.py",
)
_spec = importlib.util.spec_from_file_location("servo_capture_script", _SCRIPT)
sc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sc)

CHANNELS = [
    {"name": "cycle_index", "dtype": "u64", "offset": 0},
    {"name": "flags", "dtype": "u8", "offset": 8},
    {"name": "target_counts", "dtype": "i32", "offset": 9},
    {"name": "position_demand", "dtype": "i32", "offset": 13},
    {"name": "position_actual", "dtype": "i32", "offset": 17},
    {"name": "following_error", "dtype": "i32", "offset": 21},
    {"name": "torque_actual", "dtype": "i16", "offset": 25},
    {"name": "statusword", "dtype": "u16", "offset": 27},
    {"name": "error_code", "dtype": "u16", "offset": 29},
]


def synth_capture(tmp_path, n=4000, move=(1000, 2000), freq_hz=80.0):
    """1 kHz capture: flat, then a move with an 80 Hz error tone, then a
    post-move exponential decay (settling), then flat."""
    fs = 1000.0
    t = np.arange(n) / fs
    ferr = np.zeros(n)
    ms, me = move
    ferr[ms:me] = 200.0 * np.sin(2 * np.pi * freq_hz * t[ms:me])
    decay = 150.0 * np.exp(-(t[me:] - t[me]) / 0.05)
    ferr[me:] = decay * np.cos(2 * np.pi * 30.0 * (t[me:] - t[me]))
    flags = np.zeros(n, dtype=np.uint8)
    flags[:] = 1  # torque enabled
    flags[ms:me] |= 2  # motion active
    target = np.cumsum(np.where(flags & 2, 100, 0)).astype(np.int64)
    torque = np.zeros(n, dtype=np.int16)
    torque[ms:me] = 950  # saturated during the move

    header = {
        "version": 1,
        "cycle_ns": 1_000_000,
        "record_size": 31,
        "started_utc": "2026-06-10T12:00:00Z",
        "started_mono_ns": 0,
        "drives": [{"name": "x", "counts_per_mm": 3276.8}],
        "channels": CHANNELS,
    }
    path = os.path.join(str(tmp_path), "synth.scap")
    with open(path, "wb") as f:
        f.write((json.dumps(header) + "\n").encode())
        for i in range(n):
            fe = int(round(ferr[i]))
            tgt = int(target[i])
            f.write(
                struct.pack(
                    "<QBiiiihHH",
                    i,
                    int(flags[i]),
                    tgt,
                    tgt,
                    tgt - fe,
                    fe,
                    int(torque[i]),
                    0x0627,
                    0,
                )
            )
    return path, ferr


def test_load_capture_reads_header_and_records(tmp_path):
    path, _ = synth_capture(tmp_path)
    header, data = sc.load_capture(path)
    assert header["version"] == 1
    assert len(data) == 4000
    assert data["cycle_index"][0] == 0
    assert data["cycle_index"][-1] == 3999


def test_refuses_failed_capture(tmp_path):
    path, _ = synth_capture(tmp_path)
    failed = path.replace(".scap", ".failed.scap")
    os.rename(path, failed)
    with pytest.raises(SystemExit):
        sc.load_capture(failed)


def test_truncated_file_parses_to_last_whole_record(tmp_path):
    path, _ = synth_capture(tmp_path)
    size = os.path.getsize(path)
    with open(path, "r+b") as f:
        f.truncate(size - 17)  # kill part of the last record
    _, data = sc.load_capture(path)
    assert len(data) == 3999


def test_motion_segments_found(tmp_path):
    path, _ = synth_capture(tmp_path)
    _, data = sc.load_capture(path)
    segs = sc.motion_segments(data["flags"])
    assert segs == [(1000, 2000)]


def test_following_error_rms_matches_numpy(tmp_path):
    path, ferr = synth_capture(tmp_path)
    _, data = sc.load_capture(path)
    m = sc.compute_metrics(data, settle_band=10, torque_limit=900)
    expected_rms = float(np.sqrt(np.mean(np.round(ferr[1000:2000]) ** 2)))
    assert m["moves"][0]["ferr_rms"] == pytest.approx(expected_rms, rel=0.01)
    assert m["moves"][0]["ferr_peak"] == pytest.approx(200.0, rel=0.02)


def test_resonance_peak_detected_at_80hz(tmp_path):
    path, _ = synth_capture(tmp_path)
    _, data = sc.load_capture(path)
    segs = sc.motion_segments(data["flags"])
    freqs, psd = sc.moving_psd(data, segs, fs=1000.0)
    peaks = sc.top_peaks(freqs, psd, count=3)
    assert abs(peaks[0][0] - 80.0) < 2.5, "dominant peak at the injected 80 Hz"


def test_settling_time_in_expected_range(tmp_path):
    path, _ = synth_capture(tmp_path)
    _, data = sc.load_capture(path)
    m = sc.compute_metrics(data, settle_band=10, torque_limit=900)
    # 150*exp(-t/0.05) crosses 10 counts at t = 0.05*ln(15) ~ 135 ms
    assert 80 <= m["moves"][0]["settle_ms"] <= 300


def test_torque_saturation_fraction(tmp_path):
    path, _ = synth_capture(tmp_path)
    _, data = sc.load_capture(path)
    m = sc.compute_metrics(data, settle_band=10, torque_limit=900)
    assert m["torque_saturation_pct"] == pytest.approx(25.0, abs=1.0)


def test_drive_vs_recomputed_error_consistent(tmp_path):
    path, _ = synth_capture(tmp_path)
    _, data = sc.load_capture(path)
    m = sc.compute_metrics(data, settle_band=10, torque_limit=900)
    assert m["ferr_crosscheck_max"] == 0  # synth file is self-consistent
```

- [ ] **Step 2: Run to verify failure**

Run: `python -m pytest test/test_servo_capture_analysis.py -v`
Expected: failures — script doesn't exist.

- [ ] **Step 3: Create `scripts/servo_capture.py`**

```python
#!/usr/bin/env python3
# Analyze a servo telemetry capture (.scap) produced by SERVO_CAPTURE_START.
# Prints following-error, overshoot/settling, and torque-saturation metrics;
# --fft prints resonance peaks (notch-filter candidates); --plot opens a
# time-series dashboard.
import argparse
import json
import sys

import numpy as np

DTYPE_MAP = {
    "u8": "u1",
    "u16": "<u2",
    "i16": "<i2",
    "i32": "<i4",
    "u64": "<u8",
}
FLAG_MOTION_ACTIVE = 1 << 1
SETTLE_HOLD_MS = 50


def load_capture(path):
    if path.endswith(".failed.scap"):
        raise SystemExit(
            "%s is a FAILED capture (ring overflow or writer error); its "
            "gaps would poison every metric. Re-run the capture." % (path,)
        )
    with open(path, "rb") as f:
        header = json.loads(f.readline())
        if header.get("version") != 1:
            raise SystemExit(
                "unsupported capture version %r" % (header.get("version"),)
            )
        dtype = np.dtype(
            [(c["name"], DTYPE_MAP[c["dtype"]]) for c in header["channels"]]
        )
        if dtype.itemsize != header["record_size"]:
            raise SystemExit(
                "channel descriptor (%d bytes) disagrees with record_size %d"
                % (dtype.itemsize, header["record_size"])
            )
        body = f.read()
    whole = len(body) // header["record_size"] * header["record_size"]
    data = np.frombuffer(body[:whole], dtype=dtype)
    return header, data


def motion_segments(flags):
    moving = (flags & FLAG_MOTION_ACTIVE) != 0
    edges = np.flatnonzero(np.diff(moving.astype(np.int8)))
    bounds = np.concatenate(([0], edges + 1, [len(moving)]))
    return [
        (int(bounds[i]), int(bounds[i + 1]))
        for i in range(len(bounds) - 1)
        if moving[bounds[i]]
    ]


def _settle_index(err, band, hold):
    inside = np.abs(err) <= band
    if len(inside) < hold:
        return None
    windows = np.lib.stride_tricks.sliding_window_view(inside, hold)
    ok = np.flatnonzero(windows.all(axis=1))
    return int(ok[0]) if len(ok) else None


def compute_metrics(data, settle_band, torque_limit):
    ferr = data["following_error"].astype(np.float64)
    recomputed = data["target_counts"].astype(np.int64) - data[
        "position_actual"
    ].astype(np.int64)
    segs = motion_segments(data["flags"])
    moves = []
    for idx, (s, e) in enumerate(segs):
        move_err = ferr[s:e]
        post = ferr[e:]
        settle = _settle_index(post, settle_band, SETTLE_HOLD_MS)
        moves.append(
            {
                "move": idx,
                "start_ms": s,
                "end_ms": e,
                "ferr_peak": float(np.max(np.abs(move_err))),
                "ferr_rms": float(np.sqrt(np.mean(move_err**2))),
                "overshoot": float(
                    np.max(np.abs(post[: settle if settle else len(post)]))
                )
                if len(post)
                else 0.0,
                "settle_ms": settle,
            }
        )
    torque = np.abs(data["torque_actual"].astype(np.int64))
    return {
        "samples": len(data),
        "moves": moves,
        "torque_saturation_pct": float(
            100.0 * np.count_nonzero(torque >= torque_limit) / max(len(data), 1)
        ),
        "ferr_crosscheck_max": int(
            np.max(np.abs(recomputed - ferr.astype(np.int64)))
        )
        if len(data)
        else 0,
    }


def welch_psd(x, fs, nperseg=1024):
    x = np.asarray(x, dtype=np.float64)
    nperseg = min(nperseg, len(x))
    nperseg = 2 ** int(np.log2(nperseg))
    if nperseg < 64:
        raise SystemExit(
            "segment too short for PSD (%d samples; need >= 64)" % (len(x),)
        )
    step = nperseg // 2
    win = np.hanning(nperseg)
    scale = 1.0 / (fs * np.sum(win * win))
    psds = []
    for start in range(0, len(x) - nperseg + 1, step):
        seg = (x[start : start + nperseg] - np.mean(x[start : start + nperseg])) * win
        spec = np.fft.rfft(seg)
        psds.append((spec.real**2 + spec.imag**2) * scale)
    psd = np.mean(psds, axis=0)
    psd[1:-1] *= 2.0
    return np.fft.rfftfreq(nperseg, 1.0 / fs), psd


def moving_psd(data, segs, fs):
    if not segs:
        raise SystemExit("no moving segments in capture — nothing to analyze")
    err = np.concatenate(
        [data["following_error"][s:e].astype(np.float64) for s, e in segs]
    )
    return welch_psd(err, fs)


def top_peaks(freqs, psd, count=5):
    local_max = np.flatnonzero(
        (psd[1:-1] > psd[:-2]) & (psd[1:-1] > psd[2:])
    ) + 1
    ranked = local_max[np.argsort(psd[local_max])[::-1]][:count]
    return [(float(freqs[i]), float(psd[i])) for i in ranked]


def _print_metrics(header, m, counts_per_mm):
    print("capture: %d samples, %d move(s)" % (m["samples"], len(m["moves"])))
    print(
        "torque saturation: %.1f%% of samples at/above limit"
        % (m["torque_saturation_pct"],)
    )
    print(
        "drive-vs-recomputed following error: max delta %d counts"
        % (m["ferr_crosscheck_max"],)
    )
    for mv in m["moves"]:
        settle = (
            "%d ms" % mv["settle_ms"] if mv["settle_ms"] is not None else "NEVER"
        )
        print(
            "move %d [%d..%d ms]: ferr peak %.0f counts (%.4f mm), "
            "rms %.1f counts (%.4f mm), overshoot %.0f counts, settle %s"
            % (
                mv["move"],
                mv["start_ms"],
                mv["end_ms"],
                mv["ferr_peak"],
                mv["ferr_peak"] / counts_per_mm,
                mv["ferr_rms"],
                mv["ferr_rms"] / counts_per_mm,
                mv["overshoot"],
                settle,
            )
        )


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("capture", help="path to a .scap capture file")
    p.add_argument("--settle-band", type=int, default=50,
                   help="settling band in encoder counts (default 50)")
    p.add_argument("--torque-limit", type=int, default=900,
                   help="saturation threshold, per-mille of rated (default 900)")
    p.add_argument("--fft", action="store_true",
                   help="print resonance peaks from the moving-segment PSD")
    p.add_argument("--plot", action="store_true",
                   help="show a time-series dashboard (requires matplotlib)")
    args = p.parse_args(argv)

    header, data = load_capture(args.capture)
    fs = 1e9 / header["cycle_ns"]
    counts_per_mm = header["drives"][0]["counts_per_mm"]
    m = compute_metrics(data, args.settle_band, args.torque_limit)
    _print_metrics(header, m, counts_per_mm)

    if args.fft:
        freqs, psd = moving_psd(data, motion_segments(data["flags"]), fs)
        print("resonance peaks (notch-filter candidates):")
        for f_hz, power in top_peaks(freqs, psd):
            print("  %7.1f Hz  power %.3e" % (f_hz, power))

    if args.plot:
        _plot(header, data, fs)
    return 0


def _plot(header, data, fs):
    import matplotlib.pyplot as plt

    t = np.arange(len(data)) / fs
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
    axes[0].plot(t, data["position_demand"], label="demand (6062h)")
    axes[0].plot(t, data["position_actual"], label="actual (6064h)")
    axes[0].plot(t, data["target_counts"], label="host target (607Ah)",
                 linestyle="--", alpha=0.6)
    axes[0].set_ylabel("counts")
    axes[0].legend(loc="upper right")
    axes[1].plot(t, data["following_error"], color="tab:red")
    axes[1].set_ylabel("following error (counts)")
    axes[2].plot(t, data["torque_actual"], color="tab:green")
    axes[2].set_ylabel("torque (per-mille)")
    axes[2].set_xlabel("time (s)")
    moving = (data["flags"] & FLAG_MOTION_ACTIVE) != 0
    for ax in axes:
        ax.fill_between(t, *ax.get_ylim(), where=moving, alpha=0.08,
                        color="tab:blue")
    fig.suptitle(header["drives"][0]["name"] + " — " + header["started_utc"])
    plt.show()


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 4: Run tests**

Run: `python -m pytest test/test_servo_capture_analysis.py -v`
Expected: 9 pass. The settling/PSD assertions have generous tolerances; if one fails, print the actual value and check the synthetic-signal math before touching the implementation.

- [ ] **Step 5: Commit**

```bash
git add scripts/servo_capture.py test/test_servo_capture_analysis.py
git commit -m "feat(scripts): servo capture offline analysis (metrics, PSD, plot)"
```

---

### Task 12: documentation + spec alignment

**Files:**
- Create: `docs/kalico-rewrite/servo-telemetry-capture.md`
- Modify: `docs/superpowers/specs/2026-06-10-servo-telemetry-capture-design.md`

- [ ] **Step 1: Write `docs/kalico-rewrite/servo-telemetry-capture.md`**

Sections (write full prose, matching the tone of `docs/kalico-rewrite/ethercat-bench-bringup.md`):

1. **What it is** — 1 kHz capture of drive feedback + commanded target; replacement for the vendor's Windows scope; makes tuning measurable.
2. **Commands** — `SERVO_CAPTURE_START [SERVO=<name>] [NAME=<tag>]`, `SERVO_CAPTURE_STOP`; files land in `~/printer_data/logs/servo_captures/<tag>_<timestamp>.scap`; failed captures are renamed `.failed.scap` and the STOP errors out.
3. **The M400 footgun** — START/STOP execute when the G-code runs; queued moves execute later. Always `M400` before STOP. Include the reference macro:

```
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

4. **Analysis** — `python3 scripts/servo_capture.py <file> [--fft] [--plot] [--settle-band N] [--torque-limit N]`; what each metric means for tuning (following error → gains, FFT peaks → notch filters, settle → damping, torque saturation → feedrate/accel limits); plain-English: the FFT of the error is the machine telling you which frequencies it can't follow — those are the notch candidates.
5. **File format v1** — header JSON line (fields list), then 31-byte LE records; full channel table (name/dtype/offset, the same table as the spec §2); truncated files are valid up to the last whole record; version bumps add channels via the header descriptor.
6. **Drive-side mapping** — TxPDO is the variable 1A00h (10 objects / 40-byte ceiling, currently 10/32 used), rewritten via SDO at every bringup because the drive doesn't persist it; 6062h is the drive's interpolated demand in the same reference units as 6064h/607Ah; bringup fails with rc -6 (SDO write refused) or -7 (mapped size mismatch) rather than running with a corrupt layout.

- [ ] **Step 2: Align the spec with implementation reality**

Edit `docs/superpowers/specs/2026-06-10-servo-telemetry-capture-design.md`:
- §4: `SERVO_CAPTURE START/STOP` → `SERVO_CAPTURE_START` / `SERVO_CAPTURE_STOP`, with one sentence: klippy extended G-code parameters must be KEY=VALUE, a bare `START` does not parse.
- §4: command object lives in `klippy/extras/servo_capture.py` (auto-loaded by `ethercat_node`), not in `ethercat_node.py` itself.
- §4: wire `StartCapture` carries a single `drive_name` for now; `SERVO=a,b` errors in klippy ("not implemented yet") — the file format (header `drives` array) is multi-drive ready, the wire message grows when a multi-slave endpoint exists.
- §6/§7: add a note that new message kinds change `SCHEMA_HASH` (schema_def.rs), so the rollout requires reflashing both MCUs together with the host rebuild.

- [ ] **Step 3: Commit**

```bash
git add docs/kalico-rewrite/servo-telemetry-capture.md docs/superpowers/specs/2026-06-10-servo-telemetry-capture-design.md
git commit -m "docs: servo telemetry capture reference + spec alignment"
```

---

### Task 13: full-suite verification

- [ ] **Step 1: Rust suite**

Run from `rust/`: `cargo nextest run`
Expected: full suite green (~11 s). Also `cargo test --doc` if any doc examples were touched (none planned).

- [ ] **Step 2: hw-feature typecheck**

Run: `cargo check -p kalico-ethercat-rt --features hw`
Expected: clean.

- [ ] **Step 3: Python suite**

Run from repo root: `python -m pytest test/ -x -q`
Expected: green, including pre-existing tests (`test_servo_torque.py` exercises the refactored `set_torque` path indirectly; `test_imports.py` catches syntax errors in the new extras module).

- [ ] **Step 4: Clippy / lints**

Run from `rust/`: `cargo clippy -p kalico-protocol -p kalico-ethercat-rt -p motion-bridge -- -D warnings`
Then: `cargo fmt --all --check` (re-run after any late edit; this is the last gate before push).
Expected: both clean.

- [ ] **Step 5: Commit any stragglers; do NOT merge**

Branch stays on `servo-telemetry` for review + bench validation.

---

### Task 14: bench validation (requires the user at the Neptune bench)

This task is a checklist to hand to the user — motion commands need their per-command approval, and the drive remap can only be proven on hardware.

- [ ] **Step 1: Deploy** — commit → push → pull on the Pi → rebuild (`make -j$(nproc)` for C, cargo for the endpoint + `motion_bridge_native.so`) per the standard bench flow. The MCU schema hash changed: reflash BOTH MCUs (H7 from `.config.h7.bak`, F446 from `.config.f446.test`, `make clean` between builds).
- [ ] **Step 2: Bringup proof** — restart klippy; confirm the endpoint reaches OP. A `-6` exit means the drive refused the 1A00h SDO sequence (check SDO abort codes in stderr); `-7` means the mapped sizes disagree with `in_t`/`out_t` — both are clean failures, not corruption.
- [ ] **Step 3: Feedback sanity** — with torque off, `SERVO_CAPTURE_START NAME=parked` → wait ~5 s → `SERVO_CAPTURE_STOP`. Analyze: position_demand should track position_actual (drive parked, target follows actual); following_error near zero; statusword plausible.
- [ ] **Step 4: Moving capture** — (with user approval for each motion command) run the reference macro; verify samples ≈ wall-clock × 1000, motion segments detected, `--fft` produces a spectrum, drive-vs-recomputed cross-check is small and consistent.
- [ ] **Step 5: Crash survivability spot-check** — start a capture, `kill -9` the endpoint mid-capture, confirm the partial file parses up to the last whole record.

---

## Self-Review Notes

- **Spec coverage:** §1 drive remap → Task 3/4; §2 capture engine → Task 5/7; §3 file format → Task 5 (header/records) + Task 11 (reader); §4 protocol/klippy → Tasks 2/6/9/10; §5 analysis → Task 11; §6 testing → Tasks 5/8/10/11/13/14; §7 docs → Task 12. Spec deviations (command naming, single drive_name on the wire, servo_capture.py placement, schema-hash/reflash) are folded back into the spec in Task 12.
- **Type consistency:** `CaptureConfig`/`CaptureRecord`/`DriveSample`/`StopOutcome` (Task 5) are used by Tasks 7-8; wire structs (Task 2) by Tasks 6/8/9; PyO3 `stop_servo_capture -> (i32, u64, Option<u64>)` matches klippy's 3-tuple unpack (Task 10). Error codes -320..-324 defined once in capture.rs; integration test re-declares the three it asserts on as local consts (test files cannot import another crate's private module state — they CAN import `kalico_ethercat_rt::capture::*`; executor: prefer importing the real constants).
- **Known judgment calls:** `sync_channel` as the SPSC ring (non-blocking `try_send` on the RT side; same process already does heap allocs + eprintln in-loop); unbuffered per-record writes to keep flushed == written for truncation parseability; `started_utc`/path validated host-side, endpoint re-validates JSON safety.
