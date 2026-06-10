# SERVO_PARAM SDO Access Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Raw CoE SDO read/write to EtherCAT servo drives — a declarative `params:` block in `[servo_x]` pushed at claim time, plus a `SERVO_PARAM` console command.

**Architecture:** A new SDO message pair flows through the existing four-layer path: klippy (`servo_param.py` / `ethercat_node.py`) → pyo3 bridge (`motion-bridge`) → Unix-socket wire protocol (`kalico-protocol` + `kalico-ethercat-rt/wire.rs`) → endpoint binary → `libecrt.c` → SOEM `ec_SDOread`/`ec_SDOwrite`. Probe-then-write and write-then-verify run **endpoint-side** as one command (one socket round-trip, no interleaving). The probe/verify logic lives in a new `sdo.rs` module behind an `SdoBus` trait so the hw binary (FFI bus), the stub binary (in-memory dictionary bus), and unit tests all share it.

**Tech Stack:** Rust (workspace at `rust/`, test with `cargo nextest run`), C (`bench/libecrt.c`, SOEM), Python (klippy extras, pytest).

**Spec:** `docs/superpowers/specs/2026-06-10-servo-param-sdo-design.md`

**Conventions that bind every task:**
- Result code convention (everywhere): `0` = success, `> 0` = CoE abort code from the drive, `< 0` = local error constant.
- Values travel host→endpoint as `i64`; the endpoint encodes into the object's byte width (little-endian two's complement). Raw bytes travel endpoint→host as `[u8; 4]` + `size`.
- Message kind numbers `0x0074–0x0077` (the `0x0080–0x00BF` range is reserved for events — `MessageKind::is_event`).
- Fail loudly: no retries, no clamping, no skip-and-continue.
- Commit messages: NO `Co-Authored-By` trailer (user's global rule).

---

### Task 1: Protocol messages (`kalico-protocol`)

**Files:**
- Modify: `rust/kalico-protocol/src/messages.rs`
- Create: `rust/kalico-protocol/src/messages/sdo_tests.rs`

- [ ] **Step 1: Write failing round-trip tests**

Create `rust/kalico-protocol/src/messages/sdo_tests.rs`:

```rust
use super::*;

#[test]
fn sdo_kinds_map_to_u16_and_back() {
    for kind in [
        MessageKind::SdoRead,
        MessageKind::SdoReadResponse,
        MessageKind::SdoWrite,
        MessageKind::SdoWriteResponse,
    ] {
        assert_eq!(MessageKind::from_u16(kind.as_u16()), Some(kind));
        assert!(!kind.is_event(), "SDO kinds must not be in the event range");
    }
}

#[test]
fn sdo_read_roundtrip() {
    let msg = SdoRead {
        index: 0x2002,
        subindex: 3,
    };
    assert_eq!(roundtrip(&msg), msg);
}

#[test]
fn sdo_read_response_roundtrip() {
    let msg = SdoReadResponse {
        result: 0x0601_0002,
        size: 2,
        data: [0x64, 0x00, 0x00, 0x00],
    };
    assert_eq!(roundtrip(&msg), msg);
}

#[test]
fn sdo_write_roundtrip_negative_value() {
    let msg = SdoWrite {
        index: 0x2010,
        subindex: 1,
        size: 4,
        value: -4096,
    };
    assert_eq!(roundtrip(&msg), msg);
}

#[test]
fn sdo_write_response_roundtrip() {
    let msg = SdoWriteResponse {
        result: ERR_SDO_VERIFY_MISMATCH,
        size: 2,
        data: [0xF4, 0x01, 0x00, 0x00],
    };
    assert_eq!(roundtrip(&msg), msg);
}
```

Register the module at the bottom of `rust/kalico-protocol/src/messages.rs` next to the existing test modules:

```rust
#[cfg(test)]
mod sdo_tests;
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd rust && cargo nextest run -p kalico-protocol -E 'test(sdo)'`
Expected: compile error — `SdoRead` etc. not defined.

- [ ] **Step 3: Implement messages**

In `rust/kalico-protocol/src/messages.rs`, add the four variants to the `MessageKind` enum (after `StopResponse = 0x0073,`):

```rust
    SdoRead = 0x0074,
    SdoReadResponse = 0x0075,
    SdoWrite = 0x0076,
    SdoWriteResponse = 0x0077,
```

Add the matching arms in `from_u16` (after the `0x0073` arm):

```rust
            0x0074 => Self::SdoRead,
            0x0075 => Self::SdoReadResponse,
            0x0076 => Self::SdoWrite,
            0x0077 => Self::SdoWriteResponse,
```

Add the local error constants and structs (after the `SetTorqueResponse` impls is a natural spot):

```rust
pub const ERR_SDO_UNSUPPORTED_SIZE: i32 = -801;
pub const ERR_SDO_VERIFY_MISMATCH: i32 = -802;
pub const ERR_SDO_TRANSPORT: i32 = -803;
pub const ERR_SDO_VALUE_RANGE: i32 = -804;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SdoRead {
    pub index: u16,
    pub subindex: u8,
}

impl Encode for SdoRead {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u16(out, self.index);
        put_u8(out, self.subindex);
    }
}

impl Decode for SdoRead {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            index: get_u16(c)?,
            subindex: get_u8(c)?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SdoReadResponse {
    pub result: i32,
    pub size: u8,
    pub data: [u8; 4],
}

impl Encode for SdoReadResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
        put_u8(out, self.size);
        for b in self.data {
            put_u8(out, b);
        }
    }
}

impl Decode for SdoReadResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
            size: get_u8(c)?,
            data: [get_u8(c)?, get_u8(c)?, get_u8(c)?, get_u8(c)?],
        })
    }
}

/// `size == 0` requests an endpoint-side probe (SDO upload discovers the width).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SdoWrite {
    pub index: u16,
    pub subindex: u8,
    pub size: u8,
    pub value: i64,
}

impl Encode for SdoWrite {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u16(out, self.index);
        put_u8(out, self.subindex);
        put_u8(out, self.size);
        put_u64(out, self.value as u64);
    }
}

impl Decode for SdoWrite {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            index: get_u16(c)?,
            subindex: get_u8(c)?,
            size: get_u8(c)?,
            value: get_u64(c)? as i64,
        })
    }
}

/// `size`/`data` carry the post-write readback so a verify mismatch
/// reports what the drive actually settled on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SdoWriteResponse {
    pub result: i32,
    pub size: u8,
    pub data: [u8; 4],
}

impl Encode for SdoWriteResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
        put_u8(out, self.size);
        for b in self.data {
            put_u8(out, b);
        }
    }
}

impl Decode for SdoWriteResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
            size: get_u8(c)?,
            data: [get_u8(c)?, get_u8(c)?, get_u8(c)?, get_u8(c)?],
        })
    }
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd rust && cargo nextest run -p kalico-protocol`
Expected: all PASS (new sdo tests plus all pre-existing).

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-protocol/src/messages.rs rust/kalico-protocol/src/messages/sdo_tests.rs
git commit -m "feat(protocol): SdoRead/SdoWrite message pair with probe and readback semantics"
```

---

### Task 2: SDO executor module (`kalico-ethercat-rt/src/sdo.rs`)

The probe / fit-check / write / readback-verify logic, behind an `SdoBus` trait. `DictSdoBus` (in-memory object dictionary) lives here too — shared by unit tests and the stub binary.

**Files:**
- Create: `rust/kalico-ethercat-rt/src/sdo.rs`
- Create: `rust/kalico-ethercat-rt/src/sdo/tests.rs`
- Modify: `rust/kalico-ethercat-rt/src/lib.rs` (add `pub mod sdo;` to the module list, alphabetical: after `pub mod server;`)

- [ ] **Step 1: Write failing tests**

Create `rust/kalico-ethercat-rt/src/sdo/tests.rs`:

```rust
use super::*;
use kalico_protocol::messages::{
    SdoRead, SdoWrite, ERR_SDO_UNSUPPORTED_SIZE, ERR_SDO_VALUE_RANGE, ERR_SDO_VERIFY_MISMATCH,
};

fn test_dict() -> DictSdoBus {
    DictSdoBus::new(vec![
        (
            (0x2002, 0),
            DictObject {
                size: 2,
                value: [100, 0, 0, 0],
                read_only: false,
                clamp_max: None,
            },
        ),
        (
            (0x2003, 0),
            DictObject {
                size: 2,
                value: [0, 0, 0, 0],
                read_only: false,
                clamp_max: Some(500),
            },
        ),
        (
            (0x2010, 1),
            DictObject {
                size: 4,
                value: [0; 4],
                read_only: false,
                clamp_max: None,
            },
        ),
        (
            (0x6041, 0),
            DictObject {
                size: 2,
                value: [0x37, 0x02, 0, 0],
                read_only: true,
                clamp_max: None,
            },
        ),
    ])
}

#[test]
fn read_returns_size_and_bytes() {
    let mut bus = test_dict();
    let resp = execute_sdo_read(
        &mut bus,
        &SdoRead {
            index: 0x2002,
            subindex: 0,
        },
    );
    assert_eq!(resp.result, 0);
    assert_eq!(resp.size, 2);
    assert_eq!(resp.data, [100, 0, 0, 0]);
}

#[test]
fn read_unknown_object_returns_abort_code() {
    let mut bus = test_dict();
    let resp = execute_sdo_read(
        &mut bus,
        &SdoRead {
            index: 0x7777,
            subindex: 0,
        },
    );
    assert_eq!(resp.result, COE_ABORT_NOT_FOUND);
}

#[test]
fn probed_write_discovers_size_writes_and_verifies() {
    let mut bus = test_dict();
    let resp = execute_sdo_write(
        &mut bus,
        &SdoWrite {
            index: 0x2002,
            subindex: 0,
            size: 0,
            value: 250,
        },
    );
    assert_eq!(resp.result, 0);
    assert_eq!(resp.size, 2);
    assert_eq!(resp.data, [250, 0, 0, 0]);
    assert_eq!(bus.read_count, 2, "probe + verify");
}

#[test]
fn typed_write_skips_probe() {
    let mut bus = test_dict();
    let resp = execute_sdo_write(
        &mut bus,
        &SdoWrite {
            index: 0x2002,
            subindex: 0,
            size: 2,
            value: 250,
        },
    );
    assert_eq!(resp.result, 0);
    assert_eq!(bus.read_count, 1, "verify only — no probe");
}

#[test]
fn negative_value_encodes_twos_complement() {
    let mut bus = test_dict();
    let resp = execute_sdo_write(
        &mut bus,
        &SdoWrite {
            index: 0x2010,
            subindex: 1,
            size: 4,
            value: -4096,
        },
    );
    assert_eq!(resp.result, 0);
    assert_eq!(resp.data, (-4096i32).to_le_bytes());
}

#[test]
fn value_exceeding_discovered_width_is_rejected_before_writing() {
    let mut bus = test_dict();
    let resp = execute_sdo_write(
        &mut bus,
        &SdoWrite {
            index: 0x2002,
            subindex: 0,
            size: 0,
            value: 70_000,
        },
    );
    assert_eq!(resp.result, ERR_SDO_VALUE_RANGE);
    let after = execute_sdo_read(
        &mut bus,
        &SdoRead {
            index: 0x2002,
            subindex: 0,
        },
    );
    assert_eq!(after.data, [100, 0, 0, 0], "object must be untouched");
}

#[test]
fn clamped_write_reports_verify_mismatch_with_settled_bytes() {
    let mut bus = test_dict();
    let resp = execute_sdo_write(
        &mut bus,
        &SdoWrite {
            index: 0x2003,
            subindex: 0,
            size: 2,
            value: 600,
        },
    );
    assert_eq!(resp.result, ERR_SDO_VERIFY_MISMATCH);
    assert_eq!(resp.size, 2);
    assert_eq!(resp.data, [0xF4, 0x01, 0, 0], "drive settled on 500");
}

#[test]
fn read_only_object_write_surfaces_abort_code() {
    let mut bus = test_dict();
    let resp = execute_sdo_write(
        &mut bus,
        &SdoWrite {
            index: 0x6041,
            subindex: 0,
            size: 2,
            value: 1,
        },
    );
    assert_eq!(resp.result, COE_ABORT_READ_ONLY);
}

#[test]
fn probe_reporting_oversized_object_is_rejected() {
    struct BigObjectBus;
    impl SdoBus for BigObjectBus {
        fn read(&mut self, _index: u16, _subindex: u8) -> Result<(u8, [u8; 4]), i32> {
            Ok((8, [0; 4]))
        }
        fn write(&mut self, _index: u16, _subindex: u8, _bytes: &[u8]) -> Result<(), i32> {
            panic!("must not write an unsupported-size object");
        }
    }
    let resp = execute_sdo_write(
        &mut BigObjectBus,
        &SdoWrite {
            index: 0x1008,
            subindex: 0,
            size: 0,
            value: 1,
        },
    );
    assert_eq!(resp.result, ERR_SDO_UNSUPPORTED_SIZE);
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd rust && cargo nextest run -p kalico-ethercat-rt -E 'test(sdo)'`
Expected: compile error — module `sdo` not found.

- [ ] **Step 3: Implement `sdo.rs`**

Create `rust/kalico-ethercat-rt/src/sdo.rs`:

```rust
use std::collections::BTreeMap;

use kalico_protocol::messages::{
    SdoRead, SdoReadResponse, SdoWrite, SdoWriteResponse, ERR_SDO_UNSUPPORTED_SIZE,
    ERR_SDO_VALUE_RANGE, ERR_SDO_VERIFY_MISMATCH,
};

pub const MAX_SDO_BYTES: u8 = 4;
pub const COE_ABORT_READ_ONLY: i32 = 0x0601_0002;
pub const COE_ABORT_NOT_FOUND: i32 = 0x0602_0000;
pub const COE_ABORT_LENGTH_MISMATCH: i32 = 0x0607_0010;

/// Errors are result codes: > 0 CoE abort code, < 0 local ERR_SDO_* constant.
/// `read` must only report sizes 1..=4 packed little-endian into the array
/// (a larger reported size means the object is unsupported, not truncated data).
pub trait SdoBus {
    fn read(&mut self, index: u16, subindex: u8) -> Result<(u8, [u8; 4]), i32>;
    fn write(&mut self, index: u16, subindex: u8, bytes: &[u8]) -> Result<(), i32>;
}

pub fn execute_sdo_read(bus: &mut dyn SdoBus, msg: &SdoRead) -> SdoReadResponse {
    match bus.read(msg.index, msg.subindex) {
        Ok((size, data)) => SdoReadResponse {
            result: 0,
            size,
            data,
        },
        Err(code) => SdoReadResponse {
            result: code,
            size: 0,
            data: [0; 4],
        },
    }
}

fn value_fits(value: i64, size: u8) -> bool {
    let bits = u32::from(size) * 8;
    let min = -(1i64 << (bits - 1));
    let max = (1i64 << bits) - 1;
    (min..=max).contains(&value)
}

fn encode_value(value: i64, size: u8) -> [u8; 4] {
    let le = value.to_le_bytes();
    let mut out = [0u8; 4];
    out[..usize::from(size)].copy_from_slice(&le[..usize::from(size)]);
    out
}

pub fn execute_sdo_write(bus: &mut dyn SdoBus, msg: &SdoWrite) -> SdoWriteResponse {
    let fail = |result| SdoWriteResponse {
        result,
        size: 0,
        data: [0; 4],
    };
    let size = if msg.size == 0 {
        match bus.read(msg.index, msg.subindex) {
            Ok((probed, _)) => probed,
            Err(code) => return fail(code),
        }
    } else {
        msg.size
    };
    if size == 0 || size > MAX_SDO_BYTES {
        return fail(ERR_SDO_UNSUPPORTED_SIZE);
    }
    if !value_fits(msg.value, size) {
        return fail(ERR_SDO_VALUE_RANGE);
    }
    let bytes = encode_value(msg.value, size);
    if let Err(code) = bus.write(msg.index, msg.subindex, &bytes[..usize::from(size)]) {
        return fail(code);
    }
    match bus.read(msg.index, msg.subindex) {
        Ok((rb_size, rb_data)) => {
            if rb_size == size && rb_data == bytes {
                SdoWriteResponse {
                    result: 0,
                    size,
                    data: bytes,
                }
            } else {
                SdoWriteResponse {
                    result: ERR_SDO_VERIFY_MISMATCH,
                    size: rb_size,
                    data: rb_data,
                }
            }
        }
        Err(code) => fail(code),
    }
}

pub struct DictObject {
    pub size: u8,
    pub value: [u8; 4],
    pub read_only: bool,
    pub clamp_max: Option<u32>,
}

/// In-memory object dictionary: the stub endpoint's fake drive, and the
/// unit-test bus. `read_count` exposes probe/verify traffic for assertions.
pub struct DictSdoBus {
    objects: BTreeMap<(u16, u8), DictObject>,
    pub read_count: u32,
}

impl DictSdoBus {
    pub fn new(objects: Vec<((u16, u8), DictObject)>) -> Self {
        Self {
            objects: objects.into_iter().collect(),
            read_count: 0,
        }
    }
}

impl SdoBus for DictSdoBus {
    fn read(&mut self, index: u16, subindex: u8) -> Result<(u8, [u8; 4]), i32> {
        self.read_count += 1;
        match self.objects.get(&(index, subindex)) {
            Some(o) => Ok((o.size, o.value)),
            None => Err(COE_ABORT_NOT_FOUND),
        }
    }

    fn write(&mut self, index: u16, subindex: u8, bytes: &[u8]) -> Result<(), i32> {
        let o = self
            .objects
            .get_mut(&(index, subindex))
            .ok_or(COE_ABORT_NOT_FOUND)?;
        if o.read_only {
            return Err(COE_ABORT_READ_ONLY);
        }
        if bytes.len() != usize::from(o.size) {
            return Err(COE_ABORT_LENGTH_MISMATCH);
        }
        let mut v = [0u8; 4];
        v[..bytes.len()].copy_from_slice(bytes);
        if let Some(max) = o.clamp_max {
            if u32::from_le_bytes(v) > max {
                v = max.to_le_bytes();
            }
        }
        o.value = v;
        Ok(())
    }
}

#[cfg(test)]
mod tests;
```

Add to `rust/kalico-ethercat-rt/src/lib.rs` after `pub mod scale;`:

```rust
pub mod sdo;
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd rust && cargo nextest run -p kalico-ethercat-rt`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-ethercat-rt/src/sdo.rs rust/kalico-ethercat-rt/src/sdo/tests.rs rust/kalico-ethercat-rt/src/lib.rs
git commit -m "feat(ethercat-rt): SDO executor with probe, fit-check, and readback verify behind SdoBus trait"
```

---

### Task 3: Wire codec (`kalico-ethercat-rt/src/wire.rs`)

**Files:**
- Modify: `rust/kalico-ethercat-rt/src/wire.rs`
- Modify: `rust/kalico-ethercat-rt/src/wire/tests.rs`

- [ ] **Step 1: Write failing tests**

Append to `rust/kalico-ethercat-rt/src/wire/tests.rs`:

```rust
#[test]
fn decodes_sdo_read_command() {
    let msg = SdoRead {
        index: 0x2002,
        subindex: 1,
    };
    let payload = frame_payload(MessageKind::SdoRead, 9, &msg.encoded_to_vec());
    match decode_command(0, &payload).unwrap() {
        Command::SdoRead {
            correlation_id: 9,
            msg: m,
        } => assert_eq!(m, msg),
        other => panic!("wrong variant: {other:?}"),
    }
}

#[test]
fn decodes_sdo_write_command() {
    let msg = SdoWrite {
        index: 0x2003,
        subindex: 0,
        size: 0,
        value: -42,
    };
    let payload = frame_payload(MessageKind::SdoWrite, 10, &msg.encoded_to_vec());
    match decode_command(0, &payload).unwrap() {
        Command::SdoWrite {
            correlation_id: 10,
            msg: m,
        } => assert_eq!(m, msg),
        other => panic!("wrong variant: {other:?}"),
    }
}

#[test]
fn sdo_response_frames_decode_back() {
    let frame = sdo_read_response_frame(
        11,
        &SdoReadResponse {
            result: 0,
            size: 2,
            data: [0x64, 0, 0, 0],
        },
    );
    let (chan, payload) = decode_frame(&frame).unwrap();
    assert_eq!(chan, CHANNEL_CONTROL);
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(hdr.correlation_id, 11);
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::SdoReadResponse)
    );
    let r = SdoReadResponse::decode(body).unwrap();
    assert_eq!((r.result, r.size, r.data), (0, 2, [0x64, 0, 0, 0]));

    let frame = sdo_write_response_frame(
        12,
        &SdoWriteResponse {
            result: -802,
            size: 2,
            data: [0xF4, 0x01, 0, 0],
        },
    );
    let (chan, payload) = decode_frame(&frame).unwrap();
    assert_eq!(chan, CHANNEL_CONTROL);
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(hdr.correlation_id, 12);
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::SdoWriteResponse)
    );
    let r = SdoWriteResponse::decode(body).unwrap();
    assert_eq!((r.result, r.size, r.data), (-802, 2, [0xF4, 0x01, 0, 0]));
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd rust && cargo nextest run -p kalico-ethercat-rt -E 'test(sdo)'`
Expected: compile error — `Command::SdoRead` and the frame builders not defined.

- [ ] **Step 3: Implement codec entries**

In `rust/kalico-ethercat-rt/src/wire.rs`:

Extend the `kalico_protocol::messages` import list with `SdoRead, SdoReadResponse, SdoWrite, SdoWriteResponse`.

Add to the `Command` enum (after `SetTorque`):

```rust
    SdoRead {
        correlation_id: u32,
        msg: SdoRead,
    },
    SdoWrite {
        correlation_id: u32,
        msg: SdoWrite,
    },
```

Add decode arms in `decode_command` (after the `SetTorque` arm):

```rust
        Some(MessageKind::SdoRead) => {
            let msg = SdoRead::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
            Ok(Command::SdoRead {
                correlation_id: cid,
                msg,
            })
        }
        Some(MessageKind::SdoWrite) => {
            let msg = SdoWrite::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
            Ok(Command::SdoWrite {
                correlation_id: cid,
                msg,
            })
        }
```

Add frame builders (after `set_torque_response_frame`):

```rust
pub fn sdo_read_response_frame(cid: u32, resp: &SdoReadResponse) -> Vec<u8> {
    control_frame(MessageKind::SdoReadResponse, cid, &resp.encoded_to_vec())
}

pub fn sdo_write_response_frame(cid: u32, resp: &SdoWriteResponse) -> Vec<u8> {
    control_frame(MessageKind::SdoWriteResponse, cid, &resp.encoded_to_vec())
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd rust && cargo nextest run -p kalico-ethercat-rt`
Expected: PASS — including a compile check that the two endpoint binaries' exhaustive `match cmd` still compile. They will NOT compile yet (non-exhaustive match on new variants) — fix is Task 5, so for THIS task expect: `cargo nextest run -p kalico-ethercat-rt` passes (lib + stub-less tests) but the stub binary fails to build. If the stub binary breaks the test build (integration tests depend on `CARGO_BIN_EXE_kalico-ethercat-rt-stub`), add the minimal placeholder arms to BOTH binaries now:

In `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt-stub.rs` and `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs`, inside the `match cmd`:

```rust
                Command::SdoRead { .. } | Command::SdoWrite { .. } => {
                    todo!("wired in the endpoint task")
                }
```

(These are replaced with real handlers in Task 5; `todo!` keeps failure loud if reached early.)

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-ethercat-rt/src/wire.rs rust/kalico-ethercat-rt/src/wire/tests.rs rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt-stub.rs rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs
git commit -m "feat(ethercat-rt): wire codec for SdoRead/SdoWrite commands and response frames"
```

---

### Task 4: C FFI (`bench/libecrt.c` + `ffi.rs`)

SOEM wrappers. NOTE: the C side and the `hw`-feature link cannot be exercised on this Mac (SOEM lives on the Pi). Local verification is `cargo check` (no link) — final C compile happens through the bench flow (commit → push → pull → build on Pi), per the bench-iteration rule.

**Files:**
- Modify: `bench/libecrt.h`
- Modify: `bench/libecrt.c`
- Modify: `rust/kalico-ethercat-rt/src/ffi.rs`

- [ ] **Step 1: Declare in `bench/libecrt.h`**

Add before `#endif`:

```c
/* SDO upload from slave 1. On entry *size is the buffer capacity; on success
 * it holds the object's byte count. Returns 0 on success, -1 on failure with
 * *abort_code holding the CoE abort code (0 = transport-level failure). */
int ec_rt_sdo_read(uint16_t index, uint8_t sub, uint8_t *buf, int *size,
                   uint32_t *abort_code);

/* SDO download to slave 1. Same return/abort_code convention. */
int ec_rt_sdo_write(uint16_t index, uint8_t sub, const uint8_t *buf, int size,
                    uint32_t *abort_code);
```

- [ ] **Step 2: Implement in `bench/libecrt.c`**

Add near the other `ec_rt_*` functions:

```c
static uint32_t ec_rt_pop_abort_code(void) {
    uint32_t code = 0;
    while (ec_iserror()) {
        ec_errort err;
        if (!ec_poperror(&err)) break;
        if (err.Etype == EC_ERR_TYPE_SDO_ERROR) code = err.AbortCode;
    }
    return code;
}

int ec_rt_sdo_read(uint16_t index, uint8_t sub, uint8_t *buf, int *size,
                   uint32_t *abort_code) {
    *abort_code = 0;
    int wkc = ec_SDOread(1, index, sub, FALSE, size, buf, EC_TIMEOUTRXM);
    if (wkc <= 0) {
        *abort_code = ec_rt_pop_abort_code();
        return -1;
    }
    return 0;
}

int ec_rt_sdo_write(uint16_t index, uint8_t sub, const uint8_t *buf, int size,
                    uint32_t *abort_code) {
    *abort_code = 0;
    int wkc = ec_SDOwrite(1, index, sub, FALSE, size, (void *)buf,
                          EC_TIMEOUTRXM);
    if (wkc <= 0) {
        *abort_code = ec_rt_pop_abort_code();
        return -1;
    }
    return 0;
}
```

- [ ] **Step 3: Declare in `rust/kalico-ethercat-rt/src/ffi.rs`**

Add inside the `extern "C"` block (after `ec_rt_get_following_error`):

```rust
    pub fn ec_rt_sdo_read(
        index: u16,
        sub: u8,
        buf: *mut u8,
        size: *mut c_int,
        abort_code: *mut u32,
    ) -> c_int;

    pub fn ec_rt_sdo_write(
        index: u16,
        sub: u8,
        buf: *const u8,
        size: c_int,
        abort_code: *mut u32,
    ) -> c_int;
```

- [ ] **Step 4: Verify it compiles (no link)**

Run: `cd rust && cargo check -p kalico-ethercat-rt --features hw`
Expected: clean check (cargo check does not link, so missing libecrt.a locally is fine).

- [ ] **Step 5: Commit**

```bash
git add bench/libecrt.h bench/libecrt.c rust/kalico-ethercat-rt/src/ffi.rs
git commit -m "feat(ethercat): ec_rt_sdo_read/write SOEM wrappers with CoE abort code extraction"
```

---

### Task 5: Endpoint binaries — hw FFI bus + stub dictionary bus

**Files:**
- Modify: `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs`
- Modify: `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt-stub.rs`

- [ ] **Step 1: Wire the hw binary**

In `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs`:

Add imports:

```rust
use kalico_ethercat_rt::sdo::{execute_sdo_read, execute_sdo_write, SdoBus};
use kalico_ethercat_rt::wire::{sdo_read_response_frame, sdo_write_response_frame};
use kalico_protocol::messages::{ERR_SDO_TRANSPORT, ERR_SDO_UNSUPPORTED_SIZE};
```

(merge into the existing `use kalico_ethercat_rt::wire::{...}` list rather than a second import).

Add the FFI bus after `fn arg_val` (8-byte read buffer so an oversized object is detected as such, not as a transport failure):

```rust
struct FfiSdoBus;

impl SdoBus for FfiSdoBus {
    fn read(&mut self, index: u16, subindex: u8) -> Result<(u8, [u8; 4]), i32> {
        let mut buf = [0u8; 8];
        let mut size: std::os::raw::c_int = buf.len() as std::os::raw::c_int;
        let mut abort: u32 = 0;
        let rc = unsafe {
            ffi::ec_rt_sdo_read(index, subindex, buf.as_mut_ptr(), &mut size, &mut abort)
        };
        if rc != 0 {
            return Err(if abort != 0 {
                abort as i32
            } else {
                ERR_SDO_TRANSPORT
            });
        }
        if !(1..=4).contains(&size) {
            return Err(ERR_SDO_UNSUPPORTED_SIZE);
        }
        let mut data = [0u8; 4];
        data[..size as usize].copy_from_slice(&buf[..size as usize]);
        Ok((size as u8, data))
    }

    fn write(&mut self, index: u16, subindex: u8, bytes: &[u8]) -> Result<(), i32> {
        let mut abort: u32 = 0;
        let rc = unsafe {
            ffi::ec_rt_sdo_write(
                index,
                subindex,
                bytes.as_ptr(),
                bytes.len() as std::os::raw::c_int,
                &mut abort,
            )
        };
        if rc != 0 {
            return Err(if abort != 0 {
                abort as i32
            } else {
                ERR_SDO_TRANSPORT
            });
        }
        Ok(())
    }
}
```

Add `let mut sdo_bus = FfiSdoBus;` next to `let mut gate = TorqueGate::new();` before the `'dc` loop. Replace the Task-3 placeholder arms in the `match cmd` with:

```rust
                Command::SdoRead {
                    correlation_id,
                    msg,
                } => {
                    let resp = execute_sdo_read(&mut sdo_bus, &msg);
                    if resp.result != 0 {
                        eprintln!(
                            "ec-rt: SdoRead 0x{:04x}.{} failed result={}",
                            msg.index, msg.subindex, resp.result
                        );
                    }
                    server.respond(&sdo_read_response_frame(correlation_id, &resp));
                }
                Command::SdoWrite {
                    correlation_id,
                    msg,
                } => {
                    let resp = execute_sdo_write(&mut sdo_bus, &msg);
                    if resp.result != 0 {
                        eprintln!(
                            "ec-rt: SdoWrite 0x{:04x}.{} value={} size={} failed result={}",
                            msg.index, msg.subindex, msg.value, msg.size, resp.result
                        );
                    }
                    server.respond(&sdo_write_response_frame(correlation_id, &resp));
                }
```

- [ ] **Step 2: Wire the stub binary**

In `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt-stub.rs`:

Add imports (merged into existing lists):

```rust
use kalico_ethercat_rt::sdo::{execute_sdo_read, execute_sdo_write, DictObject, DictSdoBus};
use kalico_ethercat_rt::wire::{sdo_read_response_frame, sdo_write_response_frame};
use kalico_protocol::messages::SdoReadResponse;
```

Add after `fn arg_val`:

```rust
const STUB_PROBE_COUNTER_INDEX: u16 = 0x5FFF;

fn stub_object_dictionary() -> DictSdoBus {
    DictSdoBus::new(vec![
        (
            (0x2002, 0),
            DictObject {
                size: 2,
                value: [100, 0, 0, 0],
                read_only: false,
                clamp_max: None,
            },
        ),
        (
            (0x2003, 0),
            DictObject {
                size: 2,
                value: [0, 0, 0, 0],
                read_only: false,
                clamp_max: Some(500),
            },
        ),
        (
            (0x2010, 1),
            DictObject {
                size: 4,
                value: [0; 4],
                read_only: false,
                clamp_max: None,
            },
        ),
        (
            (0x6041, 0),
            DictObject {
                size: 2,
                value: [0x37, 0x02, 0, 0],
                read_only: true,
                clamp_max: None,
            },
        ),
    ])
}
```

Add `let mut sdo_bus = stub_object_dictionary();` next to `let mut gate = TorqueGate::new();`. Replace the Task-3 placeholder arms with (the `0x5FFF.0` intercept exposes the dictionary-read count to integration tests without inflating it):

```rust
                Command::SdoRead {
                    correlation_id,
                    msg,
                } => {
                    let resp = if msg.index == STUB_PROBE_COUNTER_INDEX {
                        SdoReadResponse {
                            result: 0,
                            size: 4,
                            data: sdo_bus.read_count.to_le_bytes(),
                        }
                    } else {
                        execute_sdo_read(&mut sdo_bus, &msg)
                    };
                    server.respond(&sdo_read_response_frame(correlation_id, &resp));
                }
                Command::SdoWrite {
                    correlation_id,
                    msg,
                } => {
                    let resp = execute_sdo_write(&mut sdo_bus, &msg);
                    eprintln!(
                        "ec-rt-stub: SdoWrite 0x{:04x}.{} value={} size={} -> result={}",
                        msg.index, msg.subindex, msg.value, msg.size, resp.result
                    );
                    server.respond(&sdo_write_response_frame(correlation_id, &resp));
                }
```

- [ ] **Step 3: Verify both binaries build**

Run: `cd rust && cargo check -p kalico-ethercat-rt --features hw && cargo nextest run -p kalico-ethercat-rt`
Expected: clean check; all tests PASS.

- [ ] **Step 4: Commit**

```bash
git add rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt-stub.rs
git commit -m "feat(ethercat-rt): SDO command handling in hw endpoint (FFI bus) and stub (object dictionary)"
```

---

### Task 6: Stub integration test (`tests/sdo_lifecycle.rs`)

**Files:**
- Create: `rust/kalico-ethercat-rt/tests/sdo_lifecycle.rs`

- [ ] **Step 1: Write the test** (helpers mirror `tests/torque_lifecycle.rs`, which cannot be shared across integration-test binaries)

```rust
use std::process::{Child, Command};
use std::thread;
use std::time::{Duration, Instant};

use kalico_host_rt::native_call::NativeCall;
use kalico_host_rt::unix_native_conn::UnixNativeConn;
use kalico_protocol::codec::{Decode, Encode};
use kalico_protocol::messages::{
    MessageKind, SdoRead, SdoReadResponse, SdoWrite, SdoWriteResponse, ERR_SDO_VALUE_RANGE,
    ERR_SDO_VERIFY_MISMATCH,
};

const STUB_BIN: &str = env!("CARGO_BIN_EXE_kalico-ethercat-rt-stub");
const COE_ABORT_READ_ONLY: i32 = 0x0601_0002;
const COE_ABORT_NOT_FOUND: i32 = 0x0602_0000;

struct ChildGuard {
    child: Option<Child>,
}

impl Drop for ChildGuard {
    fn drop(&mut self) {
        if let Some(mut c) = self.child.take() {
            let _ = c.kill();
            let _ = c.wait();
        }
    }
}

fn spawn_and_claim(tag: &str) -> (ChildGuard, UnixNativeConn) {
    let path = format!("/tmp/kalico-sdo-{}-{}.sock", tag, std::process::id());
    let _ = std::fs::remove_file(&path);
    let child = Command::new(STUB_BIN)
        .args(["--socket", &path])
        .spawn()
        .expect("stub binary must spawn");
    let guard = ChildGuard { child: Some(child) };

    let deadline = Instant::now() + Duration::from_secs(5);
    while !std::path::Path::new(&path).exists() {
        assert!(Instant::now() < deadline, "stub socket did not appear");
        thread::sleep(Duration::from_millis(10));
    }
    let conn = UnixNativeConn::connect(&path).expect("connect must succeed");
    let (kind, _) = conn
        .kalico_call(
            MessageKind::ClaimHandshake,
            Vec::new(),
            Duration::from_secs(5),
        )
        .expect("ClaimHandshake must succeed");
    assert_eq!(kind, MessageKind::ClaimHandshakeReply);
    (guard, conn)
}

fn sdo_read(conn: &UnixNativeConn, index: u16, subindex: u8) -> SdoReadResponse {
    let body = SdoRead { index, subindex }.encoded_to_vec();
    let (kind, resp) = conn
        .kalico_call(MessageKind::SdoRead, body, Duration::from_secs(5))
        .expect("SdoRead call must succeed");
    assert_eq!(kind, MessageKind::SdoReadResponse);
    SdoReadResponse::decode(&resp).expect("SdoReadResponse must decode")
}

fn sdo_write(
    conn: &UnixNativeConn,
    index: u16,
    subindex: u8,
    size: u8,
    value: i64,
) -> SdoWriteResponse {
    let body = SdoWrite {
        index,
        subindex,
        size,
        value,
    }
    .encoded_to_vec();
    let (kind, resp) = conn
        .kalico_call(MessageKind::SdoWrite, body, Duration::from_secs(5))
        .expect("SdoWrite call must succeed");
    assert_eq!(kind, MessageKind::SdoWriteResponse);
    SdoWriteResponse::decode(&resp).expect("SdoWriteResponse must decode")
}

fn probe_count(conn: &UnixNativeConn) -> u32 {
    let r = sdo_read(conn, 0x5FFF, 0);
    assert_eq!(r.result, 0);
    u32::from_le_bytes(r.data)
}

#[test]
fn read_returns_preloaded_value() {
    let (_guard, conn) = spawn_and_claim("read");
    let r = sdo_read(&conn, 0x2002, 0);
    assert_eq!((r.result, r.size, r.data), (0, 2, [100, 0, 0, 0]));
}

#[test]
fn typed_write_skips_probe_untyped_probes() {
    let (_guard, conn) = spawn_and_claim("probe");
    let before = probe_count(&conn);
    let r = sdo_write(&conn, 0x2002, 0, 2, 250);
    assert_eq!(r.result, 0);
    let after_typed = probe_count(&conn);
    assert_eq!(after_typed - before, 1, "typed write: verify read only");
    let r = sdo_write(&conn, 0x2002, 0, 0, 300);
    assert_eq!(r.result, 0);
    assert_eq!(r.data, [44, 1, 0, 0]);
    let after_untyped = probe_count(&conn);
    assert_eq!(after_untyped - after_typed, 2, "untyped write: probe + verify");
}

#[test]
fn clamping_object_fails_verify_with_settled_value() {
    let (_guard, conn) = spawn_and_claim("clamp");
    let r = sdo_write(&conn, 0x2003, 0, 2, 600);
    assert_eq!(r.result, ERR_SDO_VERIFY_MISMATCH);
    assert_eq!((r.size, r.data), (2, [0xF4, 0x01, 0, 0]));
}

#[test]
fn read_only_and_unknown_objects_surface_abort_codes() {
    let (_guard, conn) = spawn_and_claim("abort");
    assert_eq!(sdo_write(&conn, 0x6041, 0, 2, 1).result, COE_ABORT_READ_ONLY);
    assert_eq!(sdo_read(&conn, 0x7777, 0).result, COE_ABORT_NOT_FOUND);
    assert_eq!(
        sdo_write(&conn, 0x2002, 0, 0, 70_000).result,
        ERR_SDO_VALUE_RANGE
    );
}
```

- [ ] **Step 2: Run the test**

Run: `cd rust && cargo nextest run -p kalico-ethercat-rt -E 'binary(sdo_lifecycle)'`
Expected: 4 tests PASS.

- [ ] **Step 3: Commit**

```bash
git add rust/kalico-ethercat-rt/tests/sdo_lifecycle.rs
git commit -m "test(ethercat-rt): SDO lifecycle against stub object dictionary"
```

---

### Task 7: Bridge — `servo_sdo.rs`, pyo3 methods, klippy wrapper

**Files:**
- Create: `rust/motion-bridge/src/servo_sdo.rs`
- Create: `rust/motion-bridge/src/servo_sdo/tests.rs`
- Modify: `rust/motion-bridge/src/lib.rs` (or wherever sibling `pub mod servo_torque;` is declared — add `pub mod servo_sdo;` next to it)
- Modify: `rust/motion-bridge/src/bridge.rs`
- Modify: `klippy/motion_bridge.py`

- [ ] **Step 1: Write failing test for the failure-text mapping**

Create `rust/motion-bridge/src/servo_sdo/tests.rs`:

```rust
use super::*;

#[test]
fn failure_text_maps_codes() {
    assert!(failure_text(0x0601_0002).contains("CoE abort 0x06010002"));
    assert!(failure_text(ERR_SDO_VERIFY_MISMATCH).contains("readback mismatch"));
    assert!(failure_text(ERR_SDO_UNSUPPORTED_SIZE).contains("size"));
    assert!(failure_text(ERR_SDO_TRANSPORT).contains("transport"));
    assert!(failure_text(ERR_SDO_VALUE_RANGE).contains("does not fit"));
}
```

- [ ] **Step 2: Implement `servo_sdo.rs`**

```rust
use std::time::Duration;

use kalico_host_rt::native_call::NativeCall as _;
use kalico_host_rt::unix_native_conn::UnixNativeConn;
use kalico_protocol::codec::{Decode as _, Encode as _};
use kalico_protocol::messages::{
    MessageKind, SdoRead, SdoReadResponse, SdoWrite, SdoWriteResponse, ERR_SDO_TRANSPORT,
    ERR_SDO_UNSUPPORTED_SIZE, ERR_SDO_VALUE_RANGE, ERR_SDO_VERIFY_MISMATCH,
};

const SDO_TIMEOUT: Duration = Duration::from_secs(5);

pub fn send_sdo_read(
    conn: &UnixNativeConn,
    index: u16,
    subindex: u8,
) -> Result<SdoReadResponse, String> {
    let body = SdoRead { index, subindex }.encoded_to_vec();
    let (kind, resp) = conn
        .kalico_call(MessageKind::SdoRead, body, SDO_TIMEOUT)
        .map_err(|e| format!("SdoRead transport: {e:?}"))?;
    if kind != MessageKind::SdoReadResponse {
        return Err(format!(
            "SdoRead: unexpected response kind 0x{:04x}",
            kind.as_u16()
        ));
    }
    SdoReadResponse::decode(&resp).map_err(|e| format!("SdoReadResponse decode: {e:?}"))
}

pub fn send_sdo_write(
    conn: &UnixNativeConn,
    index: u16,
    subindex: u8,
    size: u8,
    value: i64,
) -> Result<SdoWriteResponse, String> {
    let body = SdoWrite {
        index,
        subindex,
        size,
        value,
    }
    .encoded_to_vec();
    let (kind, resp) = conn
        .kalico_call(MessageKind::SdoWrite, body, SDO_TIMEOUT)
        .map_err(|e| format!("SdoWrite transport: {e:?}"))?;
    if kind != MessageKind::SdoWriteResponse {
        return Err(format!(
            "SdoWrite: unexpected response kind 0x{:04x}",
            kind.as_u16()
        ));
    }
    SdoWriteResponse::decode(&resp).map_err(|e| format!("SdoWriteResponse decode: {e:?}"))
}

pub fn failure_text(result: i32) -> String {
    match result {
        ERR_SDO_UNSUPPORTED_SIZE => "object size unsupported (must be 1..=4 bytes)".into(),
        ERR_SDO_VERIFY_MISMATCH => "readback mismatch".into(),
        ERR_SDO_TRANSPORT => "SDO transport failure (no CoE abort code)".into(),
        ERR_SDO_VALUE_RANGE => "value does not fit the object width".into(),
        code if code > 0 => format!("CoE abort 0x{:08x}", code as u32),
        code => format!("endpoint error {code}"),
    }
}

#[cfg(test)]
mod tests;
```

Declare the module next to `servo_torque` in the crate root (check `rust/motion-bridge/src/lib.rs` for `pub mod servo_torque;` / `mod servo_torque;` and match its visibility):

```rust
pub mod servo_sdo;
```

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(failure_text)'`
Expected: PASS.

- [ ] **Step 3: Add pyo3 methods to `bridge.rs`**

First, in the `#[pymethods]` impl, refactor the connection lookup out of `set_torque` (the `let conn = { ... }` block at `bridge.rs:820-831`) and replace it with `let conn = self.ethercat_conn(mcu_handle, "set_torque")?;`. Add a plain (non-pymethods) impl block near the pymethods impl:

```rust
impl MotionBridge {
    fn ethercat_conn(
        &self,
        mcu_handle: u32,
        what: &str,
    ) -> PyResult<Arc<kalico_host_rt::unix_native_conn::UnixNativeConn>> {
        let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
        let mc = mcus.get(&mcu_handle).ok_or_else(|| {
            PyRuntimeError::new_err(format!("{what}: unknown mcu_handle {mcu_handle}"))
        })?;
        mc.endpoint_conn.clone().ok_or_else(|| {
            PyRuntimeError::new_err(format!(
                "{what}: mcu {mcu_handle} ({}) is not an EtherCAT endpoint",
                mc.label
            ))
        })
    }
}
```

(If `MotionBridge` already has a plain impl block, add the method there. Match the actual struct name and `Arc` import style used in the file.)

Then add to the `#[pymethods]` impl (after `set_torque`):

```rust
    fn sdo_read(&self, mcu_handle: u32, index: u16, subindex: u8) -> PyResult<(u8, u32)> {
        let conn = self.ethercat_conn(mcu_handle, "sdo_read")?;
        let r = crate::servo_sdo::send_sdo_read(&conn, index, subindex)
            .map_err(PyRuntimeError::new_err)?;
        if r.result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "SDO read 0x{index:04x}.{subindex}: {}",
                crate::servo_sdo::failure_text(r.result)
            )));
        }
        Ok((r.size, u32::from_le_bytes(r.data)))
    }

    fn sdo_write(
        &self,
        mcu_handle: u32,
        index: u16,
        subindex: u8,
        size: u8,
        value: i64,
    ) -> PyResult<(u8, u32)> {
        let conn = self.ethercat_conn(mcu_handle, "sdo_write")?;
        tracing::info!(
            subsystem = "bridge",
            event = "servo_sdo_write",
            mcu_handle,
            index,
            subindex,
            size,
            value,
            "servo SDO write"
        );
        let r = crate::servo_sdo::send_sdo_write(&conn, index, subindex, size, value)
            .map_err(PyRuntimeError::new_err)?;
        if r.result != 0 {
            tracing::error!(
                subsystem = "bridge",
                event = "servo_sdo_write_failed",
                mcu_handle,
                index,
                subindex,
                value,
                result = r.result,
                "servo SDO write failed"
            );
            let readback = u32::from_le_bytes(r.data);
            return Err(PyRuntimeError::new_err(format!(
                "SDO write 0x{index:04x}.{subindex} = {value}: {} (drive reports raw 0x{readback:x})",
                crate::servo_sdo::failure_text(r.result)
            )));
        }
        Ok((r.size, u32::from_le_bytes(r.data)))
    }
```

- [ ] **Step 4: Extend `klippy/motion_bridge.py`**

Add after `set_torque` in `MotionBridgeWrapper`:

```python
    def sdo_read(self, mcu_handle, index, subindex):
        return self._bridge.sdo_read(mcu_handle, index, subindex)

    def sdo_write(self, mcu_handle, index, subindex, size, value):
        return self._bridge.sdo_write(mcu_handle, index, subindex, size, value)
```

Add `"sdo_read",` and `"sdo_write",` to the `_STUB_MOTION_METHODS` frozenset (they issue real endpoint traffic and must raise under the stub bridge).

- [ ] **Step 5: Verify**

Run: `cd rust && cargo check -p motion-bridge && cargo nextest run -p motion-bridge`
Expected: clean check, tests PASS.

- [ ] **Step 6: Commit**

```bash
git add rust/motion-bridge/src/servo_sdo.rs rust/motion-bridge/src/servo_sdo/tests.rs rust/motion-bridge/src/lib.rs rust/motion-bridge/src/bridge.rs klippy/motion_bridge.py
git commit -m "feat(bridge): sdo_read/sdo_write pyo3 entry points over the endpoint socket"
```

---

### Task 8: Klippy — param grammar + SERVO_PARAM command (`servo_param.py`)

**Files:**
- Create: `klippy/extras/servo_param.py`
- Create: `test/test_servo_param.py`

- [ ] **Step 1: Write failing parser/format tests**

Create `test/test_servo_param.py`:

```python
import pytest

from klippy.extras import servo_param


def test_parse_address():
    assert servo_param.parse_address("0x2002.0") == (0x2002, 0)
    assert servo_param.parse_address("0x6041.0x1F") == (0x6041, 0x1F)


@pytest.mark.parametrize(
    "bad", ["2002", "0x2002.0.1", "0x12345.0", "0x2002.300", "x.y"]
)
def test_parse_address_rejects(bad):
    with pytest.raises(ValueError):
        servo_param.parse_address(bad)


def test_parse_param_entry_probed():
    assert servo_param.parse_param_entry("0x2002.0: 100") == (0x2002, 0, 0, 100)


def test_parse_param_entry_typed():
    assert servo_param.parse_param_entry("0x2003.0: u16 250") == (
        0x2003,
        0,
        2,
        250,
    )
    assert servo_param.parse_param_entry("0x2010.1: i32 -4096") == (
        0x2010,
        1,
        4,
        -4096,
    )


def test_parse_param_entry_hex_value():
    assert servo_param.parse_param_entry("0x2002.0: u16 0x64") == (
        0x2002,
        0,
        2,
        0x64,
    )


@pytest.mark.parametrize(
    "bad",
    [
        "0x2002.0 100",  # missing colon
        "0x2002.0: u16 -5",  # negative with unsigned type
        "0x2002.0: i8 200",  # out of i8 range
        "0x2002.0: q16 1",  # unknown type token
        "0x2002.0: u16 1 2",  # trailing junk
        "0x2002.0: 0x1_0000_0000",  # exceeds 32-bit probed range
    ],
)
def test_parse_param_entry_rejects(bad):
    with pytest.raises(ValueError):
        servo_param.parse_param_entry(bad)


def test_parse_params_block_skips_blanks():
    text = "\n0x2002.0: 100\n\n0x2003.0: u16 250\n"
    assert servo_param.parse_params_block(text) == [
        (0x2002, 0, 0, 100),
        (0x2003, 0, 2, 250),
    ]


def test_format_value_untyped_shows_both_interpretations():
    out = servo_param.format_value(0x2002, 0, 2, 0xFFFE, None)
    assert out == "0x2002.0 = 0xfffe (u16: 65534, i16: -2)"


def test_format_value_typed_shows_one():
    assert (
        servo_param.format_value(0x2002, 0, 2, 0xFFFE, "i16")
        == "0x2002.0 = 0xfffe (i16: -2)"
    )
    assert (
        servo_param.format_value(0x2010, 1, 4, 100, "u32")
        == "0x2010.1 = 0x00000064 (u32: 100)"
    )
```

Run: `python3 -m pytest test/test_servo_param.py -q`
Expected: FAIL — module does not exist.

- [ ] **Step 2: Implement `klippy/extras/servo_param.py`**

```python
TYPE_TOKENS = {
    "u8": (1, 0, 0xFF),
    "u16": (2, 0, 0xFFFF),
    "u32": (4, 0, 0xFFFFFFFF),
    "i8": (1, -(1 << 7), (1 << 7) - 1),
    "i16": (2, -(1 << 15), (1 << 15) - 1),
    "i32": (4, -(1 << 31), (1 << 31) - 1),
}
PROBED_MIN = -(1 << 31)
PROBED_MAX = (1 << 32) - 1


def _parse_int(text):
    t = text.strip().lower()
    if t.startswith("0x") or t.startswith("-0x"):
        return int(t, 16)
    return int(t, 10)


def parse_address(text):
    parts = text.strip().split(".")
    if len(parts) != 2:
        raise ValueError("address %r: expected 0xINDEX.SUB" % (text,))
    try:
        index = int(parts[0], 16)
        subindex = _parse_int(parts[1])
    except ValueError:
        raise ValueError("address %r: expected 0xINDEX.SUB" % (text,))
    if not 0 <= index <= 0xFFFF:
        raise ValueError("address %r: index out of 16-bit range" % (text,))
    if not 0 <= subindex <= 0xFF:
        raise ValueError("address %r: subindex out of 8-bit range" % (text,))
    return index, subindex


def check_value(value, type_token):
    if type_token is None:
        if not PROBED_MIN <= value <= PROBED_MAX:
            raise ValueError("value %d out of 32-bit range" % (value,))
        return 0
    size, vmin, vmax = TYPE_TOKENS[type_token]
    if not vmin <= value <= vmax:
        raise ValueError(
            "value %d out of range for %s [%d..%d]"
            % (value, type_token, vmin, vmax)
        )
    return size


def parse_param_entry(line):
    addr_text, sep, rest = line.partition(":")
    if not sep:
        raise ValueError(
            "param %r: expected '0xINDEX.SUB: [type] value'" % (line,)
        )
    index, subindex = parse_address(addr_text)
    fields = rest.split()
    if len(fields) == 1:
        type_token = None
        value_text = fields[0]
    elif len(fields) == 2:
        type_token = fields[0]
        if type_token not in TYPE_TOKENS:
            raise ValueError(
                "param %r: unknown type %r (use u8/u16/u32/i8/i16/i32)"
                % (line, type_token)
            )
        value_text = fields[1]
    else:
        raise ValueError(
            "param %r: expected '0xINDEX.SUB: [type] value'" % (line,)
        )
    try:
        value = _parse_int(value_text)
    except ValueError:
        raise ValueError("param %r: bad value %r" % (line, value_text))
    try:
        size = check_value(value, type_token)
    except ValueError as e:
        raise ValueError("param %r: %s" % (line, e))
    return index, subindex, size, value


def parse_params_block(text):
    entries = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        entries.append(parse_param_entry(line))
    return entries


def format_value(index, subindex, size, raw, type_token):
    bits = 8 * size
    unsigned = raw & ((1 << bits) - 1)
    signed = unsigned - (1 << bits) if unsigned >> (bits - 1) else unsigned
    hex_text = "0x%0*x" % (size * 2, unsigned)
    if type_token is not None:
        shown = signed if type_token.startswith("i") else unsigned
        return "0x%04x.%d = %s (%s: %d)" % (
            index,
            subindex,
            hex_text,
            type_token,
            shown,
        )
    return "0x%04x.%d = %s (u%d: %d, i%d: %d)" % (
        index,
        subindex,
        hex_text,
        bits,
        unsigned,
        bits,
        signed,
    )


class ServoParam:
    cmd_SERVO_PARAM_help = (
        "Read/write a raw CoE SDO object on an EtherCAT servo drive"
    )

    def __init__(self, config):
        self.printer = config.get_printer()
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SERVO_PARAM",
            self.cmd_SERVO_PARAM,
            desc=self.cmd_SERVO_PARAM_help,
        )

    def _resolve_node(self, servo_name):
        from . import servo_axis

        toolhead = self.printer.lookup_object("toolhead")
        for rail in getattr(toolhead.get_kinematics(), "rails", ()):
            if (
                isinstance(rail, servo_axis.ServoRail)
                and rail.get_name() == servo_name
            ):
                return self.printer.lookup_object(
                    "ethercat_node " + rail.get_node_name()
                )
        raise self.printer.command_error(
            "SERVO_PARAM: no servo rail named %r" % (servo_name,)
        )

    def cmd_SERVO_PARAM(self, gcmd):
        node = self._resolve_node(gcmd.get("SERVO"))
        handle = node.get_bridge_handle()
        if handle is None:
            raise gcmd.error(
                "SERVO_PARAM: ethercat_node %s has no bridge handle"
                % (node.name,)
            )
        bridge = self.printer.lookup_object("motion_bridge")
        get_addr = gcmd.get("GET", None)
        set_addr = gcmd.get("SET", None)
        if (get_addr is None) == (set_addr is None):
            raise gcmd.error("SERVO_PARAM: specify exactly one of GET or SET")
        type_token = gcmd.get("TYPE", None)
        if type_token is not None and type_token not in TYPE_TOKENS:
            raise gcmd.error(
                "SERVO_PARAM: unknown TYPE %r (use u8/u16/u32/i8/i16/i32)"
                % (type_token,)
            )
        try:
            if get_addr is not None:
                index, subindex = parse_address(get_addr)
                size, raw = bridge.sdo_read(handle, index, subindex)
                gcmd.respond_info(
                    format_value(index, subindex, size, raw, type_token)
                )
            else:
                index, subindex = parse_address(set_addr)
                value = _parse_int(gcmd.get("VALUE"))
                size = check_value(value, type_token)
                rb_size, rb_raw = bridge.sdo_write(
                    handle, index, subindex, size, value
                )
                gcmd.respond_info(
                    "set "
                    + format_value(index, subindex, rb_size, rb_raw, type_token)
                )
        except ValueError as e:
            raise gcmd.error("SERVO_PARAM: %s" % (e,))
        except RuntimeError as e:
            raise gcmd.error("SERVO_PARAM: %s" % (e,))


def load_config(config):
    return ServoParam(config)
```

- [ ] **Step 3: Run parser tests**

Run: `python3 -m pytest test/test_servo_param.py -q`
Expected: all PASS.

- [ ] **Step 4: Add command-handler tests**

Append to `test/test_servo_param.py`:

```python
from klippy.extras import servo_axis


class FakeGcmd:
    error = RuntimeError

    def __init__(self, params):
        self.params = params
        self.responses = []

    def get(self, name, default=KeyError):
        if name in self.params:
            return self.params[name]
        if default is KeyError:
            raise RuntimeError("missing param %s" % (name,))
        return default

    def respond_info(self, msg):
        self.responses.append(msg)


class FakeBridge:
    def __init__(self):
        self.reads = []
        self.writes = []
        self.read_result = (2, 100)
        self.write_result = (2, 100)

    def sdo_read(self, handle, index, subindex):
        self.reads.append((handle, index, subindex))
        return self.read_result

    def sdo_write(self, handle, index, subindex, size, value):
        self.writes.append((handle, index, subindex, size, value))
        return self.write_result


class FakeNode:
    name = "node_x"

    def __init__(self, handle):
        self._h = handle

    def get_bridge_handle(self):
        return self._h


class FakeKin:
    def __init__(self, rails):
        self.rails = rails


class FakeToolhead:
    def __init__(self, kin):
        self.kin = kin

    def get_kinematics(self):
        return self.kin


class FakePrinter:
    command_error = RuntimeError

    def __init__(self, objs):
        self._objs = objs

    def lookup_object(self, name):
        return self._objs[name]


def make_servo_param(bridge, node):
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.name = "servo_x"
    rail.axis = "x"
    rail.node_name = "node_x"
    sp = servo_param.ServoParam.__new__(servo_param.ServoParam)
    sp.printer = FakePrinter(
        {
            "toolhead": FakeToolhead(FakeKin([rail])),
            "ethercat_node node_x": node,
            "motion_bridge": bridge,
        }
    )
    return sp


def test_cmd_get_reads_and_formats():
    bridge = FakeBridge()
    sp = make_servo_param(bridge, FakeNode(7))
    gcmd = FakeGcmd({"SERVO": "servo_x", "GET": "0x2002.0"})
    sp.cmd_SERVO_PARAM(gcmd)
    assert bridge.reads == [(7, 0x2002, 0)]
    assert gcmd.responses == ["0x2002.0 = 0x0064 (u16: 100, i16: 100)"]


def test_cmd_set_typed_passes_size():
    bridge = FakeBridge()
    bridge.write_result = (2, 250)
    sp = make_servo_param(bridge, FakeNode(7))
    gcmd = FakeGcmd(
        {"SERVO": "servo_x", "SET": "0x2002.0", "VALUE": "250", "TYPE": "u16"}
    )
    sp.cmd_SERVO_PARAM(gcmd)
    assert bridge.writes == [(7, 0x2002, 0, 2, 250)]
    assert gcmd.responses == ["set 0x2002.0 = 0x00fa (u16: 250)"]


def test_cmd_set_untyped_passes_size_zero():
    bridge = FakeBridge()
    sp = make_servo_param(bridge, FakeNode(7))
    gcmd = FakeGcmd({"SERVO": "servo_x", "SET": "0x2002.0", "VALUE": "100"})
    sp.cmd_SERVO_PARAM(gcmd)
    assert bridge.writes == [(7, 0x2002, 0, 0, 100)]


def test_cmd_requires_exactly_one_of_get_set():
    sp = make_servo_param(FakeBridge(), FakeNode(7))
    with pytest.raises(RuntimeError, match="exactly one"):
        sp.cmd_SERVO_PARAM(FakeGcmd({"SERVO": "servo_x"}))
    with pytest.raises(RuntimeError, match="exactly one"):
        sp.cmd_SERVO_PARAM(
            FakeGcmd(
                {
                    "SERVO": "servo_x",
                    "GET": "0x2002.0",
                    "SET": "0x2002.0",
                    "VALUE": "1",
                }
            )
        )


def test_cmd_fails_without_bridge_handle():
    sp = make_servo_param(FakeBridge(), FakeNode(None))
    with pytest.raises(RuntimeError, match="no bridge handle"):
        sp.cmd_SERVO_PARAM(FakeGcmd({"SERVO": "servo_x", "GET": "0x2002.0"}))


def test_cmd_unknown_servo_fails():
    sp = make_servo_param(FakeBridge(), FakeNode(7))
    with pytest.raises(RuntimeError, match="no servo rail"):
        sp.cmd_SERVO_PARAM(FakeGcmd({"SERVO": "servo_q", "GET": "0x2002.0"}))


def test_cmd_propagates_bridge_failure():
    class FailingBridge(FakeBridge):
        def sdo_write(self, *args):
            raise RuntimeError("CoE abort 0x06010002")

    sp = make_servo_param(FailingBridge(), FakeNode(7))
    with pytest.raises(RuntimeError, match="CoE abort"):
        sp.cmd_SERVO_PARAM(
            FakeGcmd({"SERVO": "servo_x", "SET": "0x6041.0", "VALUE": "1"})
        )
```

- [ ] **Step 5: Run all tests**

Run: `python3 -m pytest test/test_servo_param.py -q`
Expected: all PASS.

- [ ] **Step 6: Commit**

```bash
git add klippy/extras/servo_param.py test/test_servo_param.py
git commit -m "feat(klippy): SERVO_PARAM command and raw SDO param grammar"
```

---

### Task 9: Klippy — `params:` config block pushed at claim time

**Files:**
- Modify: `klippy/extras/servo_axis.py`
- Modify: `klippy/extras/ethercat_node.py`
- Modify: `test/test_servo_param.py`

- [ ] **Step 1: Write failing tests**

Append to `test/test_servo_param.py`:

```python
from klippy.extras import ethercat_node


class FakeConfigError(Exception):
    pass


def make_node_for_claim(bridge, rail):
    node = ethercat_node.EtherCatNode.__new__(ethercat_node.EtherCatNode)
    node.name = "node_x"
    node.bridge_handle = 5
    node.printer = FakePrinter(
        {
            "toolhead": FakeToolhead(FakeKin([rail])),
            "motion_bridge": bridge,
        }
    )
    node.printer.config_error = FakeConfigError
    return node


def make_rail_with_params(params):
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.name = "servo_x"
    rail.axis = "x"
    rail.node_name = "node_x"
    rail.sdo_params = params
    return rail


def test_claim_push_writes_params_in_order():
    bridge = FakeBridge()
    rail = make_rail_with_params([(0x2002, 0, 0, 100), (0x2003, 0, 2, 250)])
    node = make_node_for_claim(bridge, rail)
    node._push_drive_params(rail)
    assert bridge.writes == [(5, 0x2002, 0, 0, 100), (5, 0x2003, 0, 2, 250)]


def test_claim_push_failure_is_config_error_with_address():
    class FailingBridge(FakeBridge):
        def sdo_write(self, *args):
            raise RuntimeError("readback mismatch")

    rail = make_rail_with_params([(0x2003, 0, 2, 600)])
    node = make_node_for_claim(FailingBridge(), rail)
    with pytest.raises(FakeConfigError, match="0x2003.0"):
        node._push_drive_params(rail)


def test_claim_push_no_params_is_noop():
    bridge = FakeBridge()
    rail = make_rail_with_params([])
    node = make_node_for_claim(bridge, rail)
    node._push_drive_params(rail)
    assert bridge.writes == []
```

Run: `python3 -m pytest test/test_servo_param.py -q`
Expected: new tests FAIL — `_push_drive_params` does not exist.

- [ ] **Step 2: Implement `servo_axis.py` changes**

Add at the top of `klippy/extras/servo_axis.py` (after `import collections`):

```python
from . import servo_param
```

In `ServoRail.__init__`, after the `position_max` parsing:

```python
        try:
            self.sdo_params = servo_param.parse_params_block(
                config.get("params", "")
            )
        except ValueError as e:
            raise config.error("servo_%s params: %s" % (self.axis, e))
```

Add a getter next to `get_counts_per_mm`:

```python
    def get_sdo_params(self):
        return self.sdo_params
```

- [ ] **Step 3: Implement `ethercat_node.py` changes**

Rename `_derive_counts_per_mm` to `_find_rail` and have it return the rail:

```python
    def _find_rail(self):
        toolhead = self.printer.lookup_object("toolhead")
        for rail in getattr(toolhead.get_kinematics(), "rails", ()):
            if (
                isinstance(rail, servo_axis.ServoRail)
                and rail.get_node_name() == self.name
            ):
                return rail
        raise self.printer.config_error(
            "ethercat_node %s: no [servo_*] section with node=%s — "
            "cannot derive counts_per_mm" % (self.name, self.name)
        )
```

(Keep the explanatory comment about toolhead rails that currently sits on `_derive_counts_per_mm`.)

In `__init__`, after the `register_event_handler` line, load the command module so SERVO_PARAM exists whenever an ethercat node is configured:

```python
        self.printer.load_object(config, "servo_param")
```

In `_claim`, replace `self._counts_per_mm = self._derive_counts_per_mm()` with:

```python
        rail = self._find_rail()
        self._counts_per_mm = rail.get_counts_per_mm()
```

and after the existing `logging.info(...)` claim-success log, add:

```python
        self._push_drive_params(rail)
```

Add the method:

```python
    def _push_drive_params(self, rail):
        params = rail.get_sdo_params()
        if not params:
            return
        bridge = self.printer.lookup_object("motion_bridge")
        for index, subindex, size, value in params:
            try:
                bridge.sdo_write(
                    self.bridge_handle, index, subindex, size, value
                )
            except RuntimeError as e:
                raise self.printer.config_error(
                    "ethercat_node %s: claim-time drive param "
                    "0x%04x.%d = %d failed: %s"
                    % (self.name, index, subindex, value, e)
                )
            logging.info(
                "ethercat_node %s: drive param 0x%04x.%d = %d pushed",
                self.name,
                index,
                subindex,
                value,
            )
```

- [ ] **Step 4: Run all python tests**

Run: `python3 -m pytest test/test_servo_param.py test/test_servo_torque.py -q`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/servo_axis.py klippy/extras/ethercat_node.py test/test_servo_param.py
git commit -m "feat(klippy): declarative drive params in [servo_*] pushed at claim time"
```

---

### Task 10: Docs + full-suite verification

**Files:**
- Modify: `docs/kalico-rewrite/ethercat-bench-bringup.md`

- [ ] **Step 1: Document the feature**

In `docs/kalico-rewrite/ethercat-bench-bringup.md`, extend the sample `[servo_x]` config section (around lines 76–95) with the `params:` option and add a short section after the claim-sequence description:

```markdown
## Drive parameters (SDO)

Drive tuning lives in config, not drive EEPROM. `params:` entries are raw CoE
object addresses pushed to drive RAM (never EEPROM) on every claim, after
bringup succeeds and before the claim is reported healthy. Each write is read
back; a mismatch (drive clamped or rejected the value) fails the claim.

```ini
[servo_x]
# ... existing options ...
params:
    0x2002.0: 100          # size probed via SDO upload (one extra round-trip)
    0x2003.0: u16 250      # explicit type skips the probe
    0x2010.1: i32 -4096
```

Ad-hoc access while tuning:

```
SERVO_PARAM SERVO=servo_x GET=0x2002.0
SERVO_PARAM SERVO=servo_x GET=0x2002.0 TYPE=i16
SERVO_PARAM SERVO=servo_x SET=0x2002.0 VALUE=100 TYPE=u16
```

GET without TYPE prints raw hex plus both unsigned and signed decimal
interpretations. Objects wider than 4 bytes (strings, segmented transfers)
are unsupported and fail loudly. To deliberately persist parameters to drive
EEPROM, SET the drive's store-parameters object (A6-EC manual, object
0x1010) — kalico never does this implicitly.
```

(Adjust the store-parameters object reference if the bringup doc names the drive family differently; 0x1010 is the standard CiA 301 store object.)

- [ ] **Step 2: Full verification**

Run from `rust/`: `cargo nextest run`
Expected: full suite PASS (~11 s).

Run: `cd rust && cargo check -p kalico-ethercat-rt --features hw && cargo check -p motion-bridge`
Expected: clean.

Run from repo root: `python3 -m pytest test/test_servo_param.py test/test_servo_torque.py -q`
Expected: all PASS.

- [ ] **Step 3: Commit**

```bash
git add docs/kalico-rewrite/ethercat-bench-bringup.md
git commit -m "docs: SERVO_PARAM and declarative drive params in bench bringup guide"
```

---

## Bench validation (after merge to the Pi flow)

Not part of this plan's automated steps — the C side and hw binary only build on the Pi (commit → push → pull → `make` on Pi → flash, per the bench rule). First hardware smoke test, **with explicit user permission for any commands sent to the printer**:

1. Build on the Pi (libecrt.a + `cargo build --release --features hw -p kalico-ethercat-rt` + motion-bridge cdylib).
2. Restart klippy with a `params:` block containing one harmless known object; verify claim succeeds and the structured log shows the push.
3. `SERVO_PARAM SERVO=servo_x GET=0x6041.0` (statusword — read-only, always present) to validate the read path.
4. Verify a deliberate bad write (`SET=0x6041.0 VALUE=1`) surfaces a CoE abort, not a hang.
