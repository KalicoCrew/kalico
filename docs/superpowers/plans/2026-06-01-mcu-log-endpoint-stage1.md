# MCU Log Endpoint — Stage 1 (Host-Only) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Dispatch implementer subagents **strictly serially** (never in parallel — they share one worktree). Use the `rust-engineer` subagent for every Rust task.

**Goal:** Land all host-side infrastructure for the MCU structured-log endpoint. After this stage the host can decode synthetic `0x0084` frames, resolve code/event names, stamp RFC3339 `_time` via clock-sync, and write `events/mcu-h7.jsonl`. Nothing emits yet — no MCU C code, no engine call sites (Stage 2/3). The implementation is fully host-unit-testable with synthetic frames.

**Spec:** `docs/superpowers/specs/2026-06-01-mcu-log-endpoint-design.md` — authoritative; read it before touching any task.

**Branch/worktree:** `observability` (`.worktrees/observability`).

**Verify commands (run from `rust/` in the worktree):**

```
cargo test -p kalico-protocol
cargo test -p kalico-host-rt
cargo test -p runtime
cargo test -p motion-bridge
cargo clippy -p kalico-protocol -p kalico-host-rt -p runtime -p motion-bridge --all-targets -- -D warnings
```

No firmware/sim build needed for Stage 1.

---

## Conventions

- Commit after every task with `feat(mcu-log):` prefix. **No `Co-Authored-By` trailer.**
- Fail-loudly is the project rule: no silent error suppression, no `let _ = ...` on paths that can legitimately fail.
- The workspace uses `-D warnings` / clippy pedantic. `unsafe_code = "deny"` workspace-wide.
- TDD order within each task: write the failing test first, then the impl, then confirm green.
- All file paths in this document are relative to the worktree root.

---

## Hash arithmetic (pre-computed)

The current canonical text (stale `PushPiecesResponse v1` with only `result:i32`) hashes to:
`8e1db554e33035bf8912031b2b5732d02b52ee8e7de11455794a07423699feda`

After fixing `PushPiecesResponse` to `v2` with the two `u64` fields the hash becomes:
`f88f3438e338e125d4cb25670fac2e7298ace606af346d1ee12cd8c67080f495`

After also adding `McuLog 0x0084` the final hash becomes:
`093aaa9625709d92255db44c2a82a317c68b474e47cde737439e956f7269a1a0`

The schema-hash test in `rust/kalico-protocol/tests/schema_hash.rs` must always agree with the compiled constant; the plan tells you the exact value to update it to in Tasks 1 and 2.

---

## Task 1 — Fix the stale `PushPiecesResponse` schema entry and update the hash test

**Why first:** the schema hash is downstream of `schema_def.rs`. The `PushPiecesResponse` entry in `schema_def.rs` currently lists only `result:i32` (version 1), but `messages.rs:246–274` has wired `result + arrival_clock:u64 + front_start_time:u64` since the simple-MCU-contract merge. The lockstep guarantee is already broken; fix it before adding anything new so the hash covers the real wire.

**Files:**
- Modify: `rust/kalico-protocol/schema_def.rs`
- Modify: `rust/kalico-protocol/tests/schema_hash.rs` (update the pinned hash constant used in the determinism test)

**Step 1 — Write the failing assertion**

Open `rust/kalico-protocol/tests/schema_hash.rs`. The test `schema_hash_is_deterministic_and_matches_published_constant` asserts `h1 == kalico_protocol::SCHEMA_HASH`. After the fix the hash will change, so the test will start failing; that is the intended red state.

Confirm the current test passes:

```
cd rust && cargo test -p kalico-protocol --test schema_hash
```

All four tests should pass. Record the current `SCHEMA_HASH_HEX` value (`8e1db554e33035bf8912031b2b5732d02b52ee8e7de11455794a07423699feda`) — that is what we are moving away from.

**Step 2 — Fix `schema_def.rs`**

In `rust/kalico-protocol/schema_def.rs`, find the `PushPiecesResponse` entry (lines 83–91 in the original). Change it from:

```rust
    SchemaMessage {
        type_tag: 0x0061,
        name: "PushPiecesResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
        ],
    },
```

to:

```rust
    SchemaMessage {
        type_tag: 0x0061,
        name: "PushPiecesResponse",
        version: 2,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
            SchemaField { name: "arrival_clock", ty: "u64" },
            SchemaField { name: "front_start_time", ty: "u64" },
        ],
    },
```

**Step 3 — Run the build so `build.rs` regenerates the hash**

```
cd rust && cargo build -p kalico-protocol
```

The schema-hash test will now fail (hash mismatch). Confirm:

```
cargo test -p kalico-protocol --test schema_hash -- schema_hash_is_deterministic_and_matches_published_constant
```

Expected: FAILED (hash mismatch).

**Step 4 — Update the schema-hash test**

The `schema_hash_is_deterministic_and_matches_published_constant` test has no pinned literal to update — it asserts `h1 == kalico_protocol::SCHEMA_HASH`, which is compiled from `build.rs`. After the `schema_def.rs` change, `cargo build` regenerated the constant to the post-fix value; the test directly compares against `kalico_protocol::SCHEMA_HASH`. So the test should now pass without any test-file change. Confirm:

```
cargo test -p kalico-protocol --test schema_hash
```

All four tests pass. If the `SCHEMA_CANONICAL` comparison in the determinism test uses any independently stored string — confirm there is no pinned hex string literal in the test file itself that needs updating. The current `tests/schema_hash.rs` has no such literal; it uses the compiled constant throughout.

**Step 5 — Verify C header was regenerated**

```
grep -n "0x0061" ../../src/kalico_protocol_schema.h
```

The `#define KALICO_MSG_PUSH_PIECES_RESPONSE 0x0061` line should be present. The hash bytes array should change from the old value to the post-fix value (`f88f3438...`).

**Step 6 — Run full protocol test suite**

```
cargo test -p kalico-protocol
cargo clippy -p kalico-protocol --all-targets -- -D warnings
```

Both must pass cleanly.

**Commit:**

```
feat(mcu-log): fix stale PushPiecesResponse schema_def.rs entry (v2 + two u64 fields)
```

---

## Task 2 — Add `KALICO_MSG_LOG (0x0084)` to `messages.rs` and `schema_def.rs`

**What this does:** adds the new wire message both to the codec layer (`messages.rs`) and to the schema layer (`schema_def.rs`) so `build.rs` emits the `#define KALICO_MSG_LOG 0x0084` in the C header and bumps the schema hash to the final Stage-1 value.

**Files:**
- Modify: `rust/kalico-protocol/src/messages.rs`
- Modify: `rust/kalico-protocol/schema_def.rs`
- Modify: `rust/kalico-protocol/tests/schema_hash.rs` (verify hash — no pinned literal to change)

### 2a — Write failing codec tests first

Add a `#[cfg(test)] mod mcu_log_tests;` at the bottom of `rust/kalico-protocol/src/messages.rs` (alongside the existing `mod tests;`), and create `rust/kalico-protocol/src/messages/mcu_log_tests.rs`:

```rust
use super::*;

#[test]
fn mcu_log_encode_decode_round_trip() {
    use crate::codec::{Cursor, Decode, Encode};

    let orig = McuLog {
        mcu_tick: 0x0001_2345_6789_ABCDu64,
        level: 2,
        subsystem: 1,
        event: 0x0003,
        code: 0xFECC,  // sign-wrapped -308 (PieceStartInPast)
        seq: 7,
        args: [0xDEAD_BEEF, 0x0000_0042],
    };

    let mut buf = Vec::new();
    orig.encode(&mut buf);
    // Fixed layout: 8+1+1+2+2+2+4+4 = 24 bytes
    assert_eq!(buf.len(), 24);

    let mut c = Cursor::new(&buf);
    let decoded = McuLog::decode_from(&mut c).expect("decode must succeed");
    assert_eq!(decoded, orig);
    // No trailing bytes
    assert_eq!(c.remaining(), 0);
}

#[test]
fn mcu_log_is_event_kind() {
    assert!(MessageKind::McuLog.is_event());
    assert_eq!(MessageKind::McuLog.as_u16(), 0x0084);
    assert_eq!(MessageKind::from_u16(0x0084), Some(MessageKind::McuLog));
    assert_eq!(MessageKind::from_u16(0x0085), None);
}

#[test]
fn mcu_log_is_schema_validated() {
    assert!(MessageKind::McuLog.is_schema_validated());
}

#[test]
fn mcu_log_zero_args_round_trip() {
    use crate::codec::{Cursor, Decode, Encode};
    let orig = McuLog {
        mcu_tick: 0,
        level: 0,
        subsystem: 0,
        event: 0,
        code: 0,
        seq: 0,
        args: [0, 0],
    };
    let mut buf = Vec::new();
    orig.encode(&mut buf);
    let decoded = McuLog::decode_from(&mut Cursor::new(&buf)).unwrap();
    assert_eq!(decoded, orig);
}
```

Confirm the file doesn't compile yet (the type `McuLog` and the variant `MessageKind::McuLog` don't exist):

```
cd rust && cargo test -p kalico-protocol 2>&1 | head -20
```

Expected: compile error on `McuLog`.

### 2b — Add the `McuLog` struct and codec impl to `messages.rs`

Add a new section at the bottom of `rust/kalico-protocol/src/messages.rs` (before `#[cfg(test)] mod tests;`):

```rust
// =============================================================================
// McuLog (0x0084) — MCU → Host structured log event.
//
// Wire layout (little-endian), fixed 24 bytes:
//   mcu_tick:  u64  (bytes  0..8)  — MCU-pre-widened clock at log-emit
//   level:     u8   (byte   8)     — 0=trace 1=debug 2=warn 3=error
//   subsystem: u8   (byte   9)     — subsystem id (resolved host-side)
//   event:     u16  (bytes 10..12) — event code (resolved host-side)
//   code:      u16  (bytes 12..14) — fault code as_u16 (sign-wrapped; 0 = none)
//   seq:       u16  (bytes 14..16) — per-MCU monotonic sequence for drop detection
//   arg0:      u32  (bytes 16..20) — first numeric argument
//   arg1:      u32  (bytes 20..24) — second numeric argument
//
// Total body = 24 bytes.
// =============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct McuLog {
    /// MCU-pre-widened clock ticks at emit time.
    pub mcu_tick: u64,
    /// Log level (0=trace, 1=debug, 2=warn, 3=error).
    pub level: u8,
    /// Subsystem id (resolved to name on host).
    pub subsystem: u8,
    /// Event code (resolved to name/template on host).
    pub event: u16,
    /// Fault code sign-wrapped as u16 via `FaultCode::as_u16`. 0 = no fault code.
    pub code: u16,
    /// Per-MCU monotonic sequence number for host drop detection.
    pub seq: u16,
    /// Numeric arguments [arg0, arg1].
    pub args: [u32; 2],
}

impl Encode for McuLog {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u64(out, self.mcu_tick);
        put_u8(out, self.level);
        put_u8(out, self.subsystem);
        put_u16(out, self.event);
        put_u16(out, self.code);
        put_u16(out, self.seq);
        put_u32(out, self.args[0]);
        put_u32(out, self.args[1]);
    }
}

impl Decode for McuLog {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            mcu_tick: get_u64(c)?,
            level: get_u8(c)?,
            subsystem: get_u8(c)?,
            event: get_u16(c)?,
            code: get_u16(c)?,
            seq: get_u16(c)?,
            args: [get_u32(c)?, get_u32(c)?],
        })
    }
}
```

Add `McuLog = 0x0084` to the `MessageKind` enum and to `from_u16`. The enum block becomes:

```rust
pub enum MessageKind {
    Identify = 0x0001,
    IdentifyResponse = 0x0002,
    ConfigureAxes = 0x0030,
    ConfigureAxesResponse = 0x0031,
    QueryRuntimeCaps = 0x0040,
    RuntimeCapsResponse = 0x0041,
    PushPieces = 0x0060,
    PushPiecesResponse = 0x0061,
    FaultEvent = 0x0082,
    StatusHeartbeat = 0x0083,
    McuLog = 0x0084,
}
```

Add `0x0084 => Self::McuLog,` to `from_u16`. Export `McuLog` from `rust/kalico-protocol/src/lib.rs`:

```rust
pub use messages::{
    FaultEvent, McuLog, MessageKind, PushPieces, PushPiecesResponse, RuntimeCapsResponse,
    StatusHeartbeat,
};
```

### 2c — Add the `SchemaMessage` entry to `schema_def.rs`

Append after the `StatusHeartbeat` entry (maintaining ascending type-tag order):

```rust
    SchemaMessage {
        type_tag: 0x0084,
        name: "McuLog",
        version: 1,
        channel: "events",
        fields: &[
            SchemaField { name: "mcu_tick", ty: "u64" },
            SchemaField { name: "level", ty: "u8" },
            SchemaField { name: "subsystem", ty: "u8" },
            SchemaField { name: "event", ty: "u16" },
            SchemaField { name: "code", ty: "u16" },
            SchemaField { name: "seq", ty: "u16" },
            SchemaField { name: "arg0", ty: "u32" },
            SchemaField { name: "arg1", ty: "u32" },
        ],
    },
```

Note the field names in `schema_def.rs` are `arg0`/`arg1` (the schema-canonical names used in the C header and NDJSON output), while the Rust struct uses `args: [u32; 2]` with individual encode/decode — this is intentional and matches the spec.

### 2d — Verify

```
cd rust && cargo build -p kalico-protocol
cargo test -p kalico-protocol
```

After the build the `SCHEMA_HASH` compiled constant will be the final value `093aaa9625709d92255db44c2a82a317c68b474e47cde737439e956f7269a1a0`. The schema-hash test compares against it directly — no test-file edits needed. All codec round-trip tests must pass.

Verify the C header now includes `#define KALICO_MSG_MCU_LOG 0x0084`:

```
grep -n "MCU_LOG\|0x0084" ../../src/kalico_protocol_schema.h
```

Run clippy:

```
cargo clippy -p kalico-protocol --all-targets -- -D warnings
```

**Commit:**

```
feat(mcu-log): add KALICO_MSG_LOG 0x0084 to messages.rs + schema_def.rs; final schema hash 093aaa96
```

---

## Task 3 — Fix `build.rs` fail-loud header write

**What this does:** when `src/` exists but `fs::write` fails (disk full, permissions, etc.), the current `build.rs` swallows the error with a `cargo:warning`. Per spec §4.2 and the fail-loudly project rule, that must be a hard error.

**Files:**
- Modify: `rust/kalico-protocol/build.rs`

The current lines 119–136 read:

```rust
    if let Some(parent) = header_path.parent() {
        if !parent.exists() {
            println!(
                "cargo:warning=kalico-protocol: skipping C header generation; {} does not exist",
                parent.display()
            );
            return;
        }
    }
    if let Err(e) = fs::write(&header_path, h) {
        println!(
            "cargo:warning=kalico-protocol: failed to write {}: {e}",
            header_path.display()
        );
    }
```

Replace with:

```rust
    // Guard: skip header generation only when `src/` doesn't exist at all —
    // that is the standalone-crate-publish path where the firmware tree is
    // absent. Any other failure (disk full, permissions, …) is a hard error:
    // a silently stale C header violates the deploy-lockstep guarantee.
    if let Some(parent) = header_path.parent() {
        if !parent.exists() {
            // Standalone-crate-publish: firmware src/ absent — skip silently.
            println!(
                "cargo:warning=kalico-protocol: skipping C header generation; {} does not exist \
                 (standalone-crate-publish path — expected)",
                parent.display()
            );
            return;
        }
    }
    // src/ exists — any write failure is fatal.
    fs::write(&header_path, h).unwrap_or_else(|e| {
        panic!(
            "kalico-protocol build.rs: failed to write C header {}: {e}\n\
             (src/ exists but write failed — disk full, permissions issue, or path wrong)",
            header_path.display()
        )
    });
```

**Verify:**

```
cd rust && cargo build -p kalico-protocol
cargo clippy -p kalico-protocol --all-targets -- -D warnings
```

No unit test covers this path (it requires simulating a write failure in `build.rs`, which is not practical from a test harness). The change is structural and the correctness is self-evident.

**Commit:**

```
feat(mcu-log): build.rs fail-loud on C header write failure when src/ exists
```

---

## Task 4 — `FaultCode::from_u16` + `code_name` in `runtime/src/error.rs`

**What this does:** gives the host a way to sign-extend a `u16` code from the wire back to a `FaultCode` variant, and resolve it to a `&'static str` name. The sign-extend path (`u16 → i16 → i32 → match`) is the inverse of `FaultCode::as_u16` (`i32 → i16 → u16`).

**Files:**
- Modify: `rust/runtime/src/error.rs`
- Modify: `rust/runtime/src/error/tests.rs`

### 4a — Write failing tests first

Add to `rust/runtime/src/error/tests.rs`:

```rust
#[test]
fn fault_code_from_u16_round_trip_positive_zero() {
    assert_eq!(FaultCode::from_u16(0), Some(FaultCode::None));
}

#[test]
fn fault_code_from_u16_sign_wrap_piece_start_in_past() {
    // -308 as i16 = -308; as u16 = 0xFF2C... wait, let's be precise:
    // -308i32 as i16 = -308i16; -308i16 as u16 = 65228 = 0xFECC
    let wire = FaultCode::PieceStartInPast.as_u16();
    assert_eq!(wire, 0xFECC);
    assert_eq!(FaultCode::from_u16(wire), Some(FaultCode::PieceStartInPast));
}

#[test]
fn fault_code_from_u16_sign_wrap_tick_interval_exceeded() {
    let wire = FaultCode::TickIntervalExceeded.as_u16();
    // -311 as i16 = -311; as u16 = 65225 = 0xFEC9
    assert_eq!(wire, 0xFEC9);
    assert_eq!(FaultCode::from_u16(wire), Some(FaultCode::TickIntervalExceeded));
}

#[test]
fn fault_code_from_u16_sign_wrap_host_disconnect() {
    let wire = FaultCode::HostDisconnect.as_u16();
    assert_eq!(FaultCode::from_u16(wire), Some(FaultCode::HostDisconnect));
}

#[test]
fn fault_code_from_u16_unknown_returns_none() {
    // 0x1234 does not correspond to any FaultCode discriminant
    assert_eq!(FaultCode::from_u16(0x1234), None);
}

#[test]
fn code_name_piece_start_in_past() {
    assert_eq!(FaultCode::PieceStartInPast.code_name(), "PieceStartInPast");
}

#[test]
fn code_name_none() {
    assert_eq!(FaultCode::None.code_name(), "None");
}

#[test]
fn code_name_tick_interval_exceeded() {
    assert_eq!(FaultCode::TickIntervalExceeded.code_name(), "TickIntervalExceeded");
}

#[test]
fn from_u16_then_code_name_for_all_step8_codes() {
    // Every Step-8 code must survive the round-trip as_u16 -> from_u16 -> code_name
    let codes = [
        FaultCode::StepQueueOverflow,
        FaultCode::SpiQueueOverflow,
        FaultCode::MathNonFinite,
        FaultCode::PieceAdvanceUnderflow,
        FaultCode::SampleRateMisconfigured,
        FaultCode::PositionCountOverflow,
        FaultCode::JogParametersInvalid,
        FaultCode::StepRateExceedsMcuCeiling,
        FaultCode::PieceStartInPast,
        FaultCode::RingFull,
        FaultCode::StepsPerSampleExceeded,
        FaultCode::TickIntervalExceeded,
    ];
    for code in codes {
        let wire = code.as_u16();
        let recovered = FaultCode::from_u16(wire).unwrap_or_else(|| {
            panic!("from_u16 returned None for {code:?} (wire=0x{wire:04x})")
        });
        assert_eq!(recovered, code, "round-trip mismatch for {code:?}");
        // code_name must return a non-empty non-"unknown" string for known variants
        let name = recovered.code_name();
        assert!(!name.is_empty(), "code_name empty for {code:?}");
        assert_ne!(name, "unknown", "code_name returned 'unknown' for known variant {code:?}");
    }
}
```

Confirm the tests fail to compile:

```
cd rust && cargo test -p runtime --lib 2>&1 | grep "error"
```

### 4b — Implement `from_u16` and `code_name`

Add to the `impl FaultCode` block in `rust/runtime/src/error.rs`, after the existing `as_u16` method:

```rust
    /// Reconstruct a `FaultCode` from its sign-wrapped `u16` wire encoding.
    ///
    /// The wire carries `(self as i32 as i16) as u16` (see `as_u16`). To
    /// invert: sign-extend `u16 → i16 → i32`, then match against discriminants.
    /// Returns `None` for values that do not correspond to any known variant.
    #[allow(clippy::cast_possible_wrap)]
    pub fn from_u16(v: u16) -> Option<Self> {
        let i = i32::from(v as i16);
        Some(match i {
            0    => Self::None,
            -1   => Self::QueueFull,
            -2   => Self::InvalidCurve,
            -3   => Self::InvalidHandle,
            -4   => Self::InvalidDuration,
            -5   => Self::InvalidKinematics,
            -6   => Self::NullPtr,
            -7   => Self::NotInit,
            -8   => Self::FaultLatched,
            -9   => Self::Internal,
            -21  => Self::StepBurstExceeded,
            -22  => Self::ZeroDurationSegment,
            -23  => Self::HomingTrip,
            -24  => Self::CapabilityMissing,
            -25  => Self::NoStep,
            -26  => Self::InvalidArg,
            -29  => Self::PhaseModeNotAvailable,
            -30  => Self::CurveLoadInvalid,
            -31  => Self::MotionInProgress,
            -100 => Self::BadCrc,
            -101 => Self::FramingViolation,
            -102 => Self::Disconnect,
            -103 => Self::ProtocolVersionUnsupported,
            -110 => Self::ClockSyncQuality,
            -111 => Self::ClockSyncTimeout,
            -120 => Self::ArmTimeout,
            -121 => Self::ArmRejected,
            -122 => Self::CrossMcuDesync,
            -130 => Self::Underrun,
            -131 => Self::QueueOverrun,
            -132 => Self::LivenessStalled,
            -133 => Self::TraceOverflow,
            -140 => Self::StreamStateViolation,
            -141 => Self::SegmentIdNonMonotonic,
            -150 => Self::TStartInPast,
            -151 => Self::TEndBeforeTStart,
            -152 => Self::SegmentTooShort,
            -153 => Self::SegmentTooLong,
            -160 => Self::InvalidCurveHandle,
            -161 => Self::CurveReloadRejected,
            -162 => Self::CurveFormatInvalid,
            -170 => Self::NanInfOutput,
            -171 => Self::BoundaryLoopOverflow,
            -172 => Self::InternalInvariant,
            -200 => Self::HostDisconnect,
            -201 => Self::HostRetransmitExhausted,
            -202 => Self::HostDispatcherTimeout,
            -300 => Self::StepQueueOverflow,
            -301 => Self::SpiQueueOverflow,
            -302 => Self::MathNonFinite,
            -303 => Self::PieceAdvanceUnderflow,
            -304 => Self::SampleRateMisconfigured,
            -305 => Self::PositionCountOverflow,
            -306 => Self::JogParametersInvalid,
            -307 => Self::StepRateExceedsMcuCeiling,
            -308 => Self::PieceStartInPast,
            -309 => Self::RingFull,
            -310 => Self::StepsPerSampleExceeded,
            -311 => Self::TickIntervalExceeded,
            _    => return None,
        })
    }

    /// Human-readable name for use in structured log output. Returns the
    /// variant's name as a `&'static str`; never allocates.
    pub fn code_name(self) -> &'static str {
        match self {
            Self::None                      => "None",
            Self::QueueFull                 => "QueueFull",
            Self::InvalidCurve              => "InvalidCurve",
            Self::InvalidHandle             => "InvalidHandle",
            Self::InvalidDuration           => "InvalidDuration",
            Self::InvalidKinematics         => "InvalidKinematics",
            Self::NullPtr                   => "NullPtr",
            Self::NotInit                   => "NotInit",
            Self::FaultLatched              => "FaultLatched",
            Self::Internal                  => "Internal",
            Self::StepBurstExceeded         => "StepBurstExceeded",
            Self::ZeroDurationSegment       => "ZeroDurationSegment",
            Self::HomingTrip                => "HomingTrip",
            Self::CapabilityMissing         => "CapabilityMissing",
            Self::NoStep                    => "NoStep",
            Self::InvalidArg                => "InvalidArg",
            Self::PhaseModeNotAvailable     => "PhaseModeNotAvailable",
            Self::CurveLoadInvalid          => "CurveLoadInvalid",
            Self::MotionInProgress          => "MotionInProgress",
            Self::BadCrc                    => "BadCrc",
            Self::FramingViolation          => "FramingViolation",
            Self::Disconnect                => "Disconnect",
            Self::ProtocolVersionUnsupported => "ProtocolVersionUnsupported",
            Self::ClockSyncQuality          => "ClockSyncQuality",
            Self::ClockSyncTimeout          => "ClockSyncTimeout",
            Self::ArmTimeout                => "ArmTimeout",
            Self::ArmRejected               => "ArmRejected",
            Self::CrossMcuDesync            => "CrossMcuDesync",
            Self::Underrun                  => "Underrun",
            Self::QueueOverrun              => "QueueOverrun",
            Self::LivenessStalled           => "LivenessStalled",
            Self::TraceOverflow             => "TraceOverflow",
            Self::StreamStateViolation      => "StreamStateViolation",
            Self::SegmentIdNonMonotonic     => "SegmentIdNonMonotonic",
            Self::TStartInPast              => "TStartInPast",
            Self::TEndBeforeTStart          => "TEndBeforeTStart",
            Self::SegmentTooShort           => "SegmentTooShort",
            Self::SegmentTooLong            => "SegmentTooLong",
            Self::InvalidCurveHandle        => "InvalidCurveHandle",
            Self::CurveReloadRejected       => "CurveReloadRejected",
            Self::CurveFormatInvalid        => "CurveFormatInvalid",
            Self::NanInfOutput              => "NanInfOutput",
            Self::BoundaryLoopOverflow      => "BoundaryLoopOverflow",
            Self::InternalInvariant         => "InternalInvariant",
            Self::HostDisconnect            => "HostDisconnect",
            Self::HostRetransmitExhausted   => "HostRetransmitExhausted",
            Self::HostDispatcherTimeout     => "HostDispatcherTimeout",
            Self::StepQueueOverflow         => "StepQueueOverflow",
            Self::SpiQueueOverflow          => "SpiQueueOverflow",
            Self::MathNonFinite             => "MathNonFinite",
            Self::PieceAdvanceUnderflow     => "PieceAdvanceUnderflow",
            Self::SampleRateMisconfigured   => "SampleRateMisconfigured",
            Self::PositionCountOverflow     => "PositionCountOverflow",
            Self::JogParametersInvalid      => "JogParametersInvalid",
            Self::StepRateExceedsMcuCeiling => "StepRateExceedsMcuCeiling",
            Self::PieceStartInPast          => "PieceStartInPast",
            Self::RingFull                  => "RingFull",
            Self::StepsPerSampleExceeded    => "StepsPerSampleExceeded",
            Self::TickIntervalExceeded      => "TickIntervalExceeded",
        }
    }
```

**Verify:**

```
cd rust && cargo test -p runtime
cargo clippy -p runtime --all-targets -- -D warnings
```

Note: `runtime` has `no_std` MCU features; `from_u16` and `code_name` contain no `std`-only constructs. Confirm the MCU feature builds still compile:

```
cargo build -p runtime --no-default-features --features mcu-h7 --target thumbv7em-none-eabihf 2>&1 | grep "^error" || echo "MCU build ok"
```

(If the MCU target is not installed, skip this check — the host test is sufficient for Stage 1.)

**Commit:**

```
feat(mcu-log): FaultCode::from_u16 + code_name in runtime/src/error.rs
```

---

## Task 5 — Subsystem and event code tables in `runtime` crate

**What this does:** defines the initial set of subsystem IDs and per-subsystem event codes with `&'static str` names and template strings. These tables live in the `runtime` crate (no_std + host) so MCU emit sites and the host resolver share one source.

**Files:**
- Create: `rust/runtime/src/log_codes.rs`
- Modify: `rust/runtime/src/lib.rs` (add `pub mod log_codes;`)

### 5a — Define the module

Create `rust/runtime/src/log_codes.rs`:

```rust
//! Subsystem and event code tables for the MCU structured-log endpoint.
//!
//! Subsystem IDs and event codes are wire-stable u8/u16 discriminants.
//! Names and templates are resolved host-side from these tables.
//! This module compiles for both no_std MCU targets and the host.
//!
//! Naming convention for templates: `{arg0}` and `{arg1}` are the two
//! numeric arguments transmitted in the `McuLog` frame.

#![allow(dead_code)] // tables grow as emit sites are added in Stage 3

// ── Subsystem IDs (u8) ──────────────────────────────────────────────────────

pub const SUBSYSTEM_RUNTIME: u8 = 0;
pub const SUBSYSTEM_MOTION: u8 = 1;
pub const SUBSYSTEM_TICK: u8 = 2;
pub const SUBSYSTEM_ENDSTOP: u8 = 3;

/// Resolve a subsystem id to its `&'static str` name.
/// Returns `"unknown"` for unrecognised ids — never fails.
pub fn subsystem_name(id: u8) -> &'static str {
    match id {
        SUBSYSTEM_RUNTIME => "runtime",
        SUBSYSTEM_MOTION  => "motion",
        SUBSYSTEM_TICK    => "tick",
        SUBSYSTEM_ENDSTOP => "endstop",
        _                 => "unknown",
    }
}

// ── Event codes (u16) per subsystem ─────────────────────────────────────────
//
// Convention: EVENT_<SUBSYSTEM>_<NAME>. Codes are unique within each subsystem
// but may repeat across subsystems (the (subsystem, event) pair is the key).
// Start at 1; 0 is reserved as "no event".

// runtime subsystem events
pub const EVENT_RUNTIME_FAULT_LATCHED: u16   = 1;
pub const EVENT_RUNTIME_ENGINE_RESET: u16    = 2;

// motion subsystem events
pub const EVENT_MOTION_PIECE_START_PAST: u16 = 1;
pub const EVENT_MOTION_RING_FULL: u16        = 2;

// tick subsystem events
pub const EVENT_TICK_INTERVAL_EXCEEDED: u16  = 1;
pub const EVENT_TICK_UNDERRUN: u16           = 2;

// endstop subsystem events
pub const EVENT_ENDSTOP_TRIP: u16            = 1;
pub const EVENT_ENDSTOP_ARM_TIMEOUT: u16     = 2;

/// Resolve a `(subsystem, event)` pair to a `(name, template)` tuple.
///
/// `name` is the stable event key (e.g. `"tick.interval_exceeded"`).
/// `template` is a human-readable format string; `{arg0}` and `{arg1}`
/// are placeholders for the two numeric args.
///
/// Returns `("unknown", "")` for unrecognised pairs — never fails.
pub fn event_info(subsystem: u8, event: u16) -> (&'static str, &'static str) {
    match (subsystem, event) {
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_FAULT_LATCHED)   =>
            ("runtime.fault_latched",  "fault latched code={arg0} detail={arg1}"),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_ENGINE_RESET)    =>
            ("runtime.engine_reset",   "engine reset epoch={arg0}"),
        (SUBSYSTEM_MOTION, EVENT_MOTION_PIECE_START_PAST)  =>
            ("motion.piece_start_past","piece start in past start_time={arg0} now={arg1}"),
        (SUBSYSTEM_MOTION, EVENT_MOTION_RING_FULL)         =>
            ("motion.ring_full",       "axis ring full axis={arg0}"),
        (SUBSYSTEM_TICK, EVENT_TICK_INTERVAL_EXCEEDED)     =>
            ("tick.interval_exceeded", "TIM5 inter-arrival exceeded: got={arg0} limit={arg1}"),
        (SUBSYSTEM_TICK, EVENT_TICK_UNDERRUN)              =>
            ("tick.underrun",          "tick underrun segment={arg0}"),
        (SUBSYSTEM_ENDSTOP, EVENT_ENDSTOP_TRIP)            =>
            ("endstop.trip",           "endstop tripped arm={arg0} source={arg1}"),
        (SUBSYSTEM_ENDSTOP, EVENT_ENDSTOP_ARM_TIMEOUT)     =>
            ("endstop.arm_timeout",    "endstop arm timeout arm={arg0}"),
        _                                                   =>
            ("unknown",                ""),
    }
}

/// Compose the `_msg` string from a template and two args.
///
/// Substitutes `{arg0}` with `arg0` and `{arg1}` with `arg1`.
/// Returns the template unchanged when there are no placeholders.
/// Allocates a `String`; called on the host only (no MCU use).
#[cfg(feature = "host")]
pub fn compose_msg(template: &str, arg0: u32, arg1: u32) -> alloc::string::String {
    template
        .replace("{arg0}", &alloc::format!("{arg0}"))
        .replace("{arg1}", &alloc::format!("{arg1}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subsystem_name_known() {
        assert_eq!(subsystem_name(SUBSYSTEM_RUNTIME), "runtime");
        assert_eq!(subsystem_name(SUBSYSTEM_MOTION), "motion");
        assert_eq!(subsystem_name(SUBSYSTEM_TICK), "tick");
        assert_eq!(subsystem_name(SUBSYSTEM_ENDSTOP), "endstop");
    }

    #[test]
    fn subsystem_name_unknown_returns_unknown() {
        assert_eq!(subsystem_name(0xFF), "unknown");
    }

    #[test]
    fn event_info_tick_interval_exceeded() {
        let (name, tmpl) = event_info(SUBSYSTEM_TICK, EVENT_TICK_INTERVAL_EXCEEDED);
        assert_eq!(name, "tick.interval_exceeded");
        assert!(tmpl.contains("{arg0}"));
    }

    #[test]
    fn event_info_unknown_pair() {
        let (name, _) = event_info(0xFF, 0x7FFF);
        assert_eq!(name, "unknown");
    }

    #[cfg(feature = "host")]
    #[test]
    fn compose_msg_substitutes_args() {
        let msg = compose_msg("got={arg0} limit={arg1}", 5, 10);
        assert_eq!(msg, "got=5 limit=10");
    }

    #[cfg(feature = "host")]
    #[test]
    fn compose_msg_no_placeholders() {
        let msg = compose_msg("engine reset", 0, 0);
        assert_eq!(msg, "engine reset");
    }
}
```

Note on `alloc`: the `runtime` crate is `no_std` on MCU targets. `compose_msg` is gated on `#[cfg(feature = "host")]`. On host builds the `host` feature is active (see `Cargo.toml`: `default = ["host"]`), which enables `nurbs/host`, and the workspace `std` is available. For the `compose_msg` function to use `alloc::string::String` in a `no_std` + `alloc` context, add `extern crate alloc;` at the top of the module under `#![cfg_attr(not(feature = "host"), no_std)]`. However, since the runtime crate's `lib.rs` currently uses `std` on host builds, the simpler approach is to restrict `compose_msg` to `#[cfg(feature = "host")]` and use `std::string::String` directly. Check `rust/runtime/src/lib.rs` — if it has `#![no_std]` unconditionally, use `alloc`; if not, use `std`. Given the crate has `default = ["host"]` and the host feature enables `nurbs/host` (which uses `std`), the crate effectively links `std` in the default build. Use `String` and `format!` directly; the `#[cfg(feature = "host")]` guard on `compose_msg` ensures it is stripped on MCU targets.

Add to `rust/runtime/src/lib.rs`:

```rust
pub mod log_codes;
```

**Verify:**

```
cd rust && cargo test -p runtime
cargo clippy -p runtime --all-targets -- -D warnings
```

**Commit:**

```
feat(mcu-log): subsystem + event code tables in runtime/src/log_codes.rs
```

---

## Task 6 — `RuntimeEvent::McuLog` + `McuLogEvent` type + decode in `kalico_native.rs`

**What this does:** defines the `McuLogEvent` struct and the `RuntimeEvent::McuLog` variant in `kalico-host-rt`, and wires the `0x0084` decode path in `kalico_native.rs`.

**Files:**
- Modify: `rust/kalico-host-rt/src/host_io/runtime_events.rs`
- Modify: `rust/kalico-host-rt/src/host_io/kalico_native.rs`
- Modify: `rust/kalico-host-rt/src/host_io/runtime_events/tests.rs` (add a decode test)

### 6a — Write the failing test

Add to `rust/kalico-host-rt/src/host_io/runtime_events/tests.rs`:

```rust
#[test]
fn lifts_mcu_log_via_kalico_native_decode() {
    use std::time::Instant;
    use kalico_protocol::{Encode, McuLog, MessageKind};
    use crate::host_io::kalico_native::{KalicoNativeState, KalicoDispatchResult, dispatch_kalico_frame};
    use kalico_native_transport::CHANNEL_EVENTS;

    let frame_payload = {
        // Build the per-message header manually (type:u16 LE, version:u8, cid:u32 LE = 0)
        let mut payload = Vec::new();
        payload.extend_from_slice(&(MessageKind::McuLog as u16).to_le_bytes());
        payload.push(1u8); // version
        payload.extend_from_slice(&0u32.to_le_bytes()); // correlation_id = 0 (event)

        let log_msg = McuLog {
            mcu_tick: 0x0000_1234_5678_9ABCu64,
            level: 2,
            subsystem: 2, // tick
            event: 1,     // interval_exceeded
            code: 0xFEC9, // -311 TickIntervalExceeded
            seq: 42,
            args: [100, 200],
        };
        log_msg.encode(&mut payload);
        payload
    };

    let mut state = KalicoNativeState::default();
    let before = Instant::now();
    let result = dispatch_kalico_frame(&mut state, CHANNEL_EVENTS, &frame_payload);
    let after = Instant::now();

    match result {
        KalicoDispatchResult::Event(RuntimeEvent::McuLog(e)) => {
            assert_eq!(e.mcu_tick, 0x0000_1234_5678_9ABCu64);
            assert_eq!(e.level, 2);
            assert_eq!(e.subsystem, 2);
            assert_eq!(e.event, 1);
            assert_eq!(e.code, 0xFEC9);
            assert_eq!(e.seq, 42);
            assert_eq!(e.args, [100, 200]);
            // host_recv must be within the test window
            assert!(e.host_recv >= before);
            assert!(e.host_recv <= after);
        }
        other => panic!("expected McuLog event, got {:?}", other),
    }
}
```

Confirm the compile fails (no `McuLogEvent`, no `RuntimeEvent::McuLog`):

```
cd rust && cargo test -p kalico-host-rt --lib 2>&1 | head -20
```

### 6b — Define `McuLogEvent` and the `RuntimeEvent::McuLog` variant

Add to `rust/kalico-host-rt/src/host_io/runtime_events.rs` (before the existing `RuntimeEvent` enum):

```rust
/// Decoded `KALICO_MSG_LOG (0x0084)` frame. The MCU pre-widens the tick to
/// `u64`; the host stamps `host_recv` at decode time.
#[derive(Debug, Clone)]
pub struct McuLogEvent {
    /// MCU-pre-widened clock ticks at log-emit.
    pub mcu_tick: u64,
    /// Log level (0=trace, 1=debug, 2=warn, 3=error).
    pub level: u8,
    /// Subsystem id — resolved to name by `runtime::log_codes::subsystem_name`.
    pub subsystem: u8,
    /// Event code — resolved to `(name, template)` by `runtime::log_codes::event_info`.
    pub event: u16,
    /// Fault code sign-wrapped as `u16`. 0 = no fault code.
    pub code: u16,
    /// Per-MCU monotonic sequence number for drop detection.
    pub seq: u16,
    /// Numeric args `[arg0, arg1]`.
    pub args: [u32; 2],
    /// Host-side `Instant` stamped at decode (in the reactor dispatch loop).
    pub host_recv: Instant,
}
```

Add the variant to the `RuntimeEvent` enum:

```rust
    /// Decoded `KALICO_MSG_LOG (0x0084)` — MCU structured log event.
    McuLog(McuLogEvent),
```

### 6c — Wire the decode in `kalico_native.rs`

In `rust/kalico-host-rt/src/host_io/kalico_native.rs`, add to the `use kalico_protocol` import block:

```rust
use kalico_protocol::{
    Decode, FaultEvent as KFaultEvent, McuLog as KMcuLog, MessageKind, PROTO_VERSION, SCHEMA_HASH,
    StatusHeartbeat as KStatusHeartbeat,
};
```

Add to the imports from `crate::host_io::runtime_events`:

```rust
use crate::host_io::runtime_events::{FaultEvent, McuLogEvent, RuntimeEvent};
```

In `lift_event_to_runtime_event`, add a new match arm before the `_ =>` catch:

```rust
        MessageKind::McuLog => match KMcuLog::decode(body) {
            Ok(msg) => KalicoDispatchResult::Event(RuntimeEvent::McuLog(McuLogEvent {
                mcu_tick: msg.mcu_tick,
                level: msg.level,
                subsystem: msg.subsystem,
                event: msg.event,
                code: msg.code,
                seq: msg.seq,
                args: msg.args,
                host_recv: Instant::now(),
            })),
            Err(e) => {
                log::warn!("kalico McuLog decode failed: {e:?}");
                KalicoDispatchResult::Ignored
            }
        },
```

The `Instant::now()` call mirrors the pattern used for `add_piggyback_sample` in `clock_sync.rs:235`. The `host_recv` is stamped as close to decode as possible, inside the reactor dispatch loop.

The `EventDispatcher::dispatch` in `events.rs` must also handle the new variant — add a forwarding arm:

```rust
            RuntimeEvent::McuLog(ref _e) => {
                // McuLog is handled by the mcu_log_hook (Task 7).
                // Forward to the general runtime channel so callers that
                // subscribe to take_runtime_event_subscription can also observe it.
                self.runtime_event_dispatcher.dispatch(event);
                // Then fire the hook if set (handled in task 7).
            }
```

Actually the hook firing and forwarding are both covered in Task 7. For now, add only the forwarding branch to `EventDispatcher::dispatch` so the compiler is satisfied with the non-exhaustive match:

```rust
            RuntimeEvent::McuLog(_) => {
                self.runtime_event_dispatcher.dispatch(event);
            }
```

**Verify:**

```
cd rust && cargo test -p kalico-host-rt
cargo clippy -p kalico-host-rt --all-targets -- -D warnings
```

**Commit:**

```
feat(mcu-log): RuntimeEvent::McuLog + McuLogEvent + 0x0084 decode in kalico_native.rs
```

---

## Task 7 — `ClockSyncEstimator::wall_time_at_mcu` + `(SystemTime, Instant)` anchor

**What this does:** gives the clock-sync estimator the ability to convert an MCU tick count to a host-side `OffsetDateTime` (RFC3339-formattable). The estimator today works entirely in `Instant`/seconds space and cannot produce wall-clock times.

**Files:**
- Modify: `rust/kalico-host-rt/src/clock_sync.rs`
- Modify: `rust/kalico-host-rt/src/clock_sync/clock_seam_tests.rs` (add new tests)
- Modify: `rust/kalico-host-rt/Cargo.toml` (add `time` dependency)

### 7a — Add `time` dependency

In `rust/kalico-host-rt/Cargo.toml` `[dependencies]`:

```toml
time = { version = "0.3", default-features = false, features = ["std"] }
```

(The `kalico-host-rt` crate needs `OffsetDateTime` which requires the `std` feature of the `time` crate. No macros or formatting needed at this layer — those live in `motion-bridge`.)

### 7b — Write failing tests first

Add to `rust/kalico-host-rt/src/clock_sync/clock_seam_tests.rs` (or as a new inline `#[cfg(test)]` block at the bottom of `clock_sync.rs`):

```rust
#[cfg(test)]
mod wall_time_tests {
    use super::*;
    use std::time::{Duration, SystemTime};
    use time::OffsetDateTime;

    #[test]
    fn wall_time_at_mcu_returns_none_with_zero_samples() {
        let est = ClockSyncEstimator::new(100_000_000.0);
        // No samples added — must return None
        assert!(est.wall_time_at_mcu(0).is_none());
        assert!(est.wall_time_at_mcu(100_000_000).is_none());
    }

    #[test]
    fn wall_time_at_mcu_inside_window_returns_some_false() {
        use crate::clock::MockClock;
        let clock = Arc::new(MockClock::new());
        let mut est = ClockSyncEstimator::new_with_clock(100_000_000.0, clock.clone());

        // Feed 30 samples at 1s intervals so the window is full.
        // MCU ticks at 100 MHz → 100_000_000 ticks/s.
        for i in 0..30u64 {
            clock.advance(Duration::from_secs(1));
            let mcu_clock = (i + 1) * 100_000_000;
            est.add_piggyback_sample_at_now(mcu_clock);
        }

        // Query a tick inside the regression window.
        let result = est.wall_time_at_mcu(15 * 100_000_000);
        assert!(result.is_some(), "expected Some after 30 samples");
        let (dt, estimated) = result.unwrap();
        // estimated=false because tick is within the window
        assert!(!estimated, "should be inside window: estimated={estimated}");
        // The returned time must be a valid OffsetDateTime (not epoch, not
        // wildly in the past or future relative to the test's wall clock).
        let now_sys = SystemTime::now();
        let now_dt = OffsetDateTime::from(now_sys);
        let diff = (now_dt - dt).abs();
        // Allow ±60 s — the mock clock advances but the epoch anchor is
        // captured from the real SystemTime at construction.
        assert!(diff.whole_seconds().abs() < 60,
            "returned time {dt} too far from now {now_dt}");
    }

    #[test]
    fn wall_time_at_mcu_extrapolate_returns_some_true() {
        use crate::clock::MockClock;
        let clock = Arc::new(MockClock::new());
        let mut est = ClockSyncEstimator::new_with_clock(100_000_000.0, clock.clone());

        for i in 0..30u64 {
            clock.advance(Duration::from_secs(1));
            est.add_piggyback_sample_at_now((i + 1) * 100_000_000);
        }

        // Query a tick 60s beyond the window's most recent sample.
        let far_future = 90 * 100_000_000u64;
        let result = est.wall_time_at_mcu(far_future);
        assert!(result.is_some());
        let (_dt, estimated) = result.unwrap();
        assert!(estimated, "extrapolation outside window must set estimated=true");
    }
}
```

Confirm the tests fail to compile (`wall_time_at_mcu` does not exist):

```
cd rust && cargo test -p kalico-host-rt 2>&1 | head -20
```

### 7c — Add the `(SystemTime, Instant)` anchor and `wall_time_at_mcu`

Add two fields to `ClockSyncEstimator`:

```rust
    /// Wall-clock anchor captured at estimator construction.
    /// Paired with `epoch` (the `Instant` at the same moment) so we can
    /// map any `Instant`-relative host-time offset to a `SystemTime` and
    /// thence to an `OffsetDateTime` for RFC3339 formatting.
    wall_epoch: std::time::SystemTime,
```

The `wall_epoch` and `epoch` (already an `Instant`) together form the anchor: both are captured at `new` / `new_with_clock` time. In `new_with_clock`:

```rust
        Self {
            epoch,
            wall_epoch: std::time::SystemTime::now(),
            // ...existing fields...
        }
```

Add `wall_time_at_mcu` to the `impl ClockSyncEstimator` block:

```rust
    /// Convert an MCU tick count to a host-side wall-clock time.
    ///
    /// Returns `None` when no samples have been received (estimator not
    /// yet converged — caller should fall back to `Instant::now()` stamped
    /// at decode, with `time_estimated = true`).
    ///
    /// Returns `Some((dt, estimated))` otherwise:
    /// - `estimated = false` when `mcu_ticks` falls within the regression
    ///   window (min..=max mcu_clock of the current sample set).
    /// - `estimated = true` when extrapolating outside the window.
    ///
    /// The inverse formula:
    ///   host_secs_since_epoch = (mcu_ticks - anchor_mcu_clock) / freq + anchor_host_time
    ///   wall_time = wall_epoch + (epoch + host_secs_since_epoch)
    ///
    /// Because `anchor_host_time` is already expressed in seconds since
    /// `epoch`, the full offset from `wall_epoch` is simply
    /// `host_secs_since_epoch` seconds (both anchors share the same epoch).
    pub fn wall_time_at_mcu(&self, mcu_ticks: u64) -> Option<(time::OffsetDateTime, bool)> {
        if self.samples.is_empty() {
            return None;
        }

        // Determine whether the query is within the regression window.
        let min_mcu = self.samples.iter().map(|s| s.mcu_clock).min().unwrap_or(0);
        let max_mcu = self.samples.iter().map(|s| s.mcu_clock).max().unwrap_or(0);
        let estimated = mcu_ticks < min_mcu || mcu_ticks > max_mcu;

        // Invert the regression: host_time_secs = anchor_host_time +
        //   (mcu_ticks - anchor_mcu_clock) / freq
        let freq = self.clock_freq_estimate;
        if freq.abs() < 1e-6 {
            // Degenerate estimator (single sample or zero slope) — extrapolate
            // with the construction-time anchor: just use epoch + 0 offset.
            let dt = time::OffsetDateTime::from(self.wall_epoch);
            return Some((dt, true));
        }

        #[allow(clippy::cast_precision_loss)]
        let delta_ticks = (mcu_ticks as f64) - (self.anchor_mcu_clock as f64);
        let host_secs_offset = self.anchor_host_time + delta_ticks / freq;

        // Wall time = wall_epoch + host_secs_offset seconds.
        // Use `Duration::from_secs_f64` for sub-second precision.
        let duration = if host_secs_offset >= 0.0 {
            self.wall_epoch
                .checked_add(std::time::Duration::from_secs_f64(host_secs_offset))
                .unwrap_or(self.wall_epoch)
        } else {
            self.wall_epoch
                .checked_sub(std::time::Duration::from_secs_f64(-host_secs_offset))
                .unwrap_or(self.wall_epoch)
        };

        Some((time::OffsetDateTime::from(duration), estimated))
    }
```

**Verify:**

```
cd rust && cargo test -p kalico-host-rt
cargo clippy -p kalico-host-rt --all-targets -- -D warnings
```

**Commit:**

```
feat(mcu-log): ClockSyncEstimator::wall_time_at_mcu + (SystemTime,Instant) anchor
```

---

## Task 8 — `EventDispatcher.mcu_log_hook` field + setter

**What this does:** adds the injection slot for the re-emit closure to `EventDispatcher`, mirroring the `heartbeat_callback` pattern.

**Files:**
- Modify: `rust/kalico-host-rt/src/host_io/events.rs`
- Modify: `rust/kalico-host-rt/src/host_io/runtime_events.rs` (re-export `McuLogEvent` if needed)
- Modify: `rust/kalico-host-rt/src/host_io/events/dispatch_tests.rs`

### 8a — Write the failing test

Add to `rust/kalico-host-rt/src/host_io/events/dispatch_tests.rs`:

```rust
#[test]
fn mcu_log_hook_is_called_on_mcu_log_event() {
    use std::sync::{Arc, Mutex};
    use std::time::Instant;
    use arc_swap::ArcSwap;
    use crate::host_io::runtime_events::{McuLogEvent, RuntimeEvent, StatusEvent};
    use crate::host_io::events::EventDispatcher;

    let snapshot = Arc::new(ArcSwap::from_pointee(StatusEvent::default()));
    let mut dispatcher = EventDispatcher::new(snapshot, 16, 8);

    let received: Arc<Mutex<Vec<McuLogEvent>>> = Arc::new(Mutex::new(Vec::new()));
    let received_clone = received.clone();
    dispatcher.set_mcu_log_hook(move |e: McuLogEvent| {
        received_clone.lock().unwrap().push(e);
    });

    let event = RuntimeEvent::McuLog(McuLogEvent {
        mcu_tick: 12345,
        level: 2,
        subsystem: 1,
        event: 1,
        code: 0xFEC9,
        seq: 1,
        args: [0, 0],
        host_recv: Instant::now(),
    });
    dispatcher.dispatch(event);

    let got = received.lock().unwrap();
    assert_eq!(got.len(), 1);
    assert_eq!(got[0].mcu_tick, 12345);
}

#[test]
fn mcu_log_without_hook_does_not_panic() {
    use std::time::Instant;
    use arc_swap::ArcSwap;
    use crate::host_io::runtime_events::{McuLogEvent, RuntimeEvent, StatusEvent};
    use crate::host_io::events::EventDispatcher;

    let snapshot = Arc::new(ArcSwap::from_pointee(StatusEvent::default()));
    let mut dispatcher = EventDispatcher::new(snapshot, 16, 8);
    // No hook set — must not panic.
    dispatcher.dispatch(RuntimeEvent::McuLog(McuLogEvent {
        mcu_tick: 0,
        level: 0,
        subsystem: 0,
        event: 0,
        code: 0,
        seq: 0,
        args: [0, 0],
        host_recv: Instant::now(),
    }));
}
```

### 8b — Add the field and setter

Add to `EventDispatcher` struct:

```rust
    /// Optional hook fired on every `McuLog (0x0084)` event with the decoded
    /// frame. Pump-private: the event is consumed here. Set via
    /// `set_mcu_log_hook`. The hook receives an owned `McuLogEvent` so the
    /// closure can move it into a channel or process it without holding a borrow
    /// on the dispatcher.
    pub mcu_log_hook: Option<Box<dyn Fn(McuLogEvent) + Send + Sync>>,
```

Add `mcu_log_hook: None,` to `EventDispatcher::new`.

Add the setter method:

```rust
    /// Attach a closure that fires on every decoded `McuLog (0x0084)` event.
    /// Replaces any previously set hook. Use `None` (by not calling this) or
    /// call again with a new closure to update.
    pub fn set_mcu_log_hook<F>(&mut self, f: F)
    where
        F: Fn(McuLogEvent) + Send + Sync + 'static,
    {
        self.mcu_log_hook = Some(Box::new(f));
    }
```

Update `EventDispatcher::dispatch` — replace the stub `McuLog` arm from Task 6 with:

```rust
            RuntimeEvent::McuLog(e) => {
                if let Some(hook) = &self.mcu_log_hook {
                    hook(e.clone());
                }
                // Also forward to the general runtime channel so callers that
                // subscribe to take_runtime_event_subscription can observe it.
                self.runtime_event_dispatcher
                    .dispatch(RuntimeEvent::McuLog(e));
            }
```

Add `McuLogEvent` to the `use` import in `events.rs`:

```rust
use crate::host_io::runtime_events::{CreditFreedEvent, McuLogEvent, RuntimeEvent, StatusEvent, TraceEvent};
```

**Verify:**

```
cd rust && cargo test -p kalico-host-rt
cargo clippy -p kalico-host-rt --all-targets -- -D warnings
```

**Commit:**

```
feat(mcu-log): EventDispatcher.mcu_log_hook field + set_mcu_log_hook setter
```

---

## Task 9 — Re-emit closure in `motion-bridge`: `ClockSyncEstimator` sharing + dedicated JSONL writer + hook wiring

**What this does:** upgrades the `spawn_periodic_clock_sync` function to share the `ClockSyncEstimator` behind an `Arc<RwLock<...>>`, constructs a dedicated `Arc<Mutex<RotatingJsonlWriter>>` for `events/mcu-h7.jsonl`, and wires the re-emit closure into `EventDispatcher` via `set_mcu_log_hook`.

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs`
- Create: `rust/motion-bridge/tests/mcu_log_reemit.rs`

### 9a — Write the integration test first

Create `rust/motion-bridge/tests/mcu_log_reemit.rs`:

```rust
//! Verifies the re-emit closure produces a schema-conformant NDJSON line
//! for a synthetic McuLog event, with a real RFC3339 `_time`.

use std::io::BufRead;
use std::sync::{Arc, Mutex, RwLock};
use std::time::Instant;
use time::OffsetDateTime;

use kalico_host_rt::clock_sync::ClockSyncEstimator;
use kalico_host_rt::host_io::runtime_events::McuLogEvent;
use motion_bridge_native::logging::schema::format_time;
use motion_bridge_native::logging::writer::{RotatingJsonlWriter, DEFAULT_MAX_BYTES, DEFAULT_BACKUP_COUNT, FSYNC_INTERVAL};
use motion_bridge_native::logging::context;
use motion_bridge_native::mcu_log::build_mcu_log_hook;

fn tmp_jsonl(name: &str) -> std::path::PathBuf {
    let mut p = std::env::temp_dir();
    p.push(format!("kalico-mcu-log-test-{}-{name}", std::process::id()));
    std::fs::create_dir_all(&p).unwrap();
    p.push("mcu-h7.jsonl");
    p
}

#[test]
fn re_emit_closure_produces_schema_conformant_line() {
    // Set up a deterministic session context.
    context::set_context("k-test-session".to_string(), "print-42".to_string());

    let path = tmp_jsonl("reemit");
    let writer = Arc::new(Mutex::new(
        RotatingJsonlWriter::new(&path, DEFAULT_MAX_BYTES, DEFAULT_BACKUP_COUNT, FSYNC_INTERVAL)
            .unwrap(),
    ));

    // Build an estimator with 30 samples so wall_time_at_mcu works.
    let est = Arc::new(RwLock::new(ClockSyncEstimator::new(100_000_000.0)));
    {
        let mut guard = est.write().unwrap();
        let now = Instant::now();
        for i in 0..30u64 {
            let t = now + std::time::Duration::from_secs(i);
            guard.add_piggyback_sample(t, (i + 1) * 100_000_000);
        }
    }

    let hook = build_mcu_log_hook(
        Arc::clone(&est),
        Arc::clone(&writer),
        "mcu-h7".to_string(),
    );

    let event = McuLogEvent {
        mcu_tick: 15 * 100_000_000u64,
        level: 2, // warn
        subsystem: 2, // tick
        event: 1, // interval_exceeded
        code: 0xFEC9, // -311 TickIntervalExceeded
        seq: 7,
        args: [100, 200],
        host_recv: Instant::now(),
    };

    hook(event);

    // Flush the writer so we can read the file.
    {
        let mut w = writer.lock().unwrap();
        use std::io::Write;
        w.flush().unwrap();
    }

    let content = std::fs::read_to_string(&path).unwrap();
    let line = content.lines().next().expect("at least one line");
    let rec: serde_json::Value = serde_json::from_str(line).expect("valid JSON");

    // Schema conformance checks.
    assert_eq!(rec["source"], "mcu-h7");
    assert_eq!(rec["level"], "warn");
    assert_eq!(rec["subsystem"], "tick");
    assert_eq!(rec["event"], "tick.interval_exceeded");
    assert_eq!(rec["session_id"], "k-test-session");
    assert_eq!(rec["print_id"], "print-42");
    assert_eq!(rec["seq"], 7);
    assert_eq!(rec["code"], 0xFEC9u64);
    assert_eq!(rec["code_name"], "TickIntervalExceeded");
    assert!(rec["_msg"].as_str().unwrap().contains("100"));
    // _time must be a real RFC3339 string with trailing Z.
    let time_str = rec["_time"].as_str().unwrap();
    assert!(time_str.ends_with('Z'), "_time must end with Z: {time_str}");
    // Verify it parses as a valid OffsetDateTime.
    OffsetDateTime::parse(time_str, &time::format_description::well_known::Rfc3339)
        .unwrap_or_else(|e| panic!("_time '{time_str}' is not valid RFC3339: {e}"));
    // time_estimated field present (false or true, either is valid — just must exist).
    assert!(rec.get("time_estimated").is_some());
    // arg0, arg1 present.
    assert_eq!(rec["arg0"], 100u64);
    assert_eq!(rec["arg1"], 200u64);
}
```

This test references `motion_bridge_native::mcu_log::build_mcu_log_hook` — a new public function we will create in Task 9b.

Confirm the test fails to compile:

```
cd rust && cargo test -p motion-bridge --test mcu_log_reemit 2>&1 | head -20
```

### 9b — Share `ClockSyncEstimator` from `spawn_periodic_clock_sync`

The `spawn_periodic_clock_sync` function in `bridge.rs` currently owns its `ClockSyncEstimator` as a local variable. We need to share it with the mcu_log_hook closure.

**Pattern:** pass an `Arc<RwLock<ClockSyncEstimator>>` into `spawn_periodic_clock_sync` (it is given the Arc at call time from `attach_serial`). The thread takes a write lock for mutations; the hook takes a read lock for `wall_time_at_mcu`.

Change `spawn_periodic_clock_sync`'s signature and body:

```rust
fn spawn_periodic_clock_sync(
    mcu_handle_raw: u32,
    host_io: Arc<KalicoHostIo>,
    router: Arc<Mutex<PassthroughRouter>>,
    clock_freqs: Arc<Mutex<HashMap<u32, f64>>>,
    stop: Arc<AtomicBool>,
    estimator: Arc<RwLock<ClockSyncEstimator>>,  // NEW parameter
) -> JoinHandle<()> {
```

Inside the thread body, remove the local `let mut estimator = ClockSyncEstimator::new(initial_freq);` and replace all `estimator.method()` calls with `estimator.write().unwrap().method()` (for mutations) or `estimator.read().unwrap().method()` (for reads). The existing logic is unchanged; only the locking wrapper is added.

Callers of `spawn_periodic_clock_sync` in `attach_serial` need to construct the `Arc<RwLock<ClockSyncEstimator>>` and store a clone on `McuConnection` so the mcu_log_hook closure can reach it:

Add to `McuConnection`:

```rust
    /// Shared clock-sync estimator. `None` for stock-Klipper MCUs.
    clock_sync_estimator: Option<Arc<RwLock<ClockSyncEstimator>>>,
```

In `attach_serial` (wherever `spawn_periodic_clock_sync` is called), construct:

```rust
let estimator = Arc::new(RwLock::new(ClockSyncEstimator::new(initial_freq)));
// store a clone on the McuConnection
mcu.clock_sync_estimator = Some(Arc::clone(&estimator));
// pass to the thread
let join = spawn_periodic_clock_sync(
    mcu_handle_raw, host_io_clone, router_clone,
    clock_freqs_clone, stop_clone, Arc::clone(&estimator),
);
```

### 9c — Create `rust/motion-bridge/src/mcu_log.rs`

This module owns `build_mcu_log_hook`:

```rust
//! Re-emit closure factory for MCU structured-log events.
//!
//! `build_mcu_log_hook` constructs the closure injected into
//! `EventDispatcher::set_mcu_log_hook`. It captures:
//!   - `Arc<RwLock<ClockSyncEstimator>>` — shared with the clock-sync thread
//!   - `Arc<Mutex<RotatingJsonlWriter>>` — dedicated NDJSON writer for
//!     `events/<source>.jsonl` (separate from `host-rust.jsonl`)
//!   - `source: String` — `"mcu-h7"` or `"mcu-f4"`

use std::io::Write;
use std::sync::{Arc, Mutex, RwLock};
use std::time::Instant;

use serde_json::{Map, Value};
use time::OffsetDateTime;

use kalico_host_rt::clock_sync::ClockSyncEstimator;
use kalico_host_rt::host_io::runtime_events::McuLogEvent;
use runtime::error::FaultCode;
use runtime::log_codes::{compose_msg, event_info, subsystem_name};

use crate::logging::context::load_context;
use crate::logging::schema::{format_time, level_str};
use crate::logging::writer::RotatingJsonlWriter;

/// Map a raw `u8` level to a `&'static str` for the schema `level` field.
fn mcu_level_str(level: u8) -> &'static str {
    match level {
        0 => "trace",
        1 => "debug",
        2 => "warn",
        3 => "error",
        _ => "error", // unknown → treat as error (fail-loudly posture)
    }
}

/// Build the re-emit closure for MCU log events.
///
/// The returned closure is `Fn(McuLogEvent) + Send + Sync + 'static` and
/// can be passed directly to `EventDispatcher::set_mcu_log_hook`.
pub fn build_mcu_log_hook(
    clock: Arc<RwLock<ClockSyncEstimator>>,
    writer: Arc<Mutex<RotatingJsonlWriter>>,
    source: String,
) -> impl Fn(McuLogEvent) + Send + Sync + 'static {
    move |e: McuLogEvent| {
        // 1. Resolve timestamp.
        let (time_str, time_estimated) = {
            let guard = clock.read().unwrap_or_else(|p| p.into_inner());
            match guard.wall_time_at_mcu(e.mcu_tick) {
                Some((dt, estimated)) => (format_time(dt), estimated),
                None => {
                    // No clock-sync samples yet — fall back to host_recv.
                    // `host_recv` is an `Instant`; convert via SystemTime.
                    let sys = std::time::SystemTime::now()
                        .checked_sub(e.host_recv.elapsed())
                        .unwrap_or(std::time::SystemTime::UNIX_EPOCH);
                    (format_time(OffsetDateTime::from(sys)), true)
                }
            }
        };

        // 2. Resolve names.
        let subsys_name = subsystem_name(e.subsystem);
        let (event_name, template) = event_info(e.subsystem, e.event);
        let msg = compose_msg(template, e.args[0], e.args[1]);

        // 3. Resolve fault code.
        let (code_val, code_name_val) = if e.code != 0 {
            let name = FaultCode::from_u16(e.code)
                .map(|fc| fc.code_name())
                .unwrap_or("unknown");
            (Some(e.code), Some(name))
        } else {
            (None, None)
        };

        // 4. Stamp session context.
        let ctx = load_context();

        // 5. Compose the NDJSON record.
        let mut rec = Map::new();
        rec.insert("_time".into(), Value::String(time_str));
        rec.insert("_msg".into(), Value::String(msg));
        rec.insert("level".into(), Value::String(mcu_level_str(e.level).into()));
        rec.insert("source".into(), Value::String(source.clone()));
        rec.insert("subsystem".into(), Value::String(subsys_name.into()));
        rec.insert("event".into(), Value::String(event_name.into()));
        rec.insert("session_id".into(), Value::String(ctx.session_id.clone()));
        rec.insert("print_id".into(), Value::String(ctx.print_id.clone()));
        rec.insert("target".into(), Value::String(format!("mcu::{subsys_name}")));
        rec.insert("mcu_tick".into(), Value::from(e.mcu_tick));
        rec.insert("seq".into(), Value::from(e.seq));
        rec.insert("arg0".into(), Value::from(e.args[0]));
        rec.insert("arg1".into(), Value::from(e.args[1]));
        rec.insert("time_estimated".into(), Value::Bool(time_estimated));
        if let Some(code) = code_val {
            rec.insert("code".into(), Value::from(code));
        }
        if let Some(name) = code_name_val {
            rec.insert("code_name".into(), Value::String(name.into()));
        }

        let mut line = serde_json::to_string(&Value::Object(rec))
            .unwrap_or_else(|e| format!("{{\"_msg\":\"mcu-log serialize error: {e}\"}}"));
        line.push('\n');

        // 6. Write to the dedicated MCU JSONL file.
        let mut w = writer.lock().unwrap_or_else(|p| p.into_inner());
        if let Err(e) = w.write_all(line.as_bytes()) {
            // Fail-loudly on write error: log to stderr (the tracing subscriber
            // may not be reachable at this callsite since we are in a closure
            // fired from the reactor dispatch thread).
            eprintln!("[mcu-log] JSONL write failed: {e}");
        }
        if let Err(e) = w.flush() {
            eprintln!("[mcu-log] JSONL flush failed: {e}");
        }
    }
}
```

Add `pub mod mcu_log;` to `rust/motion-bridge/src/lib.rs`.

Add `runtime = { path = "../runtime" }` to `rust/motion-bridge/Cargo.toml` `[dependencies]` — this is already present in the Cargo.toml, so no change needed. Confirm `use runtime::log_codes` compiles.

### 9d — Wire the hook in `attach_serial`

In `bridge.rs`, after populating `mcu.clock_sync_estimator` and constructing the `host_io`, wire the hook:

```rust
// Construct the dedicated MCU JSONL writer.
let events_dir = /* read from the global logging dir, or pass through */ ...;
```

The writer needs the events directory. The `init_logging` PyO3 method (from Stage 2) receives the `events_dir`. Pass it or store it on `PyMotionBridge`. A minimal approach: store `events_dir: Option<std::path::PathBuf>` on `PyMotionBridge`, set by `init_logging`, and use it here. The directory is always set before `attach_serial` is called (the logging init happens at `MotionBridgeWrapper.__init__`).

If `events_dir` is not yet set when `attach_serial` is called (edge case during testing), skip hook installation and log a warning.

```rust
if let Some(ref dir) = *self.events_dir.lock().unwrap() {
    let source = if mcu_handle_raw == h7_handle { "mcu-h7" } else { "mcu-f4" };
    let jsonl_path = dir.join(format!("{source}.jsonl"));
    match RotatingJsonlWriter::new(
        &jsonl_path,
        DEFAULT_MAX_BYTES,
        DEFAULT_BACKUP_COUNT,
        FSYNC_INTERVAL,
    ) {
        Ok(writer) => {
            let arc_writer = Arc::new(Mutex::new(writer));
            let arc_clock = Arc::clone(est_arc); // the shared estimator Arc
            let hook = build_mcu_log_hook(arc_clock, arc_writer, source.to_string());
            if let Some(host_io) = &mcu_conn.host_io {
                // Access event_dispatcher via KalicoHostIo's attach hook
                // mechanism. Because EventDispatcher is owned inside KalicoHostIo's
                // reactor, the idiomatic path is via a ReactorCommand.
                // See how heartbeat_callback is attached (bridge.rs AttachHeartbeatCallback).
                host_io.set_mcu_log_hook(Box::new(hook));
            }
        }
        Err(e) => {
            log::warn!("mcu-log: failed to open {}: {e}", jsonl_path.display());
        }
    }
}
```

Mirror the `AttachHeartbeatCallback` / `attach_heartbeat_callback` pattern exactly:

1. In `rust/kalico-host-rt/src/host_io/mod.rs`, add a newtype wrapper and a `ReactorCommand` variant, parallel to the existing ones at lines 80 and 105:

```rust
pub struct McuLogHook(pub Box<dyn Fn(McuLogEvent) + Send + Sync>);

impl std::fmt::Debug for McuLogHook {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("McuLogHook(<fn>)")
    }
}
```

Add `SetMcuLogHook(McuLogHook),` to `ReactorCommand`.

2. Add to `impl KalicoHostIo` in `mod.rs` (after `attach_heartbeat_callback`):

```rust
    /// Attach a hook fired on every decoded `McuLog (0x0084)` event.
    /// Runs on the reactor thread — must be non-blocking.
    pub fn set_mcu_log_hook(&self, hook: Box<dyn Fn(McuLogEvent) + Send + Sync>) {
        let _ = self
            .submission_tx
            .send(ReactorCommand::SetMcuLogHook(McuLogHook(hook)));
    }
```

3. In `rust/kalico-host-rt/src/host_io/reactor.rs`, in `handle_command`, add the arm alongside `AttachHeartbeatCallback`:

```rust
            ReactorCommand::SetMcuLogHook(wrapper) => {
                self.event_dispatcher.set_mcu_log_hook(move |e| (wrapper.0)(e));
            }
```

**Verify:**

```
cd rust && cargo test -p motion-bridge
cargo clippy -p motion-bridge --all-targets -- -D warnings
```

The integration test `mcu_log_reemit` must pass, producing a valid schema-conformant NDJSON line.

**Commit:**

```
feat(mcu-log): re-emit closure + dedicated mcu-h7.jsonl writer + ClockSyncEstimator sharing
```

---

## Task 10 — Final verification pass

Run the full verification suite:

```
cd /Users/daniladergachev/Developer/kalico/.worktrees/observability/rust

cargo test -p kalico-protocol
cargo test -p kalico-host-rt
cargo test -p runtime
cargo test -p motion-bridge

cargo clippy \
  -p kalico-protocol \
  -p kalico-host-rt \
  -p runtime \
  -p motion-bridge \
  --all-targets -- -D warnings
```

All four test suites and clippy must pass cleanly. Specifically confirm:

- `schema_hash_is_deterministic_and_matches_published_constant` passes (hash = `093aaa9625709d92255db44c2a82a317c68b474e47cde737439e956f7269a1a0`)
- `mcu_log_encode_decode_round_trip` passes
- `fault_code_from_u16_sign_wrap_piece_start_in_past` passes (`0xFECC` → `PieceStartInPast`)
- `wall_time_at_mcu_returns_none_with_zero_samples` passes
- `re_emit_closure_produces_schema_conformant_line` passes
- C header contains `#define KALICO_MSG_MCU_LOG 0x0084`

**Commit:**

```
feat(mcu-log): Stage 1 complete — host decode, clock-sync, re-emit verified
```

---

## Dependency graph summary

```
Task 1 (schema_def PushPiecesResponse fix)
  └─ Task 2 (add McuLog 0x0084 to messages.rs + schema_def.rs)
       └─ Task 3 (build.rs fail-loud)  [independent; commit after T2]

Task 4 (FaultCode::from_u16 + code_name)  [independent of T1-T3]
  └─ Task 5 (subsystem/event tables)       [depends on T4 for context; can start after T4]

Task 6 (RuntimeEvent::McuLog + decode)  [depends on T2 for McuLog type]
  └─ Task 7 (ClockSyncEstimator::wall_time_at_mcu)  [independent; can run after T6]
       └─ Task 8 (EventDispatcher.mcu_log_hook)  [depends on T6]
            └─ Task 9 (re-emit closure + bridge wiring)  [depends on T4, T5, T7, T8]
                 └─ Task 10 (final verification)
```

Tasks 3 and Tasks 4-5 are independent of each other and of Task 6; they can be committed as soon as their individual verify commands pass, without waiting for the sequence. The serial order above is the safest for the shared worktree.
