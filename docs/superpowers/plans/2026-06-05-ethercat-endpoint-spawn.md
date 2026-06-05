# EtherCAT Endpoint Spawn-on-Claim Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the manual `bench-hw-up.sh` pre-launch flow with klippy-owned endpoint lifecycle: `claim_ethercat_node` spawns the endpoint binary, waits for its socket, handshakes with structured per-slave status, and tears it down cleanly on release.

**Architecture:** The wire handshake reply gains a per-slave status list (encoded as `[(slave_idx: u8, state: u8, fault_code: u16)]`) so the bridge can map drive-offline/faulted outcomes to named klippy errors. `McuConnection` in `motion-bridge` gains an optional `Child` handle; `release_mcu` sends SIGTERM, waits bounded, then SIGKILLs. The stub binary gets a `--fail-bringup slave=1` flag so integration tests can exercise the failure path without hardware.

**Tech Stack:** Rust 2024 edition (`kalico-ethercat-rt`, `motion-bridge`, `kalico-protocol`), Python 3 (`klippy/extras/ethercat_node.py`, `klippy/extras/servo_axis.py`, `klippy/motion_bridge.py`), `libc` crate for SIGTERM/SIGKILL.

---

## Task 1 — Wire: add `ClaimHandshake` message kind and per-slave status type to `kalico-protocol`

**Files:**
- Modify: `rust/kalico-protocol/src/messages.rs`
- Modify: `rust/kalico-protocol/src/lib.rs`

### 1-A — Write the failing test

Add to `rust/kalico-protocol/src/messages.rs` inside the existing `#[cfg(test)]` block at the bottom of the file:

```rust
#[cfg(test)]
mod claim_handshake_tests {
    use super::*;

    #[test]
    fn claim_handshake_reply_roundtrips_ok_slave() {
        let reply = ClaimHandshakeReply {
            slave_statuses: vec![SlaveStatus { slave_idx: 1, state: SlaveState::Ok, fault_code: 0 }],
        };
        let mut buf = Vec::new();
        reply.encode(&mut buf);
        let decoded = ClaimHandshakeReply::decode(&buf).unwrap();
        assert_eq!(decoded.slave_statuses.len(), 1);
        assert_eq!(decoded.slave_statuses[0].slave_idx, 1);
        assert_eq!(decoded.slave_statuses[0].state, SlaveState::Ok);
        assert_eq!(decoded.slave_statuses[0].fault_code, 0);
    }

    #[test]
    fn claim_handshake_reply_roundtrips_fault_slave() {
        let reply = ClaimHandshakeReply {
            slave_statuses: vec![SlaveStatus {
                slave_idx: 1,
                state: SlaveState::Fault,
                fault_code: 0x0102,
            }],
        };
        let mut buf = Vec::new();
        reply.encode(&mut buf);
        let decoded = ClaimHandshakeReply::decode(&buf).unwrap();
        assert_eq!(decoded.slave_statuses[0].state, SlaveState::Fault);
        assert_eq!(decoded.slave_statuses[0].fault_code, 0x0102);
    }

    #[test]
    fn unknown_slave_state_byte_is_hard_error() {
        // state=0xFF is not defined; must reject, not default-to-ok.
        let mut buf = Vec::new();
        buf.push(1u8); // slave_count = 1
        buf.push(1u8); // slave_idx = 1
        buf.push(0xFFu8); // state = unknown
        buf.extend_from_slice(&0u16.to_le_bytes()); // fault_code = 0
        let result = ClaimHandshakeReply::decode(&buf);
        assert!(result.is_err(), "unknown state byte must be a hard error");
    }

    #[test]
    fn empty_slave_list_is_hard_error() {
        let mut buf = Vec::new();
        buf.push(0u8); // slave_count = 0 — missing status list
        let result = ClaimHandshakeReply::decode(&buf);
        assert!(result.is_err(), "empty slave status list must be a hard error");
    }

    #[test]
    fn message_kind_claim_handshake_roundtrips() {
        let raw = MessageKind::ClaimHandshakeReply.as_u16();
        assert_eq!(MessageKind::from_u16(raw), Some(MessageKind::ClaimHandshakeReply));
    }
}
```

### 1-B — Run to fail

```sh
cd /path/to/kalico/rust && cargo test -p kalico-protocol 2>&1 | grep -E "error|FAILED"
```

Expected: compile error — `ClaimHandshakeReply`, `SlaveStatus`, `SlaveState`, `MessageKind::ClaimHandshakeReply` not found.

### 1-C — Implement

First, in `rust/kalico-protocol/src/codec.rs`, add a `BadDiscriminant` variant to `DecodeError` (after `TrailingBytes`, line ~26) and its `Display` arm:

```rust
    /// A field carried a value outside its defined discriminant set
    /// (e.g. an unknown SlaveState byte). Fail loudly — never default.
    BadDiscriminant { field: &'static str, raw: u32 },
```

```rust
            Self::BadDiscriminant { field, raw } => {
                write!(f, "bad discriminant for {field}: {raw:#x}")
            }
```

Then in `rust/kalico-protocol/src/messages.rs`, add after the `McuLog` impl block (after line 340 approximately):

```rust
// ClaimHandshakeReply (0x0090): sent once by the endpoint after bringup,
// before entering the DC loop. Contains one entry per slave.
// Fail loudly: slave_count == 0 or unknown state byte are hard DecodeError.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SlaveState {
    Ok      = 0x00,
    Offline = 0x01,
    Fault   = 0x02,
}

impl SlaveState {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x00 => Some(Self::Ok),
            0x01 => Some(Self::Offline),
            0x02 => Some(Self::Fault),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SlaveStatus {
    pub slave_idx:  u8,
    pub state:      SlaveState,
    pub fault_code: u16,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClaimHandshakeReply {
    pub slave_statuses: Vec<SlaveStatus>,
}

impl Encode for ClaimHandshakeReply {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.slave_statuses.len() as u8);
        for s in &self.slave_statuses {
            put_u8(out, s.slave_idx);
            put_u8(out, s.state as u8);
            put_u16(out, s.fault_code);
        }
    }
}

impl ClaimHandshakeReply {
    pub fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        let mut c = Cursor::new(buf);
        let count = get_u8(&mut c)?;
        if count == 0 {
            return Err(DecodeError::BadDiscriminant {
                field: "slave_count",
                raw: 0,
            });
        }
        let mut statuses = Vec::with_capacity(count as usize);
        for _ in 0..count {
            let slave_idx  = get_u8(&mut c)?;
            let state_raw  = get_u8(&mut c)?;
            let state = SlaveState::from_u8(state_raw).ok_or(DecodeError::BadDiscriminant {
                field: "SlaveState",
                raw: state_raw as u32,
            })?;
            let fault_code = get_u16(&mut c)?;
            statuses.push(SlaveStatus { slave_idx, state, fault_code });
        }
        Ok(Self { slave_statuses: statuses })
    }
}
```

Add `MessageKind::ClaimHandshakeReply = 0x0090` to the `MessageKind` enum and its `from_u16` match in `messages.rs`.

In `rust/kalico-protocol/src/lib.rs`, add to the `pub use messages::{...}` re-export line:
```rust
pub use messages::{
    ClaimHandshakeReply, FaultEvent, McuLog, MessageKind, PushPieces, PushPiecesResponse,
    RuntimeCapsResponse, SlaveState, SlaveStatus, StatusHeartbeat,
};
```

### 1-D — Run to pass

```sh
cd /path/to/kalico/rust && cargo test -p kalico-protocol
```

Expected: all tests pass including the new `claim_handshake_tests`.

### 1-E — Commit

```sh
git add rust/kalico-protocol/src/messages.rs rust/kalico-protocol/src/lib.rs
git commit -m "feat(protocol): add ClaimHandshakeReply message with per-slave status list"
```

---

## Task 2 — Wire helpers: add `claim_handshake_reply_frame` builder and `Command::ClaimHandshake` to `kalico-ethercat-rt`

**Files:**
- Modify: `rust/kalico-ethercat-rt/src/wire.rs` (add variant ~line 30, decoder ~line 60, builder ~end of file)

### 2-A — Write the failing test

Append to the `#[cfg(test)]` block at the bottom of `rust/kalico-ethercat-rt/src/wire.rs`:

```rust
    #[test]
    fn claim_handshake_reply_frame_decodes() {
        use kalico_protocol::messages::{ClaimHandshakeReply, SlaveState, SlaveStatus};
        use kalico_native_transport::frame::decode_frame;

        let reply = ClaimHandshakeReply {
            slave_statuses: vec![SlaveStatus {
                slave_idx: 1,
                state: SlaveState::Ok,
                fault_code: 0,
            }],
        };
        let frame = claim_handshake_reply_frame(0, &reply);
        let (chan, payload) = decode_frame(&frame).unwrap();
        assert_eq!(chan, CHANNEL_CONTROL);
        let (hdr, body) = decode_message_header(payload).unwrap();
        assert_eq!(
            MessageKind::from_u16(hdr.kind_raw),
            Some(MessageKind::ClaimHandshakeReply)
        );
        let decoded = ClaimHandshakeReply::decode(body).unwrap();
        assert_eq!(decoded.slave_statuses[0].state, SlaveState::Ok);
    }

    #[test]
    fn decode_command_yields_claim_handshake_variant() {
        let payload = frame_payload(MessageKind::ClaimHandshake, 99, &[]);
        match decode_command(0, &payload).unwrap() {
            Command::ClaimHandshake { correlation_id: 99 } => {}
            other => panic!("expected ClaimHandshake, got {other:?}"),
        }
    }
```

### 2-B — Run to fail

```sh
cd /path/to/kalico/rust && cargo test -p kalico-ethercat-rt 2>&1 | grep -E "error|FAILED"
```

Expected: compile errors for `Command::ClaimHandshake` and `claim_handshake_reply_frame`.

### 2-C — Implement

In `rust/kalico-ethercat-rt/src/wire.rs`:

Add `use kalico_protocol::messages::{ClaimHandshakeReply, SlaveState, SlaveStatus};` to the imports.

Add variant to `Command` enum (after `QueryRuntimeCaps`):
```rust
    ClaimHandshake {
        correlation_id: u32,
    },
```

In `decode_command`, add a match arm before the `Unknown` fallthrough:
```rust
        Some(MessageKind::ClaimHandshake) => Ok(Command::ClaimHandshake {
            correlation_id: cid,
        }),
```

Add the frame builder after `identify_response_frame`:
```rust
pub fn claim_handshake_reply_frame(cid: u32, reply: &ClaimHandshakeReply) -> Vec<u8> {
    let mut body = Vec::new();
    reply.encode(&mut body);
    control_frame(MessageKind::ClaimHandshakeReply, cid, &body)
}
```

Also add `ClaimHandshake = 0x0091` to `MessageKind` in `kalico-protocol` — this is the request the host sends before bringup begins; the endpoint answers with `ClaimHandshakeReply`.

### 2-D — Run to pass

```sh
cd /path/to/kalico/rust && cargo test -p kalico-ethercat-rt
```

### 2-E — Commit

```sh
git add rust/kalico-ethercat-rt/src/wire.rs rust/kalico-protocol/src/messages.rs
git commit -m "feat(wire): ClaimHandshake command + ClaimHandshakeReply frame builder"
```

---

## Task 3 — `FrameServer`: expose `disconnect_detected` and `respond_then_close`

**Files:**
- Modify: `rust/kalico-ethercat-rt/src/server.rs`

The endpoint needs to send the handshake reply and then exit on disconnect. `FrameServer` currently sets `self.conn = None` silently on EOF. We need callers to detect that.

### 3-A — Write the failing test

Append to `rust/kalico-ethercat-rt/tests/stub_loop.rs` (or a new file `tests/server_disconnect.rs`):

```rust
#[test]
fn frame_server_detects_client_disconnect() {
    use kalico_ethercat_rt::server::FrameServer;
    use std::os::unix::net::UnixStream;

    let socket_path = format!("/tmp/kalico-srv-disc-{}.sock", std::process::id());
    let _ = std::fs::remove_file(&socket_path);

    let mut server = FrameServer::bind(&socket_path).expect("bind");

    // Connect a client, then drop it.
    let client = UnixStream::connect(&socket_path).expect("connect");
    // Give the listener a tick to accept.
    std::thread::sleep(std::time::Duration::from_millis(5));
    let _ = server.poll_commands(); // triggers accept
    assert!(!server.client_disconnected(), "should not be disconnected yet");
    drop(client);
    std::thread::sleep(std::time::Duration::from_millis(5));
    let _ = server.poll_commands(); // reads EOF, sets flag
    assert!(server.client_disconnected(), "EOF must set disconnected flag");

    let _ = std::fs::remove_file(&socket_path);
}
```

### 3-B — Run to fail

```sh
cd /path/to/kalico/rust && cargo test -p kalico-ethercat-rt server_disconnect 2>&1 | grep -E "error|FAILED"
```

Expected: `client_disconnected` method not found.

### 3-C — Implement

In `rust/kalico-ethercat-rt/src/server.rs`, add a `disconnected: bool` field to `FrameServer` (init `false`), set it to `true` in `poll_commands` where `Ok(0)` is matched (the EOF branch), and expose:

```rust
pub fn client_disconnected(&self) -> bool {
    self.disconnected
}
```

Also add a `respond_then_accept_next` helper that after writing the frame, calls `conn = None` so the server is ready for a new connection on the next accept — this is used by the endpoint to close cleanly after sending the handshake when bringup fails:

```rust
pub fn respond_and_close(&mut self, frame: &[u8]) {
    self.respond(frame);
    self.conn = None;
    self.disconnected = true;
}
```

### 3-D — Run to pass

```sh
cd /path/to/kalico/rust && cargo test -p kalico-ethercat-rt
```

### 3-E — Commit

```sh
git add rust/kalico-ethercat-rt/src/server.rs rust/kalico-ethercat-rt/tests/
git commit -m "feat(server): expose client_disconnected() and respond_and_close() on FrameServer"
```

---

## Task 4 — hw endpoint: replace `exit(1)` with handshake-reply-then-exit; add SIGTERM handler and disconnect-triggered shutdown

**Files:**
- Modify: `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs`

This task is hw-feature-gated. All changes are inside `#[cfg(feature = "hw")]` blocks or under the existing `unsafe { ffi::... }` calls.

### 4-A — No separate test (hw binary cannot run in CI); review gate is manual on bench. Integration test coverage comes from Task 8 (stub analog).

### 4-B — Implement

Replace the bringup failure block (current lines 54–58):

```rust
    // Bind first so the bridge can connect immediately after spawn.
    let mut server = FrameServer::bind(&socket).expect("bind socket");
    eprintln!("ec-rt: socket {socket}, cycle {cycle_us}us, counts/mm {counts_per_mm}");

    // Install SIGTERM handler: set a flag the DC loop checks.
    // Module level (above main):
    //
    //   use std::sync::atomic::{AtomicBool, Ordering};
    //
    //   static SIGTERM_RECEIVED: AtomicBool = AtomicBool::new(false);
    //
    //   extern "C" fn on_sigterm(_: libc::c_int) {
    //       // SAFETY: storing to a static AtomicBool is async-signal-safe.
    //       SIGTERM_RECEIVED.store(true, Ordering::SeqCst);
    //   }
    //
    // In main, after the FrameServer bind:
    // SAFETY: on_sigterm only touches a static AtomicBool.
    unsafe {
        libc::signal(libc::SIGTERM, on_sigterm as libc::sighandler_t);
    }

    // Shared helper (in the binary, above main): wait for the bridge's
    // ClaimHandshake with a bounded deadline; returns its correlation_id.
    //
    //   fn wait_for_claim(
    //       server: &mut FrameServer,
    //       deadline: std::time::Instant,
    //   ) -> Option<u32> {
    //       loop {
    //           if std::time::Instant::now() >= deadline {
    //               return None;
    //           }
    //           for cmd in server.poll_commands() {
    //               if let Command::ClaimHandshake { correlation_id } = cmd {
    //                   return Some(correlation_id);
    //               }
    //           }
    //           std::thread::sleep(std::time::Duration::from_millis(1));
    //       }
    //   }
    //
    // And a reply helper:
    //
    //   fn slave1_reply(state: SlaveState, fault_code: u16) -> ClaimHandshakeReply {
    //       ClaimHandshakeReply {
    //           slave_statuses: vec![SlaveStatus { slave_idx: 1, state, fault_code }],
    //       }
    //   }

    // Bringup blocks until CiA402 operation-enabled (or error).
    let cif = CString::new(ifname.clone()).expect("ifname must not contain NUL");
    let rc = unsafe { ffi::ec_rt_bringup(cif.as_ptr(), cycle_ns, rt_cpu, rt_prio) };

    let claim_deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    if rc != 0 {
        eprintln!("ec-rt: bringup failed rc={rc}, sending handshake-fail then exiting");
        // Answer the bridge so it gets a named error instead of a timeout.
        if let Some(cid) = wait_for_claim(&mut server, claim_deadline) {
            let reply = slave1_reply(SlaveState::Offline, rc.unsigned_abs() as u16);
            server.respond_and_close(&claim_handshake_reply_frame(cid, &reply));
            eprintln!("ec-rt: sent offline handshake reply, exiting");
        } else {
            eprintln!("ec-rt: bridge did not send ClaimHandshake within 5 s; giving up");
        }
        std::process::exit(1);
    }
    eprintln!("ec-rt: drive enabled");

    // Send the ok handshake reply before entering the DC loop.
    match wait_for_claim(&mut server, claim_deadline) {
        Some(cid) => {
            server.respond(&claim_handshake_reply_frame(cid, &slave1_reply(SlaveState::Ok, 0)));
        }
        None => {
            eprintln!("ec-rt: bridge did not send ClaimHandshake within 5 s; aborting");
            unsafe {
                ffi::ec_rt_disable();
                ffi::ec_rt_shutdown();
            }
            std::process::exit(1);
        }
    }
    eprintln!("ec-rt: handshake ok, entering DC loop");
```

The DC loop's `wkc != 3` break and the fault-latch exit path both already call `ffi::ec_rt_disable()` and `ffi::ec_rt_shutdown()` — that pattern is correct and stays. Add a sigterm check and disconnect check at the top of the DC loop body:

```rust
    loop {
        if SIGTERM_RECEIVED.load(Ordering::SeqCst) {
            eprintln!("ec-rt: SIGTERM received — disabling drive and exiting");
            break;
        }
        if server.client_disconnected() {
            eprintln!("ec-rt: bridge disconnected — disabling drive and exiting");
            break;
        }
        // ... existing loop body unchanged ...
    }
```

The existing shutdown at the end of `main` (`ffi::ec_rt_disable()` / `ffi::ec_rt_shutdown()`) already covers this path.

### 4-C — Commit (after manual bench verification gate in Task 10)

```sh
git add rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs
git commit -m "feat(hw-endpoint): replace exit(1) with handshake-reply-then-exit; SIGTERM + disconnect shutdown"
```

---

## Task 5 — Stub: add `--fail-bringup slave=N` flag and matching handshake behavior

**Files:**
- Modify: `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt-stub.rs`

The stub must mirror the hw endpoint's socket-bind-then-handshake-then-loop structure so the same bridge spawn path exercises both binaries identically.

### 5-A — Write the failing test

Add to `rust/kalico-ethercat-rt/tests/` a new file `tests/stub_spawn.rs` (tests are in Task 8; this step only adds the `--fail-bringup` argument parsing test):

```rust
#[test]
fn stub_arg_fail_bringup_parses() {
    // Smoke test: the stub accepts --fail-bringup slave=1 without panicking.
    // Full integration testing is in stub_lifecycle.rs (Task 8).
    let args: Vec<String> = vec![
        "kalico-ethercat-rt-stub".into(),
        "--fail-bringup".into(),
        "slave=1".into(),
        "--socket".into(),
        "/tmp/stub-parse-smoke.sock".into(),
    ];
    // We can't run main() here, but we can call the arg parser directly
    // if we extract it to a pub fn. See Task 5-C.
    let fail_slave = parse_fail_bringup(&args);
    assert_eq!(fail_slave, Some(1u8));
}
```

This requires extracting the arg parsing to `pub fn parse_fail_bringup(args: &[String]) -> Option<u8>` in the stub binary (or a lib function). The simpler approach is a `pub(crate)` function in a new `src/stub_args.rs` module gated by `#[cfg(not(feature = "hw"))]`.

### 5-B — Run to fail

```sh
cd /path/to/kalico/rust && cargo test -p kalico-ethercat-rt stub_arg_fail_bringup 2>&1 | grep -E "error|FAILED"
```

### 5-C — Implement

Rewrite `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt-stub.rs` main to:

1. Parse `--fail-bringup slave=N` (N is the slave index, currently always 1).
2. Bind the socket (same `FrameServer::bind` call).
3. Wait for a `ClaimHandshake` command from the bridge (bounded 10 s or configurable).
4. If `fail_bringup_slave` is `Some(n)`, reply with `SlaveState::Offline` for that slave, then call `server.respond_and_close(...)` and `std::process::exit(1)`.
5. Otherwise reply `SlaveState::Ok` and enter the existing stub DC loop.
6. Add disconnect detection: at top of loop, `if server.client_disconnected() { break; }` — then the loop exits cleanly without needing SIGTERM.

```rust
fn main() {
    let args: Vec<String> = std::env::args().collect();
    let socket = arg_val(&args, "--socket")
        .unwrap_or_else(|| "/tmp/kalico-ethercat.sock".into());
    let fail_slave: Option<u8> = args.iter()
        .position(|a| a == "--fail-bringup")
        .and_then(|i| args.get(i + 1))
        .and_then(|v| v.strip_prefix("slave="))
        .and_then(|n| n.parse().ok());

    let mut ring = AxisRing::new();
    let mut last_sent_retired: u32 = 0;
    let mut heartbeat_sent = false;

    let mut server = FrameServer::bind(&socket).expect("bind socket");
    eprintln!("ec-rt-stub: socket {socket} (NO HARDWARE)");

    // Wait for the bridge to send ClaimHandshake.
    let handshake_deadline = std::time::Instant::now() + Duration::from_secs(10);
    let mut handshook = false;
    while !handshook {
        if std::time::Instant::now() >= handshake_deadline {
            eprintln!("ec-rt-stub: timed out waiting for ClaimHandshake");
            std::process::exit(1);
        }
        for cmd in server.poll_commands() {
            if let Command::ClaimHandshake { correlation_id } = cmd {
                let state = if fail_slave.is_some() {
                    SlaveState::Offline
                } else {
                    SlaveState::Ok
                };
                let reply = ClaimHandshakeReply {
                    slave_statuses: vec![SlaveStatus {
                        slave_idx: fail_slave.unwrap_or(1),
                        state,
                        fault_code: 0,
                    }],
                };
                if fail_slave.is_some() {
                    server.respond_and_close(&claim_handshake_reply_frame(correlation_id, &reply));
                    eprintln!("ec-rt-stub: sent offline reply (--fail-bringup), exiting");
                    std::process::exit(1);
                } else {
                    server.respond(&claim_handshake_reply_frame(correlation_id, &reply));
                    handshook = true;
                }
            }
        }
        if !handshook {
            sleep(Duration::from_millis(1));
        }
    }
    eprintln!("ec-rt-stub: handshake ok, entering loop");

    loop {
        if server.client_disconnected() {
            eprintln!("ec-rt-stub: bridge disconnected, exiting");
            break;
        }
        // ... existing loop body (poll_commands, ring.sample, heartbeat) ...
    }
}
```

### 5-D — Run to pass

```sh
cd /path/to/kalico/rust && cargo test -p kalico-ethercat-rt
```

All existing stub_loop.rs tests must still pass (the `ClaimHandshake` wait is bounded, and those tests talk to a `FrameServer` directly, not to the stub binary).

### 5-E — Commit

```sh
git add rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt-stub.rs
git commit -m "feat(stub): socket-bind-then-handshake flow; --fail-bringup slave=N simulation"
```

---

## Task 6 — `servo_axis.py`: add `encoder_counts_per_rev` config key; `ethercat_node.py`: add `interface` and `endpoint` config keys

The spec requires `counts_per_mm` to be computed from servo config and passed at spawn. Neither `rotation_distance / rev ÷ encoder_counts_per_rev` nor `interface` / `endpoint` exist yet in any config file.

**Files:**
- Modify: `klippy/extras/servo_axis.py`
- Modify: `klippy/extras/ethercat_node.py`

### 6-A — No automated klippy unit test harness exists for extras (there is no pytest suite for `klippy/extras/`). Verification is manual: a correctly-parsed config does not raise `config.error()`; a missing required key raises during klippy startup. Document the manual check in Task 6-D.

### 6-B — Implement in `servo_axis.py`

In `ServoRail.__init__`, after reading `rotation_distance`, add:

```python
        # Encoder counts per motor revolution. Used by [ethercat_node] to
        # compute --counts-per-mm for the endpoint binary.
        self.encoder_counts_per_rev = config.getint(
            "encoder_counts_per_rev", minval=1
        )

    def get_counts_per_mm(self):
        # counts_per_mm = encoder_counts_per_rev / rotation_distance
        return self.encoder_counts_per_rev / self.rotation_distance
```

### 6-C — Implement in `ethercat_node.py`

Replace the constructor to add `interface` (required), `endpoint` (optional, default to repo-relative release binary), and a `counts_per_mm` attribute populated at claim time:

```python
import logging
import os
import subprocess
import time


class EtherCatNode:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]

        socket_path = config.get("socket").strip()
        if not socket_path:
            raise config.error(
                "ethercat_node %s: 'socket' must be a non-empty path"
                % (self.name,)
            )
        self.socket_path = socket_path

        self.interface = config.get("interface").strip()
        if not self.interface:
            raise config.error(
                "ethercat_node %s: 'interface' must be a non-empty interface name"
                % (self.name,)
            )

        # Default: the repo-relative release binary, two directory levels up
        # from klippy/ (i.e. <repo>/rust/target/release/kalico-ethercat-rt).
        _repo_root = os.path.normpath(
            os.path.join(os.path.dirname(__file__), "..", "..")
        )
        _default_endpoint = os.path.join(
            _repo_root, "rust", "target", "release", "kalico-ethercat-rt"
        )
        self.endpoint = config.get("endpoint", _default_endpoint).strip()

        self.bridge_handle = None
        self._endpoint_proc = None  # set by _claim; cleared by _release
        self.printer.register_event_handler("klippy:mcu_identify", self._claim)
        self.printer.register_event_handler("klippy:disconnect", self._release)

    def _claim(self):
        if self.bridge_handle is not None:
            return
        bridge = self.printer.lookup_object("motion_bridge")

        # Compute counts_per_mm from the servo rail that references this node.
        counts_per_mm = self._resolve_counts_per_mm()

        try:
            handle, proc = bridge.claim_ethercat_node(
                self.name,
                self.socket_path,
                self.interface,
                self.endpoint,
                counts_per_mm,
            )
        except RuntimeError as e:
            raise self.printer.config_error(str(e))

        self.bridge_handle = handle
        self._endpoint_proc = proc
        logging.info(
            "ethercat_node %s: claimed handle=%s socket=%s",
            self.name,
            self.bridge_handle,
            self.socket_path,
        )

    def _release(self):
        bridge = self.printer.lookup_object("motion_bridge", None)
        if bridge is not None and self.bridge_handle is not None:
            bridge.release_mcu(self.bridge_handle)
        self.bridge_handle = None

    def _resolve_counts_per_mm(self):
        from . import servo_axis
        for obj_name in self.printer.lookup_objects(module="servo_axis"):
            try:
                rail = self.printer.lookup_object(obj_name)
            except Exception:
                continue
            if (
                isinstance(rail, servo_axis.ServoRail)
                and rail.get_node_name() == self.name
            ):
                return rail.get_counts_per_mm()
        raise self.printer.config_error(
            "ethercat_node %s: no [servo_*] section with node=%s — "
            "cannot derive counts_per_mm" % (self.name, self.name)
        )

    def get_bridge_handle(self):
        return self.bridge_handle
```

Note: `printer.lookup_objects(module=...)` iterates all config sections of that module prefix. If that API is not available, iterate `printer.objects` and filter by name prefix `"servo_"`.

### 6-D — Manual verification steps (no automated harness)

These checks are performed on the Pi after the motion-bridge rebuild (Task 7):

1. Config missing `interface:` → klippy startup prints `"ethercat_node node_x: 'interface' must be a non-empty interface name"` and stops.
2. Config missing `encoder_counts_per_rev:` in `[servo_x]` → klippy prints a config error for that option and stops.
3. Valid config → klippy reaches `ready`; `klippy.log` shows `"ethercat_node node_x: claimed handle=1"`.

### 6-E — Commit

```sh
git add klippy/extras/ethercat_node.py klippy/extras/servo_axis.py
git commit -m "feat(config): interface + endpoint options on ethercat_node; encoder_counts_per_rev on servo_axis"
```

---

## Task 7 — Bridge: `claim_ethercat_node` spawns, polls, handshakes; `McuConnection` owns the child; `release_mcu` tears it down

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (~lines 32–46 `McuConnection`, ~line 574 `claim_ethercat_node`, ~line 598 `release_mcu`)
- Modify: `rust/motion-bridge/Cargo.toml` (no new deps needed; `libc` is already a dep)

### 7-A — Write the failing test

Add to `rust/motion-bridge/src/bridge.rs` in the `#[cfg(test)]` module at the bottom of the file:

```rust
    #[test]
    fn spawn_ethercat_node_reports_endpoint_not_found() {
        // A non-existent binary path must surface as a hard error string,
        // not a panic or silent ignore.
        let result = spawn_ethercat_endpoint(
            "/nonexistent/kalico-ethercat-rt",
            "eth0",
            "/tmp/kalico-test-missing.sock",
            3276.8,
        );
        assert!(result.is_err(), "missing binary must error");
        let msg = result.unwrap_err();
        assert!(
            msg.contains("spawn") || msg.contains("No such file") || msg.contains("not found"),
            "error must describe spawn failure, got: {msg}"
        );
    }
```

### 7-B — Run to fail

```sh
cd /path/to/kalico/rust && cargo test -p motion-bridge spawn_ethercat_node 2>&1 | grep -E "error|FAILED"
```

Expected: `spawn_ethercat_endpoint` function not found.

### 7-C — Implement

**`McuConnection` struct** — add two new fields after `ethercat_socket`:

```rust
    ethercat_socket:   Option<String>,
    endpoint_process:  Option<std::process::Child>,
    endpoint_conn:     Option<std::sync::Arc<kalico_host_rt::unix_native_conn::UnixNativeConn>>,
```

`endpoint_conn` is `Arc` because `init_planner` MUST reuse this exact connection
(see 7-C-bis): the endpoint exits when its client disconnects, so dropping the
handshake connection and re-connecting later would kill the endpoint between
claim and `init_planner`. One socket, one client, one connection for the whole
session. The serial-MCU `McuConnection` initializer (~line 567) gains
`endpoint_process: None, endpoint_conn: None`.

**New free function `spawn_ethercat_endpoint`** (private, `pub(crate)` for tests):

```rust
fn spawn_ethercat_endpoint(
    binary: &str,
    interface: &str,
    socket_path: &str,
    counts_per_mm: f64,
) -> Result<std::process::Child, String> {
    std::process::Command::new(binary)
        .arg(interface)
        .arg("--socket")
        .arg(socket_path)
        .arg("--counts-per-mm")
        .arg(counts_per_mm.to_string())
        .spawn()
        .map_err(|e| format!("spawn {binary}: {e}"))
}
```

**New free function `poll_socket_ready`**:

```rust
fn poll_socket_ready(path: &str, deadline: std::time::Instant) -> Result<(), String> {
    loop {
        if std::time::Instant::now() >= deadline {
            return Err(format!("endpoint socket {path} did not appear within deadline"));
        }
        if std::path::Path::new(path).exists() {
            return Ok(());
        }
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
}
```

**New free function `handshake_ethercat_endpoint`**:

```rust
use kalico_host_rt::unix_native_conn::UnixNativeConn;
use kalico_protocol::messages::{ClaimHandshakeReply, SlaveState};
use kalico_protocol::MessageKind;
use kalico_host_rt::native_call::NativeCall;

#[derive(Debug)]
enum EndpointClaimError {
    BusDead,
    DriveOffline { slave_idx: u8, fault_code: u16 },
    DriveFault   { slave_idx: u8, fault_code: u16 },
    Protocol(String),
}

impl std::fmt::Display for EndpointClaimError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::BusDead => write!(f, "no slaves responding on EtherCAT bus"),
            Self::DriveOffline { slave_idx, .. } =>
                write!(f, "drive (slave {slave_idx}) offline — check drive power, then FIRMWARE_RESTART"),
            Self::DriveFault { slave_idx, fault_code } =>
                write!(f, "drive (slave {slave_idx}) fault 0x{fault_code:04x} — check drive, then FIRMWARE_RESTART"),
            Self::Protocol(s) => write!(f, "endpoint protocol error: {s}"),
        }
    }
}

fn handshake_ethercat_endpoint(
    socket_path: &str,
    deadline: std::time::Instant,
) -> Result<UnixNativeConn, EndpointClaimError> {
    let remaining = deadline.saturating_duration_since(std::time::Instant::now());
    if remaining.is_zero() {
        return Err(EndpointClaimError::Protocol("handshake deadline exceeded before connect".into()));
    }
    let conn = UnixNativeConn::connect(socket_path)
        .map_err(|e| EndpointClaimError::Protocol(format!("connect {socket_path}: {e}")))?;

    let (kind, body) = conn
        .kalico_call(MessageKind::ClaimHandshake, vec![], remaining)
        .map_err(|e| EndpointClaimError::Protocol(format!("ClaimHandshake call: {e:?}")))?;

    if kind != MessageKind::ClaimHandshakeReply {
        return Err(EndpointClaimError::Protocol(
            format!("expected ClaimHandshakeReply, got {kind:?}"),
        ));
    }

    let reply = ClaimHandshakeReply::decode(&body)
        .map_err(|e| EndpointClaimError::Protocol(format!("decode ClaimHandshakeReply: {e:?}")))?;

    for s in &reply.slave_statuses {
        match s.state {
            SlaveState::Ok => {}
            SlaveState::Offline => {
                return Err(EndpointClaimError::DriveOffline {
                    slave_idx:  s.slave_idx,
                    fault_code: s.fault_code,
                });
            }
            SlaveState::Fault => {
                return Err(EndpointClaimError::DriveFault {
                    slave_idx:  s.slave_idx,
                    fault_code: s.fault_code,
                });
            }
        }
    }

    Ok(conn)
}
```

**Updated `claim_ethercat_node` PyO3 method signature and body**:

```rust
    #[pyo3(signature = (label, socket_path, interface, endpoint_binary, counts_per_mm))]
    fn claim_ethercat_node(
        &self,
        label:            &str,
        socket_path:      &str,
        interface:        &str,
        endpoint_binary:  &str,
        counts_per_mm:    f64,
    ) -> PyResult<u32> {
        const SOCKET_POLL_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(15);
        const HANDSHAKE_TIMEOUT:   std::time::Duration = std::time::Duration::from_secs(10);

        let mut child = spawn_ethercat_endpoint(endpoint_binary, interface, socket_path, counts_per_mm)
            .map_err(|e| PyRuntimeError::new_err(
                format!("ethercat {label}: endpoint failed to start — {e}")
            ))?;

        let socket_deadline = std::time::Instant::now() + SOCKET_POLL_TIMEOUT;
        if let Err(e) = poll_socket_ready(socket_path, socket_deadline) {
            let _ = child.kill();
            let _ = child.wait();
            return Err(PyRuntimeError::new_err(
                format!("ethercat {label}: {e}")
            ));
        }

        let handshake_deadline = std::time::Instant::now() + HANDSHAKE_TIMEOUT;
        let conn = match handshake_ethercat_endpoint(socket_path, handshake_deadline) {
            Ok(c) => c,
            Err(e) => {
                let _ = child.kill();
                let _ = child.wait();
                let msg = match &e {
                    EndpointClaimError::DriveOffline { slave_idx, .. } =>
                        format!("ethercat {label}: drive '{label}' (slave {slave_idx}) offline — check drive power, then FIRMWARE_RESTART"),
                    EndpointClaimError::DriveFault { slave_idx, fault_code } =>
                        format!("ethercat {label}: drive '{label}' (slave {slave_idx}) fault 0x{fault_code:04x} — check drive, then FIRMWARE_RESTART"),
                    EndpointClaimError::BusDead =>
                        format!("ethercat {label}: no slaves responding on {interface}"),
                    EndpointClaimError::Protocol(s) =>
                        format!("ethercat {label}: endpoint protocol error — {s}"),
                };
                return Err(PyRuntimeError::new_err(msg));
            }
        };

        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let handle = router.claim_mcu(label);
        let raw = handle.raw();
        self.mcus.lock().unwrap_or_else(|p| p.into_inner()).insert(
            raw,
            McuConnection {
                label:             label.to_owned(),
                serial_path:       String::new(),
                baud:              0,
                host_io:           None,
                runtime_rx:        None,
                runtime_caps:      None,
                identify_caps:     0,
                kalico_native_supported: true,
                clock_sync_stop:   None,
                clock_sync_thread: None,
                clock_sync_estimator: None,
                ethercat_socket:   Some(socket_path.to_owned()),
                endpoint_process:  Some(child),
                endpoint_conn:     Some(std::sync::Arc::new(conn)),
            },
        );
        Ok(raw)
    }
```

**7-C-bis — `init_planner` reuses the claim connection.** In the `ec_conns`
block (bridge.rs ~lines 1725–1766), replace the fresh
`UnixNativeConn::connect(&socket)` with the stored connection — connecting a
second time would race the endpoint's disconnect-detection. Collect
`(handle, Arc<UnixNativeConn>)` pairs instead of `(handle, socket_string)`:

```rust
        let ec_conns: HashMap<u32, Arc<UnixNativeConn>> = {
            let conn_by_id: Vec<(u32, Arc<UnixNativeConn>)> = {
                let mcus_lock = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
                mcus.iter()
                    .filter_map(|(handle, _, _)| {
                        mcus_lock
                            .get(handle)
                            .and_then(|c| c.endpoint_conn.as_ref().map(|conn| (*handle, Arc::clone(conn))))
                    })
                    .collect()
            };

            let mut out = HashMap::new();
            for (mcu_id, conn) in conn_by_id {
                let caps = query_ethercat_runtime_caps(&conn, std::time::Duration::from_secs(5))
                    .map_err(|e| {
                        PyRuntimeError::new_err(format!(
                            "init_planner: QueryRuntimeCaps failed for ethercat mcu \
                                 {mcu_id}: {e} — endpoint must respond with \
                                 RuntimeCapsResponse; is kalico-ethercat-rt running?"
                        ))
                    })?;
                log::debug!(
                    "[caps-trace] init_planner: ethercat mcu {mcu_id} caps \
                     total_piece_memory={}",
                    caps.total_piece_memory,
                );
                {
                    let mut mcus_lock = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
                    if let Some(c) = mcus_lock.get_mut(&mcu_id) {
                        c.runtime_caps = Some(caps);
                    }
                }
                out.insert(mcu_id, conn);
            }
            out
        };
```

If `query_ethercat_runtime_caps` takes `&UnixNativeConn`, `&conn` derefs from
the `Arc` unchanged. The two other `ethercat_socket.is_some()` checks
(bridge.rs ~1797, ~2333) stay as they are — `ethercat_socket` remains the
"is this an ethercat node" marker.

**Updated `release_mcu`** — after the existing clock-sync teardown, before the router call, add endpoint teardown:

```rust
    fn release_mcu(&self, handle: u32) -> PyResult<()> {
        let (stop, join, mut child) = {
            let mut mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn_opt = mcus.remove(&handle);
            match conn_opt {
                Some(mut c) => (c.clock_sync_stop.take(), c.clock_sync_thread.take(), c.endpoint_process.take()),
                None => (None, None, None),
            }
        };
        if let Some(stop) = stop {
            stop.store(true, Ordering::Release);
        }
        if let Some(join) = join {
            let _ = join.join();
        }

        // Tear down the endpoint process: SIGTERM first, bounded wait, SIGKILL backstop.
        if let Some(ref mut proc) = child {
            #[cfg(unix)]
            {
                let pid = proc.id();
                unsafe { libc::kill(pid as libc::pid_t, libc::SIGTERM) };
            }
            let reap_deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
            loop {
                match proc.try_wait() {
                    Ok(Some(_)) => break,
                    Ok(None) if std::time::Instant::now() >= reap_deadline => {
                        eprintln!("ethercat endpoint did not exit after SIGTERM — sending SIGKILL");
                        let _ = proc.kill();
                        let _ = proc.wait();
                        break;
                    }
                    Ok(None) => std::thread::sleep(std::time::Duration::from_millis(50)),
                    Err(e) => {
                        eprintln!("ethercat endpoint wait error: {e}");
                        break;
                    }
                }
            }
        }

        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router.release_mcu(mcu_handle_from_raw(handle));
        self.handlers
            .lock()
            .unwrap()
            .retain(|&(mcu, _, _), _| mcu != handle);
        Ok(())
    }
```

### 7-D — Run to pass

```sh
cd /path/to/kalico/rust && cargo test -p motion-bridge
```

### 7-E — Update `klippy/motion_bridge.py`

The Python wrapper for `claim_ethercat_node` currently passes two arguments; update to five:

```python
    def claim_ethercat_node(self, label, socket_path, interface, endpoint, counts_per_mm):
        return self._bridge.claim_ethercat_node(
            label, socket_path, interface, endpoint, counts_per_mm
        )
```

### 7-F — Commit

```sh
git add rust/motion-bridge/src/bridge.rs klippy/motion_bridge.py
git commit -m "feat(bridge): claim_ethercat_node spawns endpoint, handshakes, maps per-slave errors; release_mcu tears down process"
```

---

## Task 8 — Integration tests: spawn lifecycle (no orphan) and bringup-failure propagation

**Files:**
- Create: `rust/kalico-ethercat-rt/tests/stub_lifecycle.rs`

These tests compile and run the stub binary from the test binary, so they require `cargo build -p kalico-ethercat-rt --bin kalico-ethercat-rt-stub` to have been run, or a `build.rs` / `xtask` step that guarantees it. Use `env!("CARGO_BIN_EXE_kalico-ethercat-rt-stub")` to locate the binary (cargo sets this for `[[bin]]` targets in the same workspace).

Note: `env!("CARGO_BIN_EXE_kalico-ethercat-rt-stub")` is set by Cargo when compiling tests within the same package that defines the binary. Since `stub_lifecycle.rs` is an integration test in `kalico-ethercat-rt`, and `kalico-ethercat-rt-stub` is a `[[bin]]` in that package, Cargo guarantees the binary is built and the env var is set.

### 8-A — Write the failing tests (both at once)

Create `rust/kalico-ethercat-rt/tests/stub_lifecycle.rs`:

```rust
use std::os::unix::net::UnixStream;
use std::time::Duration;
use kalico_host_rt::native_call::NativeCall;
use kalico_host_rt::unix_native_conn::UnixNativeConn;
use kalico_protocol::messages::{ClaimHandshakeReply, SlaveState, MessageKind};
use kalico_protocol::codec::Encode;
use kalico_native_transport::wire_helpers::{encode_message_header, MESSAGE_VERSION_DEFAULT};
use kalico_native_transport::frame::encode_frame;
use kalico_native_transport::frame::CHANNEL_CONTROL;

fn stub_bin() -> &'static str {
    env!("CARGO_BIN_EXE_kalico-ethercat-rt-stub")
}

fn send_claim_handshake(conn: &UnixNativeConn, timeout: Duration)
    -> Result<ClaimHandshakeReply, String>
{
    let (kind, body) = conn
        .kalico_call(MessageKind::ClaimHandshake, vec![], timeout)
        .map_err(|e| format!("{e:?}"))?;
    if kind != MessageKind::ClaimHandshakeReply {
        return Err(format!("expected ClaimHandshakeReply got {kind:?}"));
    }
    ClaimHandshakeReply::decode(&body).map_err(|e| format!("{e:?}"))
}

#[test]
fn stub_claim_succeeds_and_disconnect_terminates_process() {
    let id = std::process::id();
    let socket_path = format!("/tmp/kalico-lifecycle-ok-{id}.sock");
    let _ = std::fs::remove_file(&socket_path);

    let mut child = std::process::Command::new(stub_bin())
        .arg("--socket").arg(&socket_path)
        .spawn()
        .expect("spawn stub");

    // Poll for socket (up to 5 s).
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    loop {
        if std::path::Path::new(&socket_path).exists() { break; }
        assert!(
            std::time::Instant::now() < deadline,
            "stub socket did not appear"
        );
        std::thread::sleep(Duration::from_millis(10));
    }

    let conn = UnixNativeConn::connect(&socket_path).expect("connect");
    let reply = send_claim_handshake(&conn, Duration::from_secs(5))
        .expect("ClaimHandshakeReply");
    assert_eq!(reply.slave_statuses.len(), 1);
    assert_eq!(reply.slave_statuses[0].state, SlaveState::Ok, "stub must reply Ok");

    // Drop connection: this triggers disconnect detection in the stub.
    drop(conn);

    // Stub must exit within 3 s (no orphan).
    let exit_deadline = std::time::Instant::now() + Duration::from_secs(3);
    loop {
        match child.try_wait().expect("try_wait") {
            Some(status) => {
                // Any exit is acceptable (0 or non-zero due to test scaffolding).
                let _ = status;
                break;
            }
            None if std::time::Instant::now() >= exit_deadline => {
                child.kill().ok();
                panic!("stub did not exit after client disconnect — orphan process");
            }
            None => std::thread::sleep(Duration::from_millis(50)),
        }
    }

    let _ = std::fs::remove_file(&socket_path);
}

#[test]
fn stub_fail_bringup_propagates_offline_error() {
    let id = std::process::id();
    let socket_path = format!("/tmp/kalico-lifecycle-fail-{id}.sock");
    let _ = std::fs::remove_file(&socket_path);

    let mut child = std::process::Command::new(stub_bin())
        .arg("--socket").arg(&socket_path)
        .arg("--fail-bringup").arg("slave=1")
        .spawn()
        .expect("spawn stub with --fail-bringup");

    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    loop {
        if std::path::Path::new(&socket_path).exists() { break; }
        assert!(std::time::Instant::now() < deadline, "stub socket did not appear");
        std::thread::sleep(Duration::from_millis(10));
    }

    let conn = UnixNativeConn::connect(&socket_path).expect("connect");
    let reply = send_claim_handshake(&conn, Duration::from_secs(5))
        .expect("ClaimHandshakeReply");
    assert_eq!(reply.slave_statuses.len(), 1);
    assert_eq!(
        reply.slave_statuses[0].state, SlaveState::Offline,
        "--fail-bringup must produce Offline in the handshake reply"
    );

    // Stub must exit on its own after sending the failure reply.
    let exit_deadline = std::time::Instant::now() + Duration::from_secs(3);
    loop {
        match child.try_wait().expect("try_wait") {
            Some(_) => break,
            None if std::time::Instant::now() >= exit_deadline => {
                child.kill().ok();
                panic!("stub did not exit after sending failure reply");
            }
            None => std::thread::sleep(Duration::from_millis(50)),
        }
    }

    let _ = std::fs::remove_file(&socket_path);
}
```

### 8-B — Run to fail

```sh
cd /path/to/kalico/rust && cargo test -p kalico-ethercat-rt stub_lifecycle 2>&1 | grep -E "error|FAILED"
```

Expected: compile errors from undefined types until Tasks 1–5 complete; after those tasks, tests should fail at runtime (stub doesn't have `ClaimHandshake` handling yet at this point in the sequence).

### 8-C — Run to pass (after Tasks 1–5 are complete)

```sh
cd /path/to/kalico/rust && cargo test -p kalico-ethercat-rt stub_lifecycle
```

Both tests must pass. The existing `stub_loop.rs` and `real_socket_streaming.rs` tests must also still pass (run the full suite):

```sh
cd /path/to/kalico/rust && cargo test -p kalico-ethercat-rt
```

### 8-D — Commit

```sh
git add rust/kalico-ethercat-rt/tests/stub_lifecycle.rs
git commit -m "test(stub): integration tests — spawn/handshake/disconnect lifecycle and bringup-failure propagation"
```

---

## Task 9 — Build: `setcap` target in `Makefile.kalico` for the hw binary

**Files:**
- Modify: `Makefile.kalico`

### 9-A — No test. The target is a bench-side manual step; CI must not require it. Verification is: `getcap rust/target/release/kalico-ethercat-rt` on the bench shows the expected capabilities.

### 9-B — Implement

Add to `Makefile.kalico`:

```makefile
# Hardware EtherCAT endpoint — builds only with --features hw on a Pi with
# the SOEM/libecrt prerequisites. Never built in CI.
ethercat-endpoint-hw:
	cd rust && cargo build -p kalico-ethercat-rt --features hw --bin kalico-ethercat-rt --release

# Apply capabilities so the endpoint can run unprivileged.
# Requires sudo once per binary build. Run this on the bench after every rebuild
# that changes the binary. Not run in CI (no sudo, no hw feature).
setcap-ethercat:
	sudo setcap cap_net_raw,cap_sys_nice,cap_ipc_lock+ep \
		rust/target/release/kalico-ethercat-rt
	@echo "setcap applied to rust/target/release/kalico-ethercat-rt"

.PHONY: ethercat-endpoint-hw setcap-ethercat
```

The bench flow becomes:
```
make -f Makefile.kalico ethercat-endpoint-hw
make -f Makefile.kalico setcap-ethercat   # sudo, once per binary rebuild
```

### 9-C — Commit

```sh
git add Makefile.kalico
git commit -m "build: Makefile.kalico targets for hw endpoint build and setcap"
```

---

## Task 10 — Docs: update `ethercat-bench-bringup.md`; note bench-host deletion of `bench-hw-up.sh`

**Files:**
- Modify: `docs/kalico-rewrite/ethercat-bench-bringup.md`

### 10-A — No test.

### 10-B — Implement

Replace the "Bring-up sequence" section of `ethercat-bench-bringup.md` with:

```markdown
## Bring-up sequence

### 1. Deploy + flash (low risk — stepper path unchanged)
- Commit/push the branch; pull on the Pi; build there (never cross-compile + scp).
- Flash **both** MCUs (H7 from `.config.h7.bak`, F446 from `.config.f446.test`);
  `make clean` between them. `make -j$(nproc)`.

### 2. Build the motion-bridge and stub
```sh
make -f Makefile.kalico motion-bridge
cd rust && cargo build -p kalico-ethercat-rt --bin kalico-ethercat-rt-stub --release
```

### 3. Stub validation FIRST (no drive — zero hardware risk)
Update `[ethercat_node node_x]` in your printer.cfg:
```ini
[ethercat_node node_x]
socket:    /tmp/kalico-ethercat.sock
interface: eth0                                     # the EtherCAT NIC (ignored by stub)
endpoint:  /home/pi/kalico/rust/target/release/kalico-ethercat-rt-stub
encoder_counts_per_rev: 131072                       # your drive's encoder resolution
```
Start klipper. klippy now spawns the stub automatically, connects, and handshakes.
Confirm it reaches **`ready`**.
- `SET_KINEMATIC_POSITION X=100`, then `G1 X105 F600`, `M400`.
- Watch stub's stderr: `PushPieces` frames arrive, `retired` counts advance.

### 4. Real drive (supervised)
Build the hw endpoint:
```sh
make -f Makefile.kalico ethercat-endpoint-hw
make -f Makefile.kalico setcap-ethercat     # sudo — once per rebuild
```
Update `endpoint:` in `[ethercat_node node_x]` to the hw binary path:
```ini
endpoint: /home/pi/kalico/rust/target/release/kalico-ethercat-rt
```
- **Dark-drive test:** start klipper with the drive off. You should see:
  `ethercat node_x: drive 'node_x' (slave 1) offline — check drive power, then FIRMWARE_RESTART`
  This confirms the handshake failure path works.
- Power the drive. `FIRMWARE_RESTART`. klippy re-spawns the endpoint; if the
  drive reaches operation-enabled, klippy reaches `ready`.
- Do a small supervised jog. Watch for `engine_state == Fault (3)` or `wkc != 3`.

### 5. Recovery
Any fault or restart: `FIRMWARE_RESTART` — klippy tears down and re-spawns the endpoint cleanly.
No manual pre-launch step is needed. `bench-hw-up.sh` is no longer present on the bench host.
```

Also update the "Sample config" section to include `interface:`, `endpoint:`, and `encoder_counts_per_rev:`.

Remove the `bench-hw-up.sh` deletion note (it is a bench-side one-time manual step: `rm ~/bin/bench-hw-up.sh` — mention it in the docs but do not perform it from this plan's implementation steps, since it is not a repo file).

### 10-C — Commit

```sh
git add docs/kalico-rewrite/ethercat-bench-bringup.md
git commit -m "docs(bench): update ethercat bringup to spawn-on-claim flow; drop bench-hw-up.sh reference"
```

---

## Task 11 — Final wiring: rebuild and verify full suite

**Files:** (none new; this task runs the tests)

### 11-A — Run the full Rust test suite

```sh
cd /path/to/kalico/rust && cargo test -p kalico-protocol -p kalico-ethercat-rt -p motion-bridge
```

All tests must pass. Specifically confirm:
- `kalico-protocol`: `claim_handshake_tests::*` (4 tests)
- `kalico-ethercat-rt`: existing `stub_loop`, `real_socket_streaming`, new `stub_lifecycle` (2 tests), new `server_disconnect` (1 test)
- `motion-bridge`: existing tests + `spawn_ethercat_node_reports_endpoint_not_found`

### 11-B — Verify clippy::pedantic compliance

```sh
cd /path/to/kalico/rust && cargo clippy -p kalico-protocol -p kalico-ethercat-rt -p motion-bridge -- -D warnings
```

### 11-C — Final commit

```sh
git add .  # any remaining unstaged fixups
git commit -m "chore: final wiring — all spawn-on-claim tasks complete, full suite green"
```

---

## Out of scope (per spec)

De-energize-and-track: the `ec_rt_disable()` primitive added in Task 4 is the building block; the host-side flow is future work and is not planned here.

---

## Self-review

**Spec coverage check:**

| Spec requirement | Task(s) |
|---|---|
| Endpoint: replace exit(1) with handshake-reply-then-exit | Task 4 |
| SIGTERM handler + socket-disconnect → disable+shutdown+exit | Task 4 |
| Stub analog of SIGTERM/disconnect handling | Task 5 |
| Wire: per-slave status list `[(slave_idx, state)]` | Task 1, 2 |
| Fail loudly on unknown state / missing list | Task 1 (tests), Task 7 (handshake mapping) |
| Bridge: spawn configured binary with `<interface>`, `--socket`, `--counts-per-mm` | Task 7 |
| Bridge: poll socket with bounded deadline | Task 7 |
| Bridge: handshake with bounded deadline; hung endpoint killed | Task 7 |
| Bridge: per-slave status → typed errors | Task 7 |
| Bridge: release/restart → close socket, SIGTERM, bounded reap, SIGKILL backstop | Task 7 |
| klippy: `interface` (required) and `endpoint` (default) config options | Task 6 |
| klippy: claim errors raise startup error with verbatim named-drive message | Task 7 (error strings), Task 6 (Python caller) |
| `counts_per_mm` from servo config, not hardcoded | Task 6 (`encoder_counts_per_rev` in servo_axis) |
| Build: `setcap` as Makefile target; note sudo; CI safe | Task 9 |
| Stub: same spawn machinery; `--fail-bringup slave=1` | Task 5 |
| Test: claim spawns process, handshake succeeds, disconnect terminates | Task 8 |
| Test: bringup-failure propagates named-drive error | Task 8 |
| Existing tests untouched | All tasks (verified in Task 11) |
| Docs: update `ethercat-bench-bringup.md` | Task 10 |
| Docs: note `bench-hw-up.sh` deletion (bench-host step) | Task 10 |
| Out of scope: de-energize-and-track | Not planned |

**Placeholder scan:** None. Every code block uses real type names from the codebase (`FrameServer`, `UnixNativeConn`, `ClaimHandshakeReply`, `McuConnection`, `PyMotionBridge`, `MessageKind`, `kalico_call`, `SlaveState`).

**Type consistency:** `ClaimHandshakeReply` and `SlaveState` are defined once in `kalico-protocol/src/messages.rs` and re-exported from `kalico-protocol/src/lib.rs`. Both `kalico-ethercat-rt/src/wire.rs` and `motion-bridge/src/bridge.rs` import from `kalico_protocol::messages::*`. No divergent definitions.

---

## Deviations from spec forced by existing code

1. **`ClaimHandshake` is a new `MessageKind` (request from bridge to endpoint).** The spec describes a handshake "reply" but does not specify the request direction. The existing `Identify` / `QueryRuntimeCaps` pattern is: bridge sends a command, endpoint replies. The plan follows that pattern — bridge sends `ClaimHandshake` (0x0091), endpoint replies with `ClaimHandshakeReply` (0x0090). This means the endpoint sits in its pre-loop waiting for the bridge to initiate, which aligns with "socket-bind-before-bringup" and the natural bridge-poll-then-connect order.

2. **`DecodeError` gains a `BadDiscriminant { field, raw }` variant** (Task 1-C, `codec.rs`). Abusing `ArrayLengthExceedsBuffer` for unknown state bytes would produce a misleading error message — the fail-loudly constraint requires the error to say what actually went wrong.

3. **Python `printer.lookup_objects(module=...)` API.** The plan calls this API in `ethercat_node.py._resolve_counts_per_mm()`. If this method does not exist on the klippy `printer` object, filter `printer.objects` by the prefix `"servo_"` instead. The fallback is noted inline.

4. **Single connection per endpoint — RESOLVED.** `init_planner` (bridge.rs ~1740) previously opened its own `UnixNativeConn` per ethercat socket. Under spawn-on-claim that would race the endpoint's disconnect-detection (the endpoint exits when its client drops). Resolution: the claim handshake's connection is stored as `Arc<UnixNativeConn>` in `McuConnection.endpoint_conn` and `init_planner` reuses it (step 7-C-bis). The connection lives exactly as long as the session; its drop in `release_mcu` is the endpoint's disconnect signal, backstopped by SIGTERM.
