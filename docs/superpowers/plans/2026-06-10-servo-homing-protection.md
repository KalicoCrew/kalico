# Servo Homing Protection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Homing-scoped drive protection (following-error window `6065h` + torque cap `6072h`) for EtherCAT servo axes, with a limit trip surfacing as a G28 error instead of a session kill. Spec: `docs/superpowers/specs/2026-06-10-servo-homing-protection-design.md`.

**Architecture:** Two new endpoint commands (`SetDriveLimits` / `RestoreDriveLimits`) do SDO writes; the endpoint stops exiting on drive faults — it parks a new `Faulted` torque-gate state and reports the drive's error code via the heartbeat `fault_code` field; the bridge routes that by context (homing active → homing-error channel; idle → fatal abort). klippy wraps the servo trip move with set-limits / restore-in-finally.

**Tech Stack:** Rust (kalico-protocol, kalico-ethercat-rt, kalico-host-rt, motion-bridge), C (`bench/libecrt.c`, Pi-only build), Python (klippy), cargo nextest, pytest.

**Branch:** continue on `servo-homing` (in the worktree).

**Build-system fact:** the hw endpoint binary has `required-features = ["hw"]` and links `bench/libecrt.a` + SOEM — it does NOT build on macOS. All local `cargo nextest run` invocations cover the lib, stub, and tests; the hw binary + C changes are compile-verified on the Pi (`make -f Makefile.kalico ethercat-endpoint-hw`) in Task 9.

**Conventions:** no explanatory comments; fail loudly; tests in separate files; `cargo nextest run` from `rust/`; commit messages without any Claude/Anthropic trailer.

---

### Task 1: Protocol — four new messages

**Files:**
- Modify: `rust/kalico-protocol/src/messages.rs`
- Test: `rust/kalico-protocol/src/messages/tests.rs`

- [ ] **Step 1.1: Failing tests** — append to `messages/tests.rs`:

```rust
#[test]
fn set_drive_limits_round_trips() {
    let msg = SetDriveLimits {
        following_error_counts: 8192,
        max_torque_tenth_pct: 500,
    };
    let bytes = msg.encoded_to_vec();
    let decoded = SetDriveLimits::decode(&bytes).unwrap();
    assert_eq!(decoded, msg);
}

#[test]
fn drive_limits_responses_round_trip() {
    let r = SetDriveLimitsResponse { result: -315 };
    assert_eq!(
        SetDriveLimitsResponse::decode(&r.encoded_to_vec()).unwrap(),
        r
    );
    let r = RestoreDriveLimitsResponse { result: 0 };
    assert_eq!(
        RestoreDriveLimitsResponse::decode(&r.encoded_to_vec()).unwrap(),
        r
    );
}

#[test]
fn drive_limits_message_kinds_round_trip() {
    for kind in [
        MessageKind::SetDriveLimits,
        MessageKind::SetDriveLimitsResponse,
        MessageKind::RestoreDriveLimits,
        MessageKind::RestoreDriveLimitsResponse,
    ] {
        assert_eq!(MessageKind::from_u16(kind.as_u16()), Some(kind));
    }
}
```

(Match the import style at the top of the existing tests file.)

- [ ] **Step 1.2: Verify red** — `cargo nextest run -p kalico-protocol -E 'test(drive_limits)'` → compile error.

- [ ] **Step 1.3: Implement** in `messages.rs`:

Add to `MessageKind` (after `StopResponse = 0x0073`) and to `from_u16`:

```rust
    SetDriveLimits = 0x0074,
    SetDriveLimitsResponse = 0x0075,
    RestoreDriveLimits = 0x0076,
    RestoreDriveLimitsResponse = 0x0077,
```

Add structs + codecs following the `SetTorque`/`StopResponse` patterns exactly (same put_/get_ helpers):

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SetDriveLimits {
    pub following_error_counts: u32,
    pub max_torque_tenth_pct: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SetDriveLimitsResponse {
    pub result: i32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RestoreDriveLimits;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RestoreDriveLimitsResponse {
    pub result: i32,
}
```

`RestoreDriveLimits` encodes/decodes as an empty body (copy the `Stop` impl shape).

- [ ] **Step 1.4: Verify green** — `cargo nextest run -p kalico-protocol` all pass.
- [ ] **Step 1.5: Commit** — `protocol: SetDriveLimits / RestoreDriveLimits message pairs`

---

### Task 2: Endpoint wire — decode the commands, heartbeat carries fault_code

**Files:**
- Modify: `rust/kalico-ethercat-rt/src/wire.rs`
- Test: `rust/kalico-ethercat-rt/src/wire/tests.rs`

- [ ] **Step 2.1: Failing tests** — append:

```rust
#[test]
fn decodes_set_drive_limits_command() {
    let msg = SetDriveLimits {
        following_error_counts: 8192,
        max_torque_tenth_pct: 500,
    };
    let payload = frame_payload(MessageKind::SetDriveLimits, 3, &msg.encoded_to_vec());
    match decode_command(0, &payload).unwrap() {
        Command::SetDriveLimits {
            correlation_id: 3,
            msg: m,
        } => {
            assert_eq!(m.following_error_counts, 8192);
            assert_eq!(m.max_torque_tenth_pct, 500);
        }
        other => panic!("expected SetDriveLimits, got {other:?}"),
    }
}

#[test]
fn decodes_restore_drive_limits_command() {
    let payload = frame_payload(MessageKind::RestoreDriveLimits, 4, &[]);
    match decode_command(0, &payload).unwrap() {
        Command::RestoreDriveLimits { correlation_id: 4 } => {}
        other => panic!("expected RestoreDriveLimits, got {other:?}"),
    }
}

#[test]
fn drive_limits_response_frames_round_trip() {
    let frame = set_drive_limits_response_frame(6, -315);
    let (chan, payload) = decode_frame(&frame).unwrap();
    assert_eq!(chan, CHANNEL_CONTROL);
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(hdr.correlation_id, 6);
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::SetDriveLimitsResponse)
    );
    assert_eq!(SetDriveLimitsResponse::decode(body).unwrap().result, -315);

    let frame = restore_drive_limits_response_frame(7, 0);
    let (_, payload) = decode_frame(&frame).unwrap();
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::RestoreDriveLimitsResponse)
    );
    assert_eq!(
        RestoreDriveLimitsResponse::decode(body).unwrap().result,
        0
    );
    assert_eq!(hdr.correlation_id, 7);
}

#[test]
fn status_heartbeat_frame_carries_fault_code() {
    let frame = status_heartbeat_frame(1, 0x8611, &[5u32]);
    let (_, payload) = decode_frame(&frame).unwrap();
    let (_, body) = decode_message_header(payload).unwrap();
    let hb = StatusHeartbeat::decode(body).unwrap();
    assert_eq!(hb.fault_code, 0x8611);
    assert_eq!(hb.engine_state, 1);
}
```

- [ ] **Step 2.2: Verify red** — `cargo nextest run -p kalico-ethercat-rt --lib` (bins will break on the new heartbeat arity until Task 3/5 — like the Stop work, fold bin fixes into the later commits if the package won't build; run `--lib` scoped meanwhile).

- [ ] **Step 2.3: Implement** in `wire.rs`:

- Import `RestoreDriveLimitsResponse, SetDriveLimits, SetDriveLimitsResponse` (RestoreDriveLimits has no body to decode).
- `Command` variants:

```rust
    SetDriveLimits {
        correlation_id: u32,
        msg: SetDriveLimits,
    },
    RestoreDriveLimits {
        correlation_id: u32,
    },
```

- Decode arms (before Unknown), mirroring `SetTorque` / `Stop`.
- Response frame builders mirroring `set_torque_response_frame`:

```rust
pub fn set_drive_limits_response_frame(cid: u32, result: i32) -> Vec<u8> {
    let body = SetDriveLimitsResponse { result }.encoded_to_vec();
    control_frame(MessageKind::SetDriveLimitsResponse, cid, &body)
}

pub fn restore_drive_limits_response_frame(cid: u32, result: i32) -> Vec<u8> {
    let body = RestoreDriveLimitsResponse { result }.encoded_to_vec();
    control_frame(MessageKind::RestoreDriveLimitsResponse, cid, &body)
}
```

- `status_heartbeat_frame(engine_state: u8, fault_code: u16, retired_counts: &[u32])` — new middle parameter feeding the existing `fault_code` field (today hardcoded 0). Update all callers in wire tests; bin callers are updated in Tasks 3/5.

- [ ] **Step 2.4: Verify green** — `cargo nextest run -p kalico-ethercat-rt --lib` (or full package if bins already fixed).
- [ ] **Step 2.5: Commit** (may fold with Task 3 if bins block) — `ethercat-rt: drive-limits commands on the wire; heartbeat fault_code`

---

### Task 3: TorqueGate — `Faulted` state

Drive faults park the gate in a third state that: accepts enable (the CiA ladder pulses fault reset), accepts a scheduled disable (drive already de-energized; the ramp is a no-op landing in Parked), does NOT raise pieces-while-parked, and rejects nothing silently. New error code `ERR_PIECES_WHILE_FAULTED: i32 = -314` for pieces arriving while Faulted (used by the bins to reject PushPieces without exiting).

**Files:**
- Modify: `rust/kalico-ethercat-rt/src/torque.rs`
- Test: `rust/kalico-ethercat-rt/src/torque/tests.rs`

- [ ] **Step 3.1: Failing tests** — append to `torque/tests.rs`:

```rust
#[test]
fn drive_fault_parks_in_faulted_and_clears_pending_disable() {
    let mut g = TorqueGate::new();
    assert_eq!(g.on_set_torque(true, 0, 0), CommandAction::Enable);
    g.enable_finished(true);
    assert_eq!(g.on_set_torque(false, 100, 50), CommandAction::ScheduleDisable);
    g.on_drive_fault();
    assert_eq!(g.state(), TorqueState::Faulted);
    assert_eq!(g.on_tick(200, true), TickAction::None);
}

#[test]
fn faulted_tick_with_pieces_is_not_a_fault() {
    let mut g = TorqueGate::new();
    g.on_drive_fault();
    assert_eq!(g.on_tick(0, false), TickAction::None);
}

#[test]
fn enable_from_faulted_recovers() {
    let mut g = TorqueGate::new();
    g.on_drive_fault();
    assert_eq!(g.on_set_torque(true, 0, 0), CommandAction::Enable);
    g.enable_finished(true);
    assert_eq!(g.state(), TorqueState::Enabled);
}

#[test]
fn disable_from_faulted_schedules_and_lands_parked() {
    let mut g = TorqueGate::new();
    g.on_drive_fault();
    assert_eq!(
        g.on_set_torque(false, 100, 50),
        CommandAction::ScheduleDisable
    );
    assert_eq!(g.on_tick(150, true), TickAction::ExecuteDisable);
    g.disable_finished();
    assert_eq!(g.state(), TorqueState::Parked);
}
```

- [ ] **Step 3.2: Verify red** — `cargo nextest run -p kalico-ethercat-rt -E 'test(faulted) or test(drive_fault)'`.

- [ ] **Step 3.3: Implement** in `torque.rs`:

```rust
pub const ERR_PIECES_WHILE_FAULTED: i32 = -314;
```

- `TorqueState` gains `Faulted`.
- New method:

```rust
    pub fn on_drive_fault(&mut self) {
        self.state = TorqueState::Faulted;
        self.pending_disable_at = None;
    }
```

- `on_set_torque`: enable branch — change the reject condition so only `Enabled`-without-pending-disable rejects (Faulted and Parked both proceed to `Enable`). Disable branch — accept from `Enabled` or `Faulted` (without pending disable); keep the in-past and double-schedule rejections.
- `on_tick`: the `Parked && !ring_empty` fault must NOT fire for `Faulted` (match on `TorqueState::Parked` specifically). Pending-disable handling stays; when the deadline passes in Faulted with a non-empty ring, that is still `Fault { ERR_PIECES_WHILE_PARKED }`? No — in Faulted the bins reject pieces at arrival, so the ring is empty by construction; keep the existing pending-disable logic untouched apart from state checks.

- [ ] **Step 3.4: Verify green** — full `cargo nextest run -p kalico-ethercat-rt --lib`.
- [ ] **Step 3.5: Commit** — `ethercat-rt: TorqueGate Faulted state (drive fault parks, enable recovers)`

---

### Task 4: C `libecrt` + FFI — SDO limit read/write

C changes are compile-verified on the Pi in Task 9; keep them minimal and mirror the file's existing SDO style (`bench/libecrt.c:114-127`).

**Files:**
- Modify: `bench/libecrt.c`, `bench/libecrt.h`
- Modify: `rust/kalico-ethercat-rt/src/ffi.rs`

- [ ] **Step 4.1: `bench/libecrt.h`** — add below `ec_rt_get_following_error`:

```c
/* SDO-read 6065h/6066h/6072h. 0 on success; -1/-2/-3 per failing object. */
int ec_rt_read_limits(uint32_t *ferr_counts, uint16_t *ferr_timeout_ms,
                      uint16_t *torque_tenth_pct);

/* SDO-write 6065h and 6072h. 0 on success; -1/-2 per failing object. */
int ec_rt_write_limits(uint32_t ferr_counts, uint16_t torque_tenth_pct);
```

- [ ] **Step 4.2: `bench/libecrt.c`** — in the bringup SDO phase (beside the `0x6060` write at line ~123), add:

```c
    uint16_t ferr_timeout_ms = 0;
    ec_SDOwrite(1, 0x6066, 0x00, FALSE, sizeof(ferr_timeout_ms),
                &ferr_timeout_ms, EC_TIMEOUTRXM);
```

and add the two functions (file scope, near the other getters):

```c
int ec_rt_read_limits(uint32_t *ferr_counts, uint16_t *ferr_timeout_ms,
                      uint16_t *torque_tenth_pct)
{
    int sz = sizeof(*ferr_counts);
    if (ec_SDOread(1, 0x6065, 0x00, FALSE, &sz, ferr_counts, EC_TIMEOUTRXM) <= 0)
        return -1;
    sz = sizeof(*ferr_timeout_ms);
    if (ec_SDOread(1, 0x6066, 0x00, FALSE, &sz, ferr_timeout_ms, EC_TIMEOUTRXM) <= 0)
        return -2;
    sz = sizeof(*torque_tenth_pct);
    if (ec_SDOread(1, 0x6072, 0x00, FALSE, &sz, torque_tenth_pct, EC_TIMEOUTRXM) <= 0)
        return -3;
    return 0;
}

int ec_rt_write_limits(uint32_t ferr_counts, uint16_t torque_tenth_pct)
{
    if (ec_SDOwrite(1, 0x6065, 0x00, FALSE, sizeof(ferr_counts), &ferr_counts,
                    EC_TIMEOUTRXM) <= 0)
        return -1;
    if (ec_SDOwrite(1, 0x6072, 0x00, FALSE, sizeof(torque_tenth_pct),
                    &torque_tenth_pct, EC_TIMEOUTRXM) <= 0)
        return -2;
    return 0;
}
```

(SDO mailbox transfers briefly pause PDO exchange — acceptable: limits are only written at bringup and around homing, always at standstill, and the drive's SM watchdog tolerates ~100 ms gaps. State this in the commit message body, not a comment.)

- [ ] **Step 4.3: `ffi.rs`** — add to the extern block:

```rust
    pub fn ec_rt_read_limits(
        ferr_counts: *mut u32,
        ferr_timeout_ms: *mut u16,
        torque_tenth_pct: *mut u16,
    ) -> c_int;

    pub fn ec_rt_write_limits(ferr_counts: u32, torque_tenth_pct: u16) -> c_int;
```

- [ ] **Step 4.4: Verify** — `cargo check -p kalico-ethercat-rt` (non-hw surface unaffected); C compile deferred to Task 9.
- [ ] **Step 4.5: Commit** — `libecrt: SDO read/write for 6065h/6072h protection limits; 6066h=0 at bringup`

---

### Task 5: Endpoint binaries — limits commands, drive-fault-without-exit

**Files:**
- Modify: `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs`
- Modify: `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt-stub.rs`
- Test: `rust/kalico-ethercat-rt/tests/torque_lifecycle.rs`

- [ ] **Step 5.1: Failing integration tests (stub)** — add helpers + tests to `torque_lifecycle.rs`:

```rust
fn set_drive_limits(conn: &UnixNativeConn, counts: u32, tenth_pct: u16) -> i32 {
    let body = SetDriveLimits {
        following_error_counts: counts,
        max_torque_tenth_pct: tenth_pct,
    }
    .encoded_to_vec();
    let (kind, resp) = conn
        .kalico_call(MessageKind::SetDriveLimits, body, Duration::from_secs(5))
        .expect("SetDriveLimits call must succeed");
    assert_eq!(kind, MessageKind::SetDriveLimitsResponse);
    SetDriveLimitsResponse::decode(&resp).expect("decode").result
}

fn restore_drive_limits(conn: &UnixNativeConn) -> i32 {
    let (kind, resp) = conn
        .kalico_call(
            MessageKind::RestoreDriveLimits,
            Vec::new(),
            Duration::from_secs(5),
        )
        .expect("RestoreDriveLimits call must succeed");
    assert_eq!(kind, MessageKind::RestoreDriveLimitsResponse);
    RestoreDriveLimitsResponse::decode(&resp).expect("decode").result
}
```

```rust
#[test]
fn drive_limits_set_and_restore_round_trip() {
    let (mut guard, conn, path) = spawn_and_claim("limits-rt", &[]);
    assert_eq!(set_drive_limits(&conn, 8192, 500), 0);
    assert_eq!(restore_drive_limits(&conn), 0);
    assert_eq!(set_drive_limits(&conn, 4096, 300), 0);
    drop(conn);
    let _ = guard.defuse().wait();
    let _ = std::fs::remove_file(&path);
}

#[test]
fn simulated_drive_fault_parks_keeps_serving_and_recovers() {
    let (mut guard, conn, path) =
        spawn_and_claim("drive-fault", &["--drive-fault-after-pieces", "1"]);

    let r = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(r, 0);
    push_one_piece(&conn, now_ns());

    thread::sleep(Duration::from_millis(100));

    let r = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(
        r, 0,
        "enable from Faulted must run the ladder and return 0, got {r}"
    );

    drop(conn);
    let _ = guard.defuse().wait();
    let _ = std::fs::remove_file(&path);
}
```

(The second test proves: fault did not kill the process; the gate reached Faulted — a plain re-enable from Enabled would return -312, so result 0 after the fault is only possible via Faulted→Enable.)

- [ ] **Step 5.2: Verify red** — `cargo nextest run -p kalico-ethercat-rt -E 'test(drive_limits) or test(drive_fault)'`.

- [ ] **Step 5.3: Implement — stub** (`kalico-ethercat-rt-stub.rs`):

- Parse `--drive-fault-after-pieces` (optional u32) via `arg_val`.
- Track `sampled_pieces: u32` — increment when `ring.sample(now)` returns `Some`.
- When the threshold is reached (once): `gate.on_drive_fault(); ring.reset();` emit `status_heartbeat_frame(0, 0x8611, &[ring.retired_count()])`; eprintln a `drive fault simulated` line; DO NOT exit.
- `Command::SetDriveLimits` → remember `(counts, tenth_pct)` in a local, respond `set_drive_limits_response_frame(cid, 0)`, eprintln the values.
- `Command::RestoreDriveLimits` → respond `restore_drive_limits_response_frame(cid, 0)`, eprintln.
- `Command::PushPieces` while `gate.state() == TorqueState::Faulted` → do not push; respond with `result = ERR_PIECES_WHILE_FAULTED` (import from torque module).
- Update the two existing `status_heartbeat_frame` call sites for the new arity (walker-fault site passes its fault code low 16 bits: `(fault_val & 0xFFFF) as u16`; steady-state site passes 0).

- [ ] **Step 5.4: Implement — hw binary** (`kalico-ethercat-rt.rs`):

- After successful bringup, before the claim wait: read + log prior limits, apply session-wide CLI values if present:

```rust
    let run_limits: Option<(u32, u16)> = {
        let mut ferr = 0u32;
        let mut tmo = 0u16;
        let mut tq = 0u16;
        let rc = unsafe { ffi::ec_rt_read_limits(&mut ferr, &mut tmo, &mut tq) };
        if rc != 0 {
            eprintln!("ec-rt: SDO read of protection limits failed rc={rc} — aborting bringup");
            unsafe {
                ffi::ec_rt_disable();
                ffi::ec_rt_shutdown();
            }
            std::process::exit(1);
        }
        eprintln!("ec-rt: drive limits at bringup: 6065h={ferr} counts, 6066h={tmo} ms, 6072h={tq} (0.1%)");
        let cli_ferr: Option<u32> = arg_val(&args, "--following-error-counts").and_then(|s| s.parse().ok());
        let cli_tq: Option<u16> = arg_val(&args, "--max-torque-tenth-pct").and_then(|s| s.parse().ok());
        let run = (cli_ferr.unwrap_or(ferr), cli_tq.unwrap_or(tq));
        if cli_ferr.is_some() || cli_tq.is_some() {
            let rc = unsafe { ffi::ec_rt_write_limits(run.0, run.1) };
            if rc != 0 {
                eprintln!("ec-rt: SDO write of session limits failed rc={rc} — aborting bringup");
                unsafe {
                    ffi::ec_rt_disable();
                    ffi::ec_rt_shutdown();
                }
                std::process::exit(1);
            }
            eprintln!("ec-rt: session limits applied: 6065h={} 6072h={}", run.0, run.1);
        }
        Some(run)
    };
    let run_limits = run_limits.expect("limits read or exit");
```

- `Command::SetDriveLimits` → `ec_rt_write_limits(msg.following_error_counts, msg.max_torque_tenth_pct)`; rc → respond result (0 or rc); on rc != 0 respond and continue (host turns it into a homing error), eprintln either way.
- `Command::RestoreDriveLimits` → `ec_rt_write_limits(run_limits.0, run_limits.1)`; respond rc.
- `Command::PushPieces` while Faulted → reject with `ERR_PIECES_WHILE_FAULTED`, don't push (same as stub).
- **Drive-fault policy change:** the telemetry-block `if err != 0 { ... break; }` moves out of the 500 ms telemetry block into an every-cycle check, and stops breaking the loop:

```rust
        let drive_err = unsafe { ffi::ec_rt_get_error_code() };
        if drive_err != 0 && gate.state() != TorqueState::Faulted {
            eprintln!("ec-rt: DRIVE FAULT err=0x{drive_err:04x} — parking, reporting via heartbeat");
            gate.on_drive_fault();
            ring.reset();
            cmap = None;
            server.respond(&status_heartbeat_frame(0, drive_err, &[ring.retired_count()]));
        }
```

(The drive de-energized itself; no `ec_rt_disable()` needed, and the DC loop keeps running PDO so SDO restore + the recovery enable still work.)
- Update remaining `status_heartbeat_frame` call sites for the new arity (walker-fault: low 16 bits of `fault_val`; steady-state: 0).
- The `#[cfg(feature = "hw")]` walker-fault exit behavior stays exactly as is.

- [ ] **Step 5.5: Verify green** — `cargo nextest run -p kalico-ethercat-rt` (stub tests; hw bin compiles only on Pi — `cargo check -p kalico-ethercat-rt` locally does not cover it, that's Task 9).
- [ ] **Step 5.6: Commit** — `ethercat-rt: drive-limits commands; drive fault parks and reports instead of exiting`

---

### Task 6: kalico-host-rt — heartbeat callback carries the full StatusHeartbeat

**Files:**
- Modify: `rust/kalico-host-rt/src/unix_native_conn.rs` (callback type + `dispatch_frame`, lines ~296-316)
- Modify: caller in `rust/motion-bridge/src/bridge.rs:2330` (adapt closure signature; behavior change lands in Task 7)
- Test: `rust/kalico-host-rt/src/unix_native_conn/tests.rs` (existing tests module)

- [ ] **Step 6.1: Failing test** — in the conn tests, find the existing `dispatch_frame`/heartbeat test and add:

```rust
#[test]
fn dispatch_frame_passes_fault_code_to_callback() {
    // Build a StatusHeartbeat frame with fault_code 0x8611 using the same
    // encode helpers the existing heartbeat test uses, dispatch it, and
    // assert the callback received engine_state, fault_code, and counts.
}
```

Write it concretely against the existing test-file idiom (read the file first; it already constructs heartbeat frames for the retired-counts test — extend that pattern, asserting `hb.fault_code == 0x8611`).

- [ ] **Step 6.2: Implement** — change the callback type from `dyn Fn(&[u32]) + Send + Sync` to `dyn Fn(&StatusHeartbeat) + Send + Sync` everywhere in the conn (field, `attach_heartbeat_callback`, `dispatch_frame` body: `cb(&hb)`). Update the bridge call site mechanically:

```rust
                conn.attach_heartbeat_callback(Arc::new(move |hb: &StatusHeartbeat| {
                    let _ = pump_tx_hb.send(crate::pump::PumpMsg::Heartbeat(
                        crate::pump::HeartbeatMsg {
                            mcu_id,
                            retired_counts: hb.retired_counts.clone(),
                        },
                    ));
                    for (axis, &r) in hb.retired_counts.iter().enumerate() {
                        drain_hb.set_retired(mcu_id, axis as u8, r);
                    }
                }));
```

(with the import added; fault routing is Task 7).

- [ ] **Step 6.3: Verify** — `cargo nextest run -p kalico-host-rt -p motion-bridge` green.
- [ ] **Step 6.4: Commit** — `host-rt: heartbeat callback receives the full StatusHeartbeat`

---

### Task 7: Bridge — fault routing + drive-limits pyfunctions + spawn args

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs`
- Modify: `rust/motion-bridge/src/servo_torque.rs` (add the two send helpers beside `send_set_torque`)
- Test: `rust/motion-bridge/src/homing/tests.rs` (routing decision fn) and `rust/motion-bridge/src/servo_torque/tests.rs`

- [ ] **Step 7.1: Failing tests** — routing decision as a pure function in `homing.rs`:

```rust
mod drive_fault_routing_tests {
    use crate::homing::{route_drive_fault, DriveFaultRoute};

    #[test]
    fn homing_active_on_faulting_mcu_routes_to_homing_error() {
        assert_eq!(
            route_drive_fault(7, Some(7)),
            DriveFaultRoute::HomingError
        );
    }

    #[test]
    fn homing_on_other_mcu_is_fatal() {
        assert_eq!(route_drive_fault(7, Some(3)), DriveFaultRoute::Fatal);
    }

    #[test]
    fn idle_fault_is_fatal() {
        assert_eq!(route_drive_fault(7, None), DriveFaultRoute::Fatal);
    }
}
```

- [ ] **Step 7.2: Implement routing** in `homing.rs`:

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveFaultRoute {
    HomingError,
    Fatal,
}

pub fn route_drive_fault(fault_mcu: u32, homing_axis_mcu: Option<u32>) -> DriveFaultRoute {
    if homing_axis_mcu == Some(fault_mcu) {
        DriveFaultRoute::HomingError
    } else {
        DriveFaultRoute::Fatal
    }
}
```

- [ ] **Step 7.3: Wire it into the heartbeat callback** (bridge.rs:2330 site, extended from Task 6). Clone `Arc`s of `self.homing_run`, `self.active_drip_cohort`, and the pump sender into the closure. On `hb.fault_code != 0`:

```rust
                    if hb.fault_code != 0 {
                        let run_opt = {
                            let mut guard =
                                homing_run_hb.lock().unwrap_or_else(|p| p.into_inner());
                            match guard.as_ref().map(|r| r.axis_key.mcu_id) {
                                Some(axis_mcu)
                                    if crate::homing::route_drive_fault(
                                        mcu_id,
                                        Some(axis_mcu),
                                    ) == crate::homing::DriveFaultRoute::HomingError =>
                                {
                                    guard.take()
                                }
                                _ => None,
                            }
                        };
                        match run_opt {
                            Some(run) => {
                                *active_cohort_hb
                                    .lock()
                                    .unwrap_or_else(|p| p.into_inner()) = None;
                                let _ = pump_tx_fault
                                    .send(crate::pump::PumpMsg::Flush(run.all_axis_keys.clone()));
                                let _ = pump_tx_fault
                                    .send(crate::pump::PumpMsg::DripDisarm(run.cohort));
                                let _ = run.notify.send(Err(format!(
                                    "drive fault 0x{:04x} during homing — \
                                     following-error/torque limit exceeded (endstop failure?)",
                                    hb.fault_code
                                )));
                            }
                            None => {
                                tracing::error!(
                                    mcu_id,
                                    fault_code = hb.fault_code,
                                    "EXIT_ON_FAULT — ethercat drive fault outside homing; \
                                     aborting klippy so systemd restarts it"
                                );
                                let _ = std::io::Write::flush(&mut std::io::stderr());
                                if std::env::var_os("KALICO_NO_EXIT_ON_FAULT").is_none() {
                                    std::process::abort();
                                }
                            }
                        }
                        return;
                    }
```

(Variable names: clone the Arcs before the closure as `homing_run_hb`, `active_cohort_hb`, `pump_tx_fault`. Match the abort style of `on_endpoint_death` exactly.)

- [ ] **Step 7.4: send helpers + pyfunctions.** In `servo_torque.rs`, beside `send_set_torque` (read it first; same shape):

```rust
pub fn send_drive_limits(
    conn: &UnixNativeConn,
    following_error_counts: u32,
    max_torque_tenth_pct: u16,
) -> Result<i32, String>

pub fn send_restore_drive_limits(conn: &UnixNativeConn) -> Result<i32, String>
```

each doing `kalico_call` with the matching MessageKind, decoding the matching response, returning `result`. Add round-trip-style unit tests only if `servo_torque/tests.rs` already tests `send_set_torque` against a fake — otherwise rely on the stub integration tests (read the file and follow it).

In `bridge.rs`, mirror the `set_torque` pyfunction (lines ~803-850):

```rust
fn set_drive_limits(&self, mcu_handle: u32, following_error_counts: u32, max_torque_tenth_pct: u16) -> PyResult<()>
fn restore_drive_limits(&self, mcu_handle: u32) -> PyResult<()>
```

— look up `endpoint_conn` (same error text shape as set_torque's "not an EtherCAT endpoint"), call the helper, nonzero result → PyRuntimeError naming the SDO failure. Log via `tracing::info!(subsystem = "bridge", event = "servo_drive_limits", ...)` like `servo_torque_command`.

- [ ] **Step 7.5: spawn args.** `claim_ethercat_node` gains two trailing optional params `following_error_counts: Option<u32>, max_torque_tenth_pct: Option<u16>`; `spawn_ethercat_endpoint` appends `--following-error-counts N` / `--max-torque-tenth-pct N` when present. Update `klippy/motion_bridge.py`'s wrapper passthrough (read it; it's a thin pyo3 surface).

- [ ] **Step 7.6: Verify** — `cargo nextest run -p motion-bridge -p kalico-host-rt -p kalico-ethercat-rt` green; `cargo clippy -p motion-bridge --all-targets` clean.
- [ ] **Step 7.7: Commit** — `motion-bridge: drive-fault routing (homing error vs fatal) + drive-limits plumbing`

---

### Task 8: klippy — config, conversions, homing wrap

**Files:**
- Modify: `klippy/extras/servo_axis.py`
- Modify: `klippy/extras/ethercat_node.py`
- Modify: `klippy/extras/homing.py`
- Test: `test/test_servo_homing.py`

- [ ] **Step 8.1: Failing tests** — append:

```python
def make_homing_servo_rail():
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.axis = "x"
    rail.name = "servo_x"
    rail.rotation_distance = 40.0
    rail.encoder_counts_per_rev = 131072
    rail.homing_following_error = 2.5
    rail.homing_max_torque = 50.0
    return rail


def test_homing_drive_limits_convert_units():
    rail = make_homing_servo_rail()
    counts, tenth_pct = rail.get_homing_drive_limits()
    assert counts == 8192
    assert tenth_pct == 500


class FakeLimitsBridge:
    def __init__(self):
        self.calls = []

    def set_drive_limits(self, handle, counts, tenth_pct):
        self.calls.append(("set", handle, counts, tenth_pct))

    def restore_drive_limits(self, handle):
        self.calls.append(("restore", handle))


def test_homing_limits_guard_sets_and_restores():
    bridge = FakeLimitsBridge()
    with homing_mod._servo_drive_limits(bridge, 7, (8192, 500)):
        assert bridge.calls == [("set", 7, 8192, 500)]
    assert bridge.calls == [("set", 7, 8192, 500), ("restore", 7)]


def test_homing_limits_guard_restores_on_error():
    bridge = FakeLimitsBridge()
    try:
        with homing_mod._servo_drive_limits(bridge, 7, (8192, 500)):
            raise RuntimeError("trip move failed")
    except RuntimeError:
        pass
    assert bridge.calls[-1] == ("restore", 7)


def test_homing_limits_guard_noop_without_limits():
    bridge = FakeLimitsBridge()
    with homing_mod._servo_drive_limits(bridge, None, None):
        pass
    assert bridge.calls == []
```

- [ ] **Step 8.2: Verify red**, then implement:

**servo_axis.py** — in the `endstop_pin is not None` branch of `__init__`:

```python
            self.homing_following_error = config.getfloat(
                "homing_following_error", 2.5, above=0.0
            )
            self.homing_max_torque = config.getfloat(
                "homing_max_torque", 50.0, above=0.0, maxval=400.0
            )
```

(and in the None branch set both to `0.0`). Session-wide options, read unconditionally in `__init__`:

```python
        self.following_error = config.getfloat("following_error", None, above=0.0)
        self.max_torque = config.getfloat("max_torque", None, above=0.0, maxval=400.0)
```

Conversion methods:

```python
    def get_homing_drive_limits(self):
        counts_per_mm = self.encoder_counts_per_rev / self.rotation_distance
        return (
            int(round(self.homing_following_error * counts_per_mm)),
            int(round(self.homing_max_torque * 10.0)),
        )

    def get_session_drive_limits(self):
        counts_per_mm = self.encoder_counts_per_rev / self.rotation_distance
        counts = None
        if self.following_error is not None:
            counts = int(round(self.following_error * counts_per_mm))
        tenth_pct = None
        if self.max_torque is not None:
            tenth_pct = int(round(self.max_torque * 10.0))
        return counts, tenth_pct
```

**ethercat_node.py** — `_claim` derives the session limits from its rail (extend `_derive_counts_per_mm`'s rail walk or add a `_find_rail` helper returning the rail; derive both counts_per_mm and session limits from it) and passes them to `bridge.claim_ethercat_node(...)` as the two new arguments.

**homing.py** — module-level context manager below `_enable_homing_motors`:

```python
import contextlib


@contextlib.contextmanager
def _servo_drive_limits(bridge, handle, limits):
    if handle is None or limits is None:
        yield
        return
    bridge.set_drive_limits(handle, limits[0], limits[1])
    try:
        yield
    finally:
        bridge.restore_drive_limits(handle)
```

(put the `import contextlib` at the top with the other imports). In `_home_axis`, wrap the trip move:

```python
        servo_handle = None
        servo_limits = None
        if hasattr(rail, "get_node_name"):
            node = self.printer.lookup_object(
                "ethercat_node " + rail.get_node_name()
            )
            servo_handle = node.get_bridge_handle()
            servo_limits = rail.get_homing_drive_limits()

        with _servo_drive_limits(bridge, servo_handle, servo_limits):
            trip_pos, final_pos = self.trip_move(
                gcmd, toolhead, bridge, axis, direction, speed, max_travel, entry
            )
        if servo_handle is not None:
            fault = bridge.take_drive_fault(servo_handle)
            if fault is not None:
                raise gcmd.error(
                    "%s homing: drive fault 0x%04x at endstop contact — "
                    "following-error/torque limit exceeded" % ("XYZ"[axis], fault)
                )
```

(`take_drive_fault` exists from the race-fix commit `bae5096fc`: a trip and a
deviation fault can be the same physical contact; the bridge latches the
late-arriving fault for exactly this check. Add a test with a fake bridge
returning a fault → the error raises; returning None → no error. The
`_servo_drive_limits` restore must also be raise-safe when the body raised:
restore inside `except BaseException` → log restore failures at warning and
re-raise the original; on the success path a restore failure raises normally.)

- [ ] **Step 8.3: Verify green** — `python3 -m pytest test/test_servo_homing.py test/test_servo_torque.py -v`; `ruff check` on the three files.
- [ ] **Step 8.4: Commit** — `klippy: homing-scoped servo drive limits (config, conversion, trip-move wrap)`

---

### Task 9: Full verification + Pi compile of the C/hw surface + PR update

- [ ] **Step 9.1:** `cargo nextest run` (full workspace) from `rust/` — green.
- [ ] **Step 9.2:** `python3 -m pytest test/ -q` — green. `cargo clippy --workspace --all-targets` — clean. `ruff check` changed py files — clean.
- [ ] **Step 9.3:** push the branch; on the Pi (`dderg@ethercatpi5.local`, repo `~/kalico`): `git pull`, then `make -f Makefile.kalico ethercat-endpoint-hw && make -f Makefile.kalico ethercat-stub && make -f Makefile.kalico motion-bridge`, then `sudo make -f Makefile.kalico setcap-ethercat`. This is the compile gate for `bench/libecrt.c` + the hw binary. Fix any C compile errors and re-push.
- [ ] **Step 9.4:** PR #35 gets the new commits automatically (same branch); update the PR body's feature list with the protection work.

---

### Task 10: Hardware validation (user-gated — every motion command needs a per-command yes)

- [ ] Restart klippy; bringup log shows the drive's prior 6065h/6066h/6072h values.
- [ ] Normal `G28 X` — works as before; bridge log shows limits set + restored around the move.
- [ ] Deliberate trip: `_HOME_TEST AXIS=X` homing into open travel (small MAX_TRAVEL so the carriage contacts the frame at low speed) — axis must stop within ~2.5 mm of contact at ≤50% torque, G28 errors with the drive-fault message, and a follow-up `G28 X` recovers without FIRMWARE_RESTART.

---

## Self-review notes

- Spec coverage: config (Task 8), SetDriveLimits/RestoreDriveLimits (Tasks 1/2/5/7), bringup read+log+6066h=0+session writes (Tasks 4/5), drive-fault park-and-report + Faulted gate (Tasks 3/5), bridge routing homing-vs-fatal (Task 7), homing.py wrap with finally (Task 8), stub fault injection + all four test categories (Tasks 1-8), hardware (Task 10).
- Type consistency: `(u32 counts, u16 tenth_pct)` everywhere; `ERR_PIECES_WHILE_FAULTED = -314`; `status_heartbeat_frame(engine_state, fault_code, retired)` arity used consistently in Tasks 2/5.
- Known judgment calls encoded: every-cycle drive-error check (was 500 ms telemetry); no `ec_rt_disable()` on drive fault (drive already de-energized, keeps SDO/recovery alive); pieces rejected at arrival while Faulted so the gate's ring-empty invariant holds.
