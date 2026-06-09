# Servo Axis Homing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `G28` work for an EtherCAT servo axis (and stop breaking for the other axes when a servo is configured), per `docs/superpowers/specs/2026-06-09-servo-axis-homing-design.md`.

**Architecture:** The endpoint (`kalico-ethercat-rt`) gains the existing protocol `Stop`/`StopResponse` command (discard ring, reply with `discard_clock` in its ns domain). The bridge's endstop-trip handler routes the Stop broadcast per transport (serial `host_io` vs EtherCAT `UnixNativeConn`) through a new pure `broadcast_stop` helper. klippy's `homing.py` learns to build endstop entries from `[servo_<axis>]` sections and to torque-enable a steppers-less rail; `servo_axis.py` gains real homing config (`endstop_pin`, `position_endstop`, `homing_speed`, inferred direction).

**Tech Stack:** Rust (`kalico-ethercat-rt`, `motion-bridge`), Python (klippy), `cargo nextest`, pytest.

**Branch:** create `servo-homing` off `sota-motion` before Task 1: `git checkout -b servo-homing`.

**Conventions (from CLAUDE.md):** no explanatory comments — express intent through names; fail loudly; unit tests in separate files; run Rust tests with `cargo nextest run` from `rust/`.

---

### Task 1: Endpoint wire protocol — decode `Stop`, encode `StopResponse`

`MessageKind::Stop = 0x0072` / `StopResponse { result: i32, discard_clock: u64 } = 0x0073` already exist in `kalico-protocol` (serial MCUs use them). The endpoint's decoder currently drops Stop into `Command::Unknown`.

**Files:**
- Modify: `rust/kalico-ethercat-rt/src/wire.rs`
- Test: `rust/kalico-ethercat-rt/src/wire/tests.rs`

- [ ] **Step 1.1: Write the failing tests**

Append to `rust/kalico-ethercat-rt/src/wire/tests.rs`:

```rust
#[test]
fn decodes_stop_command() {
    let payload = frame_payload(MessageKind::Stop, 11, &[]);
    match decode_command(0, &payload).unwrap() {
        Command::Stop { correlation_id: 11 } => {}
        other => panic!("expected Stop, got {other:?}"),
    }
}

#[test]
fn stop_response_frame_round_trips() {
    let frame = stop_response_frame(5, 0, 123_456_789);
    let (chan, payload) = decode_frame(&frame).unwrap();
    assert_eq!(chan, CHANNEL_CONTROL);
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(hdr.correlation_id, 5);
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::StopResponse)
    );
    let r = StopResponse::decode(body).unwrap();
    assert_eq!(r.result, 0);
    assert_eq!(r.discard_clock, 123_456_789);
}
```

Add `StopResponse` to the test imports if not reachable via `super::*` (it will be once Step 1.3 imports it in `wire.rs`).

- [ ] **Step 1.2: Run tests to verify they fail**

Run from `rust/`: `cargo nextest run -p kalico-ethercat-rt -E 'test(stop)'`
Expected: compile error — `Command::Stop` and `stop_response_frame` do not exist.

- [ ] **Step 1.3: Implement**

In `rust/kalico-ethercat-rt/src/wire.rs`:

Extend the `kalico_protocol::messages` import:

```rust
use kalico_protocol::messages::{
    ClaimHandshakeReply, MessageKind, PushPieces, PushPiecesResponse, RuntimeCapsResponse,
    SetTorque, SetTorqueResponse, StatusHeartbeat, StopResponse,
};
```

Add to `enum Command`:

```rust
    Stop {
        correlation_id: u32,
    },
```

Add a decode arm in `decode_command` (before the `_ => Unknown` fallback):

```rust
        Some(MessageKind::Stop) => Ok(Command::Stop {
            correlation_id: cid,
        }),
```

Add beside `set_torque_response_frame`:

```rust
pub fn stop_response_frame(cid: u32, result: i32, discard_clock: u64) -> Vec<u8> {
    let body = StopResponse {
        result,
        discard_clock,
    }
    .encoded_to_vec();
    control_frame(MessageKind::StopResponse, cid, &body)
}
```

- [ ] **Step 1.4: Run tests to verify they pass**

Run from `rust/`: `cargo nextest run -p kalico-ethercat-rt`
Expected: new tests PASS. The two binaries will now FAIL to compile only if a `match cmd` is non-exhaustive — they match on `Command` with explicit arms and a `Command::Unknown` arm, not `_`, so add nothing yet if it compiles; if the build errors with "non-exhaustive patterns: `Command::Stop`", that is Task 2's work — temporarily confirm only the lib tests:
`cargo nextest run -p kalico-ethercat-rt --lib`
(If the bins block even lib tests, proceed straight to Task 2 and run both tasks' tests together at Step 2.4 — commit then covers both.)

- [ ] **Step 1.5: Commit** (skip if Step 1.4 deferred to Task 2; then commit both together)

```bash
git add rust/kalico-ethercat-rt/src/wire.rs rust/kalico-ethercat-rt/src/wire/tests.rs
git commit -m "ethercat-rt: decode Stop, encode StopResponse"
```

---

### Task 2: Endpoint binaries handle `Stop` (hw + stub)

Stop semantics: discard all ring pieces (`ring.reset()` — `RingDescriptor::drain` keeps the retired counter monotonic, so pump in-flight accounting stays consistent), reply `StopResponse { result: 0, discard_clock: monotonic_ns() }`. Torque state is untouched. Stop while parked/idle succeeds trivially.

**Files:**
- Modify: `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs`
- Modify: `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt-stub.rs`
- Test: `rust/kalico-ethercat-rt/tests/torque_lifecycle.rs` (stub lifecycle suite — Stop is lifecycle)

- [ ] **Step 2.1: Write the failing integration tests**

In `rust/kalico-ethercat-rt/tests/torque_lifecycle.rs`, extend the protocol import:

```rust
use kalico_protocol::messages::{
    ClaimHandshakeReply, MessageKind, PushPieces, SetTorque, SetTorqueResponse, StopResponse,
};
```

Add a helper beside `set_torque`:

```rust
fn send_stop(conn: &UnixNativeConn) -> (i32, u64) {
    let (kind, resp) = conn
        .kalico_call(MessageKind::Stop, Vec::new(), Duration::from_secs(5))
        .expect("Stop call must succeed");
    assert_eq!(
        kind,
        MessageKind::StopResponse,
        "expected StopResponse, got 0x{:04x}",
        kind.as_u16()
    );
    let r = StopResponse::decode(&resp).expect("StopResponse must decode");
    (r.result, r.discard_clock)
}
```

Add the tests:

```rust
#[test]
fn stop_while_parked_succeeds_and_keeps_session() {
    let (mut guard, conn, path) = spawn_and_claim("stop-parked", &[]);

    let t0 = now_ns();
    let (result, discard_clock) = send_stop(&conn);
    let t1 = now_ns();
    assert_eq!(result, 0, "Stop while parked must return 0, got {result}");
    assert!(
        discard_clock >= t0 && discard_clock <= t1,
        "discard_clock {discard_clock} outside [{t0}, {t1}]"
    );

    let r = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(r, 0, "enable after Stop must return 0 (session alive), got {r}");

    drop(conn);
    let _ = guard.defuse().wait();
    let _ = std::fs::remove_file(&path);
}

#[test]
fn stop_discards_queued_pieces() {
    let (mut guard, conn, path) = spawn_and_claim("stop-discard", &[]);

    let r = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(r, 0, "enable must return 0, got {r}");

    push_one_piece(&conn, now_ns() + 10_000_000_000);

    let (result, _clock) = send_stop(&conn);
    assert_eq!(result, 0, "Stop mid-stream must return 0, got {result}");

    let r = set_torque(&conn, false, now_ns() + 200_000_000);
    assert_eq!(r, 0, "scheduling disable after Stop must return 0, got {r}");

    thread::sleep(Duration::from_millis(400));

    let r = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(
        r, 0,
        "re-enable must return 0 — nonzero means pieces survived Stop \
         and the scheduled disable faulted, got {r}"
    );

    drop(conn);
    let _ = guard.defuse().wait();
    let _ = std::fs::remove_file(&path);
}
```

(`stop_discards_queued_pieces` proves the discard: a 10-s-future piece left in the ring would make the scheduled disable fault the torque gate and kill the stub, so the final enable would fail.)

- [ ] **Step 2.2: Run tests to verify they fail**

Run from `rust/`: `cargo nextest run -p kalico-ethercat-rt -E 'test(stop_)'`
Expected: FAIL — both binaries don't handle `Command::Stop` (non-exhaustive match compile error, or the stub logs "ignoring kind" and `send_stop` times out — depends on whether the match has an `Unknown` catch; either failure mode is the correct red).

- [ ] **Step 2.3: Implement in both binaries**

In `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs`, add `stop_response_frame` to the `wire` import list, and add an arm in the command match (beside `Command::SetTorque`):

```rust
                Command::Stop { correlation_id } => {
                    let now_ns = monotonic_ns();
                    ring.reset();
                    cmap = None;
                    eprintln!("ec-rt: Stop — ring discarded, discard_clock={now_ns}");
                    server.respond(&stop_response_frame(correlation_id, 0, now_ns));
                }
```

In `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt-stub.rs`, same import addition, and the same arm minus the `cmap` line (the stub has no CountMap):

```rust
                Command::Stop { correlation_id } => {
                    let now_ns = monotonic_ns();
                    ring.reset();
                    eprintln!("ec-rt-stub: Stop — ring discarded, discard_clock={now_ns}");
                    server.respond(&stop_response_frame(correlation_id, 0, now_ns));
                }
```

- [ ] **Step 2.4: Run tests to verify they pass**

Run from `rust/`: `cargo nextest run -p kalico-ethercat-rt`
Expected: all PASS, including both new `stop_*` tests and the pre-existing lifecycle tests.

- [ ] **Step 2.5: Commit**

```bash
git add rust/kalico-ethercat-rt/src/bin/ rust/kalico-ethercat-rt/tests/torque_lifecycle.rs
git commit -m "ethercat-rt: Stop discards the ring and reports discard_clock (hw + stub)"
```

---

### Task 3: Bridge — per-transport Stop broadcast in the trip handler

Extract the broadcast loop from `handle_endstop_trip` into a pure, closure-parameterized `broadcast_stop` in `homing.rs` (testable without transports), then rewire `handle_endstop_trip` to build a per-MCU transport map: serial `host_io` **or** EtherCAT `endpoint_conn`. This removes the "Stop: no host_io for mcu N" failure for EtherCAT nodes; an MCU with neither transport stays a loud error.

**Files:**
- Modify: `rust/motion-bridge/src/homing.rs`
- Modify: `rust/motion-bridge/src/bridge.rs:3021-3109` (`handle_endstop_trip`)
- Test: `rust/motion-bridge/src/homing/tests.rs`

- [ ] **Step 3.1: Write the failing unit tests**

Append to `rust/motion-bridge/src/homing/tests.rs`:

```rust
mod broadcast_stop_tests {
    use crate::homing::broadcast_stop;
    use kalico_protocol::messages::StopResponse;
    use std::collections::HashSet;

    #[test]
    fn collects_discard_clock_from_the_axis_mcu() {
        let ids: HashSet<u32> = [1, 2].into_iter().collect();
        let clock = broadcast_stop(&ids, 2, |mcu_id| {
            Ok(StopResponse {
                result: 0,
                discard_clock: u64::from(mcu_id) * 100,
            })
        })
        .unwrap();
        assert_eq!(clock, 200);
    }

    #[test]
    fn missing_transport_fails_loudly() {
        let ids: HashSet<u32> = [1, 7].into_iter().collect();
        let err = broadcast_stop(&ids, 1, |mcu_id| {
            if mcu_id == 7 {
                Err("Stop: no transport for mcu 7".to_owned())
            } else {
                Ok(StopResponse {
                    result: 0,
                    discard_clock: 42,
                })
            }
        })
        .unwrap_err();
        assert!(err.contains("no transport for mcu 7"), "got: {err}");
        assert!(err.contains("Stop broadcast failed"), "got: {err}");
    }

    #[test]
    fn rejected_result_is_an_error() {
        let ids: HashSet<u32> = [1].into_iter().collect();
        let err = broadcast_stop(&ids, 1, |_| {
            Ok(StopResponse {
                result: -5,
                discard_clock: 0,
            })
        })
        .unwrap_err();
        assert!(
            err.contains("Stop rejected by mcu 1: result=-5"),
            "got: {err}"
        );
    }

    #[test]
    fn axis_mcu_without_a_discard_clock_is_an_error() {
        let ids: HashSet<u32> = [2].into_iter().collect();
        let err = broadcast_stop(&ids, 9, |_| {
            Ok(StopResponse {
                result: 0,
                discard_clock: 5,
            })
        })
        .unwrap_err();
        assert!(err.contains("did not report a discard clock"), "got: {err}");
    }
}
```

- [ ] **Step 3.2: Run tests to verify they fail**

Run from `rust/`: `cargo nextest run -p motion-bridge -E 'test(broadcast_stop)'`
Expected: compile error — `broadcast_stop` does not exist.

- [ ] **Step 3.3: Implement `broadcast_stop` in `homing.rs`**

Add to `rust/motion-bridge/src/homing.rs`:

```rust
pub fn broadcast_stop<F>(
    mcu_ids: &std::collections::HashSet<u32>,
    axis_mcu: u32,
    call: F,
) -> Result<u64, String>
where
    F: Fn(u32) -> Result<kalico_protocol::messages::StopResponse, String>,
{
    let mut errors: Vec<String> = Vec::new();
    let mut axis_discard_clock: Option<u64> = None;
    for &mcu_id in mcu_ids {
        match call(mcu_id) {
            Ok(resp) if resp.result != 0 => {
                errors.push(format!(
                    "Stop rejected by mcu {mcu_id}: result={}",
                    resp.result
                ));
            }
            Ok(resp) => {
                if mcu_id == axis_mcu {
                    axis_discard_clock = Some(resp.discard_clock);
                }
            }
            Err(e) => errors.push(e),
        }
    }
    if !errors.is_empty() {
        return Err(format!(
            "EndstopTrip Stop broadcast failed: {}",
            errors.join("; ")
        ));
    }
    axis_discard_clock.ok_or_else(|| {
        format!("EndstopTrip: axis MCU {axis_mcu} did not report a discard clock")
    })
}
```

- [ ] **Step 3.4: Run tests to verify they pass**

Run from `rust/`: `cargo nextest run -p motion-bridge -E 'test(broadcast_stop)'`
Expected: 4 PASS.

- [ ] **Step 3.5: Rewire `handle_endstop_trip`**

In `rust/motion-bridge/src/bridge.rs`, inside `handle_endstop_trip`:

Replace the `mcu_ios` construction (currently lines 3021-3026):

```rust
        enum StopTransport {
            Serial(Arc<KalicoHostIo>),
            EtherCat(Arc<UnixNativeConn>),
        }

        let transports: HashMap<u32, StopTransport> = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            mcus.iter()
                .filter_map(|(&id, conn)| {
                    if let Some(io) = conn.host_io.as_ref() {
                        Some((id, StopTransport::Serial(Arc::clone(io))))
                    } else {
                        conn.endpoint_conn
                            .as_ref()
                            .map(|ec| (id, StopTransport::EtherCat(Arc::clone(ec))))
                    }
                })
                .collect()
        };
```

Inside the `homing-trip-handler` thread closure, replace the whole stop loop — from `let mut stop_errors: Vec<String> = Vec::new();` through the `let discard_clock = match axis_discard_clock { ... };` block (currently lines 3049-3109) — with:

```rust
                let stop_call = |mcu_id: u32| -> Result<
                    kalico_protocol::messages::StopResponse,
                    String,
                > {
                    use kalico_host_rt::native_call::NativeCall as _;
                    use kalico_protocol::codec::Decode as _;
                    let transport = transports
                        .get(&mcu_id)
                        .ok_or_else(|| format!("Stop: no transport for mcu {mcu_id}"))?;
                    let (_kind, body) = match transport {
                        StopTransport::Serial(io) => io
                            .kalico_call(
                                kalico_protocol::MessageKind::Stop,
                                Vec::new(),
                                stop_timeout,
                            )
                            .map_err(|e| {
                                format!("Stop call failed for mcu {mcu_id}: {e:?}")
                            })?,
                        StopTransport::EtherCat(conn) => conn
                            .kalico_call(
                                kalico_protocol::MessageKind::Stop,
                                Vec::new(),
                                stop_timeout,
                            )
                            .map_err(|e| {
                                format!("Stop call failed for mcu {mcu_id}: {e:?}")
                            })?,
                    };
                    kalico_protocol::messages::StopResponse::decode(&body)
                        .map_err(|e| format!("Stop decode failed for mcu {mcu_id}: {e:?}"))
                };

                let discard_clock = match crate::homing::broadcast_stop(
                    &stepper_mcu_ids,
                    run.axis_key.mcu_id,
                    stop_call,
                ) {
                    Ok(c) => c,
                    Err(e) => {
                        let _ = run.notify.send(Err(e));
                        return;
                    }
                };
```

(`transports` moves into the thread closure; the `mcu_ios` variable disappears. The existing `MessageKind::Stop` import path used by the old code — `kalico_protocol::MessageKind` — is already available.)

- [ ] **Step 3.6: Run the full motion-bridge + ethercat suites**

Run from `rust/`: `cargo nextest run -p motion-bridge -p kalico-ethercat-rt`
Expected: all PASS (the trip-handler change is compile-checked; behavior covered by `broadcast_stop` tests + endpoint integration tests).

- [ ] **Step 3.7: Commit**

```bash
git add rust/motion-bridge/src/homing.rs rust/motion-bridge/src/homing/tests.rs rust/motion-bridge/src/bridge.rs
git commit -m "motion-bridge: route homing Stop broadcast per transport (serial / ethercat)"
```

---

### Task 4: `servo_axis.py` — real homing config

`[servo_<axis>]` gains `endstop_pin` (read here for presence; consumed for its value by homing.py in Task 5), `position_endstop`, `homing_speed`. Direction inferred: endstop at `position_min` → negative, at `position_max` → positive, anything else a config error.

**Files:**
- Modify: `klippy/extras/servo_axis.py`
- Test: `test/test_servo_homing.py` (new)

- [ ] **Step 4.1: Write the failing tests**

Create `test/test_servo_homing.py`:

```python
import pytest

from klippy.extras import servo_axis


class FakeErrConfig:
    error = RuntimeError


def test_infer_positive_dir_at_min_is_negative():
    cfg = FakeErrConfig()
    assert servo_axis.infer_positive_dir(cfg, "x", -6.0, -6.0, 235.0) is False


def test_infer_positive_dir_at_max_is_positive():
    cfg = FakeErrConfig()
    assert servo_axis.infer_positive_dir(cfg, "x", 235.0, -6.0, 235.0) is True


def test_infer_positive_dir_mid_range_is_config_error():
    cfg = FakeErrConfig()
    with pytest.raises(RuntimeError, match="position_endstop"):
        servo_axis.infer_positive_dir(cfg, "x", 100.0, -6.0, 235.0)


def test_get_homing_info_reflects_homing_config():
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.position_endstop = -6.0
    rail.homing_speed = 50.0
    rail.homing_positive_dir = False
    hi = rail.get_homing_info()
    assert hi.speed == 50.0
    assert hi.position_endstop == -6.0
    assert hi.positive_dir is False
```

- [ ] **Step 4.2: Run tests to verify they fail**

Run from repo root: `python3 -m pytest test/test_servo_homing.py -v`
Expected: FAIL — `servo_axis` has no `infer_positive_dir`; `get_homing_info` returns the zeros stub.

- [ ] **Step 4.3: Implement**

In `klippy/extras/servo_axis.py`:

Add a module-level function (above `class ServoRail`):

```python
def infer_positive_dir(config, axis, position_endstop, position_min, position_max):
    if position_endstop == position_min:
        return False
    if position_endstop == position_max:
        return True
    raise config.error(
        "servo_%s: position_endstop %.3f must equal position_min (%.3f) "
        "or position_max (%.3f)"
        % (axis, position_endstop, position_min, position_max)
    )
```

In `ServoRail.__init__`, replace the line `self.position_endstop = 0.0` with:

```python
        self.endstop_pin = config.get("endstop_pin", None)
        if self.endstop_pin is None:
            self.position_endstop = 0.0
            self.homing_speed = 0.0
            self.homing_positive_dir = False
        else:
            self.position_endstop = config.getfloat("position_endstop")
            self.homing_speed = config.getfloat("homing_speed", 5.0, above=0.0)
            self.homing_positive_dir = infer_positive_dir(
                config,
                self.axis,
                self.position_endstop,
                self.position_min,
                self.position_max,
            )
```

Replace `get_homing_info`'s stub fields:

```python
    def get_homing_info(self):
        return _homing_info(
            speed=self.homing_speed,
            position_endstop=self.position_endstop,
            retract_speed=0.0,
            retract_dist=0.0,
            positive_dir=self.homing_positive_dir,
            second_homing_speed=0.0,
            use_sensorless_homing=False,
            min_home_dist=0.0,
            accel=None,
        )
```

- [ ] **Step 4.4: Run tests to verify they pass**

Run: `python3 -m pytest test/test_servo_homing.py -v`
Expected: 4 PASS.
Also run the existing servo suite: `python3 -m pytest test/test_servo_torque.py -v` — all PASS.

- [ ] **Step 4.5: Commit**

```bash
git add klippy/extras/servo_axis.py test/test_servo_homing.py
git commit -m "servo_axis: homing config (endstop_pin, position_endstop, homing_speed)"
```

---

### Task 5: `homing.py` — servo endstop entries + steppers-less torque enable

Two seams: (a) the endstop-entry loop reads `[servo_<axis>]` alongside `[stepper_<axis>]`; (b) pre-home enable falls back to the rail's registered motor name when the rail has no steppers (`servo_x` is registered in `stepper_enable` by `register_torque_enable`; `EnableTracking.motor_enable` is guarded, so enabling an already-enabled servo is a no-op).

**Files:**
- Modify: `klippy/extras/homing.py`
- Test: `test/test_servo_homing.py`

- [ ] **Step 5.1: Write the failing tests**

Append to `test/test_servo_homing.py`:

```python
from klippy.extras import homing as homing_mod


class FakeSectionsConfig:
    def __init__(self, sections):
        self._sections = sections

    def has_section(self, name):
        return name in self._sections


def test_endstop_section_finds_stepper():
    cfg = FakeSectionsConfig({"stepper_x"})
    assert homing_mod._endstop_section(cfg, "x") == "stepper_x"


def test_endstop_section_finds_servo():
    cfg = FakeSectionsConfig({"servo_x"})
    assert homing_mod._endstop_section(cfg, "x") == "servo_x"


def test_endstop_section_none_when_axis_absent():
    cfg = FakeSectionsConfig({"stepper_y"})
    assert homing_mod._endstop_section(cfg, "x") is None


class FakeStepperEnable:
    def __init__(self):
        self.calls = []

    def motor_debug_enable(self, name, enable):
        self.calls.append((name, enable))


class FakeStepper:
    def __init__(self, name):
        self._name = name

    def get_name(self):
        return self._name


class FakeRail:
    def __init__(self, steppers, name):
        self._steppers = steppers
        self._name = name

    def get_steppers(self):
        return self._steppers

    def get_name(self, short=False):
        return self._name


def test_enable_homing_motors_enables_each_stepper():
    se = FakeStepperEnable()
    rail = FakeRail([FakeStepper("stepper_x"), FakeStepper("stepper_x1")], "stepper_x")
    homing_mod._enable_homing_motors(se, rail)
    assert se.calls == [("stepper_x", True), ("stepper_x1", True)]


def test_enable_homing_motors_enables_servo_rail_by_name():
    se = FakeStepperEnable()
    rail = FakeRail([], "servo_x")
    homing_mod._enable_homing_motors(se, rail)
    assert se.calls == [("servo_x", True)]
```

- [ ] **Step 5.2: Run tests to verify they fail**

Run: `python3 -m pytest test/test_servo_homing.py -v`
Expected: the 5 new tests FAIL — `_endstop_section` / `_enable_homing_motors` do not exist.

- [ ] **Step 5.3: Implement**

In `klippy/extras/homing.py`:

Add module-level helpers (below the constants):

```python
def _endstop_section(config, axis_name):
    for prefix in ("stepper_", "servo_"):
        section = prefix + axis_name
        if config.has_section(section):
            return section
    return None


def _enable_homing_motors(stepper_enable, rail):
    steppers = rail.get_steppers()
    if not steppers:
        stepper_enable.motor_debug_enable(rail.get_name(), True)
        return
    for s in steppers:
        stepper_enable.motor_debug_enable(s.get_name(), True)
```

In `Homing.__init__`, replace:

```python
            section = "stepper_" + axis_name
            if not config.has_section(section):
                continue
```

with:

```python
            section = _endstop_section(config, axis_name)
            if section is None:
                continue
```

In `_home_axis`, replace:

```python
        stepper_enable = self.printer.lookup_object("stepper_enable")
        for s in rail.get_steppers():
            stepper_enable.motor_debug_enable(s.get_name(), True)
```

with:

```python
        stepper_enable = self.printer.lookup_object("stepper_enable")
        _enable_homing_motors(stepper_enable, rail)
```

- [ ] **Step 5.4: Run tests to verify they pass**

Run: `python3 -m pytest test/test_servo_homing.py test/test_servo_torque.py -v`
Expected: all PASS.

- [ ] **Step 5.5: Commit**

```bash
git add klippy/extras/homing.py test/test_servo_homing.py
git commit -m "homing: servo axes get endstop entries and pre-home torque enable"
```

---

### Task 6: Full-suite verification

- [ ] **Step 6.1: Rust suite**

Run from `rust/`: `cargo nextest run`
Expected: all PASS (full workspace).

- [ ] **Step 6.2: Python suite**

Run from repo root: `python3 -m pytest test/ -q`
Expected: all PASS.

- [ ] **Step 6.3: Lint**

Run from `rust/`: `cargo clippy --workspace --all-targets` — no new warnings.
Run from repo root: `ruff check klippy/extras/homing.py klippy/extras/servo_axis.py test/test_servo_homing.py` — clean.

- [ ] **Step 6.4: Push and open PR**

```bash
git push -u origin servo-homing
gh pr create --base sota-motion --title "Servo axis homing: endpoint Stop + servo endstop entries" --body "Implements docs/superpowers/specs/2026-06-09-servo-axis-homing-design.md"
```

---

### Task 7: Hardware validation (Neptune bench — user-gated)

Everything here follows the bench rule: commit → push → pull on the Pi → build there. **No motion command (G28/G1/M84/SET_KINEMATIC_POSITION) is ever issued without the user's per-command yes.** The user edits `printer.cfg` themselves (servo on **X**: `endstop_pin: PA13`, `position_endstop: -6.0`, `homing_speed: 50`, `[stepper_x]` removed).

- [ ] **Step 7.1: Flash the F401** (after merge, or from the branch with user's go-ahead)

```bash
.claude/skills/neptune-bench/scripts/flash-neptune.sh <branch>
```

- [ ] **Step 7.2: Rebuild host artifacts on the Pi**

```bash
ssh dderg@ethercatpi5.local 'cd ~/kalico && make -f Makefile.kalico motion-bridge && make -f Makefile.kalico ethercat-endpoint-hw && make -f Makefile.kalico ethercat-stub'
ssh dderg@ethercatpi5.local 'cd ~/kalico && echo password | sudo -S make -f Makefile.kalico setcap-ethercat'
```

- [ ] **Step 7.3: Stub-first validation (drive dark, zero hardware risk)**

User points `endpoint:` at the stub, restarts klippy → must reach `ready`. With user approval: `G28 Y` (stepper axis with servo present — the regression case), then `_HOME_TEST`/`G28 X` against the stub only if the user wants the software path exercised further (the stub never trips the endstop — expect the 30 s homing timeout, which itself proves the broadcast no longer errors).

- [ ] **Step 7.4: Real drive (supervised, user powers it on when asked)**

Switch `endpoint:` to the hw binary, tell the user to power the servo drive, `FIRMWARE_RESTART` → claim reaches `ready`. With per-command user approval: small `G1 X` jog to confirm direction/counts, then `G28 X` into `PA13` — torque enables, drip runs, trip stops the servo, position set from trip reconstruction. Then `G28 Y`/`G28 Z` to confirm the mixed-transport broadcast end to end.

---

## Self-review notes

- **Spec coverage:** endpoint Stop (Tasks 1-2), per-transport broadcast (Task 3), homing.py entries + enable (Task 5), servo_axis homing config (Task 4), all four test categories from the spec (wire/integration Task 1-2, bridge unit Task 3, klippy Task 4-5, hardware Task 7). Stop-while-parked → other-axes-G28 regression covered by `stop_while_parked_succeeds_and_keeps_session` + Task 7.3.
- **Type consistency:** `broadcast_stop(&HashSet<u32>, u32, F) -> Result<u64, String>` used identically in Task 3 steps; `stop_response_frame(cid, result, discard_clock)` matches between Tasks 1 and 2; `_endstop_section` / `_enable_homing_motors` names match between Steps 5.1 and 5.3.
- **No placeholders:** every code step carries the full code.
