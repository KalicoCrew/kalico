# Plan B: Firmware + Rust — Software Trip, Deadline, Async Homing, Curve Retention

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **Parallel plan.** This plan runs independently of Plan A (Python trsync). Both must complete before Plan C (integration) can begin.

**Goal:** Add the MCU firmware commands, Rust runtime logic, and Rust bridge FFI surface needed for credit-windowed software-trip homing and host-side curve evaluation.

**Architecture:** Bottom-up: (1) Add `SourceKind::Software` and deadline logic to the Rust runtime endstop module, (2) add `runtime_software_trip` and `runtime_extend_homing_deadline` C command handlers, (3) add `submit_homing_move_async`, `software_trip`, `extend_homing_deadline` PyO3 methods on the bridge, (4) add curve retention and evaluation in the bridge.

**Tech Stack:** Rust (runtime, motion-bridge), C (MCU firmware)

**Spec:** `docs/kalico-rewrite/external-probe-homing.md`, Pieces B+C

---

## File Map

| File | Change |
|------|--------|
| `rust/runtime/src/endstop.rs` | `SourceKind::Software`, deadline state, `software_trip()`, `extend_deadline()` |
| `rust/runtime/src/lib.rs` | `extern "C"` FFI exports for new functions |
| `src/runtime_commands.c` | `command_runtime_software_trip`, `command_runtime_extend_homing_deadline` |
| `rust/motion-bridge/src/homing.rs` | `DeadlineExpired` terminal state, software trip handling |
| `rust/motion-bridge/src/bridge.rs` | `submit_homing_move_async`, `software_trip`, `extend_homing_deadline`, `get_homing_segment_reason` PyO3 methods; retained curve slot + `get_homing_position_at_time` |

---

### Task 1: Rust runtime — SourceKind::Software + deadline logic

**Files:**
- Modify: `rust/runtime/src/endstop.rs`

- [ ] **Step 1:** Add `Software = 2` to the `SourceKind` enum (endstop.rs:17). Update `TryFrom<u8>` implementation to handle value 2.

- [ ] **Step 2:** Add deadline state to the endstop module. This can be fields on the existing armed state or a separate struct:
```rust
struct DeadlineState {
    active: AtomicBool,
    clock: AtomicU64,
}
```
`FIXED_GRANT_TICKS` is a constant corresponding to 50ms at the MCU's clock frequency. Since clock frequency is runtime-known, this can be computed during arm: `grant_ticks = (freq as u64) / 20` (50ms = 1/20th of a second).

- [ ] **Step 3:** When arming with `SourceKind::Software`:
  - Do NOT set up GPIO polling (no physical pin to watch)
  - Set `deadline_active = false` initially (deadline starts at evaluation, not arm)
  - Store `grant_ticks` for later use

- [ ] **Step 4:** At curve evaluation start (when the modulator begins ticking through a Software-armed segment), set:
  - `deadline_clock = current_clock + grant_ticks`
  - `deadline_active = true`

- [ ] **Step 5:** In the modulator tick path (where GPIO is polled for Physical/TmcDiag sources), add a deadline check for Software arms:
```rust
if deadline_active.load(Ordering::Relaxed)
    && current_clock >= deadline_clock.load(Ordering::Relaxed)
{
    // Terminal freeze — same path as GPIO trip but with
    // a distinct reason (REASON_DEADLINE_EXPIRED = 5)
    trigger_freeze(REASON_DEADLINE_EXPIRED);
}
```

- [ ] **Step 6:** Add `pub fn software_trip(arm_id: u32) -> TripResult`:
  - Same freeze logic as GPIO trip: snapshot stepper positions, publish trip event
  - If no Software arm is active or arm_id doesn't match, return error status

- [ ] **Step 7:** Add `pub fn extend_deadline(arm_id: u32)`:
  - If `deadline_active` and arm matches: `deadline_clock = current_clock + grant_ticks`
  - If not active (pre-start): silently ignore (no error, no latch)
  - If arm_id doesn't match: silently ignore

- [ ] **Step 8:** Add unit tests:
  - Arm with Software source, verify no GPIO polling configured
  - Verify deadline fires after grant_ticks with no extension
  - Verify extend_deadline pushes deadline forward
  - Verify software_trip produces a trip event with stepper snapshots
  - Verify pre-start extend_deadline is ignored

- [ ] **Step 9:** Commit: `feat(runtime): SourceKind::Software + deadline + software_trip`

---

### Task 2: MCU firmware — command handlers

**Files:**
- Modify: `src/runtime_commands.c`
- Modify: `rust/runtime/src/lib.rs` (FFI exports)

- [ ] **Step 1:** Add `extern "C"` FFI exports in `rust/runtime/src/lib.rs` (or wherever the existing `kalico_endstop_arm` / `kalico_endstop_disarm` exports live):
```rust
#[no_mangle]
pub extern "C" fn kalico_software_trip(arm_id: u32, status: *mut u8) -> i32 {
    let result = endstop::software_trip(arm_id);
    unsafe { *status = result.status_byte(); }
    0
}

#[no_mangle]
pub extern "C" fn kalico_extend_deadline(arm_id: u32) -> i32 {
    endstop::extend_deadline(arm_id);
    0
}
```
Match the pattern of existing `kalico_endstop_arm` / `kalico_endstop_disarm` exports.

- [ ] **Step 2:** Add C command handler in `src/runtime_commands.c`:
```c
void
command_runtime_software_trip(uint32_t *args)
{
    uint32_t arm_id = args[0];
    uint8_t status = 0;
    (void)kalico_software_trip(arm_id, &status);
    sendf("kalico_software_trip_response arm_id=%u status=%c",
          arm_id, status);
}
DECL_COMMAND(command_runtime_software_trip,
    "runtime_software_trip arm_id=%u");
```

- [ ] **Step 3:** Add C command handler for deadline extension:
```c
void
command_runtime_extend_homing_deadline(uint32_t *args)
{
    uint32_t arm_id = args[0];
    (void)kalico_extend_deadline(arm_id);
}
DECL_COMMAND(command_runtime_extend_homing_deadline,
    "runtime_extend_homing_deadline arm_id=%u");
```

- [ ] **Step 4:** Build firmware for both H7 (`.config.h7.bak`) and F446 (`.config.f446.test`). Verify:
  - Compilation succeeds
  - New commands appear in the data dictionary (check `out/klipper.dict` for `runtime_software_trip` and `runtime_extend_homing_deadline`)
  - `kalico_software_trip_response` appears as a registered response

- [ ] **Step 5:** Commit: `feat(firmware): runtime_software_trip + runtime_extend_homing_deadline commands`

---

### Task 3: Rust bridge — homing state machine updates

**Files:**
- Modify: `rust/motion-bridge/src/homing.rs`

- [ ] **Step 1:** Add `DeadlineExpired` to the homing segment terminal states. Currently homing.rs has states like `Idle`, `Active`, `Completed`, `Tripped`. Add `DeadlineExpired` as a terminal alongside `Tripped` and `Completed`.

- [ ] **Step 2:** Register a response handler for `kalico_software_trip_response` in the bridge's serial handling. When received, process like a trip event: extract arm_id and status, update homing state accordingly.

- [ ] **Step 3:** Handle `REASON_DEADLINE_EXPIRED` (value 5) from the MCU's trip event. When the MCU reports a deadline-expired freeze, the homing state transitions to `DeadlineExpired` instead of `Tripped` or `Completed`. The segment is retired either way.

- [ ] **Step 4:** Commit: `feat(bridge): DeadlineExpired homing terminal state`

---

### Task 4: Rust bridge — async homing + software_trip + extend_deadline FFI

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs`

- [ ] **Step 1:** Add `submit_homing_move_async` PyO3 method. Internally: same segment submission as `submit_homing_move` but does NOT call the blocking wait. Returns immediately. The Python caller will use `get_homing_segment_status` to poll for retirement.

```rust
#[pyo3(signature = (newpos, speed, arm_ids))]
fn submit_homing_move_async(
    &self, newpos: Vec<f64>, speed: f64, arm_ids: Vec<u32>
) -> PyResult<()> {
    self.submit_homing_move_inner(&newpos, speed, &arm_ids)
    // No wait_moves — returns immediately
}
```

- [ ] **Step 2:** Add `is_homing_segment_retired` PyO3 method — returns `bool`. Python polls this alongside `drip_completion.test()` in the credit loop. Checks whether the homing segment has reached a terminal state (Completed, Tripped, or DeadlineExpired).

- [ ] **Step 3:** Add `get_homing_segment_reason` PyO3 method — returns an integer reason code. Callable after `is_homing_segment_retired` returns True. Returns bridge-private codes:
  - 1 = past end time (no trigger)
  - 2 = tripped (software_trip or GPIO)
  - 3 = deadline expired

- [ ] **Step 4:** Add `software_trip` PyO3 method. Sends `runtime_software_trip arm_id=%u` through the bridge's MCU serial path. Waits for `kalico_software_trip_response`.

- [ ] **Step 5:** Add `extend_homing_deadline` PyO3 method. Sends `runtime_extend_homing_deadline arm_id=%u` through the bridge's MCU serial path. Fire-and-forget (no response).

- [ ] **Step 6:** Register the new response message `kalico_software_trip_response` in the bridge's msgproto handling, matching the pattern of `kalico_arm_endstop_response` and `kalico_disarm_endstop_response`.

- [ ] **Step 7:** Commit: `feat(bridge): async homing submission + software_trip + extend_deadline FFI`

---

### Task 5: Rust bridge — curve retention and evaluation

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs`

- [ ] **Step 1:** Add a retained-curve slot to the bridge state:
```rust
struct RetainedHomingCurve {
    /// Per-axis Bézier control points (parameterized in planner-local seconds)
    curves: [Option<CurveParams>; 3],  // X, Y, Z
    /// Planner-local start time of the homing segment
    t_start: f64,
    /// Planner-local end time
    t_end: f64,
    /// MCU clock sync state at dispatch time (for print_time → planner_time)
    clock_freq: f64,
    clock_offset: f64,
    /// Kinematics type (0 = corexy, 1 = cartesian)
    kinematics: u8,
}

// In PyMotionBridge state:
retained_homing_curve: Mutex<Option<RetainedHomingCurve>>,
```

- [ ] **Step 2:** In `submit_homing_move_async` (or `submit_homing_move_inner`), after the segment is created and dispatched, populate the retained curve slot. Capture:
  - The segment's curve parameters (control points per axis)
  - Segment planner-local start/end times
  - Current clock sync state (freq, offset from the most recent `set_clock_est`)
  - Kinematics type

Replace any existing retained curve (single slot).

- [ ] **Step 3:** Clear the retained curve on `kalico_stream_open` (planner reset / `set_position`).

- [ ] **Step 4:** Add `get_homing_position_at_time` PyO3 method:
```rust
#[pyo3(signature = (print_time,))]
fn get_homing_position_at_time(&self, print_time: f64) -> PyResult<Vec<f64>> {
    // 1. Lock retained curve
    // 2. Convert print_time → MCU clock → planner-local time
    //    using retained clock_freq/clock_offset and the planner epoch
    // 3. Clamp to [t_start, t_end]
    // 4. Evaluate each axis curve at that parameter → [x, y, z]
    // 5. Return position vector
}
```

The conversion chain:
- `mcu_clock = print_time * clock_freq + clock_offset_ticks` (matching how dispatch.rs computes `t_start_clock`)
- `planner_time = t_start + (mcu_clock - dispatch_mcu_clock) / clock_freq`

The exact formula depends on how the bridge currently converts between planner time and MCU clock for dispatch. Read `dispatch.rs:266-267` (`t_start_clock`, `t_end_clock`) to match.

- [ ] **Step 5:** Add Rust unit test:
  - Create a synthetic linear homing curve: Z moves from 10mm to 0mm over 2 seconds
  - Retain it with known clock sync parameters
  - Evaluate at t=0 → expect Z=10mm
  - Evaluate at t=1 → expect Z=5mm
  - Evaluate at t=2 → expect Z=0mm
  - Verify position vector is in toolhead coordinates [x, y, z]

- [ ] **Step 6:** Commit: `feat(bridge): retained homing curve + position evaluation at trigger time`

---

## Verification Checklist

After all tasks:

- [ ] Firmware compiles for H7 and F446 with new commands in data dictionary
- [ ] `runtime_software_trip` and `runtime_extend_homing_deadline` registered
- [ ] `kalico_software_trip_response` registered as a response
- [ ] Rust runtime unit tests pass (deadline, software_trip, extend)
- [ ] Rust bridge unit tests pass (curve retention, evaluation)
- [ ] Bridge cdylib compiles (`make` on the Pi)
- [ ] Existing sensorless homing (G28 X, G28 Y) still works after firmware flash

Z homing with Beacon will NOT work yet — that requires Plan C (Python credit loop integration) which depends on both Plan A and Plan B.

## Build/Flash Sequence

After firmware tasks (Task 2+):
1. Commit and push to the branch
2. On trident.local: `cd ~/kalico && git pull`
3. Build host-side: `cd ~/kalico && make clean && make -j$(nproc)`
4. Build + flash H7: switch to `.config.h7.bak`, `make clean && make -j$(nproc) && make flash`
5. Build + flash F446: switch to `.config.f446.test`, `make clean && make -j$(nproc) && make flash`
6. Restart klippy: `sudo systemctl restart klipper`
7. Verify sensorless homing still works: G28 X, G28 Y
