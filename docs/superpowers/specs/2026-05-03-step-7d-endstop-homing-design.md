# Step 7-D — Endstop watch & homing (SOTA design, rev 2)

**Scope:** End-to-end sensorless and physical-switch homing for the kalico-rewrite single-MCU runtime, replacing the fall-back `motion_toolhead.drip_move` stub. Sub-modulation-period trip-to-stop, atomic per-tick snapshot, single-segment homing, one trigger-source primitive shared by physical switches and TMC StallGuard DIAG.

**Precondition:** 7-D Phase 2b-2 reached — TMC SPI bring-up passed (DUMP_TMC clean), drivers locked under `SET_STEPPER_ENABLE`, kalico bridge live on H723 with `kalico_status_v6` ping flow.

**Replaces:** `motion_toolhead.drip_move` fallback (klippy/motion_toolhead.py:220), `TriggerDispatch.start() not yet supported under the new motion path` raise (klippy/mcu.py:381–385).

**Driving principle:** State of the art, simple, elegant, robust. Cut nothing; build on the architectural advantages our single-MCU + analytic-curve runtime gives over Klipper / Prunt / LinuxCNC / Marlin.

**Revision history:**
- rev 1 → rev 2 (2026-05-03): addresses Codex round-1 review (wire format, MCU_endstop integration, CAS race, memory ordering, phase-stepping stop policy, IgnoreUntilMoving latch, wait_moves contract, multi-MCU, probing matrix, homed gate, debounce naming, polarity).
- rev 2 → rev 3 (2026-05-03): addresses Codex round-2 review — `_DECL_OUTPUT` for async events; 64-bit clocks split lo/hi; stepper binding carried over the wire; `kalico_set_homed` extended (existing no-arg command takes an optional `homed=%c`); `home_wait` bounded timeout with `REASON_PAST_END_TIME`; `submit_homing_move()` replaces the `set_next_segment_homing()` mutable-state footgun; bridge `_build_config` legacy-command suppression; opt-out for sensorless `TripImmediately`; trip event blob version byte; `AlreadyTripped` recategorized as ack status, not an `ArmState`; seqlock data stores upgraded to `Release`.

---

## 1. Property targets

| # | Property | Target | Rationale |
|---|---|---|---|
| P1 | Trip detection latency | ≤ 1 modulation period (25–50 µs at 20–40 kHz) | Matches Prunt's HRTIM ISR sampling; sensorless DIAG bandwidth doesn't reward sub-period sampling |
| P2 | Trip-to-stop latency | Same modulation tick as detection | Coercive abort, not cooperative — strictly stronger than every surveyed firmware |
| P3 | Position snapshot atomicity | Single ISR invocation, publish-with-Release | Per-stepper step counts + motion clock + tripped-source-id captured before the next sample, with explicit memory-ordering protocol (§4) |
| P4 | Trigger position fidelity (MVP) | Integer step counts after tick-N pulse commits, plus analytic `x_unshaped(t_trip)` / `x_shaped(t_trip)` from the curve evaluator | Open-loop, hybrid stepping. Phase-stepping fractional position is a Step-10 extension (§4.5) |
| P5 | Sources unified | One trip primitive for physical switches & TMC DIAG | Sensorless property gated by velocity-latch policy on the source, not a special path |
| P6 | Single-segment homing | One curve segment for whole homing move | Trip-aware `wait_moves` (§6); no drip mode, no DripModeEndSignal |
| P7 | Multi-source OR | First source to trip wins, all others disarmed | Dual-stepper X (`stepper_x` + `stepper_x1`) collapses to two DIAGs OR'd on one logical endstop |
| P8 | Phase-stepping forward-compat | Same trip primitive; Step-10 stop policy required (§4.5) | Trip handler shares modulation ISR with current synthesis; current-mode stop policy specified explicitly, not handwaved |
| P9 | Multi-MCU forward-compat | Per-MCU `Arm`s coordinated via host-side election (§7) | EtherCAT / multi-MCU support remains achievable post-MVP |
| P10 | Determinism | Renode-tested without hardware | Trip event injection at simulator level; assertions on snapshot contents |

---

## 2. Decisions

### A. Polling vs EXTI
**Decision:** Sample inside the modulation ISR. No separate EXTI line.

The modulation ISR runs at 20–40 kHz on the H723. EXTI buys nothing because the trip handler must run *inside* the modulation ISR anyway to coerce-abort the same tick's outputs.

### B. Phase-stepping stop policy
**Decision:** Trip handler co-locates with the modulation ISR. On trip, the ISR (i) freezes curve `t`, (ii) for hybrid stepping: cancels any not-yet-committed step pulse for this tick, (iii) for phase stepping: clamps the phase accumulator at its trip-instant value, holds the last electrical angle, and applies the configured holding current.

Detailed Step-10 contract is in §4.5. The MVP only needs hybrid behavior; phase-stepping is a forward-compatible extension layered on the same trip primitive.

### C. Debounce — consecutive-N
**Decision:** Configurable consecutive-N sample policy on the trigger source. Defaults: N=1 for `TmcDiag`, N=3 for `Physical`.

(Earlier wording said "N-of-M" / "2-of-3" — that's a different filter. We use **consecutive-N**: N consecutive asserted samples in a row. Resets to zero on any de-asserted sample.) Sensorless DIAG is already low-pass-filtered by SG_RESULT in the chip. Physical switches with N=3 at 25 µs = 75 µs filter.

### D. Pre-trip-at-arm — `arm_policy` enum
**Decision:** `arm_policy` per source: `TripImmediately`, `WaitForClear`, `IgnoreUntilMoving` (latched). Defaults: `TripImmediately` for `Physical`, `IgnoreUntilMoving` for `TmcDiag`.

`IgnoreUntilMoving` is a **latch**, not a continuous suppressor. Behavior: ignore all assertions until commanded velocity (selected via `velocity_axis: AxisMask`, see §4.1) has exceeded `v_min` at least once **and** the pin has cleared at least once after that point. Once both conditions are met, the latch flips to "armed-for-real" — trips count even if velocity later drops below `v_min` (e.g., during decel into the rail or under load-induced stall). Closes Codex's correctness hazard about decel suppression.

`velocity_axis` is per-source (closes the §10 open question from rev 1). Defaults: `XY` for X/Y endstops, `Z` for Z endstops.

### E. Position reporting
**Decision:** Raw step counts only over the wire; host computes cartesian via inverse kinematics.

Bridge event payload: `(trip_clock, trip_source_idx, [(stepper_id, step_count); N])`. Per-stepper metadata (steps_per_mm, signs, kinematics mapping) lives in the host's existing `MCU_stepper` registry — counts + ids are sufficient for the host to reconstruct cartesian. `stepper_id` echoes the existing bridge stepper-handle id used in `kalico_push_segment`.

### F. Phase-stepping
Covered by P8 / §2.B / §4.5.

### G. Renode test fixture
**Decision:** Codex is building the GPIO injection + virtual-time-advance + async-event-poll fixture as a separate workstream (delegated 2026-05-03). When that lands, §10's test outline is implementable as written. The fixture is a hard prereq for spec acceptance, not an aspiration.

---

## 3. Bridge protocol — Klipper msgproto

The bridge wire layer is Klipper's existing msgproto (`DECL_COMMAND` host→MCU, `_DECL_OUTPUT`/`sendf` MCU→host). All names, fields, and ids are compiled into the data dictionary at firmware build; the host parses with the dictionary, no hand-packed byte streams.

(Earlier rev 1 §3 hand-packed `op_id` byte layouts — that was wrong for this transport. Rev 2 follows existing `kalico_push_segment` and `kalico_status_v6` precedents.)

### 3.1 `kalico_arm_endstop` (host → MCU)

```c
DECL_COMMAND(command_kalico_arm_endstop,
    "kalico_arm_endstop arm_id=%u arm_clock_lo=%u arm_clock_hi=%u "
    "source_count=%c sources=%*s "
    "stepper_count=%c steppers=%*s");
```

64-bit `arm_clock` split into `arm_clock_lo / _hi` to match the existing convention in `kalico_push_segment` (`src/runtime_tick.c:503`). The arm goes live at this clock; closes the host-MCU race (mainline Klipper's `HOMING_START_DELAY` exists for this) without piggybacking on segment dispatch.

`sources=%*s` — `source_count` records, each (LE):

```
source_kind     u8   = 0 (Physical) | 1 (TmcDiag)
gpio_pin        u16       MCU pin index (resolved from kalico-side pin table)
polarity        u8   = 0 (active-low) | 1 (active-high)
arm_policy      u8   = 0 (TripImmediately) | 1 (WaitForClear) | 2 (IgnoreUntilMoving)
sample_n        u8        consecutive-N debounce (1..=8)
velocity_axis   u8   = bitmask: 0x01=X, 0x02=Y, 0x04=Z
v_min_q16       u32       Q16.16 mm/s velocity-latch threshold; 0 = no gate
```

Size: 11 bytes per source. Up to 4 sources per arm.

`steppers=%*s` — `stepper_count` records of stepper handle ids the trip event must snapshot. Each record:

```
stepper_handle  u32   handle id from kalico_push_segment's x/y/z/e_handle namespace
```

Size: 4 bytes per stepper. Up to `MAX_STEPPERS` per arm. The handle namespace is the runtime's existing per-axis stepper-handle registry (see §3.7), the same handles `kalico_push_segment` already uses to bind axes to curves. No new registry is created.

### 3.2 `kalico_arm_endstop_response` (MCU → host, sync response)

Sent via `sendf` from the command handler, matching the existing
`kalico_push_response` / `kalico_set_homed_response` pattern:

```c
sendf("kalico_arm_endstop_response arm_id=%u status=%c", arm_id, status);
```

`status` ∈ { 0 = Armed, 1 = AlreadyTripped, 2 = Rejected }. `AlreadyTripped`
here is a sync ack status (the source was asserted at arm time under
`TripImmediately`); it is NOT a runtime `ArmState` (see §4.1).

### 3.3 `kalico_endstop_tripped` (MCU → host, async)

```c
DECL_CTR("_DECL_OUTPUT "
    "kalico_endstop_tripped arm_id=%u "
    "trip_clock_lo=%u trip_clock_hi=%u "
    "trip_source_idx=%c "
    "fmt_version=%c "
    "stepper_count=%c stepper_data=%*s");
```

(Async events use `_DECL_OUTPUT` + `output()` from firmware, matching
`kalico_status_v6` / `kalico_credit_freed` / `kalico_fault` at
`src/runtime_tick.c:684–688`. Earlier rev-2 spec used `sendf`, which is the
sync-response convention — fixed in rev 3.)

`trip_clock_{lo,hi}` is the 64-bit MCU clock at the modulation tick where
the trip CAS won.

`fmt_version` is `1` for the MVP integer-counts-only format. Step-10
phase-stepping bumps this to `2` and extends each per-stepper record with
`phase_q16`. Hosts inspect `fmt_version` to dispatch parsers; pure
size-prefixing alone is not enough (Codex N7).

`stepper_data=%*s`, `fmt_version=1`:

```
stepper_handle   u32   handle from §3.1 stepper-binding list
step_count       i32   signed step counter snapshot, after tick-N-1 pulses commit
```

Size: 8 bytes per stepper.

`stepper_data=%*s`, `fmt_version=2` (Step-10 forward-compat):

```
stepper_handle   u32
step_count       i32
phase_q16        u16
```

Size: 10 bytes per stepper. Hosts implementing only `fmt_version=1` MUST
reject events with `fmt_version >= 2`.

Sent at most once per `arm_id`. After emission, the arm transitions to
terminal `TrippedSent`.

### 3.4 `kalico_disarm_endstop` (host → MCU)

```c
DECL_COMMAND(command_kalico_disarm_endstop, "kalico_disarm_endstop arm_id=%u");
```

### 3.5 `kalico_disarm_endstop_response` (MCU → host, sync response)

```c
sendf("kalico_disarm_endstop_response arm_id=%u status=%c", arm_id, status);
```

`status` ∈ { 0 = Disarmed, 1 = AlreadyTripped (trip already queued/sent), 2 = Unknown (no such arm_id) }.

### 3.6 Arm lifetime

```
                          ┌────────────────┐
                          │      Idle      │
                          └────────┬───────┘
                                   │ arm command
                                   ▼
                          ┌────────────────┐
                ┌─────────┤     Armed      ├────────────┐
                │         └────────┬───────┘            │
                │  trip CAS wins   │                    │ disarm
                ▼                  ▼                    ▼
       ┌────────────────┐  ┌────────────────┐  ┌────────────────┐
       │   Tripping     │  │ AlreadyTripped │  │   Disarmed     │
       │ (snapshot wr)  │  │   (terminal)   │  │   (terminal)   │
       └────────┬───────┘  └────────────────┘  └────────────────┘
                │ snapshot published (Release)
                ▼
       ┌────────────────┐
       │ TrippedReady   │
       │  (event queued)│
       └────────┬───────┘
                │ event drained by bridge serializer
                ▼
       ┌────────────────┐
       │ TrippedSent    │
       │   (terminal)   │
       └────────────────┘
```

**Invariant:** exactly one terminal event per `arm_id`. Either `kalico_endstop_tripped` (TrippedSent) or `kalico_disarm_endstop_ack { status=Disarmed }` (Disarmed). `disarm` issued after a trip wins the CAS observes `Tripping`/`TrippedReady`/`TrippedSent` and returns `AlreadyTripped` — the trip event must NOT be elided.

### 3.7 Stepper handle registry

The runtime already maintains a per-axis stepper-handle table consumed by
`kalico_push_segment` (`x_handle`/`y_handle`/`z_handle`/`e_handle`). The arm
command's stepper binding (§3.1) reuses these handles. No new registry; no
new allocation protocol. For a corexy-AB topology, axes A and B map to two
handles in the same namespace; the trip event's `stepper_handle` field
echoes whichever was bound by the arm.

### 3.8 Wire-schema crosscheck

The existing `kalico_push_segment` test pattern asserts that the firmware's compiled msgproto dictionary matches the host-side encoder/decoder. Add equivalent crosschecks for the four new commands plus the two response/event outputs.

---

## 4. Rust runtime — `rust/runtime/src/endstop.rs`

### 4.1 Types

```rust
#[repr(u8)]
pub enum SourceKind {
    Physical = 0,
    TmcDiag  = 1,
}

#[repr(u8)]
pub enum ArmPolicy {
    TripImmediately   = 0,
    WaitForClear      = 1,
    IgnoreUntilMoving = 2,
}

bitflags::bitflags! {
    pub struct VelocityAxis: u8 {
        const X = 0x01;
        const Y = 0x02;
        const Z = 0x04;
    }
}

pub struct Source {
    kind:           SourceKind,
    gpio:           PinId,
    polarity:       bool,            // true = active-high
    policy:         ArmPolicy,
    sample_n:       u8,              // consecutive-N debounce
    velocity_axis:  VelocityAxis,
    v_min_q16:      u32,
    // ISR-private state (only ISR mutates, foreground reads on snapshot publish):
    sample_acc:     u8,              // consecutive-asserted counter
    moved_above_v:  bool,            // velocity-latch sub-state
    cleared:        bool,            // pin de-asserted at least once after moved_above_v
}

#[repr(u8)]
#[derive(Copy, Clone, PartialEq)]
pub enum ArmState {
    Idle           = 0,
    Armed          = 1,
    Tripping       = 2, // ISR-only transient: snapshot is being written
    TrippedReady   = 3, // snapshot complete, event queued for bridge
    TrippedSent    = 4, // event drained
    Disarmed       = 5,
}

pub struct Arm {
    arm_id:        u32,
    sources:       ArrayVec<Source, 4>,
    state:         AtomicU8,            // ArmState
    arm_clock:     u64,                 // becomes effective at this MCU clock
    snapshot:      TripSnapshot,
}

pub struct TripSnapshot {
    // trip_clock_lo + trip_clock_hi form a 64-bit value protected by version seqlock.
    // 32-bit AtomicU64 is not lock-free on Cortex-M7 single-core no-std builds.
    version:           AtomicU32,       // odd = write-in-progress, even = stable
    trip_clock_lo:     AtomicU32,
    trip_clock_hi:     AtomicU32,
    trip_source_idx:   AtomicU8,
    step_count_count:  AtomicU8,
    step_counts:       [AtomicI32; MAX_STEPPERS],
}
```

There is one global `Arm` slot. The single-arm constraint matches the homing path (one `home_start` at a time per axis-group, multi-source OR within one arm). If future probing needs concurrent arms, this generalizes to `[Arm; N]`; not in MVP scope.

### 4.2 ISR hook

`endstop::tick(modulation_clock, v_per_axis_q16, stepper_counts)` is called once per modulation period from the modulation ISR, **after** any pulse decisions for tick N-1 have committed but **before** the per-tick step/current outputs for tick N are committed. The ordering is: stepper_counts represent the count after pulses up to tick N-1; trip on tick N suppresses tick N's pulses.

```rust
pub fn tick(
    clock: u64,
    v_per_axis_q16: [u32; 3],   // |vx|, |vy|, |vz|
    stepper_counts: &[i32],
) -> TripAction {
    let arm_state = ARM.state.load(Ordering::Acquire);
    if arm_state != ArmState::Armed as u8 { return TripAction::Continue; }
    if clock < ARM.arm_clock { return TripAction::Continue; }

    for (idx, src) in ARM.sources.iter_mut().enumerate() {
        // Polarity: clearer than the rev-1 XOR.
        let pin_high = read_pin(src.gpio);
        let asserted = if src.polarity { pin_high } else { !pin_high };

        // Velocity-latch sub-state (IgnoreUntilMoving)
        if matches!(src.policy, ArmPolicy::IgnoreUntilMoving) {
            let v_sel = max_axis_velocity(v_per_axis_q16, src.velocity_axis);
            if !src.moved_above_v && v_sel >= src.v_min_q16 {
                src.moved_above_v = true;
            }
            if !src.moved_above_v {
                src.sample_acc = 0;
                continue;
            }
            // Once moved above v_min, require a clear before counting trips.
            if !asserted {
                src.cleared = true;
                src.sample_acc = 0;
                continue;
            }
            if !src.cleared {
                src.sample_acc = 0;
                continue;
            }
        } else if matches!(src.policy, ArmPolicy::WaitForClear) {
            if !asserted {
                src.cleared = true;
                src.sample_acc = 0;
                continue;
            }
            if !src.cleared {
                src.sample_acc = 0;
                continue;
            }
        } else {
            // TripImmediately: count any assertion.
            if !asserted {
                src.sample_acc = 0;
                continue;
            }
        }

        // Consecutive-N debounce
        src.sample_acc = src.sample_acc.saturating_add(1);
        if src.sample_acc < src.sample_n { continue; }

        // Trip — CAS Armed → Tripping. If lost (foreground disarmed first),
        // drop the trip silently.
        if ARM.state.compare_exchange(
            ArmState::Armed as u8,
            ArmState::Tripping as u8,
            Ordering::AcqRel,
            Ordering::Acquire,
        ).is_err() {
            return TripAction::Continue;
        }

        // Snapshot under seqlock; foreground reads with version-check.
        // Data stores between the odd and even versions use Release to
        // ensure they cannot be observed before the odd marker on
        // weak-memory targets (Codex round-2 M1 partial-fix correction).
        let v0 = ARM.snapshot.version.load(Ordering::Acquire);
        ARM.snapshot.version.store(v0 | 1, Ordering::Release); // write-in-progress
        ARM.snapshot.trip_clock_lo.store(clock as u32, Ordering::Release);
        ARM.snapshot.trip_clock_hi.store((clock >> 32) as u32, Ordering::Release);
        ARM.snapshot.trip_source_idx.store(idx as u8, Ordering::Release);
        for (i, &c) in stepper_counts.iter().enumerate().take(MAX_STEPPERS) {
            ARM.snapshot.step_counts[i].store(c, Ordering::Release);
        }
        ARM.snapshot.step_count_count.store(stepper_counts.len() as u8, Ordering::Release);
        ARM.snapshot.version.store((v0 | 1).wrapping_add(1), Ordering::Release); // stable

        // Publish state.
        ARM.state.store(ArmState::TrippedReady as u8, Ordering::Release);
        TRIP_EVENT_QUEUED.store(true, Ordering::Release);

        return TripAction::AbortNow;
    }
    TripAction::Continue
}
```

**Memory ordering:**
- `state` transitions: `Armed → Tripping` is `AcqRel` CAS. `Tripping → TrippedReady` is `Release` store. Foreground-side `poll_trip` reads with `Acquire`.
- Snapshot fields are written under a seqlock (version counter). Writer toggles version to odd with `Release`, **all data stores between odd and even versions are `Release`** (corrected from rev 2's `Relaxed`, which permitted reordering past the odd marker on weak-memory targets), then writes the next-even version with `Release`. Reader loads version with `Acquire`, retries if odd or if version changed between begin and end of read.
- `arm_clock_{lo,hi}` is set once at arm time; treated as immutable while `state == Armed`.
- **Single-core ISR-vs-foreground assumption:** the ISR runs at higher priority than the foreground; on Cortex-M7 single-core, the ISR is strictly non-preemptable by foreground code. `Release`/`Acquire` semantics are therefore sufficient even without a seqlock; the seqlock is belt-and-suspenders for hypothetical multi-core / DMA-snapshot scenarios. Spec correctness scope: single-core, ISR-non-preemptable. Multi-core requires re-review.

### 4.3 Foreground side

```rust
pub fn arm(msg: ArmMsg) -> Result<(), ArmError> { ... }
pub fn disarm(arm_id: u32) -> DisarmStatus { ... }
pub fn poll_trip() -> Option<TripEvent> {
    if !TRIP_EVENT_QUEUED.swap(false, Ordering::AcqRel) { return None; }
    let s = ARM.state.load(Ordering::Acquire);
    if s != ArmState::TrippedReady as u8 { return None; }
    // Seqlock read of snapshot
    loop {
        let v_begin = ARM.snapshot.version.load(Ordering::Acquire);
        if v_begin & 1 != 0 { continue; } // write in progress
        let snap = read_snapshot_relaxed(&ARM.snapshot);
        let v_end = ARM.snapshot.version.load(Ordering::Acquire);
        if v_begin == v_end {
            ARM.state.store(ArmState::TrippedSent as u8, Ordering::Release);
            return Some(snap);
        }
    }
}
```

`disarm` CAS pattern:
- `Armed → Disarmed` succeeds: emit `kalico_disarm_endstop_ack { Disarmed }`.
- `Tripping`/`TrippedReady`/`TrippedSent` observed: emit `{ AlreadyTripped }`. The trip event still flows host-ward; the host's `MCU_endstop.home_wait` reconciles.

### 4.4 Tests

- Unit: `tick()` matrix `SourceKind × ArmPolicy × sample_n ∈ {1, 3} × velocity-gate {below, above v_min}`.
- Property: trip never fires while `state != Armed`; after trip, no further state mutations until disarm-or-reset.
- Property: disarm-during-trip races. Spawn synthetic concurrency via `loom` or equivalent at host-test build time; assert exactly-one-terminal invariant (§3.6).
- Snapshot seqlock: simulated reader/writer races; reader never returns torn data.
- Renode end-to-end: per §10.

### 4.5 Phase-stepping (Step-10) stop policy

When phase stepping lands, the modulation ISR additionally synthesizes per-tick current commands `i_a(t), i_b(t)` from the curve evaluator's `(t, x_shaped(t), ẋ_shaped(t))`. On trip:

1. `t` is frozen at `t_trip` as in MVP.
2. **Phase accumulator clamp:** the per-stepper electrical phase `φ_s(t)` is clamped at `φ_s(t_trip)`. No further advance.
3. **Current magnitude policy:** the synthesized current ramps from its trip-instant value to the configured `holding_current` over a bounded interval (default 5 ms). No discontinuous magnitude drop, no torque dump that could let the carriage drift.
4. **Snapshot extension:** `TripSnapshot` adds `phase_q16: [AtomicU16; MAX_STEPPERS]` recording fractional electrical phase at trip. Wire format extends `kalico_endstop_tripped`'s per-stepper record with a `phase_q16` u16 field (5 → 7 bytes per stepper). The host opportunistically uses this for sub-step trigger position; it MUST tolerate firmware that doesn't report it (size-prefixed blob handles forward-compat).

Step-10 implementation work is out of scope here. This section nails down the contract so Step 10 doesn't require revisiting trip semantics.

---

## 5. Python — `klippy/motion_bridge.py` and `klippy/mcu.py`

### 5.1 Goal: preserve `MCU_endstop` API

Existing callers (`klippy/extras/homing.py`, `klippy/extras/probe.py`, `klippy/extras/manual_probe.py`, load-cell/eddy probes) call `MCU_endstop.home_start(print_time, sample_time, sample_count, rest_time, triggered)` and `home_wait(home_end_time)`. Those signatures must NOT change.

### 5.2 `BridgeTriggerDispatch`

New class in `klippy/motion_bridge.py`. Replaces the legacy trsync path inside `MCU_endstop` when `_use_bridge=True`. Conforms to the `TriggerDispatch` interface used by `klippy/mcu.py:MCU_endstop`:

```python
REASON_ENDSTOP_HIT     = 1   # legacy-compatible numeric code
REASON_COMMS_TIMEOUT   = 2
REASON_HOST_REQUEST    = 3
REASON_PAST_END_TIME   = 4

class BridgeTriggerDispatch:
    def __init__(self, bridge, reactor):
        self._bridge = bridge
        self._reactor = reactor
        self._completion = reactor.completion()
        self._arm_id = bridge.next_arm_id()
        self._sources = []
        self._trip_event = None     # populated on trip
        self._reason = None         # legacy-compatible reason code

    # --- methods called by MCU_endstop.home_start (matches existing interface) ---
    def get_oid(self):
        return self._arm_id   # the arm_id stands in for the trsync oid

    def get_command_queue(self):
        return self._bridge.get_command_queue()

    def add_stepper(self, mcu_stepper):
        # Stepper handles are sent over the wire as part of the arm
        # command (§3.1). No separate bind op; no host-side bridge state
        # to track between calls.
        self._stepper_handles.append(mcu_stepper.get_bridge_handle())

    # --- new endstop-source binding (called by MCU_endstop after pin parse) ---
    def add_source(self, kind, gpio, polarity, policy, sample_n, velocity_axis, v_min_q16):
        self._sources.append((kind, gpio, polarity, policy, sample_n, velocity_axis, v_min_q16))

    # --- start/stop matching legacy TriggerDispatch surface ---
    def start(self, print_time):
        # Map print_time → MCU clock for arm_clock.
        arm_clock = self._bridge.print_time_to_clock(print_time)
        self._bridge.register_trip_handler(self._arm_id, self._on_trip)
        self._bridge.register_disarm_handler(self._arm_id, self._on_disarm_ack)
        self._bridge.arm_endstop(self._arm_id, self._sources,
                                 self._stepper_handles, arm_clock)
        return self._completion

    def _on_trip(self, evt):
        self._trip_event = evt
        self._reason = REASON_ENDSTOP_HIT
        self._completion.complete(self._reason)

    def _on_disarm_ack(self, status):
        # Only completes if no trip already won
        if self._reason is None:
            self._reason = (REASON_HOST_REQUEST if status == "Disarmed"
                            else REASON_ENDSTOP_HIT)
            self._completion.complete(self._reason)

    def stop(self):
        # Called by MCU_endstop.home_wait. Disarm if no trip yet.
        if self._reason is None:
            self._bridge.disarm_endstop(self._arm_id)
            # Wait briefly for ack.
            self._completion.wait()
        return self._reason   # legacy-compatible int

    def get_trip_event(self):
        return self._trip_event   # contains trip_clock, source_idx, [(stepper_id, count)]
```

### 5.3 `MCU_endstop.home_start` / `home_wait`

```python
def home_start(self, print_time, sample_time, sample_count, rest_time, triggered=True):
    if self._use_bridge:
        td = BridgeTriggerDispatch(self._bridge, self._reactor)
        # Map legacy params → bridge sources.
        # sample_count → sample_n (consecutive-N).
        # sample_time/rest_time: bridge samples at modulation rate.
        # These legacy args are polling-rate hints from the C-firmware
        # endstop_home command; they have no analog in the bridge ISR
        # path. Documented as ignored. If a future caller depends on
        # explicit sample timing (e.g., load-cell probes), it is
        # routed through the legacy path (§5.4) and never sees this
        # branch.
        # triggered=True → TripImmediately; triggered=False → WaitForClear.
        kind, gpio, polarity = self._resolve_pin()  # Physical or TmcDiag
        policy = (ArmPolicy.TripImmediately if triggered
                  else ArmPolicy.WaitForClear)
        if kind == 'TmcDiag' and triggered \
                and not self._sensorless_trip_immediately:
            # Sensorless default; caller opts out via setting
            # _sensorless_trip_immediately on the MCU_endstop instance
            # (configured by extras/tmc.py per [tmc5160 stepper_x] config
            # option `homing_trip_immediately: True`, off by default).
            policy = ArmPolicy.IgnoreUntilMoving
        td.add_source(kind, gpio, polarity, policy,
                      sample_n=sample_count,
                      velocity_axis=self._velocity_axis_for_pin(),
                      v_min_q16=self._sensorless_v_min_q16())
        for s in self._steppers:
            td.add_stepper(s)
        self._dispatch = td
        return td.start(print_time)
    # ...legacy path unchanged...

def home_wait(self, home_end_time):
    if self._use_bridge:
        # Bounded reactor wait: completion or print_time deadline.
        eventtime = self._reactor.monotonic()
        deadline = eventtime + (home_end_time - self._mcu.estimated_print_time(eventtime))
        completion = self._dispatch._completion
        # Reactor.completion supports wait(waketime); returns None on timeout.
        result = completion.wait(waketime=deadline)
        if result is None:
            # Timeout — no trip arrived by deadline. Disarm and report.
            self._bridge.disarm_endstop(self._dispatch._arm_id)
            self._dispatch._reason = REASON_PAST_END_TIME
            return 0
        reason = self._dispatch.stop()
        if reason == REASON_ENDSTOP_HIT:
            evt = self._dispatch.get_trip_event()
            # Per-stepper position-at-trigger from snapshot.
            for handle, count in evt.stepper_data:
                stepper = self._lookup_stepper_by_handle(handle)
                stepper.note_homing_step_count(count)
            return self._bridge.clock_to_print_time(evt.trip_clock)
        return 0   # disarmed-by-host or other non-trip terminal
    # ...legacy path unchanged...
```

`note_homing_step_count(count)` is a new lightweight method on `MCU_stepper` that converts the count to a position via the existing per-stepper step distance & sign metadata, equivalent to what `stepcompress_find_past_position` returned in legacy. Existing `homing.py:HomingMove` post-processing (`note_home_end`) consumes the position the same way.

### 5.4 Bridge-mode `_build_config` skips legacy endstop commands

`klippy/mcu.py:447,460` registers `config_endstop` and `endstop_home` MCU
commands during `_build_config` for every `MCU_endstop`. In bridge mode
(`_use_bridge=True`), these legacy commands are not sent: the kalico
firmware does not implement them, and sending them would either be
rejected or worse, conflict with the bridge arm/disarm path.

Spec contract: `MCU_endstop._build_config` checks `_use_bridge` and skips
`mcu.lookup_command(...)` for `config_endstop` / `endstop_home` /
`endstop_query_state`. `query_endstop` (the standalone-pin polling path,
used by `QUERY_ENDSTOPS`) is preserved via a bridge-side
`kalico_query_pin gpio=%u` op (added in §3.x as a follow-up; not blocking
homing implementation).

### 5.5 Probing / virtual-pin compatibility matrix

| Caller | API used | Bridge supports? | Notes |
|---|---|---|---|
| `homing.py::HomingMove` (G28) | `home_start`/`home_wait`, `multi_complete` | Yes (MVP) | Primary case |
| `probe.py::PrinterProbe` | Same `MCU_endstop` API | Yes (MVP) | Z probe via single physical/virtual endstop |
| `manual_probe.py` | `MCU_endstop` reused | Yes (MVP) | No change beyond probe inheritance |
| `bed_mesh.py` | Calls into `probe.py` | Yes (MVP) | Inherits from probe |
| `tmc.py::TMCVirtualPinHelper` | Registers DIAG as virtual endstop | Yes (MVP) | Resolved to `TmcDiag` source kind |
| `load_cell_probe.py` | Custom `MCU_endstop`-like (host-side trigger) | Out of scope | Handled by host event loop; doesn't use bridge trip path |
| `probe_eddy_current.py` | Custom virtual endstop (chip-internal trigger) | Out of scope | Host-resolves trigger; uses `MCU_endstop` only as a shell |

Out-of-scope cases continue to work because they never reach `_use_bridge=True` — they have their own host-side completion path. We ensure the bridge branch isn't accidentally engaged by checking whether the resolved pin is on the kalico bridge MCU; if not, fall through to the existing path. Guard test added.

---

## 6. Trip-aware `wait_moves` / motion_toolhead

### 6.1 Bridge runtime contract

Current `bridge.wait_moves()` calls `planner.flush()` only. That's not
trip-aware. Rev 3 contract:

- Bridge tracks `homing_segment_state: AtomicU8` (Idle / Active /
  Completed / Tripped).
- A new dedicated entry point `bridge.submit_homing_move(newpos, speed,
  arm_id)` (Python) → `submit_homing_move(...)` (Rust pyo3) submits the
  segment, marks `homing_segment_state=Active`, ties it to `arm_id`, and
  bypasses the `commanded_pos` advance the regular `submit_move` does.
  This is a single atomic API — no mutable-bridge-flag-then-submit
  footgun (Codex round-2 N5).
- Regular `submit_move` is unchanged. The bridge does NOT inspect the
  arm state; only `submit_homing_move` engages trip semantics.
- The runtime sets `homing_segment_state=Tripped` when the ISR aborts
  (via `endstop::tick` returning `AbortNow`); sets `Completed` when the
  segment retires naturally.
- `wait_moves()` blocks until `homing_segment_state` exits `Active`. It
  is the same call non-homing callers use; the only effect of `Active`
  is that the wait gets a richer terminal state to inspect.
- Bridge-side `commanded_pos` is reconciled from the trip snapshot (or
  from the segment's natural endpoint on `Completed`) inside `home_wait`,
  not at submit time.

### 6.2 `motion_toolhead.drip_move`

```python
def drip_move(self, newpos, speed, drip_completion):
    # Endstops were armed by homing.py via mcu_endstop.home_start.
    # The drip_completion's underlying BridgeTriggerDispatch already has
    # an arm_id. We submit a homing-tagged segment so the bridge runtime
    # engages trip semantics; on trip the ISR aborts and freezes the
    # curve evaluator. wait_moves() returns when the segment retires
    # (Completed or Tripped). commanded_pos reconciliation happens in
    # home_wait via the trip snapshot, not here.
    arm_id = drip_completion.get_arm_id()
    self.bridge.submit_homing_move(newpos, speed, arm_id)
    self.bridge.wait_moves()
```

`drip_completion.get_arm_id()` is added to the `multi_complete` wrapper
and the underlying `BridgeTriggerDispatch.start()` return value: the
completion exposes a stable `arm_id` accessor for the homing path. For
multi-source `multi_complete`, all sources share one arm; the wrapper
returns that arm_id.

**No `set_next_segment_homing()` flag.** The arm_id-bearing
`submit_homing_move` is the only path that engages homing semantics; you
cannot accidentally engage it by calling regular `submit_move` and you
cannot accidentally leave it engaged for a subsequent move.

---

## 7. Multi-MCU forward contract

EtherCAT / multi-MCU is out of scope for MVP; the design must not foreclose it.

**Logical-arm model.** A homing operation creates one **logical arm** at the `MCU_endstop` layer. If the steppers/sources span multiple MCUs, the host fans out one **per-MCU arm** (separate `arm_id` per MCU) sharing a synchronized `arm_clock`, expressed in each MCU's local clock domain via the existing clock-sync (Step 6).

**First-trip election.** The host receives at most one trip event per per-MCU arm. The host translates each `trip_clock` into the global host time and elects the earliest as the canonical trigger. After election, the host issues `kalico_disarm_endstop` to the other MCUs; their per-MCU arms either return `Disarmed` (no trip yet) or `AlreadyTripped` (trip event in flight, suppress on host side).

**Late-event reconciliation.** A late trip event on a non-canonical MCU is logged but does not change the elected result.

**Wire format implication.** `trip_clock` is per-MCU (no need to embed MCU id in the event; the host already knows which MCU sent it). No protocol changes needed for multi-MCU; the host orchestration handles it.

This replaces trsync's role explicitly: trsync existed to dispatch one trigger across multiple MCUs over a serial link. With our per-MCU arms and host-side election, we have the same OR semantics with strictly lower complexity.

---

## 8. `homed` gate ownership

The runtime already exposes `state::homed: AtomicBool` and a no-arg
`kalico_set_homed` command (`src/runtime_tick.c:524–534`) that
unconditionally sets it to true and emits `kalico_set_homed_response
result=%i`. Rev 3 extends — not collides with — that command.

**Wire-format extension** (firmware change required as part of this
work):

```c
// Existing (no-arg, unconditional set-true) becomes:
DECL_COMMAND(command_kalico_set_homed, "kalico_set_homed homed=%c");
// Response unchanged:
sendf("kalico_set_homed_response result=%i", r);
```

To preserve backward compatibility with any host that omits the
argument, the parser accepts the no-arg form and treats it as
`homed=1` (legacy semantics). The runtime side gains a clear path:
`homed=0` clears the gate.

**Lifecycle:**

- **Set:** host issues `kalico_set_homed homed=1` after a successful
  homing of all required axes (typically after `G28`), from
  `homing.py::HomingMove.homing_move` post-success.
- **Cleared:** runtime auto-clears on shutdown, FAULT, or any axis
  losing position (e.g., disabled stepper, reset). Host can also clear
  via `kalico_set_homed homed=0` (e.g., on `M84`).
- **Granularity (MVP):** single bool. The MVP requires all axes homed
  before motion. Per-axis flags are a Step-10 generalization; out of
  scope.
- **Failure behavior:** if homing fails (timeout, never tripped), the
  host does NOT issue `homed=1`. Runtime keeps motion gated until a
  successful homing.

The existing `kalico_set_homed_response` ack confirms the runtime
applied the change.

---

## 9. Implementation order

1. **Renode GPIO injection fixture** (in flight, delegated to Codex) — hard prereq for §10.
2. **Rust: `endstop.rs`** with types, `tick()`, `arm()`, `disarm()`, `poll_trip()`, seqlock snapshot, full state machine. Unit + property tests with `loom` (or equivalent) for the CAS race. ~500 LOC.
3. **Modulation ISR integration**: invoke `endstop::tick()` per period; honor `TripAction::AbortNow`; freeze curve `t`; tick-N pulse cancellation. Hybrid stepping only. ~50 LOC + tests.
4. **Bridge runtime contract (§6.1)**: `homing_segment_active`/`_state`, trip-aware retire, `commanded_pos` non-update for homing segments. Bridge crate test.
5. **Bridge protocol**: msgproto schemas for the four new commands + two outputs. Wire-schema crosscheck tests (§3.7).
6. **Renode end-to-end test** using fixture from step 1: arm + segment + GPIO inject + assert trip event correctness.
7. **`BridgeTriggerDispatch`** + `MCU_endstop.home_start`/`home_wait` rewiring. Unit tests with mocked bridge.
8. **`motion_toolhead.drip_move`** collapse + `kalico_set_homed` integration.
9. **Hardware bring-up (2b-3 / 2b-4)**: `G28 X` on real H723.

Each step compiles and passes its tests before the next. Steps 1–6 can be parallelized once step 1 lands.

---

## 10. Renode test outline

Pseudocode against the fixture Codex is building:

```python
def test_endstop_trip_mid_segment():
    fix = RenodeBridgeFixture()
    # Arm a Physical endstop on a known GPIO.
    arm_id = fix.arm_endstop(sources=[
        Physical(gpio=42, polarity=1, policy=TripImmediately, sample_n=2,
                 velocity_axis=XY, v_min_q16=0)
    ], arm_clock=fix.now() + 1000)
    fix.expect_arm_ack(arm_id, status=Armed)
    # Submit a long X-axis homing segment (e.g. 100 mm at 20 mm/s → 5 s).
    seg_id = fix.push_homing_segment(...)
    fix.advance_to_clock(fix.now() + int(1.5 * MCU_HZ))
    fix.gpio_set(42, 1)
    # Two modulation periods to debounce + trip.
    fix.advance_ticks(2 * MODULATION_PERIOD_TICKS)
    evt = fix.expect_tripped(arm_id)
    # Trip clock landed within (sample_n)+1 modulation periods of injection.
    expected = int(1.5 * MCU_HZ)
    assert expected <= evt.trip_clock <= expected + 3 * MODULATION_PERIOD_TICKS
    # Step counts match curve evaluator at t_trip (corexy: A=B=30mm).
    a_count = step_count_for_x(30.0)
    assert evt.stepper_data[A_STEPPER] == a_count
    assert evt.stepper_data[B_STEPPER] == a_count
```

Coverage matrix:
- `SourceKind × ArmPolicy × sample_n ∈ {1, 3}`.
- `IgnoreUntilMoving` velocity-latch: assert pre-`v_min` assertions are ignored, post-`v_min`-and-clear assertions trip.
- Multi-source OR: 2 sources, src-1 trips first, src-0 never asserts; assert `trip_source_idx=1` and src-0 latch state is irrelevant.
- Disarm vs trip race: disarm command in flight when trip CAS wins; assert exactly-one terminal event (trip wins, disarm ack returns `AlreadyTripped`).
- `AlreadyTripped` arm: GPIO already asserted at arm; `TripImmediately` returns `AlreadyTripped` synchronously.
- Phase-stepping stop-policy check (deferred to Step-10): phase clamp + holding-current ramp.
- Multi-MCU first-trip election (deferred to multi-MCU work): two per-MCU arms, both trip; host elects earlier `trip_clock`.

---

## 11. What this design explicitly does NOT do

- **Closed-loop / position correction.** The `step_counts` in the trip event are open-loop counter snapshots; they ARE the commanded position by construction.
- **Mid-print hard-limit fault.** The arm/trip primitive is reusable for hard-limit arming during normal motion (always-armed sources), but the policy decision (continue vs estop on trip during a print) is out of scope.
- **Concurrent arms.** Single global `Arm` slot. Generalizes to `[Arm; N]` if probing+homing concurrency becomes a requirement.
- **Per-microstep position snapshot at trip during phase stepping.** Step-10 work; contract specified in §4.5 so it doesn't require revisiting trip semantics.

---

## 12. References

- Klipper (mainline) `endstop.c` / `trsync.c` / `mcu.py` / `homing.py` / `extras/tmc.py` / `stepper.py`: https://github.com/Klipper3d/klipper
- Kalico fork: https://github.com/KalicoCrew/kalico
- Prunt firmware `step_generator.adb` / `input_switches.adb`: https://github.com/Prunt3D/prunt_board_2_software
- LinuxCNC `motion(9)` HAL pin reference, `control.c`: https://github.com/LinuxCNC/linuxcnc
- Marlin `ENDSTOP_INTERRUPTS_FEATURE` discussion: https://github.com/MarlinFirmware/Marlin/issues/5102
- Smoothieware `Endstops.cpp`: https://github.com/Smoothieware/Smoothieware
- Voron sensorless homing tuning: https://docs.vorondesign.com/tuning/sensorless.html

Per-section file references for current kalico-rewrite code are inline.
