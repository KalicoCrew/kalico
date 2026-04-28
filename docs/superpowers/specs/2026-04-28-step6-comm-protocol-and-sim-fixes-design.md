# Step 6 — Communication Protocol, Clock Sync, and Simulator Fixes

**Date:** 2026-04-28
**Status:** Spec — design under brainstorm review; implementation plan to follow on green-light.
**Layer:** 5 (host↔MCU communication, clock sync, telemetry transport — partial; F4x integration deferred to parallel workstream).
**Driver:** Build-order Step 6 — *"Communication protocol and clock sync — Layer 5"*. Phase 0 folds in the two Step-5-leftover simulator follow-ups (Renode GDB-attach for `load_curve` hang; software CYCCNT under `CONFIG_KALICO_SIM`).

---

## 1. Context

Step 5 shipped the MCU runtime framework with stub output, no live producer, and a known-broken simulator. Step 7 MVP needs a host that pushes real planner output to a real MCU at print pace, with deterministic-enough timing to never silently lose throughput. Step 6 is the connective tissue: the wire-level protocol, the credit-based flow control, the multi-MCU clock synchronization, the buffer-budget framework that makes all of that load-bearing under realistic load, and the simulator infrastructure that lets the rest of Step 6 iterate in seconds rather than flash cycles.

This spec was hardened across two adversarial review rounds (`codex:codex-rescue` × 2, `kalico-verifier` × 2 — see §16). Most of the brainstorm-phase decisions survived; the buffer-budget commitment was converted from "fixed numbers" to "framework + measurement protocol + parameter linkage" after the second review surfaced that the originally-proposed numbers were engineering judgment, not measurements.

### 1.1 Driving constraints (inherited)

- **Rust end-to-end** for new code; standalone Rust kalico-host-rt owns the USB-CDC fd directly. `tools/kalico_host_io.py` (Step 5) explicitly bypasses `klippy/reactor.py` + `klippy/serialhdl.py`; Step 6 ratifies this direction.
- **CLAUDE.md "Real time communication, no queue-based offload."** Interpreted as: no Klipper-style ~2-second offline queue. Bounded host-stall budget enforced by **FAULT-on-underrun**, not silent slowdown. Buffer depth is "deep enough to absorb measured host tail latency," not "deep enough to coast."
- **CLAUDE.md print-throughput-non-negotiable.** Every parameter in the buffer-budget framework derives from MCU runtime cost or measured host-stall distribution — never from soft "comms convenience" arguments. Layer-3 minimum-segment-duration is a binding hardware constraint, not a coalescing trick.
- **RT_PREEMPT not required at Step 6** (MCU does the hard real-time work; Pi 5 host stall budget is absorbed by the MCU buffer). RT_PREEMPT becomes a hard requirement at Step 14 (EtherCAT). Step 6 is RT_PREEMPT-friendly: no blocking syscalls on the segment-push path, dedicated RX thread, no Python in the data-plane critical path.
- **NURBS-on-the-wire.** Wire format is trajectory-shaped, not step-event-shaped. Avoids the host-side step-event discretization that earns Klipper its loose-real-time reputation.
- **Multi-MCU coordination architecturally designed-in.** F4x bring-up itself is a parallel workstream, but the protocol's ability to drive two MCUs in lockstep is a Step-6 design problem.

### 1.2 Non-goals

- **F4x integration testing.** Bring-up of the F4 Octopus (Z-axis, TMC2209) uses the protocol designed here but is its own workstream. Step 6 ships single-MCU validation against H723 + sim.
- **Heater PID, fans, endstops, gcode file ingest, slicer integration.** Step 7 MVP scopes the minimum non-motion surface needed for first print.
- **Host/klippy IPC boundary for non-motion subsystems.** Step 7 MVP. (How a kalico-host-rt motion fault propagates to a Klippy supervisor that owns thermals.)
- **Mid-print parameter adjustment** (live shaper retune, live PA tweak). Post-MVP.
- **Automatic recovery from FAULT** beyond identify-handshake re-establishment. Post-MVP. Faults are sticky on the MCU until power-cycle / explicit reset (which Step 6 does not ship).
- **`kalico_runtime_reset()` FFI.** Post-MVP.
- **EtherCAT-specific design work.** Step 14 territory. The data-plane format is versioned and msgproto is a framing adapter, so the Step-14 migration path is clean — see §4.2 for the design contract.

---

## 2. Architecture

### 2.1 Component layout

```
┌────────────────────────────────────────────────────────────────────────┐
│ Pi 5 host                                                              │
│                                                                        │
│  ┌─────────────────────────────────────┐                              │
│  │ kalico-host-rt (standalone Rust)    │                              │
│  │  - USB-CDC fd ownership (per MCU)   │                              │
│  │  - clock-freq estimator (per MCU)   │                              │
│  │  - per-MCU credit counter           │                              │
│  │  - segment producer (Layer 1/2/3 → wire)                           │
│  │  - status-frame consumer            │                              │
│  │  - trace consumer                   │                              │
│  │  - fault aggregator                 │                              │
│  └─────────────┬───────────────────────┘                              │
│                │ USB-CDC (msgproto envelope)                          │
└────────────────┼───────────────────────────────────────────────────────┘
                 │
   ┌─────────────┴────────────────┐
   ▼                              ▼
┌─────────────────┐         ┌─────────────────┐
│ MCU 1 (H723)    │         │ MCU 2 (F4x)     │
│ XY+E (CoreXY)   │         │ Z (TMC2209)     │
│ - 40 kHz TIM5   │         │ - timer ISR     │
│ - kalico runtime│         │ - kalico runtime│
│ - half-split    │         │ - half-split    │
│   FgState/IsrSt │         │   FgState/IsrSt │
└─────────────────┘         └─────────────────┘
```

Each MCU runs the same kalico runtime (Step-5 substrate), refactored into a half-split SPSC architecture (§12) so the foreground command-dispatch task and the ISR have non-overlapping `&mut` references to disjoint memory.

### 2.2 What flows over the wire

**Host → MCU (control plane, msgproto-typed commands):**
- `kalico_load_curve` (Step-5; remains as-is)
- `kalico_load_fixture_curve` *(NEW — sim-only, §4.3)*
- `kalico_push_segment` (Step-5; carries kalico-versioned segment payload — §5.2)
- `kalico_stream_open` *(NEW — §9)*
- `kalico_stream_arm` *(NEW — §9, multi-MCU atomic-start commit)*
- `kalico_stream_terminal` *(NEW — §9)*
- `kalico_stream_flush` *(NEW — §9, emergency abort)*
- `kalico_clock_sync_request` *(NEW — §13)*
- `kalico_query_status` (Step-5; remains)

**MCU → host (control plane responses + async events):**
- `kalico_load_curve_response result=%i` (Step-5)
- `kalico_push_response result=%i accepted_through_segment_id=%u credit_epoch=%u` *(extended)*
- `kalico_stream_*_response` *(NEW)*
- `kalico_clock_sync_response mcu_clock=%llu` *(NEW)*
- `kalico_status engine_status=%c queue_depth=%c current_segment_id=%u last_fault=%hu fault_detail=%u mcu_clock_now=%llu` *(extended; periodic ~10 Hz, primary clock-sync sample piggyback)*
- `kalico_credit_freed accepted_through_segment_id=%u free_slots=%c` *(NEW — primary flow-control event, §6)*
- `kalico_fault fault_code=%hu fault_detail=%u segment_id=%u` *(NEW — async event)*
- `kalico_trace count=%u data=%*s` (Step-5; trace ring drain — extended schema in §14)

### 2.3 Build-system shape (additive)

Step 5's workspace + cbindgen surface stays. Step 6 adds:

- New Rust modules under `rust/runtime/src/`: `comms.rs` (engine-side protocol decoder), `stream.rs` (lifecycle state machine), `clock_sync.rs` (MCU-side clock-sync responder), `gen_handle.rs` (generation-counter curve handle).
- New host crate `rust/kalico-host-rt/` (or extension of an existing host crate — the Step-7 spec will pick a final layout). Step 6 ships at least: `host_io/` (msgproto-aware fd owner), `clock_sync/` (per-MCU estimator), `credit/` (per-MCU credit counter), `producer/` (Layer-1/2/3 → wire glue), `fault/` (fault-aggregator + abort).
- Python `tools/kalico_host_io.py` continues to exist as a Step-6-test-harness adapter (drives the Rust kalico-host-rt's protocol surface from Python tests). Final production host is Rust.

### 2.4 Build invocation

Unchanged from Step 5 for the MCU staticlib. Host-side build adds:

```sh
# host build (kalico-host-rt + smoke tests)
cargo build -p kalico-host-rt
cargo test -p kalico-host-rt
```

---

## 3. Phase 0 — Simulator fixes (1-day timebox)

The Renode H7 simulator (Step-5 scaffolding, `tools/sim/`) lets us iterate protocol changes in seconds instead of flash cycles. Two known-broken paths must close before Step-6 protocol work proceeds. Per CLAUDE.md memory ("Trident is a test bench"), printer availability is not a constraint; the sim accelerates the fast-iteration loop, not the bring-up sequence.

### 3.1 Software CYCCNT under `CONFIG_KALICO_SIM`

The H743 .repl tags `DWT->CYCCNT` as opaque memory; reads return 0, so the engine widening loop ingests zero-time samples and segment evaluation never advances.

**Fix:** C-side fork in `src/stm32/kalico_h7_timer.c::kalico_h7_read_cyccnt()`.

```c
__attribute__((used, externally_visible))
uint32_t
kalico_h7_read_cyccnt(void)
{
#if CONFIG_KALICO_SIM
    extern uint32_t kalico_sim_cyccnt;  // defined in src/stm32/kalico_sim_clock.c
    return kalico_sim_cyccnt;
#else
    return DWT->CYCCNT;
#endif
}
```

Sim-side counter (`src/stm32/kalico_sim_clock.c`, gated on `CONFIG_KALICO_SIM`):
- `volatile uint32_t kalico_sim_cyccnt` initialized to 0 at runtime_init.
- Incremented by a fixed delta per TIM5 ISR fire (delta = `kalico_clock_freq / 40000`, the cycles-per-tick the production code would observe).
- Optionally seeded from a startup-monotonic source if Renode exposes one; otherwise pure ISR-incremented is fine for engine progress.

The Rust runtime is unaware of the fork. No Cargo feature, no build.rs change. The C abstraction is where the abstraction belongs because the C side already owns DWT register access.

**Test:** `tools/test_h723_first_light.py --port socket://localhost:3334` against the sim — the multi-tick segment evaluation must advance, status frame must report increasing `mcu_clock_now`, trace samples must emerge with monotone tick counters.

### 3.2 `load_curve` hang — root-cause attempt + escape hatch

**Half 1 (≤4h): GDB-attach root-cause.**

The `tools/sim/h723_sim.resc` already documents `machine StartGdbServer 3333` as the path. Procedure:

1. Boot sim with GDB server enabled.
2. From host: `arm-none-eabi-gdb out/klipper.elf -ex "target remote :3333"`.
3. Send `kalico_load_curve` from `kalico_host_io`. When firmware blocks, `Ctrl-C`. Inspect `pc`, `lr`, the call stack, and any HardFault-handler chain.
4. If `pc` lands in the `%*s` decode path (`command.c`) on a peripheral memory access — Renode platform-model hole, escalate to half 2.
5. If `pc` is in kalico C glue or Rust FFI — a kalico-side bug, fix in place.

**Half 2 (≥4h): sim-only fixed-fixture preload escape hatch.**

If GDB confirms a Renode H7 modeling hole (verifier cited renode/renode#618, #626, #649 documenting H7 peripheral gaps), bypass `load_curve` in the sim path entirely.

New command, gated on `CONFIG_KALICO_SIM`:

```c
#if CONFIG_KALICO_SIM
DECL_COMMAND(command_kalico_load_fixture_curve,
    "kalico_load_fixture_curve slot=%hu fixture_id=%hu");

void
command_kalico_load_fixture_curve(uint32_t *args)
{
    uint16_t slot = args[0];
    uint16_t fixture_id = args[1];
    int32_t r = kalico_runtime_load_fixture(kalico_rt_handle, slot, fixture_id);
    sendf("kalico_load_fixture_response result=%i", r);
}
#endif
```

Rust side: `kalico_runtime_load_fixture(rt, slot, fixture_id)` calls a static fixture table compiled into the firmware (`runtime/src/sim_fixtures.rs`, gated on a `kalico-sim` Cargo feature wired into the staticlib build via Klipper's autoconf bridge). Initial fixture set:

- `straight_line_x` (degree-1, 2 CP)
- `quarter_arc_xy` (degree-2 rational, 3 CP)
- `cubic_bezier_xy` (degree-3, 4 CP)

Production firmware never compiles `sim_fixtures.rs` (gated on Cargo feature; Cargo feature gated on `CONFIG_KALICO_SIM=y` via build.rs that reads autoconf.h or the workspace-passed env var). NEVER flash a `CONFIG_KALICO_SIM=y` image to silicon — the sim build also disables IWDG, which is a safety footgun (already documented in Step-5).

### 3.3 Acceptance gates

Phase 0 has **two distinct gates** — one for sim infrastructure (closes with the 1-day timebox), one for Step-6 features (closes after §7/§8 implementation lands and re-uses the sim for validation). Round 1 review caught this conflation; the gates are now split.

**Gate A — Phase 0 sim-fix gate (closes Phase 0; ≤1 day):**

1. Sim starts; firmware boots; identify handshake succeeds.
2. Host streams 10 segments using either `kalico_load_curve` (if root-caused) or `kalico_load_fixture_curve` (if escape hatch).
3. MCU evaluates each in order; trace stream reports monotone tick counters and correct segment_id sequence.
4. End-to-end iteration loop ≤30 seconds (vs ≥3 minutes flash-and-reboot).

**Gate B — Step-6 feature validation against sim (closes after §7/§8/§9 implementation):**

5. Status frame reports correct `current_segment_id`, `queue_depth`, `retired_through_segment_id` throughout.
6. Underrun-fault path: stop pushing while stream-open → MCU latches `KALICO_FAULT_UNDERRUN` within `MIN_SEGMENT_DURATION_MS` of last-segment retirement.
7. Trace-overflow-fault path: throttle host trace-drain → MCU latches `KALICO_FAULT_TRACE_OVERFLOW` once trace ring saturates.

Gate A is the prerequisite for proceeding with Step-6 implementation. Gate B re-validates against sim once the implementation reaches a consistent state. Both must pass before Step 6 is considered done.

**Build-system note (Round 1 review):** the `kalico-sim` Cargo feature in §3.2 requires `src/Makefile.kalico` (and possibly `src/Makefile`) to thread `CONFIG_KALICO_SIM=y` through to the cargo invocation as an additional `--features kalico-sim` flag. Plan budget: ~1h within the Phase 0 timebox.

---

## 4. Wire framing

### 4.1 Transport: Klipper msgproto (extended)

msgproto is the framing/transport adapter only. It provides:
- 16-bit CCITT CRC framing.
- 4-bit sequence number with NAK + retransmit (Klipper docs assert in-order, error-free, deduplicated delivery).
- Self-describing data dictionary at startup (identify handshake).
- `%*s` blob carriers for binary payloads.

**Architectural assertion (load-bearing for Step-14 EtherCAT migration):** msgproto is a framing adapter, not a semantic dependency. No kalico command relies on Klipper's foreground command-dispatch ordering or scheduling guarantees. All ordering invariants (e.g., "curve must be loaded before any segment referencing it is pushed") are enforced at the kalico-runtime boundary, not at the protocol layer. At Step 14, msgproto is replaced by EtherCAT's PDO-mapped cyclic frames; the segment binary format and the kalico semantic invariants carry over unchanged.

Implementation note: the Klipper msgproto 4-bit sequence window size is unspecified in public docs. `tools/kalico_host_io.py` already maintains its own `self._seq` counter and open-codes encode_msgblock to route around documented Klipper bugs (see comment at `kalico_host_io.py:241-244`). Step 6 documents this empirical contract in `docs/research/msgproto-empirical-contract.md` (write during Phase 0 by reading the Klipper source).

### 4.2 Data-plane payload format (kalico-versioned)

Every kalico-native blob payload carried inside msgproto's `%*s` is prefixed with a **1-byte format-version field**, followed by the binary struct in little-endian (matches MCU; no host-MCU endian conversion needed since both ARM-LE and Pi-LE).

```
kalico-blob-payload:
  u8  format_version    // v1 = 0x01
  ... typed binary ...
```

For Step 6, only `v1 = 0x01` is defined. Future schema evolution: bump `format_version`, document in a spec amendment, MCU rejects unknown versions with `KALICO_FAULT_PROTOCOL_VERSION_UNSUPPORTED`.

The format-version field replaces the rejected "kalico-native protocol layer parallel to msgproto" approach (Codex round 1 Claim 4, overstated per kalico-verifier). It gives us versioning and framing-adapter independence at minimal cost (1 byte + a `match` arm).

### 4.3 Command schemas

Each new kalico DECL_COMMAND uses Klipper-typed args for the control fields (named, self-described, debuggable) and `%*s` only for the kalico-native binary (segment payload, trace data, fixture preload). This split was decided in brainstorm Q5 and preserves msgproto's debuggability for the high-fan-out control surface while sidestepping `%*s` ABI issues for the data plane.

Concrete schemas listed under each respective section (§6, §9, §13, §14).

### 4.4 Endianness & alignment

- All multi-byte integers in kalico blob payloads: **little-endian**, packed (no struct padding beyond what alignment requires).
- f32 and f64: IEEE 754 little-endian (matches both ARM-LE and Pi-LE).
- Alignment: each blob is decoded into a `#[repr(C)]` Rust struct; the struct layout matches the wire layout exactly. Static asserts on `size_of::<...>()` for every wire-mapped struct gate the build.

---

## 5. Flow control (α — credit-based, MCU-authoritative)

Per brainstorm Q3 (recommendation α, accepted): the MCU is the only authoritative observer of "segment retired"; the host's clock model is always slightly stale. Credit messages are explicit MCU-emitted events; the host counts credits and pushes when credit > 0.

### 5.1 Credit message contract

`kalico_credit_freed` is emitted by the MCU foreground task when a segment retires (its `t_end` has passed and the next segment has been popped from the queue, or the segment was retired due to a flush).

```
kalico_credit_freed retired_through_segment_id=%u free_slots=%c
```

Fields:
- `retired_through_segment_id` (u32) — monotonically increasing; "the MCU has fully retired all segments with id ≤ this value." Host-side idempotency: if the host receives `retired_through=N` after already processing `retired_through=M ≥ N`, the event is a no-op.
- `free_slots` (u8) — current free queue capacity after this retirement.

**Naming discipline (Round 1 review fix):** earlier drafts reused `accepted_through_segment_id` for two distinct counters — "last enqueued/accepted by the MCU" (push-time, in `kalico_push_response`) vs "last retired/consumed by the engine" (run-time, in `kalico_credit_freed`). These are different events at different layers. Step 6 splits them: `accepted_segment_id` is push-time (the just-accepted segment's id, returned in the push-response); `retired_through_segment_id` is run-time (the last-retired segment's id, emitted in credit-freed events and reported in periodic status). Both fields appear in the periodic status frame so the host can detect divergence.

Per kalico-verifier round 1: msgproto already provides in-order, deduplicated, error-free transport. Codex's larger field list (`mcu_id, credit_epoch, queue_capacity, active_segment_id`) is over-specified at the per-event layer. `mcu_id` is implicit (the message arrives on a specific fd). `queue_capacity` is static and learned at identify time. `active_segment_id` is informational and need not gate anything.

### 5.2 Session-establishment epoch

`credit_epoch` is a **session-level** field, not a per-event field. It bumps on:
- MCU power-on / reset (segment-id space restarts).
- Stream-open / stream-flush (queue contents conceptually invalidated).

The host learns the current `credit_epoch` from the `kalico_stream_open_response` and from any `kalico_status` frame; if the host observes a `credit_epoch` change, it resets its credit counter.

### 5.3 Periodic status frame as backstop

`kalico_status` (extended from Step-5) emits at ~10 Hz with the full state:

```
kalico_status engine_status=%c queue_depth=%c current_segment_id=%u
              last_fault=%hu fault_detail=%u mcu_clock_now=%llu
              credit_epoch=%u accepted_segment_id=%u retired_through_segment_id=%u
```

The status frame:
- Carries authoritative full-state for credit reconciliation. If the host's credit count diverges from `(queue_capacity - queue_depth)`, the host re-syncs to the MCU's view and emits a `KALICO_FAULT_INTERNAL_INVARIANT` warning to telemetry.
- Provides the clock-sync sample on every frame (`mcu_clock_now` is captured at status-frame-emit instant; pairs with `host_recv_time` for the clock-sync regression — see §12.3 for the cadence policy).
- Acts as the keepalive heartbeat: if the host doesn't see a status frame for 3× the expected period, the MCU is declared `KALICO_FAULT_LIVENESS_STALLED` and disconnect-recovery triggers.
- Carries both `accepted_segment_id` (last enqueued) and `retired_through_segment_id` (last retired) so the host can independently observe queue-fill and queue-drain progress.

The status frame is **explicitly NOT primary flow control** — at 10 Hz it has 100 ms granularity, three orders of magnitude too slow for the sub-millisecond segment-retire cadence. It is a backstop and reconciliation source, parallel to the credit events.

### 5.4 Push semantics

Host segment-push path:

```
fn push_one(segment) -> Result<(), PushFault> {
    if credit[mcu] == 0 { return Err(NoCredit); }
    credit[mcu] -= 1;
    let response = wire.send(kalico_push_segment, segment).await?;
    match response.result {
        Ok => Ok(()),
        Err(reason) => {
            credit[mcu] += 1;  // rollback
            Err(reason.into())
        }
    }
}
```

The credit decrement is speculative — rolled back if the MCU rejects the push (e.g., because the segment is invalid). On accepted push, the credit stays decremented; the next `kalico_credit_freed` event re-credits.

`kalico_push_response` extended:
```
kalico_push_response result=%i accepted_segment_id=%u credit_epoch=%u
```

`accepted_segment_id` is the just-accepted segment's id (equal to the push command's `id` field on success); the host uses the periodic status frame's `accepted_segment_id` to detect if any push was silently lost (status field should equal the last successfully-pushed id). `credit_epoch` lets the host detect MCU resets.

---

## 6. Multi-MCU sync

Per brainstorm Q3-followup (settled): perfect sync requires three components, all designed-in even when only one MCU is wired up at the Step-6-validation pass.

### 6.1 Continuous clock-frequency estimation per MCU (§13)

Per-MCU sliding-window linear regression on (host_time, mcu_clock) pair samples. Anchors a host_time ↔ MCU_clock mapping for each MCU independently.

### 6.2 Per-MCU local-clock t_start/t_end domain

Per brainstorm Q9 (settled, option (i)): every segment's `t_start`, `t_end` are in the destination MCU's cycle-counter domain (u64 widened in Rust; widening logic from Step-5 `clock.rs` carries forward to the half-split refactor).

The host owns the wall-clock plan (a single Layer-2/3 trajectory description across all axes). At push time, the host converts wall-clock t → per-MCU cycles using each MCU's `clock_freq_estimate`. ISR compares `now_widened_cyccnt` against `seg.t_start` directly — no conversion math in the hot path.

Sync residual error is bounded by the clock-freq estimator's quality. On Klipper-precedented USB-CDC links, residual is sub-µs in steady state; Step 6 measures it and quotes a number (§7.3 measurement protocol). At 1000 mm/s, 100 µs residual = 100 µm of axis-relative position error — well inside print tolerance, but only because XY is on the same MCU (CoreXY) and Z's sync-tightness requirement is much looser. Step-6 spec asserts: "max admissible cross-MCU residual at commit is `MAX_RESIDUAL_US`, default pending measurement, conservative initial estimate 100 µs."

### 6.3 Atomic-start arm/commit handshake

Per kalico-verifier round 1 Claim 3 (corrected from Codex round 1): the genuinely-new failure mode that an arm-handshake catches (above and beyond Step-5's per-MCU push-acceptance) is the **commit-time clock-sync-quality check**. The handshake gates motion-arm on both per-MCU acceptance AND clock-sync quality.

State machine (host-side, multi-MCU):

```
IDLE → STREAM_OPENING (host: kalico_stream_open per MCU)
     → STREAM_OPEN_PRIMING (host pushes first N priming segments per MCU)
     → ARMING (host: kalico_stream_arm t_start_T0 per MCU)
       │
       ├─ all MCUs ack arm + clock-sync quality OK → ARMED
       └─ any MCU NACK / timeout / quality bad → ABORT (clean teardown via flush)

ARMED → RUNNING (when wall-clock now ≥ T0; each MCU autonomously transitions when its local mcu_clock_now ≥ t_start_T0)
```

Arm-acceptance criteria on the MCU side:
- Stream is in STREAM_OPEN_PRIMING.
- At least 1 priming segment is in the queue.
- The first segment's `t_start ≥ now_widened_cyccnt + MIN_ARM_LEAD_CYCLES` (default `MIN_ARM_LEAD_CYCLES = 0.1 sec × clock_freq` — gives both host and MCU time to settle).
- No latched fault.

Wire schema:
```
kalico_stream_arm t_start_t0_lo=%u t_start_t0_hi=%u arm_lead_cycles=%u
kalico_stream_arm_response result=%i armed_t_start_lo=%u armed_t_start_hi=%u
```

Quality gate (host-side, before issuing the arm command):
- For each MCU: `clock_freq_estimator.is_quality_gate_passed()` returns true.
- Cross-MCU sanity: `|fA / fB - 1| < 1e-3` (PPM-class crystals; >1e-3 means a measurement bug or hardware fault, not a real frequency difference).

If any MCU fails arm or quality gate, the host issues `kalico_stream_flush` to all MCUs, latches a `KALICO_FAULT_ARM_*` fault, and reports to the user.

### 6.4 Commit-time t_start_T0 selection

Host computes `T0_wall_clock = now_wall_clock + ARM_LEAD_TIME_MS` where `ARM_LEAD_TIME_MS` is large enough that:
- All MCUs receive their first priming segment + arm command before their respective `t_start_T0_local`.
- Clock-freq estimator sample age is below threshold for all MCUs.

Per-MCU: `t_start_T0_local[mcu] = host_to_mcu_clock[mcu](T0_wall_clock)`.

`ARM_LEAD_TIME_MS` default: 200 ms. Justification: 100 ms covers Klipper-class scheduling latency on the host, 100 ms covers segment-push round-trip + processing on each MCU. Final number derived from §7.3 measurement protocol.

**ARMING race protection (Round 1 fix):** the host MUST issue all per-MCU `kalico_stream_arm` commands and confirm all acks within `ARM_LEAD_TIME_MS / 2` (default 100 ms). If the host stalls partway through arming (e.g., MCU A is armed but MCU B's command hasn't been sent), the host aborts via flush before MCU A's `t_start_T0_local` arrives. Implementation: the host maintains an arming deadline `now_wall_clock + ARM_LEAD_TIME_MS / 2` and checks it before each `kalico_stream_arm` issuance; if the deadline has passed, abort. This prevents the "MCU A starts moving before MCU B is armed" failure mode the verifier flagged.

### 6.5 Inactive-MCU hold segments (Z idle stretches)

In a CoreXY+Z setup, the F4 (Z) MCU has long idle stretches during a print — each layer change requires one Z move; between layers, Z is stationary. The flow-control design (§5) under (α) requires `kalico_credit_freed` events to keep host credit alive, which requires the MCU to be retiring segments. Without explicit handling, the F4 stream would either: (a) underrun after the last Z move retires (FAULT), or (b) require the host to manually send dummy segments at print-time (gross).

**Hold-segment representation:** Add a `SegmentKind::Hold` variant to the segment payload (§4.2 v1 schema). A hold segment carries:
- `t_start, t_end` in MCU-local cycles (same as motion segments).
- `kinematics` field encodes "hold last position" (no curve_handle reference; CurveHandle = 0 sentinel).
- `flags` includes `HOLD_SEGMENT` bit.

ISR semantics for `Hold`:
- During its `[t_start, t_end]` window, ISR continues to fire at 40 kHz but emits no motor-position changes; the previous motor position is repeated each tick (or a no-op trace sample with `HOLD_SAMPLE` flag is emitted at lower cadence — TBD by trace bandwidth measurement).
- At `t_end`, hold retires normally; emits `kalico_credit_freed` and `SEGMENT_END` like a motion segment. Stream stays alive.

**Planner responsibility:** Layer 3 (or the host's stream coordinator) emits hold segments to the F4 to fill idle stretches. Granularity TBD: 100 ms hold segments would generate 10 Hz credit traffic on the F4 — exactly the periodic-status-frame cadence. Alternative: emit a single long hold segment (multi-second) and rely on the periodic status frame for liveness — saves wire chatter but increases the host-stall blast radius (the hold can't be safely shortened mid-stream without flush+rearm).

For Step-6 validation against H723-only sim, hold segments are designed-in but not actively exercised. F4x bring-up workstream validates the hold path against real Z motion patterns.

---

## 7. Buffer-budget framework

Per brainstorm Q7 (decision (p), settled after kalico-verifier round 2): Step 6 commits to **the framework** — parameter linkage and measurement protocol — not to specific numbers. Numbers fall out at implementation time after measurement.

### 7.1 Parameters and linkage

| Parameter | Source | Default (pending measurement) |
|---|---|---|
| `MIN_TICKS_PER_SEGMENT` | `max(4, ceil(WORST_ISR_CYCLES / CYCLES_PER_TICK))`; `WORST_ISR_CYCLES` from M2 measurement, `CYCLES_PER_TICK = clock_freq / 40000` | 4 (initial estimate; Step-5 cycle bench was ~5.5–7.3 µs / 25 µs tick → ratio &lt; 1, so floor of 4 binds) |
| `MIN_SEGMENT_DURATION_MS` | `MIN_TICKS_PER_SEGMENT × tick_period_ms` = `MIN_TICKS_PER_SEGMENT / 40` | 0.1 ms (4 ticks × 25 µs); pending M2 |
| `HOST_STALL_BUDGET_MS` | Measured p99.99 host-side tail latency on Pi 5 + Bookworm desktop + production load (M1) | 20 ms (initial estimate) |
| `Q_N_BUFFER_MS` | `max(HOST_STALL_BUDGET_MS, 4 × MIN_SEGMENT_DURATION_MS)` | derived |
| `Q_N` | Smallest power of 2 ≥ `ceil(Q_N_BUFFER_MS / MIN_SEGMENT_DURATION_MS) + 1` (heapless effective-cap = N-1 rule); subject to `Q_N ≤ CURVE_POOL_MAX = 65535` from §10.1 handle layout | derived; ≤ 65535 |
| `CURVE_POOL_N` | `Q_N` (worst case is one distinct curve per segment) | derived |
| `MAX_BOUNDARY_ITERS` | `Q_N - 1` (effective capacity); predicate `iters > MAX_BOUNDARY_ITERS` | derived |
| `MAX_RESIDUAL_US` | Measured clock-freq estimator p99.99 residual on production link (M3) | 100 µs (initial estimate) |
| `TRACE_RING_DURATION_MS` | `2 × HOST_STALL_BUDGET_MS` (safety margin against overflow under stall) | derived |
| `TRACE_RING_N` | `TRACE_RING_DURATION_MS × 40` (40 kHz sample rate) | derived |
| `TRACE_RING_LOCATION` | DTCM (§13.1 math: 128 KB DTCM has ~40 KB headroom at default sizing). AXI SRAM is contingency requiring MPU non-cacheable config — out of scope for Step 6 unless DTCM measurements force it. | DTCM |

### 7.2 Layer-3 minimum-segment-duration enforcement

Per kalico-verifier round 2 + CLAUDE.md print-throughput-non-negotiable:

- Layer 3 must NOT emit runtime segments below `MIN_SEGMENT_DURATION_MS`, except for explicit end/flush sentinel segments.
- If reparameterization × shaper convolution would produce sub-budget pieces, Layer 3 must constrain Layer-2 v(s) so the resulting runtime segments are ≥ `MIN_SEGMENT_DURATION_MS`. **Binding constraint, not coalescing.** The constraint derives from MCU hardware (40 kHz tick, sub-tick boundary loop overhead), not from "comms convenience."
- If Layer 3 cannot satisfy the constraint with current Layer-2 output, that's a hard planner error reported to the user. Not a silent slowdown.
- The Layer-3 spec (Step 8) inherits this constraint as part of its output contract. Step 6 documents it; Step 8 implements it.

### 7.3 Measurement protocol

Three measurement runs gate the parameter pinning. All run during Step-6 implementation, results checked into `docs/research/step6-buffer-budget-measurements.md`.

**M1 — Host-stall measurement.** 8h Pi-5 soak: Bookworm desktop, Wayfire compositor, Mainsail rendering trace UI, Moonraker WebSocket, journald `--persist`, simulated USB-CDC traffic at production rate (~1 kHz peak push), simulated Layer-1/2/3 planner running. Capture distribution of segment-push completion times. Report: p50, p95, p99, p99.9, p99.99, max. `HOST_STALL_BUDGET_MS` = max(p99.99, 5 ms) — the 5 ms floor is for sanity; we never go tighter than RT_PREEMPT-friendly.

**M2 — MCU runtime cost.** Re-run Step-5 cycle-budget bench (`tools/test_h723_cycle_count.py`) against Step-6's protocol-handler additions (clock-sync responder, stream-state machine, generation-handle lookup, seqlock publication). Report:
- `WORST_ISR_CYCLES` = max observed ISR cycle count over 1M ticks under stress (segment turnover, boundary loop, fault path).
- `CYCLES_PER_TICK = clock_freq / 40000` = cycles between consecutive 40 kHz fires.

Derivation (units consistent — both quantities in cycles):
```
MIN_TICKS_PER_SEGMENT = max(4, ceil(WORST_ISR_CYCLES / CYCLES_PER_TICK))
MIN_SEGMENT_DURATION_MS = MIN_TICKS_PER_SEGMENT × (1000 / 40000) = MIN_TICKS_PER_SEGMENT × 0.025
```

The floor of 4 ticks (= 0.1 ms) ensures the sub-tick boundary loop has at least 1 cycle of slack for retire+pop+resume on segment transitions. If `WORST_ISR_CYCLES > CYCLES_PER_TICK / 4`, the ISR budget is over-loaded — implementation must lighten before any segment can be reliably evaluated. (Round 1 review fix: previous formula `ceil(worst_isr_cycles / clock_freq) × MIN_TICKS_PER_SEGMENT × tick_period_us / 1000` had unit-confusion errors; this version stays in cycles until the final ms conversion.)

**M3 — Clock-sync residual.** 24h dual-MCU soak (H723 + F4x sim, since F4x hardware deferred). Measure clock-freq estimator residual distribution. Report: max residual p99.99, max drift PPM p99.99, sample-age distribution. `MAX_RESIDUAL_US` = max(p99.99, 10 µs) — the 10 µs floor is the timestamp-resolution sanity bar at 100 MHz.

### 7.4 Underrun semantics

Queue empty while stream is open (per §9 stream-open flag): `KALICO_FAULT_UNDERRUN`. Latched on MCU. Status frame and `kalico_fault` event emitted. Host aborts print.

Queue empty after explicit terminal segment retired: `Drained` status, normal end-of-stream.

This split fixes the Step-5 gap where `engine.rs:227-233` collapsed both into `Drained`. Per kalico-verifier round 2 Claim C: ~30 LOC change, low-risk, gated on `stream_open: AtomicBool`.

---

## 8. Stream lifecycle

### 8.1 Stream states (host's view)

```
DISCONNECTED  → fd not open / lost
HANDSHAKING   → identify in progress
IDLE          → identify done, no active stream
STREAM_OPENING → kalico_stream_open sent, awaiting ack
STREAM_OPEN_PRIMING → first N segments accepted, not yet armed
ARMING        → kalico_stream_arm sent, awaiting ack from all MCUs
ARMED         → arm acked, clock-sync quality OK, awaiting wall-T0
RUNNING       → motion active (now ≥ T0)
DRAINING      → terminal segment sent, awaiting drain ack
DRAINED       → end-of-stream, returned to IDLE
FAULT         → latched fault, manual reset required
```

### 8.2 MCU-side `stream_open: AtomicBool`

`Engine` (Step-5) extended with `stream_open: AtomicBool`. Set on `kalico_stream_open`. Cleared on terminal-segment retirement (success path) or `kalico_stream_flush` (abort path). Read by the boundary-loop drain branch:

```rust
let Some(next) = queue.try_pop() else {
    if self.stream_open.load(Ordering::Acquire) {
        self.fault(FaultCode::Underrun);
    } else {
        self.status.store(RuntimeStatus::Drained as u8, Ordering::Release);
    }
    return Ok(());
};
```

### 8.3 Wire commands

```
kalico_stream_open stream_id=%u
  → kalico_stream_open_response result=%i credit_epoch=%u

kalico_stream_arm t_start_t0_lo=%u t_start_t0_hi=%u arm_lead_cycles=%u
  → kalico_stream_arm_response result=%i armed_t_start_lo=%u armed_t_start_hi=%u

kalico_stream_terminal segment_id=%u
  → kalico_stream_terminal_response result=%i

kalico_stream_flush
  → kalico_stream_flush_response result=%i
```

### 8.4 Multi-MCU coordination

Host issues `kalico_stream_open` / `kalico_stream_arm` / etc. to all participating MCUs. Each per-MCU state machine advances independently; the host's aggregate state machine progresses only when all per-MCU states reach the next gate. Any per-MCU FAULT or NACK aborts the aggregate to FAULT and triggers `kalico_stream_flush` to clean up the surviving MCUs.

### 8.5 Flush atomicity and idempotency (Round 1 review fixes)

**Flush atomicity:** `kalico_stream_flush` atomically:
1. Clears `stream_open: AtomicBool` (so any in-flight boundary-loop drain branch returns Drained, not Underrun).
2. Drains the segment queue (pops all entries, no retire events emitted for flushed segments — they were never executed).
3. Resets `last_retired_gen` to current `current_gen` for all curve-pool slots (curves stay loaded but are immediately reusable).
4. Increments `credit_epoch` (so any pending credit events from before the flush are detectable as stale).
5. Returns `kalico_stream_flush_response result=0 credit_epoch=<new>`.

The atomicity is enforced by holding the foreground task lock during all five operations; the ISR observes `stream_open == false` first and treats the in-flight segment as the last one (retires normally on its t_end, no underrun).

**Command idempotency under msgproto retransmits:** msgproto's NAK+retransmit can deliver the same command payload twice if the host's first ACK was lost on the wire. The Step-6 protocol must handle this gracefully:

- `kalico_stream_open` with the same `stream_id` twice in a row: second response carries `result=KALICO_OK, credit_epoch=<unchanged>` (idempotent).
- `kalico_stream_arm` with the same `t_start_t0` twice: second response carries `result=KALICO_OK, armed_t_start=<unchanged>`.
- `kalico_stream_terminal` with the same `segment_id` twice: second response is `result=KALICO_OK`. If different `segment_id` arrives after the first, the second NACKs with `KALICO_FAULT_STREAM_STATE_VIOLATION`.
- `kalico_stream_flush`: always idempotent — flush after flush is a no-op except for incrementing `credit_epoch`.
- `kalico_push_segment` with the same `segment.id` twice: second response carries `result=KALICO_FAULT_SEGMENT_ID_NON_MONOTONIC` (the host should never re-push the same id; this is a host bug, not a transport quirk).

State-machine command violations (e.g., `kalico_stream_arm` while in IDLE without first opening): NACK with `KALICO_FAULT_STREAM_STATE_VIOLATION` in the response. This is reported as an FFI rejection code, NOT latched into the engine's FAULT state — the host gets the chance to recover by issuing the right command sequence.

---

## 9. Fault taxonomy

Step 5's `KalicoErr` enum (numeric codes in `error.rs`, exposed via `last_error: AtomicI32`) is extended with the comms-layer faults. Faults are reported via:

- `kalico_runtime_last_error()` — numeric code.
- `kalico_status` periodic frame — `last_fault: u16, fault_detail: u32`.
- `kalico_fault` async event on FAULT-state transition.

### 9.1 Fault codes (extension)

**Transport-layer:**
- `KALICO_FAULT_BAD_CRC` — msgproto retransmit-exhausted.
- `KALICO_FAULT_FRAMING_VIOLATION` — sequence/sync violation exhausted.
- `KALICO_FAULT_DISCONNECT` — fd lost / serial error.
- `KALICO_FAULT_PROTOCOL_VERSION_UNSUPPORTED` — blob format-version unknown.

**Clock-sync:**
- `KALICO_FAULT_CLOCK_SYNC_QUALITY` — residual / drift / sample-age over threshold.
- `KALICO_FAULT_CLOCK_SYNC_TIMEOUT` — no response to N consecutive sync pings.

**Multi-MCU coordination:**
- `KALICO_FAULT_ARM_TIMEOUT` — MCU didn't ack arm by deadline.
- `KALICO_FAULT_ARM_REJECTED` — MCU rejected arm (t_start too soon, latched fault, etc.).
- `KALICO_FAULT_CROSS_MCU_DESYNC` — one MCU underran while another is RUNNING.

**Buffer-budget:**
- `KALICO_FAULT_UNDERRUN` — queue empty while stream-open.
- `KALICO_FAULT_QUEUE_OVERRUN` — host pushed without credit and we accepted then ran out (defensive; should never fire if host obeys credit).
- `KALICO_FAULT_LIVENESS_STALLED` — `kalico_liveness_ok` cleared, foreground task not running.
- `KALICO_FAULT_TRACE_OVERFLOW` — TraceRing overwrote an undrained sample (host trace consumer too slow). Sized at 2× host-stall budget (§13.1) so this should never fire under stated assumptions; if it does, it indicates either a wrong host-stall budget or a host-side bug.

**Protocol/state machine (Round 1 fix):**
- `KALICO_FAULT_STREAM_STATE_VIOLATION` — stream-control command issued in wrong state (e.g., `arm` before `open`); reported as FFI rejection code, NOT latched into engine FAULT.
- `KALICO_FAULT_SEGMENT_ID_NON_MONOTONIC` — `kalico_push_segment` with `id` ≤ last accepted `id`; reported as FFI rejection code.

**Time-domain:**
- `KALICO_FAULT_T_START_IN_PAST` — push with `t_start ≤ now_widened_cyccnt`.
- `KALICO_FAULT_T_END_BEFORE_T_START` — invariant violation.
- `KALICO_FAULT_SEGMENT_TOO_SHORT` — duration below `MIN_SEGMENT_DURATION_MS`.
- `KALICO_FAULT_SEGMENT_TOO_LONG` — duration over a sanity ceiling (default 10s; planner shouldn't ever).

**Curve-pool:**
- `KALICO_FAULT_INVALID_CURVE_HANDLE` — slot not loaded, or generation mismatch.
- `KALICO_FAULT_CURVE_RELOAD_REJECTED` — live curve cannot be replaced (only released slots can).
- `KALICO_FAULT_CURVE_FORMAT_INVALID` — degree/n_cp/n_knots out of range.

**Runtime-numerical (extends Step-5):**
- `KALICO_FAULT_NAN_INF_OUTPUT` — eval produced NaN/Inf.
- `KALICO_FAULT_BOUNDARY_LOOP_OVERFLOW` — sub-tick boundary loop iterations exceeded MAX_BOUNDARY_ITERS.
- `KALICO_FAULT_INTERNAL_INVARIANT` — defensive catch-all for invariants violated by code.

### 9.2 Fault-detail field

The 32-bit `fault_detail` carries fault-code-specific context:
- `BAD_CRC`: number of consecutive failures.
- `CLOCK_SYNC_QUALITY`: encoded `(residual_us << 16 | drift_ppm)`.
- `INVALID_CURVE_HANDLE`: encoded `(slot_idx << 24 | observed_gen << 16 | expected_gen)`.
- `SEGMENT_TOO_SHORT`: actual duration in cycles.
- (etc; spec amendments can extend per fault code.)

---

## 10. Curve-pool generation handles

Per kalico-verifier round 2 Claim F (settled): Step-5's "no overwrite after load" policy fails at production scale (10K–200K segments per print). Step 6 ships generation-counter discipline. Round-3 review found two implementation-blocking bugs in the original §10.2/§10.3 — predicate inconsistency and wrap-cooldown deadlock; both fixed in this revision.

### 10.1 Handle layout

`CurveHandle` is **u32 on the wire** (not a packed bit-field), explicit `slot_idx: u16, generation: u16` so the framework's measurement-driven `CURVE_POOL_N` is not bottlenecked by handle bit-width:

```rust
#[repr(C)]
pub struct CurveHandle {
    pub slot_idx: u16,    // 0..CURVE_POOL_N (CURVE_POOL_N ≤ 65535; framework can scale)
    pub generation: u16,  // wraps modulo 65536
}
```

Wire size: 4 bytes per handle (vs prior 2 bytes packed). Cost is amortized across the segment-push payload (typically 60+ bytes) — &lt;7% increase, well worth removing the framework-vs-bit-width contention.

At `CURVE_POOL_N = 64` and 65536 generations, that's 64 × 65536 ≈ 4M curve allocations between full-cycle wraps. Worst-case wrap per print: 200K segments / 65536 gens ≈ 4 wraps over the longest reasonable print — handled by the wrap policy below.

### 10.2 Allocation predicate (consistent)

Each pool slot maintains two atomic generation counters:

```rust
struct PoolSlot {
    current_gen: AtomicU16,        // last gen issued by alloc
    last_retired_gen: AtomicU16,   // last gen confirmed retired (foreground sets)
    curve: UnsafeCell<LoadedCurve>,
}
```

**Allocation predicate:** `try_alloc(slot=N)` succeeds **iff** `current_gen == last_retired_gen` (modulo u16 — both wrap together). On success: load curve into slot, then `current_gen.store(current_gen.wrapping_add(1), Release)`. The returned handle is `(N, new_current_gen)`.

Initial state at runtime_init: `current_gen = 0, last_retired_gen = 0`. Predicate satisfied → first alloc OK.

After alloc: `current_gen = 1, last_retired_gen = 0`. Predicate fails → no further alloc on slot N until retire.

After SEGMENT_END for handle (N, gen=1): foreground confirms no queued segment references slot=N or prior, then `last_retired_gen.store(1, Release)`. Predicate satisfied → next alloc OK, advances `current_gen` to 2.

### 10.3 Wrap policy (deadlock-free)

The predicate `current_gen == last_retired_gen` is **modulo u16 wrap**: when both have advanced through 65535 → 0, the equality holds again naturally. There is no separate wrap-cooldown state machine, no special-case rule. The previous draft's "rejected for one cycle" mechanism with no defined wake-up was the deadlock; removing it removes the bug.

ABA defense: a stale handle `(N, gen=g)` cannot match a freshly-allocated `(N, gen=g)` because for that to happen, the slot's `current_gen` must have wrapped through all 65535 intermediate values. At 4M allocations between wraps and a maximum host buffer of `Q_N` outstanding segments per slot (typically ≤64), a stale handle would have to survive `(65536 - Q_N)` allocations on the same slot — physically impossible because the host releases handles via the trace pipeline, not by holding them indefinitely. If a host bug causes a stale handle to leak, the worst-case ABA window is one print's duration; mitigation is the §10.4 ISR-side validation which still catches mismatches.

### 10.4 Reclaim mechanism (with TraceRing-overflow backstop)

Primary path:
- ISR emits a `SEGMENT_END` trace sample at retirement, including the segment's `curve_handle = (slot, gen)`.
- Foreground drains trace; on `SEGMENT_END(slot=N, gen=G)`, checks: any queued segment references `(slot=N, gen=G or earlier)`? If no, `slot.last_retired_gen.store(G, Release)`.
- Producer: `try_alloc(slot=N)` succeeds per §10.2 predicate.

Backstop path (covers Round 1 review concern: TraceRing overflow can drop SEGMENT_END events under sustained host stall):
- TraceRing is dimensioned per §13.1 to **2× HOST_STALL_BUDGET_MS** (with safety margin) so overflow is operationally rare.
- If the ISR detects that it has had to drop a sample with `SEGMENT_END` flag, it latches `KALICO_FAULT_TRACE_OVERFLOW`. This is a hard fault — the print aborts. The host learns of the lost reclaim event via fault rather than silent slot starvation.
- Foreground also runs a periodic reclaim sweep (~1 Hz, on the same cadence as the periodic status frame): for each pool slot N where `current_gen != last_retired_gen` and the periodic status reports `retired_through_segment_id ≥ all queued segments referencing slot N`, foreground forces `last_retired_gen.store(current_gen, Release)`. This is a soft backstop that recovers if SEGMENT_END events were dropped *and* the trace-overflow fault didn't fire (e.g., a different fault preempted, or the overflow was a near-miss). On forced reclaim, foreground emits a warning event to telemetry.

### 10.5 ISR validation on every segment evaluation

```rust
fn lookup(handle: CurveHandle) -> Result<&LoadedCurve, FaultCode> {
    if (handle.slot_idx as usize) >= CURVE_POOL_N {
        return Err(FaultCode::InvalidCurveHandle);
    }
    let slot = &self.slots[handle.slot_idx as usize];
    if slot.current_gen.load(Ordering::Acquire) != handle.generation {
        return Err(FaultCode::InvalidCurveHandle);
    }
    // SAFETY: current_gen matches; the slot's curve is the one this handle refers to.
    // current_gen is bumped only after the new curve is fully loaded (§10.2 ordering).
    Ok(unsafe { &*slot.curve.get() })
}
```

Defends against use-after-reclaim and bit-flips.

### 10.6 Cost summary

- Per slot: `AtomicU16 × 2 + UnsafeCell<LoadedCurve>`. The atomics add 4 bytes/slot; total at 64 slots: 256 bytes (negligible).
- Per allocation: one ordered store on `current_gen` + curve memcpy.
- Per ISR lookup: one Acquire load on `current_gen` + slot-bounds check.

---

## 11. FFI aliasing UB → half-split SPSC

Per Step-5 acknowledged latent UB (`*mut KalicoRuntime → &mut RuntimeContext` produces overlapping `&mut` under Rust's strict aliasing model when the ISR preempts foreground): Step 6 closes this before live producer surfaces. Round 1 review (both Codex and kalico-verifier) confirmed that the original §11.2 pattern (two concurrent `&mut *rt` followed by field projection) is **still UB under stacked borrows** — this revision uses raw-pointer projection that never materializes a `&mut RuntimeContext`.

### 11.1 RuntimeContext split

```rust
pub struct RuntimeContext {
    fg: UnsafeCell<FgState>,
    isr: UnsafeCell<IsrState>,
    shared: SharedState,
}

pub struct FgState {
    queue_producer: SpscProducer<Segment, Q_N>,
    curve_pool_producer: CurvePoolProducer,
    trace_consumer: TraceRingConsumer,
    status_consumer: StatusFrameConsumer,
    stream_state: AtomicU8,  // host-orchestrated lifecycle
}

pub struct IsrState {
    queue_consumer: SpscConsumer<Segment, Q_N>,
    curve_pool_consumer: CurvePoolConsumer,
    trace_producer: TraceRingProducer,
    status_producer: StatusFrameProducer,
    engine: Engine,
    widen_state: ClockWidenState,  // ISR-private; foreground reads via §11.4 seqlock
}

pub struct SharedState {
    last_error: AtomicI32,
    runtime_status: AtomicU8,
    stream_open: AtomicBool,
    // §11.4 widened-clock seqlock fields (foreground reads, ISR writes)
    widened_now_lo: AtomicU32,
    widened_now_hi: AtomicU32,
    widened_now_seq: AtomicU32,
    // ... other cross-half atomics
}
```

`FgState` and `IsrState` are wrapped in `UnsafeCell` because each is mutated through a raw pointer without ever forming a `&mut RuntimeContext`. `Sync` is implemented on `RuntimeContext` (manual `unsafe impl`) with the discipline contract that `FgState` is touched only from foreground and `IsrState` only from the ISR.

### 11.2 FFI surface evolution (raw-pointer projection)

The init function returns a `*mut KalicoRuntime`, but every subsequent FFI call projects directly to the relevant half via `core::ptr::addr_of_mut!` without creating a `&mut RuntimeContext`. The opaque handle never resolves to `&mut` of the parent.

```rust
#[no_mangle]
pub extern "C" fn kalico_runtime_init() -> *mut KalicoRuntime {
    static mut RT: MaybeUninit<RuntimeContext> = MaybeUninit::uninit();
    static INIT_DONE: AtomicBool = AtomicBool::new(false);
    if INIT_DONE.compare_exchange(false, true, Acquire, Relaxed).is_err() {
        return core::ptr::null_mut();
    }
    // SAFETY: single-threaded init, before any other FFI.
    unsafe {
        let rt_ptr = RT.as_mut_ptr();
        rt_ptr.write(RuntimeContext::new(...));
        rt_ptr as *mut KalicoRuntime
    }
}

#[no_mangle]
pub extern "C" fn kalico_runtime_tick(rt: *mut KalicoRuntime, raw_cyccnt: u32) {
    // SAFETY: ISR is single-threaded entry; shared is read-only from here (atomics).
    // We never form &mut RuntimeContext.
    let ctx = rt as *mut RuntimeContext;
    let isr_ptr: *mut IsrState = unsafe {
        UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr))
    };
    let shared_ptr: *const SharedState = unsafe {
        core::ptr::addr_of!((*ctx).shared)
    };
    // SAFETY: ISR has unique access to *isr_ptr per the discipline contract.
    let isr: &mut IsrState = unsafe { &mut *isr_ptr };
    // SAFETY: SharedState is Sync; shared via &.
    let shared: &SharedState = unsafe { &*shared_ptr };
    isr.engine.tick(raw_cyccnt, shared);
}

#[no_mangle]
pub extern "C" fn kalico_runtime_push_segment(rt: *mut KalicoRuntime, /* ... */) -> i32 {
    // Symmetric: foreground re-borrows FgState only, never RuntimeContext.
    let ctx = rt as *mut RuntimeContext;
    let fg_ptr: *mut FgState = unsafe {
        UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).fg))
    };
    let shared_ptr: *const SharedState = unsafe { core::ptr::addr_of!((*ctx).shared) };
    let fg: &mut FgState = unsafe { &mut *fg_ptr };
    let shared: &SharedState = unsafe { &*shared_ptr };
    fg.push_segment(/* ... */, shared)
}
```

**Why this is sound under stacked borrows / tree borrows:**

- We never form `&mut RuntimeContext`. The parent struct is accessed only via raw pointers (`*mut RuntimeContext`).
- `core::ptr::addr_of!((*ctx).isr)` produces a raw pointer to the `isr` field without re-borrowing the parent (per Rust 1.51+ semantics; this is exactly the pattern documented in the Rustonomicon "Splitting Borrows" section).
- `UnsafeCell::raw_get` on the field's raw pointer yields `*mut IsrState` without any intermediate reference.
- Each FFI call materializes `&mut IsrState` (or `&mut FgState`) once at its entry; the discipline contract (`fg` ↔ foreground, `isr` ↔ ISR) ensures no concurrent overlapping `&mut`.
- `SharedState` is accessed only via `&` (immutable reference); mutation is via atomics, sound under aliasing.

The TIM5-disable-around-push idiom (currently used to safely access `widen_state` from foreground) goes away: `widen_state` becomes ISR-private and is touched only from the ISR; foreground reads the widened clock via §11.4's seqlock. Round 1 surfaced concern #4 was that the disable idiom needed auditing, not just the strict-aliasing fix; this revision retires the idiom completely.

### 11.3 Loom test coverage

Per Step-5 plan-changes-log open follow-up: "Loom test coverage expansion (gated to Step 6 when live producer surfaces)." Step 6 ships loom tests on:
- The half-split SPSC channels (queue, trace ring).
- The §11.4 widened-clock seqlock.
- The §10.2 generation-counter allocation predicate.
- The stream-lifecycle state machine (§8) under concurrent push/retire.

Test surface lives under `rust/runtime/tests/loom_*.rs` (host-target, `--cfg loom` build).

### 11.4 Widened-clock publication (seqlock for u64 over AtomicU32)

ARMv7-M does not provide lock-free 64-bit atomics. The ISR maintains a 64-bit widened CYCCNT in `IsrState.widen_state`; the foreground command-dispatch task reads it for the clock-sync responder (§12.1) and the periodic status frame (§5.3). The exchange uses a **standard seqlock pattern over two AtomicU32 + a sequence counter**:

```rust
// In SharedState:
//   widened_now_lo: AtomicU32,
//   widened_now_hi: AtomicU32,
//   widened_now_seq: AtomicU32,    // even = stable; odd = write in progress

// ISR writer (called from kalico_runtime_tick after widening):
fn publish_widened_now(shared: &SharedState, now: u64) {
    let seq = shared.widened_now_seq.load(Relaxed).wrapping_add(1);  // → odd
    shared.widened_now_seq.store(seq, Release);
    shared.widened_now_lo.store(now as u32, Release);
    shared.widened_now_hi.store((now >> 32) as u32, Release);
    shared.widened_now_seq.store(seq.wrapping_add(1), Release);     // → even
}

// Foreground reader (called from clock-sync responder, status-frame builder):
fn read_widened_now(shared: &SharedState) -> u64 {
    loop {
        let seq_before = shared.widened_now_seq.load(Acquire);
        if seq_before & 1 != 0 {
            // Write in progress; spin a few cycles and retry.
            core::hint::spin_loop();
            continue;
        }
        let lo = shared.widened_now_lo.load(Acquire) as u64;
        let hi = shared.widened_now_hi.load(Acquire) as u64;
        let seq_after = shared.widened_now_seq.load(Acquire);
        if seq_after == seq_before {
            return (hi << 32) | lo;
        }
        // Concurrent write; retry.
    }
}
```

Properties:
- Wait-free for the writer (ISR), bounded-retry for the reader (foreground).
- The seqlock is exactly Linux kernel's `seqcount_t`; Rust port via `AtomicU32` is straightforward.
- Foreground worst case: 2-3 retries on a contention burst; in practice 0 retries because the ISR's publish window is ~3 instructions and foreground reads happen at coarse cadence (status-frame ~10 Hz, clock-sync responder per request).
- AtomicU64 lock-free constraint sidestepped: only AtomicU32 used.
- Loom test (§11.3) gates correctness.

The seqlock is **not** used for any other ISR→foreground data; it is specifically for the widened clock. Other state is communicated via single-AtomicU32-or-smaller fields in `SharedState`, which are atomic on their own.

---

## 12. Clock-sync algorithm + quality gate

Per brainstorm Q9 (settled): per-MCU clock-frequency estimation via Klipper-style sliding-window linear regression. ARM-side u64 widening already in Step-5 `clock.rs`.

### 12.1 Wire exchange

```
kalico_clock_sync_request request_id=%u host_send_time_lo=%u host_send_time_hi=%u
  → kalico_clock_sync_response request_id=%u mcu_clock_lo=%u mcu_clock_hi=%u
```

The `request_id` lets the host correlate the response to a specific request when multiple are in flight. `mcu_clock` is captured at response-handler entry on the MCU side (read of widened CYCCNT inside the foreground command-dispatch task; widening is ISR-side only, foreground reads via SharedState atomics).

### 12.2 Estimator state (per MCU)

```rust
struct ClockSyncEstimator {
    samples: ArrayDeque<Sample, WINDOW>,  // sliding-window samples
    clock_freq_estimate: f64,             // ticks/sec
    anchor_host_time: f64,
    anchor_mcu_clock: u64,
    residual_max_in_window: f64,
    last_sample_time: Instant,
    sample_count: u32,
}

struct Sample {
    host_time_at_send: f64,
    mcu_clock_at_send: u64,  // back-calculated from response: mcu_at_response - rtt/2 × freq
    rtt_us: u32,
}
```

Linear regression: `mcu_clock = clock_freq × host_time + offset`. Update on every sample; recompute residual_max as the max abs(observed - predicted) over the current window.

### 12.3 Sample cadence

The periodic `kalico_status` frame at 10 Hz carries `mcu_clock_now` on every frame; **every status frame contributes a clock-sync sample to the regression**. This resolves the §5.3-vs-§12.3 cadence inconsistency Round 1 flagged: previously §5.3 said clock-sync rides every status frame and §12.3 said only 1-of-10 frames. The simpler design (every frame) reduces convergence time at near-zero cost (regression cost is one row update per sample; with WINDOW = ~30 the per-sample CPU cost is negligible).

- Warmup: status frame cadence is bumped to 50 Hz (every 20 ms) until `sample_count ≥ MIN_WARMUP_SAMPLES = 30` per MCU. Drops back to 10 Hz once warmup is complete.
- Steady-state: 10 Hz status frames; 10 samples/sec/MCU into the regression.
- High-residual mode: if `residual_max_in_window > 0.5 × MAX_RESIDUAL_US`, status cadence bumps to 50 Hz with hysteresis (Round 1 nice-to-have): only drops back to 10 Hz once `residual_max_in_window < 0.25 × MAX_RESIDUAL_US` for 5 consecutive samples. Prevents oscillation between modes on a transient noise spike.

Dedicated `kalico_clock_sync_request/response` (§12.1) is reserved for arm-time quality-gate validation (host needs a fresh sample with known RTT for the commit-time check) and post-fault-recovery resync. Steady-state regression is fed entirely by the status-frame piggyback.

### 12.4 Quality gate

Motion is armable when, for each MCU:
- `sample_count ≥ MIN_WARMUP_SAMPLES` (default 30).
- `last_sample_age_ms ≤ MAX_SAMPLE_AGE_MS` (default 2000ms steady, 100ms during warmup).
- `residual_max_in_window ≤ MAX_RESIDUAL_US` (default 100µs pending §7.3 measurement).
- `|drift_ppm| ≤ MAX_DRIFT_PPM` (default 100ppm pending measurement; >100ppm means the crystal is failing).

For multi-MCU: cross-MCU drift sanity `|fA / fB - 1| < 1e-3`; failure → `KALICO_FAULT_CROSS_MCU_DESYNC` at arm time.

---

## 13. Telemetry transport (TraceRing reconciliation, DTCM-resident)

Per kalico-verifier round 2 surfaced concern #1: Step-5 silently halved TraceRing from 1024 → 128 (3.2 ms headroom at 40 kHz), but spec text §4.5 still claims 25 ms. At Step-6's host-stall budget, 3.2 ms trace-ring overflow happens before the queue is exhausted — trace ring is part of the host-stall surface.

Round 1 review (kalico-verifier) found: (a) the previous draft's AXI SRAM relocation introduces a cache-coherency hazard on H7 because Klipper enables D-cache and AXI SRAM is write-back cached by default, requiring MPU configuration or explicit cache maintenance the spec didn't address; (b) the math claim that "doesn't fit DTCM" was wrong — H7 has 128 KB DTCM, Klipper's existing footprint is ~20 KB, kalico static state at default sizing is ~40 KB, total ~60 KB with ~68 KB slack. **Step 6 keeps TraceRing in DTCM.** AXI SRAM remains an option for *future* sizing pressure but with proper cache-handling design, not Step 6's problem.

### 13.1 Sizing rule

`TRACE_RING_DURATION_MS = 2 × HOST_STALL_BUDGET_MS` (overflow safety margin). Example at default 20 ms host-stall: `TRACE_RING_DURATION_MS = 40 ms`, `TRACE_RING_N = 1600` slots × 32 B = 51.2 KB. Total kalico DTCM footprint at default sizing:

| Component | Size |
|---|---|
| `TraceRing` (1600 × 32 B) | 51.2 KB |
| `Queue<Segment, Q_N=64>` (64 × ~32 B) | 2 KB |
| `CurvePool<CURVE_POOL_N=64>` (64 × ~184 B) | 11.5 KB |
| `widen_state`, `engine`, `stream_state`, atomics | ~1 KB |
| Stack | ~2 KB |
| **Subtotal kalico** | **~68 KB** |
| Klipper baseline (.bss/.data) | ~20 KB |
| **Total** | **~88 KB** |

H723 DTCM = 128 KB; ~40 KB headroom remains. If implementation-time measurements push the total over 100 KB, revisit; AXI SRAM relocation is the contingency, gated on adding MPU configuration to mark the region non-cacheable (Strongly-Ordered or Device memory) so producer/consumer cache lines stay coherent without software flush.

**Overflow handling:** If the ISR observes that the TraceRing write would overwrite an unread sample (i.e., consumer hasn't drained fast enough), it sets a `SAMPLE_DROP_PENDING` sticky flag in `SharedState`. Foreground observes this on its drain pass and emits `KALICO_FAULT_TRACE_OVERFLOW` (latched). This is a hard fault: the print aborts. The 2× safety margin makes overflow physically impossible under the stated host-stall budget; if it fires, it indicates either the host-stall budget was set wrong (host stalled longer than measured) or a consumer bug.

### 13.2 Trace schema (`repr(C)`, naturally aligned)

`TraceSample` (32 bytes, naturally aligned — no `repr(C, packed)` to avoid unaligned u64 access cost on Cortex-M7 and the aliasing-correctness traps that come with `addr_of!` on packed fields):

```rust
#[repr(C)]
struct TraceSample {
    tick: u64,                  // widened CYCCNT at sample time (8 B, 8-aligned)
    motor_a: f32,               // 4 B
    motor_b: f32,               // 4 B
    motor_e: f32,               // 4 B
    segment_id: u32,            // 4 B
    curve_handle: CurveHandle,  // 4 B (§10.1 — u16 slot + u16 gen)
    flags: u8,                  // SEGMENT_END, SEGMENT_START, FAULT, ...
    _pad: [u8; 3],              // explicit padding to 32 B
}
const _: () = assert!(core::mem::size_of::<TraceSample>() == 32);
```

Per §10.4, `SEGMENT_END` flag drives generation-handle reclaim; foreground observes `SEGMENT_END` events to update `last_retired_gen`. If a `SEGMENT_END` sample is dropped via overflow, the §10.4 backstop (periodic-status-driven reclaim sweep + `KALICO_FAULT_TRACE_OVERFLOW`) closes the gap.

---

## 14. Step-5 carryover items resolved

| Step-5 item | Step-6 resolution |
|---|---|
| MAX_BOUNDARY_ITERS / Q_N off-by-one | §7.1 `MAX_BOUNDARY_ITERS = Q_N - 1` derivation; test-only injection path required for fault branch reachability. |
| FFI aliasing UB | §11 half-split SPSC refactor. |
| Loom test coverage expansion | §11.3 — loom tests on half-split SPSC, cross-half atomics, stream-lifecycle state machine. |
| Production-context Engine constructor | Provided by half-split refactor: `Engine::new_production(...) -> (FgEngineHalf, IsrEngineHalf)`. |
| F4x integration test | Deferred to parallel workstream; uses Step-6 protocol but not its design problem. |
| Status-LED pin verification | Bring-up gate, not Step-6 spec scope. |
| Klipper full-LTO link CI | Step 7 MVP scope. |
| Cycle-budget actual numbers | §7.3 M2 measurement protocol re-runs Step-5 cycle-budget bench against Step-6 additions. |
| `Engine::Default` `#[cfg(test)]`-only | Production-context constructor (above) supersedes. |
| TraceRing 128/1024 mismatch | §13.1 — reconciled against host-stall budget; sized at 2× host-stall, kept in DTCM (Round 3 dropped AXI SRAM relocation per cache-coherency review). |
| Bench-loop manual IWDG kicking | §9 fault taxonomy includes `KALICO_FAULT_LIVENESS_STALLED`; comms-protocol benches that need to bypass liveness must be guarded by `#[cfg(test)]` and emit a warning trace event on entry. Permanent benches in production firmware are forbidden. |

---

## 15. Open follow-ups deferred

- **Host/klippy IPC boundary for non-motion subsystems** — Step 7 MVP. How a kalico-host-rt motion fault propagates to a Klippy supervisor that owns thermals/fans/probing.
- **F4x integration test + multi-MCU bring-up validation against the protocol** — parallel workstream; Step-6 protocol designed-in for it but bring-up is not Step-6 work.
- **Cross-MCU desync detection beyond simple "both arm + stay armed"** — Step 7 MVP onwards. (E.g., during RUNNING, periodic cross-MCU mcu_clock comparison; tolerance check.)
- **Skip-detection event channel architecture** — Step 11 (uses telemetry transport defined here).
- **Mid-print parameter adjustment** (live shaper retune, live PA tweak) — post-MVP.
- **`kalico_runtime_reset()` FFI for in-place fault recovery** — post-MVP.
- **UART-vs-USB-CDC framing differences** — wire-agnostic for now; Klipper's UART framing layer handles RX overrun under stress, baud rate mismatch, etc.
- **Layer-3 minimum-segment-duration enforcement implementation** — Layer-3 is Step 8; Step 6 documents the constraint as Step-3-output contract; Step-8 implements it.

---

## 16. Adversarial review history

### Round 1: Codex broad architecture review

Codex (`codex:codex-rescue`) reviewed the brainstorm direction at the post-Q6 point. Seven issues raised:

1. **Credit messages must be sequenced/epoched** — kalico-verifier confirmed *partially*; msgproto already provides sequencing/dedup at the transport layer; the genuine load-bearing requirement is `accepted_through_segment_id` per event + `credit_epoch` at session-establishment only. Codex's full field list over-specified.
2. **Buffer budget must be specified in milliseconds, not segments** — verifier confirmed *correct*. Drove the §7 framework.
3. **Atomic-start needs explicit arm/commit, not just push-with-future-t_start** — verifier confirmed *partially*; commit-time clock-sync-quality check is the genuinely-new failure mode. Push-time-acceptance was already covered by Step-5's producer protocol.
4. **Data-plane must be kalico-native from day one** — verifier confirmed *overstated*. The actual fix is a 1-byte version field on the segment struct + msgproto-as-framing-adapter assertion. Adopted as §4.2.
5. **Fault taxonomy must be enumerated** — verifier confirmed *correct*; Codex's list was incomplete (missing time-domain, curve-pool, liveness). Adopted as §9 with verifier's additions.
6. **Sim fix is one-day timebox, not half-day** — verifier confirmed *correct*. Adopted as §3.
7. **Eight unasked questions** — verifier triaged: six blocking, one already-filed-Step-5-residual, one (host/klippy IPC) deferable to Step 7 MVP.

### Round 2: Codex buffer-budget proposal review

Codex reviewed the originally-proposed buffer-budget commitment (`Q_N=64`, 20 ms host-stall, etc.). Seven pushbacks:

- A: **20 ms p99.9 stock Pi OS** — verifier confirmed correct. 20 ms is engineering judgment, not measurement. Drove §7.3 measurement protocol M1.
- B: **0.5 ms post-shaper minimum is under-justified** — verifier confirmed *partially*; structural claim correct, exact 0.02 ms arithmetic required slicer pathology. Drove §7.2 Layer-3 enforcement rule.
- C: **Drained vs Fault** — verifier confirmed *partially*; current Step-5 code is fine for Step 5 (no live producer); for Step 6 the queue-empty branch must split. Drove §8.2 `stream_open` flag + boundary-loop branch.
- D: **CurvePool slot ~180 B not 128 B** — verifier verified slot = 184 B. The proposal's 12 KB total was right by accident (two errors cancelled). `static_assert` on `size_of::<LoadedCurve>()` added as hygiene.
- E: **MAX_BOUNDARY_ITERS aligned to Q_CAP=Q_N-1** — verifier confirmed correct. Adopted in §7.1.
- F: **No-overwrite curve-pool fails at Q_N=64** — verifier confirmed correct. Generation-counter discipline in §10.
- G: **Eight missing items** — six blocking (§3.1, §7.2, §8.2, §11, §13, §7.4 derivation clarity), one should-do (§7.3 measurement), Codex Claim D variant.

### Verifier-surfaced concerns Codex missed (round 2)

- **TraceRing 128 vs 1024 spec mismatch** — adopted as §13.1.
- **MIN_SEGMENT_CYCLES = 50 µs is the only enforced floor** — adopted as §7.2 Layer-3 enforcement.
- **AtomicU64 not lock-free on ARMv7-M** — noted in §10 (handle widened to u32 in Round 3), §11.4 (seqlock for cross-half u64 publication).
- **TIM5-disable-around-push idiom audit** — adopted as §11.2 (idiom retired with half-split).
- **CLAUDE.md throughput-non-negotiable binds Layer-3 min-duration derivation** — adopted as §7.2.

### Round 3: Codex + kalico-verifier parallel review of round-2 spec

After committing the round-2 spec, both reviewers ran in parallel against the as-written document.

**Both reviewers converged on:**
- **§11.2 FFI half-split is still UB.** Two concurrent `&mut *rt` re-borrows are UB under stacked-borrows / tree-borrows even when re-projected to disjoint fields. Fix: rewrite §11.2 to use raw-pointer projection via `core::ptr::addr_of!` + `UnsafeCell::raw_get`, never materialize `&mut RuntimeContext`. Adopted in Round 3 §11.2.
- **TraceRing overflow → SEGMENT_END loss → reclaim deadlock.** Fix: size TraceRing at 2× host-stall (§13.1), latch `KALICO_FAULT_TRACE_OVERFLOW` on overflow, add periodic-status-driven reclaim sweep as backstop (§10.4).

**Codex (round 3) unique blocking findings:**
- **Flow-control field semantics contradictory** (`accepted_through_segment_id` reused for two different counters in §5.1 and §5.4). Fix: split into `retired_through_segment_id` (credit-freed event) and `accepted_segment_id` (push-response). Adopted in §5.1, §5.3, §5.4.
- **Multi-MCU sync doesn't handle inactive MCUs.** F4 has long Z-idle stretches. Fix: hold-segment representation in §6.5.
- **Phase 0 build-system work understated.** Fix: acknowledge Cargo-feature wiring to Makefile.kalico (§3.3 build-system note).
- **M2 formula has unit confusion.** Fix: rewrite in cycles until final ms conversion (§7.3).

**Verifier (round 3) unique blocking findings:**
- **§10 curve-pool predicate inconsistency** (§10.2 said "alloc when `last_reclaimed_gen != next_gen`", §10.3 said "alloc when `last_reclaimed_gen == current_gen`"; contradictory). Plus wrap-cooldown deadlock (no defined elapse mechanism). Fix: rewrite §10.2 with one consistent predicate (`current_gen == last_retired_gen` modulo u16 wrap), remove wrap-cooldown machinery (§10.3 wrap policy is now natural-modulo-wrap). Handle widened to u32 (slot+gen each u16) so framework's measurement-driven `CURVE_POOL_N` isn't bottlenecked by handle bit-width (§10.1).
- **§13.2 AXI SRAM relocation introduces cache-coherency hazard** unaddressed (Klipper enables D-cache; AXI SRAM is write-back cached by default; producer/consumer SPSC across cache lines requires MPU non-cacheable config or software cache maintenance). Plus the "doesn't fit DTCM" math was wrong — verifier's accounting showed ~40 KB headroom in 128 KB DTCM at default sizing. Fix: delete §13.2; keep TraceRing in DTCM (§13).
- **§7.1 CURVE_POOL_N=64 hardcoded ceiling vs framework.** If measurements push Q_N>64, the §10.1 handle bit-width breaks. Fix: widen handle to u32 (§10.1), update §7.1 table to allow Q_N up to 65535.
- **§3.3 Phase 0 acceptance gate items 5–6 require Step-6 features** (mis-categorized). Fix: split into Gate A (Phase 0 sim-fixes, 1 day) and Gate B (Step-6 feature validation against sim, after §7/§8/§9 implementation).
- **§11 widen-clock foreground access mechanism unspecified.** AtomicU64 forbidden on ARMv7-M. Fix: explicit seqlock pattern over two AtomicU32 + sequence counter (§11.4).
- **Multiple smaller fixes**: §5.3↔§12.3 piggyback cadence reconciliation (every status frame contributes a sample, with hysteresis on high-residual mode), §8.5 flush atomicity + command idempotency under msgproto retransmits, additional fault codes (`STREAM_STATE_VIOLATION`, `SEGMENT_ID_NON_MONOTONIC`, `TRACE_OVERFLOW`), §6.4 ARMING race protection (host commits to all-MCU arming within ARM_LEAD_TIME_MS / 2), TraceSample `repr(C)` aligned (not packed; avoids unaligned u64 access), §10.1 handle bit wording made explicit.

Net round 3: spec converged on raw-pointer FFI projection (closes the actual UB), corrected curve-pool generation predicate (deadlock-free), DTCM-only TraceRing with overflow fault, framework-flexible curve-handle layout, and explicit-seqlock widened-clock publication. Concrete numbers still fall out at Step-6 implementation time after §7.3 measurements run.

---

## 17. Summary of decisions

- **Scope:** full Layer-5 architectural shape, MVP feature subset (B). F4x bring-up parallel workstream.
- **Phase 0:** simulator fixes are Step-6's first work item; 1-day timebox; software CYCCNT C-side; load_curve fixed-fixture escape hatch if Renode platform-model hole.
- **Wire framing:** msgproto extended (control plane) + 1-byte-versioned kalico-native blobs (data plane); msgproto is a framing adapter, not a semantic dependency.
- **Flow control:** α — credit-based, MCU-authoritative, edge-triggered. Periodic status frame at 10 Hz as backstop and clock-sync sample piggyback.
- **Multi-MCU sync:** clock-freq estimation per MCU + per-MCU local-clock t_start/t_end + arm/commit handshake gating motion on commit-time clock-sync quality.
- **Buffer budget:** framework + measurement protocol; concrete numbers derived from M1/M2/M3 at implementation time; defaults pending.
- **Stream lifecycle:** open/arm/run/terminal/drain/flush + FAULT branch on underrun-while-stream-open.
- **Fault taxonomy:** transport, clock-sync, multi-MCU, buffer, time-domain, curve-pool, runtime-numerical — enumerated.
- **Curve-pool:** generation-counter handles `(slot, gen)`, deferred reclaim via SEGMENT_END trace flag.
- **FFI aliasing UB:** half-split SPSC, FgState/IsrState disjoint memory.
- **Clock sync:** Klipper-style sliding-window linear regression, 10 Hz warmup / 1 Hz steady-state, quality gate on residual + drift + sample-age.
- **Telemetry transport:** TraceRing kept in DTCM, sized at 2× host-stall budget; overflow is a hard fault. AXI SRAM relocation is a future contingency that requires MPU non-cacheable configuration (out of scope for Step 6).
- **Host runtime:** standalone Rust kalico-host-rt owns USB-CDC fd directly; Klippy-replacement non-motion subsystems are Step 7 MVP scope.
