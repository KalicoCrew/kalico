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

### 3.3 Acceptance gate for Phase 0

A round-tripped multi-segment streaming test against the sim, exercised end-to-end:

1. Sim starts; firmware boots; identify handshake succeeds.
2. Host streams 10 segments using either `kalico_load_curve` (if root-caused) or `kalico_load_fixture_curve` (if escape hatch).
3. MCU evaluates each in order; trace stream reports monotone tick counters and correct segment_id sequence.
4. Status frame reports correct `current_segment_id` and `queue_depth` throughout.
5. Underrun-fault path: stop pushing while stream-open → MCU latches `KALICO_FAULT_UNDERRUN` within MIN_SEGMENT_DURATION_MS of last-segment retirement.
6. End-to-end iteration loop ≤30 seconds (vs ≥3 minutes flash-and-reboot).

Phase 0 is complete when all six pass. Step-6 protocol work proceeds against a working sim.

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
kalico_credit_freed accepted_through_segment_id=%u free_slots=%c
```

Fields:
- `accepted_through_segment_id` (u32) — monotonically increasing; "the MCU has retired all segments with id ≤ this value." Host-side idempotency: if the host receives `accepted_through=N` after already processing `accepted_through=M ≥ N`, the event is a no-op.
- `free_slots` (u8) — current free queue capacity after this retirement.

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
              credit_epoch=%u accepted_through_segment_id=%u
```

The status frame:
- Carries authoritative full-state for credit reconciliation. If the host's credit count diverges from `(queue_capacity - queue_depth)`, the host re-syncs to the MCU's view and emits a `KALICO_FAULT_INTERNAL_INVARIANT` warning to telemetry.
- Provides the clock-sync sample piggyback (mcu_clock_now is captured at status-frame-emit instant; pairs with host_recv_time for the clock-sync regression).
- Acts as the keepalive heartbeat: if the host doesn't see a status frame for 3× the expected period, the MCU is declared `KALICO_FAULT_LIVENESS_STALLED` and disconnect-recovery triggers.

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
kalico_push_response result=%i accepted_through_segment_id=%u credit_epoch=%u
```

`accepted_through_segment_id` lets the host detect if any prior push was silently lost (accepted_through should equal the last successfully-pushed id). `credit_epoch` lets the host detect MCU resets.

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

---

## 7. Buffer-budget framework

Per brainstorm Q7 (decision (p), settled after kalico-verifier round 2): Step 6 commits to **the framework** — parameter linkage and measurement protocol — not to specific numbers. Numbers fall out at implementation time after measurement.

### 7.1 Parameters and linkage

| Parameter | Source | Default (pending measurement) |
|---|---|---|
| `MIN_SEGMENT_DURATION_MS` | MCU runtime cost (sub-tick boundary loop overhead at 40 kHz) | 0.5 ms (initial estimate) |
| `HOST_STALL_BUDGET_MS` | Measured p99.99 host-side tail latency on Pi 5 + Bookworm desktop + production load | 20 ms (initial estimate) |
| `Q_N_BUFFER_MS` | `max(HOST_STALL_BUDGET_MS, 4 × MIN_SEGMENT_DURATION_MS)` | derived |
| `Q_N` | Smallest power of 2 ≥ `ceil(Q_N_BUFFER_MS / MIN_SEGMENT_DURATION_MS) + 1` (heapless effective-cap = N-1 rule) | derived |
| `CURVE_POOL_N` | `Q_N` (worst case is one distinct curve per segment) | derived |
| `MAX_BOUNDARY_ITERS` | `Q_N - 1` (effective capacity); predicate `iters > MAX_BOUNDARY_ITERS` | derived |
| `MAX_RESIDUAL_US` | Measured clock-freq estimator p99.99 residual on production link | 100 µs (initial estimate) |
| `TRACE_RING_DURATION_MS` | `≥ HOST_STALL_BUDGET_MS` | derived |
| `TRACE_RING_N` | `TRACE_RING_DURATION_MS × 40` (40 kHz sample rate) | derived |
| `TRACE_RING_LOCATION` | DTCM if `sizeof(TraceRing) ≤ 2 KB`, else AXI SRAM | conditional |

### 7.2 Layer-3 minimum-segment-duration enforcement

Per kalico-verifier round 2 + CLAUDE.md print-throughput-non-negotiable:

- Layer 3 must NOT emit runtime segments below `MIN_SEGMENT_DURATION_MS`, except for explicit end/flush sentinel segments.
- If reparameterization × shaper convolution would produce sub-budget pieces, Layer 3 must constrain Layer-2 v(s) so the resulting runtime segments are ≥ `MIN_SEGMENT_DURATION_MS`. **Binding constraint, not coalescing.** The constraint derives from MCU hardware (40 kHz tick, sub-tick boundary loop overhead), not from "comms convenience."
- If Layer 3 cannot satisfy the constraint with current Layer-2 output, that's a hard planner error reported to the user. Not a silent slowdown.
- The Layer-3 spec (Step 8) inherits this constraint as part of its output contract. Step 6 documents it; Step 8 implements it.

### 7.3 Measurement protocol

Three measurement runs gate the parameter pinning. All run during Step-6 implementation, results checked into `docs/research/step6-buffer-budget-measurements.md`.

**M1 — Host-stall measurement.** 8h Pi-5 soak: Bookworm desktop, Wayfire compositor, Mainsail rendering trace UI, Moonraker WebSocket, journald `--persist`, simulated USB-CDC traffic at production rate (~1 kHz peak push), simulated Layer-1/2/3 planner running. Capture distribution of segment-push completion times. Report: p50, p95, p99, p99.9, p99.99, max. `HOST_STALL_BUDGET_MS` = max(p99.99, 5 ms) — the 5 ms floor is for sanity; we never go tighter than RT_PREEMPT-friendly.

**M2 — MCU runtime cost.** Re-run Step-5 cycle-budget bench (`tools/test_h723_cycle_count.py`) against Step-6's protocol-handler additions (clock-sync responder, stream-state machine, generation-handle lookup). Worst-case ISR cycle count derives `MIN_SEGMENT_DURATION_MS` lower bound: `MIN_SEGMENT_DURATION_MS = ceil(worst_isr_cycles / clock_freq) × MIN_TICKS_PER_SEGMENT × tick_period_us / 1000`, where `MIN_TICKS_PER_SEGMENT ≥ 4` (allows 1 sub-tick boundary crossing without sub-tick-loop iteration overflow).

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

Per kalico-verifier round 2 Claim F (settled): Step-5's "no overwrite after load" policy fails at production scale (10K–200K segments per print). Step 6 ships generation-counter discipline.

### 10.1 Handle layout

`CurveHandle = u16`:
- Bits 0..6 (6 bits): `slot_idx` — supports up to 64 slots. (Aligned to derived `CURVE_POOL_N` from §7.1.)
- Bits 6..16 (10 bits): `generation` — 1024 generations before wrap.

At `CURVE_POOL_N = 64`, that's 64 × 1024 = 65,536 curve allocations between full-cycle wraps. Worst-case wrap per print: 200K segments / 1024 gens ≈ 200 wraps over a long print — easily handled by the wrap policy below.

```rust
#[repr(C)]
struct CurveHandle(u16);

impl CurveHandle {
    fn slot(self) -> usize { (self.0 & 0x3F) as usize }
    fn gen(self) -> u16 { self.0 >> 6 }
    fn pack(slot: usize, gen: u16) -> Self {
        debug_assert!(slot < 64);
        debug_assert!(gen < 1024);
        Self((slot as u16) | (gen << 6))
    }
}
```

### 10.2 Wrap rule

When a slot's generation rolls over from 1023 → 0, the slot is **rejected for one allocation cycle** — the planner is forced to pick a different slot for the next allocation. This sidesteps the ABA hazard where a stale handle from generation 1023 could match a fresh allocation also at generation 0.

Formally: `try_alloc()` returns `Some(slot)` only if `slot.last_reclaimed_gen != slot.next_gen`. After the rejection, the planner picks another slot; the rejected slot becomes available next cycle once the wrap-cooldown elapses.

### 10.3 Reclaim mechanism

- ISR emits a `SEGMENT_END` trace sample at retirement, including the segment's `curve_handle`.
- Foreground task drains trace stream; observes `SEGMENT_END(slot=N, gen=G)`.
- Foreground checks: is there any pending segment in the queue referencing `(slot=N, gen=G)` or earlier? If no, `slot.last_reclaimed_gen.store(G, Release)`.
- Producer side: `try_alloc(slot=N)` succeeds only if `slot.last_reclaimed_gen.load(Acquire) == slot.current_gen.load(Acquire)`. On success, increments `current_gen`.

Cost: one `AtomicU16` per slot for `current_gen`, one `AtomicU16` per slot for `last_reclaimed_gen`. At 64 slots: 256 bytes total. Negligible.

### 10.4 ISR validation on every segment evaluation

```rust
fn lookup(handle: CurveHandle) -> Result<&LoadedCurve, FaultCode> {
    let slot = &self.slots[handle.slot()];
    if slot.current_gen.load(Ordering::Acquire) != handle.gen() {
        return Err(FaultCode::InvalidCurveHandle);
    }
    Ok(&slot.curve)
}
```

Defends against use-after-reclaim (a stale handle from a prior generation; should never happen under the producer-protocol invariants but is a defensive backstop).

---

## 11. FFI aliasing UB → half-split SPSC

Per Step-5 acknowledged latent UB (`*mut KalicoRuntime → &mut RuntimeContext` produces overlapping `&mut` under Rust's strict aliasing model when the ISR preempts foreground): Step 6 closes this before live producer surfaces. Per kalico-verifier round 2 Claim G(iv): blocking; not deferable.

### 11.1 RuntimeContext split

```rust
pub struct RuntimeContext {
    fg: FgState,    // foreground-only mutable state
    isr: IsrState,  // ISR-only mutable state
    shared: SharedState,  // SPSC channels + atomics, immutable references from both
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
    widen_state: ClockWidenState,  // currently shared, becomes ISR-private
}

pub struct SharedState {
    last_error: AtomicI32,
    runtime_status: AtomicU8,
    stream_open: AtomicBool,
    // ... other cross-half atomics
}
```

### 11.2 FFI surface evolution

```rust
#[no_mangle]
pub extern "C" fn kalico_runtime_init() -> *mut KalicoRuntime { /* ... */ }

#[no_mangle]
pub extern "C" fn kalico_runtime_tick(rt: *mut KalicoRuntime, raw_cyccnt: u32) {
    // ISR re-borrows IsrState only:
    let ctx = unsafe { &mut *(rt as *mut RuntimeContext) };
    let isr = &mut ctx.isr;
    let shared = &ctx.shared;
    isr.engine.tick(raw_cyccnt, shared);
}

#[no_mangle]
pub extern "C" fn kalico_runtime_push_segment(rt: *mut KalicoRuntime, ...) -> i32 {
    // Foreground re-borrows FgState only:
    let ctx = unsafe { &mut *(rt as *mut RuntimeContext) };
    let fg = &mut ctx.fg;
    let shared = &ctx.shared;
    fg.push_segment(/* ... */, shared)
}
```

The two `&mut` re-borrows are non-overlapping because `FgState` and `IsrState` are disjoint memory regions. `SharedState` is `&` (immutable reference) from both sides; mutation is via atomics, which are sound under aliasing.

The TIM5-disable-around-push idiom (currently used to safely access `widen_state` from foreground) goes away: `widen_state` becomes ISR-private and is touched only from the ISR, with foreground reading derived values via atomics in `SharedState`. Per kalico-verifier round 2 surfaced concern #4: this also retires the TIM5-disable side-effect (which at Q_N=64 + bursty pushes could spike ISR-off duty cycle).

### 11.3 Loom test coverage

Per Step-5 plan-changes-log open follow-up: "Loom test coverage expansion (gated to Step 6 when live producer surfaces)." Step 6 ships loom tests on the half-split SPSC channels, the cross-half atomics, and the stream-lifecycle state machine. Test surface lives under `rust/runtime/tests/loom_*.rs` (host-target, `--cfg loom` build).

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

- Warmup: 10 Hz for first 30 samples (`MIN_WARMUP_SAMPLES`).
- Steady-state: 1 Hz piggybacked on `kalico_status` (which already emits at 10 Hz; we use 1-of-10 frames as the sync sample, the others are pure status). Reduces wire overhead.
- High-residual mode: if `residual_max_in_window` rises above 50% of `MAX_RESIDUAL_US`, the host bumps cadence back to 10 Hz until residual settles.

### 12.4 Quality gate

Motion is armable when, for each MCU:
- `sample_count ≥ MIN_WARMUP_SAMPLES` (default 30).
- `last_sample_age_ms ≤ MAX_SAMPLE_AGE_MS` (default 2000ms steady, 100ms during warmup).
- `residual_max_in_window ≤ MAX_RESIDUAL_US` (default 100µs pending §7.3 measurement).
- `|drift_ppm| ≤ MAX_DRIFT_PPM` (default 100ppm pending measurement; >100ppm means the crystal is failing).

For multi-MCU: cross-MCU drift sanity `|fA / fB - 1| < 1e-3`; failure → `KALICO_FAULT_CROSS_MCU_DESYNC` at arm time.

---

## 13. Telemetry transport (TraceRing reconciliation + AXI relocation)

Per kalico-verifier round 2 surfaced concern #1: Step-5 silently halved TraceRing from 1024 → 128 (3.2 ms headroom at 40 kHz), but spec text §4.5 still claims 25 ms. At Step-6's host-stall budget (default 20 ms), 3.2 ms trace-ring overflow happens before the queue is exhausted — trace ring is part of the host-stall surface.

### 13.1 Reconciliation rule (from §7.1)

`TRACE_RING_DURATION_MS ≥ HOST_STALL_BUDGET_MS`. So `TRACE_RING_N ≥ HOST_STALL_BUDGET_MS × 40` (40 kHz tick × samples/tick).

At default 20 ms: `TRACE_RING_N = 800`, sample size 32 B, total 25.6 KB. Doesn't fit DTCM alongside `Q_N=64` queue (~4 KB) + `CURVE_POOL_N=64` (~12 KB) + stack + scratch on H723's 128 KB DTCM budget.

### 13.2 AXI SRAM relocation

Required: TraceRing storage relocates to AXI SRAM via linker-section annotation:

```rust
#[link_section = ".axi_sram"]
static mut TRACE_RING_BUFFER: [TraceSample; TRACE_RING_N] = [...; TRACE_RING_N];
```

Linker script (`out/klipper.ld` template) gets a new `.axi_sram` section pointing at AXI SRAM physical address. Per H723 reference manual: AXI SRAM is on the AXI bus, accessed slightly slower than DTCM (extra wait state) but well within ISR budget for the trace-write path (one 32-byte memcpy per tick = ~8 cycles overhead vs DTCM, negligible against the engine's per-tick cost).

The half-split refactor (§11) is a precondition: the ISR-side `trace_producer` is the only writer; the foreground-side `trace_consumer` is the only reader. AXI SRAM access from the ISR has a slight wait-state cost; from the foreground (USB-CDC drain path) the cost is negligible against USB-CDC TX time.

### 13.3 Trace schema

`TraceSample` (32 bytes, packed):
```rust
#[repr(C, packed)]
struct TraceSample {
    tick: u64,                  // widened CYCCNT at sample time
    motor_a: f32,
    motor_b: f32,
    motor_e: f32,
    segment_id: u32,
    curve_handle: u16,          // (slot, gen) for §10 reclaim accounting
    flags: u8,                  // SEGMENT_END, SEGMENT_START, FAULT, ...
    _pad: u8,
}
```

Per §10.3, `SEGMENT_END` flag drives generation-handle reclaim; foreground task observes `SEGMENT_END` events to know when to release a slot.

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
| TraceRing 128/1024 mismatch | §13.1, §13.2 — reconciled against host-stall budget, AXI SRAM relocation. |
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

- **TraceRing 128 vs 1024 spec mismatch** — adopted as §13.1, §13.2.
- **MIN_SEGMENT_CYCLES = 50 µs is the only enforced floor** — adopted as §7.2 Layer-3 enforcement.
- **AtomicU64 not lock-free on ARMv7-M** — noted in §10 (handle stays u16), §11 (cross-half atomics stay u32 or smaller).
- **TIM5-disable-around-push idiom audit** — adopted as §11.2 (idiom retired with half-split).
- **CLAUDE.md throughput-non-negotiable binds Layer-3 min-duration derivation** — adopted as §7.2.

Net: the spec converged on a framework + measurement protocol approach; concrete numbers fall out at Step-6 implementation time after §7.3 runs.

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
- **Telemetry transport:** TraceRing relocated to AXI SRAM; capacity ≥ host-stall budget.
- **Host runtime:** standalone Rust kalico-host-rt owns USB-CDC fd directly; Klippy-replacement non-motion subsystems are Step 7 MVP scope.
