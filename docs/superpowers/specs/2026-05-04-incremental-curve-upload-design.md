# Incremental curve upload — design

**Status:** rev 2 (post-review). Addresses 7 substantive issues from the plan-reviewer pass: fire-and-forget back-pressure, finalize retry policy, slot VLQ growth, `MAX_PENDING_BLOCKS` interaction, pool sizing vs trajectory worst case, ingest-state right-sizing, `send_typed` honesty.
**Date:** 2026-05-04.
**Step:** 7-D Phase 4 (bridge-mode first print).
**Supersedes the wire-fragmentation portion of:** [`2026-05-04-multi-piece-dispatch-design.md`](2026-05-04-multi-piece-dispatch-design.md) ("B.1") — the multi-piece chunker idea is preserved as an *optimization* layered on top of this protocol; the per-`Buffer`-field 255-byte cap that B.1 was sized against is **not** the binding constraint.

---

## 1. Problem

Sending a post-shape per-axis NURBS in one `kalico_load_curve` command overflows Klipper's per-frame `MESSAGE_MAX = 64`-byte cap. The host's `build_frame` truncates `msglen` (a `u8`) silently; the MCU's `extract_packet` either CRC-rejects or processes a corrupt frame; `command_kalico_load_curve` never fires; the host's producer times out at 100 ms.

Concrete reproducer (`bash tools/sim_klippy/run_local.sh "G1 X10 F1000"` then `python3 tools/sim_klippy/test_phase4_steps.py`): a degree-9 single-axis curve serializes to a 414-byte payload. Klipper's wire is hard-capped at ~50 bytes of usable payload per frame.

This is not a sizing bug in B.1's chunker constant — even one Bézier piece at degree 9 is ~120 bytes raw (10 cps + 20 knots × 4 B), which doesn't fit a single 64-byte frame at any chunker setting.

## 2. Constraints

The fix has to respect:

1. **Print throughput is non-negotiable** ([`CLAUDE.md`](../../../CLAUDE.md)). Post-shape NURBS quality cannot be downgraded (rules out polynomial-degree reduction).
2. **MCU receives shape with PA + IS pre-baked** ([`CLAUDE.md`](../../../CLAUDE.md)). Cannot offload convolution to MCU (rules out sending pre-shape + kernel).
3. **No queue-based offload** ([`CLAUDE.md`](../../../CLAUDE.md)). Each shaped segment dispatches in real time as planned.
4. **Klipper-stock protocol compatibility** — kalico runtime is linked into Klipper's MCU build; klippy bridges via PyO3. A Klipper-wide `MESSAGE_MAX` change is a fork-divergence cost we'd rather not pay for an MVP.
5. **STM32H723 USB-FS endpoint = 64 bytes** (confirmed in `src/generic/usb_cdc_ep.h`). Bumping past 64 forces multi-packet reassembly in `usbfs.c` that doesn't exist today.
6. **Worst-case post-shape degree = 9** (cubic Bézier source × degree-4 bell smoother × composition + Hermite refit → convolve; `rust/trajectory/src/kernel.rs`, `rust/nurbs/src/algebra.rs::convolve`). Locked by the smooth-shaper architecture (CLAUDE.md).
7. **MCU per-slot scratch budget — must grow.** Today: `kalico_aligned_cps[80]` + `kalico_aligned_knots[91]` = 684 B (`runtime_tick.c:422-423`), sized in Step 7-B against an empirical 64-cps measurement plus 25 % margin (`docs/superpowers/specs/2026-04-30-step7b-layer4-mcu-evaluator-design.md:104-114`). Realistic worst case for a complex move at production shaper frequency is ~100 cps + ~111 knots per axis (post-convolution piece growth ≈ `2 × hermite_pieces + 1`; for ≤10 hermite pieces this is ≤21 post-shape pieces; for degree 9 with 10 pieces: 91 cps, 100 knots; with the 25 % margin this spec applies, ≤100 cps, ≤111 knots is the design target). H723 SRAM is 564 KB; the worst-case slot scratch grows from 684 B to 844 B per slot, i.e. +160 B per slot, +10.3 KB across all 64 slots. Comfortably within budget. See §5.0.
8. **NAK / RTO / credit semantics from Step 7-C-IO** (`docs/superpowers/specs/2026-04-30-step-7c-io-design.md`) must continue to apply. Wire-layer NAK retransmits whole frames, not application-layer multi-frame uploads.
9. **Existing `dispatch_fire_and_forget` silently drops on window-full** (`reactor.rs:226-245`). This is incompatible with the multi-frame upload pattern this spec requires. The fix to `dispatch_fire_and_forget` is in scope here (§6.0), and is itself a small Step-7C-IO addendum.

## 3. Decision

Replace the single-shot `kalico_load_curve` command with a three-command **incremental upload protocol**:

```
kalico_load_curve_begin     slot=%hu degree=%c total_cps=%hu total_knots=%hu
kalico_load_curve_chunk     slot=%hu kind=%c offset=%hu data=%*s   (N times)
kalico_load_curve_finalize  slot=%hu  →  kalico_load_curve_finalize_response result=%i curve_handle_packed=%u
```

The MCU stages cps and knots in a per-slot scratch buffer until `finalize`, which validates lengths, calls into `kalico_runtime_load_curve`, and returns the slot handle. The host issues `begin → N×chunk → finalize`. Today's single-shot path becomes a degenerate "1 begin + small N + 1 finalize" case.

**Why this option:**

- **No Klipper-stock protocol change.** `MESSAGE_MAX = 64`, `MAX_PENDING_BLOCKS = 12`, `Buffer` u8 length-prefix, `serialqueue.c`, `msgproto.py`, USB-CDC EP sizes — all unchanged.
- **Throughput-neutral.** No algorithmic compromise; post-shape NURBS goes over the wire byte-for-byte identical to today, just split across frames.
- **Klipper-idiomatic precedent.** Mirrors the `allocate_oids` / `finalize_config` staging pattern in `src/basecmd.c`, and is structurally what `serialqueue.c`'s multi-block batching does for unrelated commands.
- **NAK/RTO inherit unchanged.** Each chunk is its own request/response, retried at the wire layer if a NAK arrives. No new application-layer retransmit logic.
- **Wire-format-stable.** No new schema versions, no canonical-capture invalidation beyond the load_curve commands themselves.

**Why not the alternatives** (full options-space writeup — all six options the research agent surveyed — landed at `docs/superpowers/research/2026-05-04-curve-wire-options.md` if we want the long form; otherwise summarized here):

- **MESSAGE_MAX bump (64 → 256).** Klipper-wide protocol break; USB-FS 64-byte EP boundary forces new multi-packet reassembly in `usbfs.c`; `serialqueue.c` flow accounting needs review. Best throughput (1 frame per axis) but blast radius is incompatible with the MVP scope.
- **Per-piece dispatch.** Degree-9 single piece (~120 B raw) doesn't fit one 50-byte frame even with no header. DOA.
- **Lower polynomial degree post-shape.** Post-shape refit imposes measurable trajectory error. Violates "throughput is non-negotiable."
- **Split fields across two commands** (cps in one, knots in another). Degree-9 knot vector alone is 80 B → still doesn't fit one frame. Strict subset of incremental upload.
- **Coefficient quantization (f16 cps, breakpoint-encoded knots).** Both are real wins (f16 saves ~50% on cps, breakpoint encoding is ~10× on knots since post-`extract_bezier_pieces` knots are full-multiplicity-redundant), but they're *optimizations* layered on whatever fragmentation primitive we pick. **Defer** to a follow-up — see §10.

## 4. Wire schema

### 4.1 `kalico_load_curve_begin`

```text
kalico_load_curve_begin
  version=%c slot=%hu degree=%c total_cps=%hu total_knots=%hu
```

| field | type | meaning |
|---|---|---|
| version | u8 | `0x01` (matches existing `FORMAT_VERSION_V1`) |
| slot | u16 | curve-pool slot index, host-allocated via existing `slot_pool::try_alloc` |
| degree | u8 | NURBS polynomial degree |
| total_cps | u16 | total f32 control-point count for this curve |
| total_knots | u16 | total f32 knot count for this curve |

No response. Fire-and-forget. The MCU resets the per-slot scratch state (`bytes_received_cps = 0`, `bytes_received_knots = 0`, `expected_cps_bytes = total_cps × 4`, `expected_knots_bytes = total_knots × 4`, `degree`).

Rationale for fire-and-forget: an unacked `begin` followed by chunks creates an obvious finalize-time error (length mismatch); the host doesn't need confirmation that the begin landed. Simplifies producer state — one synchronous `call_typed` only at finalize time.

Frame size (worst case): 1 (cmd-id VLQ) + 1 (version) + 3 (slot u16 worst case) + 1 (degree) + 3 (total_cps u16) + 3 (total_knots u16) = 12 B payload + 5 B envelope = **17 B**. Comfortably under 64.

### 4.2 `kalico_load_curve_chunk`

```text
kalico_load_curve_chunk
  slot=%hu kind=%c offset=%hu data=%*s
```

| field | type | meaning |
|---|---|---|
| slot | u16 | same slot as `begin` |
| kind | u8 | `0` = cps, `1` = knots |
| offset | u16 | byte offset into the destination scratch buffer |
| data | buffer (≤ 255 B) | f32 payload chunk, little-endian, contiguous |

No response. Fire-and-forget. The MCU bounds-checks `offset + data.len() ≤ expected_<kind>_bytes`, `memcpy`s into the scratch slot, increments a `received_<kind>_bytes` counter.

Why no response: chunk validation is purely local (offset arithmetic + memcpy), and any failure surfaces at finalize time as a length mismatch. Per-chunk acks would inflate frame count by 2× and serialize the host pipeline behind RTT for no benefit beyond what wire-layer NAK already provides.

Worst-case payload size budget per chunk:
- Frame envelope: 5 B (`MESSAGE_MIN`)
- Command-id VLQ: ~1 B
- slot VLQ (u16): ≤3 B
- kind VLQ (u8): 1 B
- offset VLQ (u16): ≤3 B
- buffer length prefix: 1 B
- **→ Frame overhead ≈ 14 B; data budget = 64 − 14 = 50 B = 12 f32 per chunk.**

Conservative target: **40 B data per chunk = 10 f32**, leaving slack for VLQ growth as `offset` increases.

### 4.3 `kalico_load_curve_finalize`

```text
kalico_load_curve_finalize slot=%hu
  →  kalico_load_curve_finalize_response result=%i curve_handle_packed=%u
```

The MCU validates `received_cps_bytes == expected_cps_bytes` and `received_knots_bytes == expected_knots_bytes`; on mismatch returns `result = -2` (`KALICO_ERR_INVALID_CURVE`). On success, calls `kalico_runtime_load_curve(slot, scratch_cps, total_cps, scratch_knots, total_knots, degree, &handle)`, returns the result code and packed handle.

Frame size: 1 + 3 = **4 B payload + 5 B envelope = 9 B**. Response is the same shape as today's `kalico_load_curve_response`, ~7 B payload. Both fit one frame.

### 4.4 Retire single-shot `kalico_load_curve`

The existing `kalico_load_curve` command is **removed**. There is no degenerate "small curve uses single-shot" fast path. Rationale:

- Two code paths means two test surfaces and two failure modes for one operation.
- The MCU's per-slot scratch state is identical for "1 chunk" and "16 chunks"; the begin/finalize overhead is ~26 B = 0.4 ms at 250 kbaud, negligible compared to the dispatch cadence.
- Phase 4 has not yet shipped any non-test consumer of the single-shot command.
- Keeps the MCU-side ingest state machine simple (one entry path).

The `python-diff-test` and canonical-capture corpus regenerate against the new schema. The Step 7-C-IO test battery is unaffected (it covers wire-frame mechanics, not application-level commands).

## 5. MCU-side state

### 5.0 Pool-size bump (prerequisite)

Bump the per-slot scratch ceilings in `runtime_tick.c:422-423` and the corresponding constants in `rust/runtime/src/curve_pool.rs`:

| constant | today | new | rationale |
|---|---|---|---|
| `MAX_DEGREE` | 10 | 10 | unchanged — degree 9 fits with 1 margin |
| `MAX_CONTROL_POINTS` | 80 | 100 | empirical worst case 91 (10 pieces × 9 + 1) + ~10 % margin |
| `MAX_KNOT_VECTOR_LEN` | 91 | 111 | derived per existing invariant `MAX_CONTROL_POINTS + MAX_DEGREE + 1` = 100 + 10 + 1 (`rust/runtime/src/curve_pool.rs:25`) |

Per-slot scratch grows from 80 + 91 = 171 f32 = 684 B to 100 + 111 = 211 f32 = 844 B. With `CURVE_POOL_N = 64` slots: pool RAM grows from 43.7 KB to 54.0 KB (+10.3 KB). Within H723 SRAM budget (564 KB) and within F446 SRAM budget (128 KB) by a wide margin.

This bump is what unlocks "retire B.1's chunker" (§6.3) — without it, the trajectory layer's worst-case 100-knot output overflows the pool, and either multi-NURBS-per-segment dispatch or a tighter trajectory-side piece cap is forced. Growing RAM is the right tradeoff per the project ethos that compute/RAM is spent in service of trajectory optimality, not the other way around.

The Step 7-B canonical capture corpus needs regeneration anyway after this spec lands; the bumped pool sizes do not alter the corpus's wire-level structure (slot scratch is host-invisible).

### 5.1 Ingest state

The host is single-threaded for curve uploads (the bridge dispatch closure issues `producer::load_curve` calls sequentially per shaped segment per MCU). Therefore at most **one** upload is in flight per MCU at any time. The MCU keeps a single global ingest context, not per-slot state:

```c
struct kalico_curve_ingest {
    uint16_t slot;
    uint16_t expected_cps_bytes;
    uint16_t expected_knots_bytes;
    uint16_t received_cps_bytes;
    uint16_t received_knots_bytes;
    uint8_t  degree;
    uint8_t  in_progress;   // 1 between begin and finalize/abort
};
```

Sized once: **12 B static**. The existing `kalico_aligned_cps[100]` / `kalico_aligned_knots[110]` static buffers (post-bump per §5.0) hold the in-flight upload's payload.

If `begin slot=A` arrives while a different `slot=B` is `in_progress`, the new begin **wins** (overwrites context). Rationale: this only happens on a host bug or a reconnect; "latest begin wins" is the simplest, most predictable recovery.

**Memory delta from today: +156 B (per-slot scratch growth from §5.0) + 12 B (ingest context) = +168 B static.** No per-slot ingest array.

### 5.2 Foreground-only dispatch

All three new handlers run in the same command-dispatch foreground context as today's `command_kalico_load_curve`. No ISR or background thread touches scratch state. The atomic commit point remains `kalico_runtime_load_curve` inside `finalize`'s handler.

### 5.3 Ingest lifecycle

Single global ingest context per §5.1. `S` denotes the slot index recorded in `begin`.

| event | ingest state |
|---|---|
| `begin slot=S` (any current state) | reset counters; set `slot = S`, `in_progress = 1`, `expected_*` |
| `chunk slot=X` while `in_progress = 0` | drop silently, log warning (stale frame after disconnect) |
| `chunk slot=X` while `in_progress = 1`, `X != slot` | drop silently, log warning (host bug — concurrent upload) |
| `chunk slot=S` while `in_progress = 1` | bounds-check `offset + data.len() ≤ expected_<kind>_bytes`; memcpy; bump received counter |
| `finalize slot=X` while `in_progress = 0` | return `KALICO_ERR_INVALID_CURVE` |
| `finalize slot=X` while `in_progress = 1`, `X != slot` | return `KALICO_ERR_INVALID_CURVE`; do **not** clear `in_progress` (the active upload's `finalize` is still pending) |
| `finalize slot=S` length-mismatch | return `KALICO_ERR_INVALID_CURVE`; clear `in_progress`; do not install |
| `finalize slot=S` success | call `kalico_runtime_load_curve`; clear `in_progress`; return handle |
| MCU shutdown / restart | ingest state cleared alongside curve pool |

### 5.4 Error codes

Reuse existing codes from `kalico_runtime_load_curve`:
- `-2 KALICO_ERR_INVALID_CURVE` — length mismatch, missing begin, malformed degree
- `-7 KALICO_ERR_RUNTIME_NOT_INITIALIZED` — `kalico_rt_handle == NULL`
- `-103 KALICO_ERR_PROTOCOL_VERSION_UNSUPPORTED` — `version != 0x01` (reported on `begin`; `chunk` and `finalize` cannot fail-version since version is only on begin)

## 6. Host-side dispatch

### 6.0 Backpressure-respecting fire-and-forget (Step-7C-IO addendum)

`reactor.rs::dispatch_fire_and_forget` (lines 226-245) currently silently drops the frame when `unacked_window.is_full()`:

```rust
if self.unacked_window.is_full() {
    log::warn!("dispatch_fire_and_forget: unacked window full, dropping frame");
    return Ok(());
}
```

This is incompatible with multi-frame upload: a dropped chunk surfaces as a finalize-time length mismatch with no automatic recovery. The fix:

1. Add a `pending_fire_and_forget: VecDeque<Vec<u8>>` to the reactor, sibling of `pending_submissions`. When `dispatch_fire_and_forget` is called and the unacked window is full, push the payload into this queue instead of dropping.
2. Extend `drain_pending_submissions` (today: lines 247-274) to also drain `pending_fire_and_forget` after `pending_submissions`. Order matters: submissions (which the caller is blocked on) take priority over fire-and-forget.
3. Cap `pending_fire_and_forget` at `PENDING_FIRE_AND_FORGET_CEILING = 256` (twice the existing `PENDING_SUBMISSION_CEILING`, since chunks are smaller and arrive in bursts). Overflow → log error + return error to caller (host-side bug, not a wire-side condition).

This generalizes the existing back-pressure behavior across both call modes. The semantics that "frames written via `dispatch_fire_and_forget` are guaranteed to reach the wire eventually unless the host overruns the ceiling" is now uniform with `dispatch_submission`. Step-7C-IO's NAK / RTO / unacked-window / receive-window invariants are unaffected — this is a pre-write enqueue, not a post-write retransmit.

The Step-7C-IO test battery (A1–A7) needs one new test: **A8** — fire-and-forget enqueue under window-full does not drop, drains in order after window opens. ~30 lines of test code mirroring A4 (NAK retransmit).

This is a small, isolated change to Step-7C-IO. The alternative — keep drop semantics and add an application-layer retry-on-length-mismatch loop in `producer::load_curve` — is messier (host has to track which chunks were dropped, retry with new offsets, slot-pool generation gymnastics) and shifts complexity from the wire layer (where back-pressure naturally lives) to the application layer (where it's foreign). Backpressure is a wire-mechanics concern; fix it once at the wire layer.

### 6.1 `producer::load_curve` rewrite

Replace the single `call_typed` with:

```rust
pub fn load_curve<T: Transport>(
    io: &T,
    slot: u16,
    params: &CurveLoadParams,
    timeout: Duration,
) -> Result<u32, ProducerError> {
    let cps_buf = params.cps_bytes();      // total_cps × 4 bytes
    let knots_buf = params.knots_bytes();  // total_knots × 4 bytes

    // 1) begin (fire-and-forget)
    io.send_typed("kalico_load_curve_begin", &[
        ("version",      FieldValue::Byte(FORMAT_VERSION_V1)),
        ("slot",         FieldValue::U16(slot)),
        ("degree",       FieldValue::Byte(params.degree)),
        ("total_cps",    FieldValue::U16(params.cps_f32.len() as u16)),
        ("total_knots",  FieldValue::U16(params.knots_f32.len() as u16)),
    ])?;

    // 2) chunks (fire-and-forget, conservative 40-byte data budget)
    const CHUNK_BYTES: usize = 40;
    for (kind, buf) in [(0u8, &cps_buf), (1u8, &knots_buf)] {
        for (chunk_idx, chunk) in buf.chunks(CHUNK_BYTES).enumerate() {
            let offset = (chunk_idx * CHUNK_BYTES) as u16;
            io.send_typed("kalico_load_curve_chunk", &[
                ("slot",   FieldValue::U16(slot)),
                ("kind",   FieldValue::Byte(kind)),
                ("offset", FieldValue::U16(offset)),
                ("data",   FieldValue::Buffer(chunk)),
            ])?;
        }
    }

    // 3) finalize (synchronous; only this call has a timeout)
    let resp = io.call_typed(
        "kalico_load_curve_finalize",
        &[("slot", FieldValue::U16(slot))],
        "kalico_load_curve_finalize_response",
        timeout,
    )?;
    // unchanged result/handle parsing
}
```

`send_typed` is a new typed-args fire-and-forget entry point on `KalicoHostIo`. **Not a thin wrapper.** Today's `send_fire_and_forget` (`host_io/mod.rs:389`) takes a pre-encoded command string and runs it through `parser.encode`; `call_typed` (line 285) takes typed `FieldValue` args and runs them through `parser.encode_typed`. `send_typed` needs the typed-args encoding path of `call_typed` joined with the fire-and-forget delivery path of `send_fire_and_forget` — concretely, a new `ReactorCommand::FireAndForgetTyped { payload: Vec<u8> }` variant whose payload is the already-encoded blob, and the reactor handler routes it through §6.0's enqueue path. ~30 lines on the reactor side, plus the public `send_typed` method on `KalicoHostIo`. Worth ~2 hours of careful code; not a freebie.

### 6.2 Frame budget per shaped segment

For the reproducer (degree 9, 1 piece per axis, single-axis-X move):
- cps = 10 f32 = 40 B → 1 chunk
- knots = 20 f32 = 80 B → 2 chunks
- total: 1 begin + 3 chunks + 1 finalize = **5 frames** (vs today's 1 oversize frame that doesn't work).

Worst case from the trajectory layer (degree 9, ≤10 pieces per axis from a complex move; bound is the §5.0 pool ceiling):
- cps ≤ 100 f32 = 400 B → ≤10 chunks (CHUNK_BYTES = 40 = 10 f32)
- knots ≤ 111 f32 = 444 B → ≤12 chunks
- total: 1 + 22 + 1 = **≤24 frames per axis per logical move per MCU.**

Two-axis MCU (Octopus Pro, X+Y) worst case: ≤48 frames per logical move per MCU. Klipper's wire window is `MAX_PENDING_BLOCKS = 12` frames; a 48-frame burst saturates the window 4× and relies on §6.0's back-pressure path for ordered drainage. Per-RTT throughput ≈ 12 × 64 B = 768 B; 48 frames × 64 B ≈ 3 KB ≈ ~4 RTTs at the wire layer. **At a typical 1 ms USB-CDC RTT this is ≈4 ms wire time per logical move per 2-axis MCU.** Well within the 100 ms dispatch lead-time budget but worth measuring on real hardware (§9.5).

### 6.3 B.1 chunker disposition

The B.1 multi-piece chunker (`rust/motion-bridge/src/curve_chunker.rs`) was sized against the per-Buffer-field cap (255 B) which is no longer the binding constraint. Two options:

**(a) Retire the chunker.** Send the entire post-shape NURBS via incremental upload as one logical curve. Simpler. The MCU's per-slot scratch is sized at 80 cps + 91 knots = comfortably above the trajectory-layer worst-case piece-count for a single logical move.

**(b) Keep the chunker.** Split the curve into K sub-NURBS, send each via incremental upload, dispatch K push_segments per logical move. Inherits B.1's design complexity without the wire-fit motivation.

**Decision: (a) — retire the chunker, conditional on §5.0 pool bump.** The wire-fit reason for chunking is gone, *and* the MCU pool is grown (§5.0) to fit the trajectory layer's worst-case 100-cp / 110-knot output, so K=1 segment per logical move is sufficient. Without the §5.0 bump this option is unsafe (the existing 91-knot ceiling overflows on ~10-piece complex moves); the spec couples the two changes deliberately.

Removes `curve_chunker.rs`, `build_chunked_push_plans`, `ChunkedMcuPlan`, `McuChunkPlan`, `set_axis_handle` — about 600 LoC and 8 tests retired. The bezier-piece-extraction primitives in `rust/nurbs/src/bezier.rs` (`extract_bezier_pieces`, `bezier_pieces_to_nurbs`, `split_piece_at`) stay — they're needed by the breakpoint-encoding optimization (§10) and are general-purpose enough to outlive the chunker.

The B.1 work is not wasted: the diagnosis (correct identification of an oversize-frame failure mode) and the cross-axis-breakpoint-union infrastructure are documented for future reference in `docs/superpowers/specs/2026-05-04-multi-piece-dispatch-design.md` even after the code is removed.

The B.1 work is not wasted — the bezier-piece-extraction primitives in `rust/nurbs/src/bezier.rs` (`extract_bezier_pieces`, `bezier_pieces_to_nurbs`, `split_piece_at`) stay and are needed by the breakpoint-encoding optimization (§10).

### 6.4 Slot-pool semantics

Unchanged. The host allocates a slot via `slot_pool.try_alloc()` before `begin`, releases it on `finalize` failure, registers it for retirement on `finalize` success. `kalico_credit_freed` events still drive retirement.

## 7. Failure modes

| failure | host sees | MCU state |
|---|---|---|
| `begin` frame dropped on wire | wire-layer NAK retransmits begin; chunks land after | clean once begin finally lands |
| `chunk` frame dropped on wire | wire-layer NAK retransmits chunk | clean once chunk lands |
| `chunk` lands before `begin` | (impossible — wire is in-order; serialqueue preserves frame order) | n/a |
| Length mismatch at finalize | `ProducerError::McuRejected(-2)` | `in_progress = 0`, scratch reset |
| Host crashes mid-upload, reconnects | re-allocates slot via fresh slot pool view; sends new `begin` (which resets MCU scratch) | reset on first new begin |
| MCU crashes mid-upload | host's `compute_ack_clock` detects stall, fault-latches, host tears down slot pool | full reset |
| Two concurrent uploads to same slot (host bug) | second `begin` resets first | latest-wins; first upload is lost (acceptable — it's a host bug) |

The wire-layer NAK semantics inherit from Step 7-C-IO unchanged. No new application-layer retransmit logic.

### 7.1 Bridge dispatch policy on `KALICO_ERR_INVALID_CURVE` from finalize

After §6.0's back-pressure fix, a length-mismatch at finalize is **not a normal wire-failure path** — it indicates either a host bug (encoded `total_cps` ≠ summed chunks) or extreme `pending_fire_and_forget` overrun (which §6.0 makes return-error, not silent-drop). Both are bug conditions.

**Policy: fail loud.** The bridge dispatch closure (`bridge.rs::dispatch`) treats `ProducerError::McuRejected(-2)` from `producer::load_curve` as a fault: release the slot via `slot_pool.release`, return `Err("load_curve mcu={mcu_id}: invalid curve (host/MCU desync)")` to the planner, which surfaces as a `dispatch error` to klippy. No retry. The user sees the print abort; the bug gets fixed.

This is consistent with the existing dispatch-error contract (mcu rejections at `bridge.rs:1124-1134` are surfaced verbatim) and avoids the trap of looping on a deterministic bug.

## 8. Interaction with Step 7-C-IO

- **AwaitingResponse** — only `finalize` registers an entry; `begin` and `chunk` are fire-and-forget. Per-segment AwaitingResponse pressure goes from 1 entry per `load_curve` to 1 entry per `finalize` = same.
- **Unacked window** — every frame (begin + chunks + finalize) consumes one slot in the unacked window. Worst case ~24 frames per axis × 2 axes = 48 frames in flight per logical move per MCU. Klipper's wire window is `MAX_PENDING_BLOCKS = 12` (`klippy/chelper/serialqueue.c:93`, enforced at line 524). **A 48-frame burst will saturate the 12-frame window 4× per logical move** and rely on §6.0's back-pressure path for ordered drainage. This is intentional — the spec takes that latency hit (≈4 RTTs of dispatch time per segment, dominated by MCU wire ack cadence) in exchange for not changing Klipper-stock `MESSAGE_MAX`. See §9.5 for the validation gate that measures whether the resulting per-segment dispatch latency stays within the 100 ms lead-cycles budget.
- **Backpressure** — `dispatch_submission` enqueues in `pending_submissions` on full window (`reactor.rs:185-194`); §6.0 extends `dispatch_fire_and_forget` to enqueue in `pending_fire_and_forget` symmetrically. Both queues drain in `drain_pending_submissions` post-window-open. Finalize blocks behind both queues by construction (it's a `call_typed` going through the same window).
- **Credit/retirement** — unchanged. `kalico_credit_freed` retirement happens on segment retirement, not chunk retirement.
- **Slot VLQ growth** — `slot: u16` would VLQ-encode to 2 bytes once `slot ≥ 128`. With `CURVE_POOL_N = 64`, slots are dense in `0..63` by construction, so slot is always 1 VLQ byte. The chunk-frame budget (§4.2, §11 Q3) **assumes slot < 128 and degree-9 worst case**. If `CURVE_POOL_N` is ever bumped past 128, `CHUNK_BYTES` must shrink by 1 to stay within the 50 B usable payload. Pin in the unit test asserted in §11 Q3.
- **Canonical capture corpus** — needs regeneration for the new commands. Not a Phase 4 blocker (corpus regeneration was already on the Step 7-D punch list).

## 9. Validation gates

In order:

1. **Unit tests in `rust/kalico-host-rt/src/producer.rs`** — exercise `load_curve` with mock `Transport`, verify the begin/N×chunk/finalize sequence, verify chunk sizing and offsets.
2. **MCU-side unit-equivalent in `rust/kalico-runtime/`** — none (the MCU handlers live in `src/runtime_tick.c` and are integration-tested via the sim).
3. **Linux-sim end-to-end** — the existing `tools/sim_klippy/test_phase4_steps.py` reproducer must show non-zero step counts after the fix.
4. **Phase-4 hardware bring-up** — single-axis G1 on the Octopus Pro test bench produces step pulses (this is the next 7-D milestone regardless).
5. **Throughput + window-saturation measurement on real H723 over USB-CDC.** Two metrics: (a) per-segment dispatch wire time (target: ≤ 20 ms for the 2-axis worst-case 48-frame burst, well under the 100 ms lead-cycles budget); (b) `pending_fire_and_forget` queue depth distribution under typical OrcaSlicer print load (target: peak depth ≤ 64, sustained mean ≤ 8 — well below the 256 ceiling). If (a) eats meaningfully into lead time, raise `lead_cycles` before reaching for §10's protocol-level optimizations. If (b) approaches the ceiling, the §10 optimizations move from "nice-to-have" to "needed."

No 24-hour soak required for this fix specifically; the existing 7-C soaks cover the wire mechanics.

## 10. Optional optimizations (post-MVP)

These reduce frame count without changing the protocol shape. **Not in scope for this spec — listed so the design choice here doesn't preclude them.**

- **Breakpoint-encoded knots.** Post-`extract_bezier_pieces` knot vectors are full-multiplicity-redundant: for N pieces of degree d, the (N+1) breakpoints fully determine the (d·N + d + 2)-element knot vector. Sending breakpoints + degree + N saves ~10× on knot bytes (e.g. degree 9 / 5 pieces: 60 → 6 f32). MCU expands at load time. ~50 lines of code on each side.
- **f16 control points.** Halves cps bytes. f16 covers ±65 504 with ~3 decimal digits, ample for control points in mm at toolhead scale. Would require keeping knots in f32 (knot domain in seconds at 40 kHz tick crosses f16 dynamic range). MCU per-tick eval cost: ~10–20 cycles per coefficient — negligible at 40 kHz × 4 axes × ~10 cps.
- Combined, the two would bring the worst-case 22-frame-per-axis case down to ~6 frames per axis without protocol surgery.

## 11. Spec-level open questions

1. **Should `chunk` carry a sequence number, or rely on offset?** Argument for seq: detect drop in a way that's independent of payload arithmetic. Argument for offset (current proposal): wire is in-order; offset doubles as bounds-check input; one fewer field. *Tentative: offset only.*
2. **Should we keep the single-shot `kalico_load_curve` as a fast path for "1 chunk fits"?** Argument for: zero overhead for the common case of degree-3 short curves (which exist in compat-layer-driven prints). Argument against: two code paths. *Tentative: no — simplicity wins.*
3. **`CHUNK_BYTES` constant — 40 B or 50 B? Pinned to 40.** Leaves slack for VLQ growth as `offset` crosses 128 and 16384 (3- and 4-byte VLQ thresholds); 50 maxes out the budget today and breaks once `offset > 127` (every chunk past the 4th in a 400-byte payload). 40 has comfortable headroom for both `offset` growth (up to 14 KB worth of payload) and `slot` growth (if `CURVE_POOL_N` is ever bumped past 128). **Unit test asserts every encoded frame for every realistic `(slot, offset, data)` triple is ≤ `MESSAGE_MAX − 5 = 59` bytes.**
4. **Should `begin` include a content-addressed hash for finalize-time consistency?** Argument for: detect host bugs (wrong total_cps vs actual chunk sum) earlier. Argument against: counter mismatch already detects this; hashing adds MCU CPU on the load path. *Tentative: no.*

## 12. Implementation work breakdown

Rough order. Items 1–3 are prerequisites for items 4+; otherwise each piece is independently testable.

1. **Step-7C-IO addendum (§6.0):** add `pending_fire_and_forget` VecDeque + `PENDING_FIRE_AND_FORGET_CEILING`, extend `dispatch_fire_and_forget` to enqueue on window-full, extend `drain_pending_submissions` to drain it. New test A8 mirroring A4. ~80 LoC including test.
2. **`send_typed` plumbing:** new `ReactorCommand::FireAndForgetTyped { payload }` variant; reactor handler routes through §6.0's enqueue path; new public `KalicoHostIo::send_typed` method matching `call_typed`'s typed-args interface. ~30 LoC + unit test.
3. **MCU pool size bump (§5.0):** change `MAX_DEGREE`/`MAX_CONTROL_POINTS`/`MAX_KNOT_VECTOR_LEN` constants in `src/runtime_tick.c` *and* `rust/runtime/src/curve_pool.rs`; bump `kalico_aligned_cps[100]` / `kalico_aligned_knots[110]`. Verify no regression in existing pool tests.
4. **MCU command handlers:** new `command_kalico_load_curve_begin`, `command_kalico_load_curve_chunk`, `command_kalico_load_curve_finalize` in `src/runtime_tick.c`. Single global `kalico_curve_ingest` struct (§5.1). Delete the existing `command_kalico_load_curve`. ~120 LoC.
5. **`producer::load_curve` rewrite (§6.1):** begin/chunk/finalize sequence using `send_typed` + `call_typed`. Unit tests with mock `Transport` covering chunk sizing, offset arithmetic, length-mismatch finalize. ~60 LoC + tests.
6. **Bridge dispatch error policy (§7.1):** verify `bridge.rs::dispatch` surfaces `ProducerError::McuRejected(-2)` cleanly with the new error string. May be a no-op if existing pass-through is sufficient.
7. **Retire `rust/motion-bridge/src/curve_chunker.rs`** and the `build_chunked_push_plans` path; revert `bridge.rs` dispatch closure to per-axis-per-segment. ~600 LoC + 8 tests deleted.
8. **Sim-end-to-end gate:** `test_phase4_steps.py` shows non-zero step counts (Phase 4 unblock).
9. **Regenerate canonical capture corpus** for `kalico_load_curve_*` frames.

Steps 1–4 are the bulk. Step 7 is mostly deletion. Step 8 is the gate. Steps 1, 2, 3 can be landed independently as prep PRs before the integration PR for steps 4–8.
