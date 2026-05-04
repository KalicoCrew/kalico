# Multi-piece dispatch — design memo for shaped per-axis NURBS

**Date:** 2026-05-04
**Author:** B.1 design subagent (kalico Phase 4 unblock)
**Status:** Design ready; implementation pending.
**Prerequisite:** B.2 — `kalico_credit_freed` handler wiring (committed `a551d8eaf`).

## 1. Goals and non-goals

**Goal.** Land Option B so `dispatch` in `rust/motion-bridge/src/bridge.rs::init_planner` can ship a shaped per-axis `ScalarNurbs<f64>` whose Bézier-piece count exceeds the wire/MCU caps. Today's path tries to encode the full curve into one `kalico_load_curve` and either trips the host-side `u8` length-prefix at `parser.rs:481` (knots field max 63 f32 ≈ 5 Bézier pieces at degree 9) or the MCU's `MAX_CONTROL_POINTS=80` / `MAX_KNOT_VECTOR_LEN=91` (`runtime/src/curve_pool.rs:24-25`). Multi-piece dispatch splits one shaped axis curve into K sub-`ScalarNurbs<f64>`s, each ≤5 Bézier pieces, and emits K `load_curve` + K `push_segment` calls per logical move per MCU.

**Non-goals.**
- No wire-schema change (ruled out: would invalidate the canonical capture corpus).
- No `trajectory/` change. The shaped output remains one `ScalarNurbs<f64>` per axis per logical `ShapedSegment`.
- No slot-retirement work (parallel task — once retirement lands, the K× higher slot-allocation rate becomes harmless).
- No MCU evaluator change. Verified below: the existing engine already handles N consecutive segments with contiguous `[t_start, t_end]`.

**Binding numerical bounds.**
Wire `FieldValue::Buffer` length-prefix is `0..=255` bytes (`host_io/parser.rs:481`) → 63 f32 values per buffer field. For a degree-9 NURBS in piecewise-Bézier form (full-multiplicity interior knots), N pieces require `9N + 1` CPs and `9N + 10` knots. Knots is the binding cap: `9N + 10 ≤ 63 ⇒ N ≤ 5`. **The chunker uses `MAX_PIECES_PER_CHUNK = 5` for degree 9** (parameterize as `(63 - degree - 1) / degree` so `smooth_zv` lower-degree post-shape curves get correspondingly larger chunks). MCU caps (80 / 91) are looser and need no separate enforcement once the wire bound holds.

## 2. Chunking algorithm

**Site.** New file `rust/motion-bridge/src/curve_chunker.rs`. Reasons against the alternatives: trajectory-layer chunking leaks dispatch concerns into the math layer (and the spec forbids touching it); inlining into the dispatch closure pushes that closure past 200 lines of nested loops. A standalone module is unit-testable in isolation against synthetic NURBS, has no I/O dependencies, and exports one pure function.

**Boundary semantics — pick Option β** (each chunk is a `ScalarNurbs<f64>` reassembled from `[i*chunk_size .. (i+1)*chunk_size]` Bézier pieces via `bezier_pieces_to_nurbs`). Justification:

- The MCU evaluator (`runtime/src/engine.rs:400-485`) advances between segments via the SPSC queue's `dequeue`. On boundary, it sets `current.t_start = now.saturating_sub(delta_t)` from the next segment's record and re-evaluates from `u = 0`. Per-axis curves are looked up fresh each tick by handle. There is no cross-segment state in the curve domain — the only state that persists is `prev_x` / `prev_y` / `e_accumulator`, all of which are values in motor space, not curve coefficients. So as long as chunk_N+1's `X(0)` equals chunk_N's `X(1)` numerically, the integrated arc length stays continuous and the E-following math (`engine.rs:702-708`) does not glitch.
- Option β achieves that exactly because `extract_bezier_pieces` + `bezier_pieces_to_nurbs` round-trip preserves the boundary CP (the proof: `bezier_pieces_to_nurbs` skips the first Bernstein CP of every piece after the first; the boundary CP is shared by construction; that boundary value is the post-shape X(t_boundary) value computed in f64 in the trajectory layer, then truncated to f32 once when each side encodes its own buffer). The two adjacent f32 truncations of the same f64 value are bit-identical, so X is C⁰ across the boundary at f32 precision.
- Option α (sub-range knot windowing without piece reassembly) does not help — `extract_bezier_pieces` already does the knot-domain analysis we need, and reusing those pieces is cheaper than another knot-vector slicing pass.
- Option γ (knot overlap) is rejected: the MCU evaluator has no concept of cross-segment blending; it would just double-evaluate the overlap region.

**Per-piece extraction is unchanged**: call `nurbs::bezier::extract_bezier_pieces(curve)` once per axis-curve. This already does Boehm refinement to full multiplicity and converts Bernstein → Pascal-shifted monomial.

**Pseudocode** (the function the dispatch closure calls):

```rust
pub const fn max_pieces_per_chunk(degree: u8) -> usize {
    // Wire cap: 63 f32 per buffer field. Knots is the binding constraint.
    // num_knots(N pieces, degree d) = d*N + d + 1.
    // Solve d*N + d + 1 <= 63  =>  N <= (62 - d) / d.
    ((63 - 1 - degree as usize) / degree.max(1) as usize).max(1)
}

pub struct ChunkedAxisCurve {
    pub chunks: Vec<nurbs::ScalarNurbs<f64>>,  // each ≤ MAX_PIECES_PER_CHUNK pieces
    /// Per-chunk time-domain windows in the same units as the source NURBS knots.
    /// chunks[i] spans windows[i] = (u_start, u_end).
    pub windows: Vec<(f64, f64)>,
}

pub fn chunk_scalar_nurbs(curve: &nurbs::ScalarNurbs<f64>) -> ChunkedAxisCurve {
    let degree = curve.degree();
    let max_pieces = max_pieces_per_chunk(degree);
    let pieces = nurbs::bezier::extract_bezier_pieces(curve);

    if pieces.len() <= max_pieces {
        let u_start = curve.knots().first().copied().unwrap_or(0.0);
        let u_end = curve.knots().last().copied().unwrap_or(1.0);
        return ChunkedAxisCurve {
            chunks: vec![curve.clone()],
            windows: vec![(u_start, u_end)],
        };
    }

    let mut chunks = Vec::new();
    let mut windows = Vec::new();
    for batch in pieces.chunks(max_pieces) {
        let u_start = batch.first().unwrap().u_start;
        let u_end = batch.last().unwrap().u_end;
        let chunk = nurbs::bezier::bezier_pieces_to_nurbs(batch);
        chunks.push(chunk);
        windows.push((u_start, u_end));
    }
    ChunkedAxisCurve { chunks, windows }
}
```

`bezier_pieces_to_nurbs` builds a clamped knot vector with full-multiplicity interior breakpoints — exactly the format `CurveLoadParams::from_scalar_nurbs_normalized` already consumes.

## 3. Dispatch loop changes

Today (`bridge.rs:986-1113`) the closure runs:

```text
for plan in build_push_params(seg, ...):
    compute t_start_clock / t_end_clock for this MCU
    allocate segment_id
    for (axis_idx, curve_params) in plan.curves_to_load:
        slot = pool.try_alloc()
        load_curve(slot, curve_params)         // wait for response
        plan.set_handle(axis_idx, handle)
    pool.register_segment(*, plan.params.id)
    push_segment_fire_and_forget(plan.params)
```

The new structure adds an outer chunk loop and pre-chunks each axis curve before plan construction. Concretely:

1. **Pre-chunk per axis**, before `build_push_params` runs. Replace the call site with a wrapper that produces `ShapedSegment`-like views, one per chunk. The cleanest factoring: a new `dispatch::build_chunked_push_plans(seg, mcu_configs) -> Vec<ChunkedMcuPlan>` where:

    ```rust
    pub struct ChunkedMcuPlan {
        pub mcu_id: u32,
        pub kinematics: u8,
        pub e_mode: u8,
        pub extrusion_ratio: f32,
        pub chunks: Vec<McuChunkPlan>,
    }
    pub struct McuChunkPlan {
        pub curves_to_load: Vec<(usize, CurveLoadParams)>,
        pub t_start_s: f64,
        pub t_end_s: f64,
    }
    ```

2. **Aligning chunk boundaries across axes.** Each axis's `ScalarNurbs<f64>` is *the same shaped curve evaluated in different output dimensions*. Chunk per axis independently, then take the union of all per-axis chunk-boundary t-values across axes used by this MCU, and split every axis at every union boundary.

    ```rust
    let mut all_breaks: BTreeSet<NotNan<f64>> = BTreeSet::new();
    let mut per_axis_chunked = Vec::new();
    for axis_idx in cfg.axes {
        let curve = &shaped.axes[axis_idx];
        if is_trivially_constant(curve) { continue; }
        let chunked = chunk_scalar_nurbs(curve);
        for (_, u_end) in &chunked.windows[..chunked.windows.len()-1] {
            all_breaks.insert(NotNan::new(*u_end).unwrap());
        }
        per_axis_chunked.push((axis_idx, chunked));
    }
    // Re-split each axis at the union breakpoints.
    ```

    For the MVP single-axis-X-move and pure-Z-move paths, this collapses to the trivial case (one axis non-trivial, others constant) so the per-axis breakpoints *are* the union. Defensive but cheap.

3. **Dispatch loop, new shape:**

    ```text
    plans = build_chunked_push_plans(seg, mcu_configs)
    for plan in plans:                                 // one per MCU
        for chunk_idx in 0 .. plan.chunks.len():       // NEW outer loop
            chunk = &plan.chunks[chunk_idx]
            (t_start_clock, t_end_clock) = chunk_clocks(plan.mcu_id, seg, chunk)
            allocate segment_id for this chunk
            allocated_slots = []
            for (axis_idx, curve_params) in chunk.curves_to_load:
                slot = pool.try_alloc()  // on Err → release allocated_slots, abort
                allocated_slots.push(slot)
                handle = load_curve(slot, curve_params)
                    .map_err(|e| { for s in &allocated_slots { pool.release(*s) }; e })?
                params.set_handle(axis_idx, handle)
            for slot in &allocated_slots:
                pool.register_segment(*slot, params.id)
            push_segment_fire_and_forget(params)
                .map_err(|e| { for s in &allocated_slots { pool.release(*s) }; e })?
    ```

    Per-chunk allocate-load-push (not allocate-all-then-load-all-then-push-all). MCU starts executing chunk_0 the instant chunk_0's `push_segment` lands, overlapping with the host loading chunk_1.

4. **Per-chunk clock derivation.** Replace the `segment_clock_cache` keyed on `mcu_id` with per-`(mcu_id, chunk_idx)` derivation:

    ```rust
    let rel_start = ((seg.t_start + chunk.t_start_s) * freq).round().max(0.0) as u64;
    let rel_end   = ((seg.t_start + chunk.t_end_s)   * freq).round().max(0.0) as u64;
    ```

5. **Normalization is per-chunk too.** `CurveLoadParams::from_scalar_nurbs_normalized(chunk.nurbs, chunk.t_start_s + seg.t_start, chunk.t_end_s + seg.t_start)` — the normalization remaps each chunk's knot domain to `[0, 1]` independently.

## 4. Schema interactions

**`kalico_push_segment`.** Used unchanged, K times per logical move per MCU. No schema bump.

**Segment-id allocation rate.** K× faster (typical K = 4–6). 32-bit space at even 1 kHz push rate lasts >49 days. No change needed.

**`current_segment_id` / `retired_through_segment_id`.** The host's notion of "logical move" no longer maps 1:1 to segment_id. This is fine — host code never relied on that mapping; only telemetry surfaces it.

**Homing (`HomingState::mark_dispatched_segment`, `complete_if_retired`).** `mark_dispatched_segment` records the *latest* dispatched segment_id; `complete_if_retired` compares against `retired_through ≥ active_segment_id`. Latest-id-wins preserves correctness across chunks. **No change to homing required.**

**Telemetry / `dispatched_segments` counter.** Keep as logical-move counter. **The counter increment moves to *after* the inner chunk loop**, not per-chunk.

## 5. Failure handling

**Slot release on `load_curve` failure.** Track `allocated_slots` per chunk; release all on any error path before propagating.

**Mid-burst abort.** If `push_segment_fire_and_forget` for chunk_i fails after chunks 0..i-1 are in flight:
- Earlier chunks: do not release; let `kalico_credit_freed` retire them naturally.
- Chunk_i (push failed after register_segment): **call `pool.release(slot)` for every slot in `allocated_slots`** — defensive cleanup that today's single-push path also wants but doesn't currently do.
- Chunks i+1..K-1: never dispatched; planner returns Err, halts the printer. Better than executing garbage motion.

**Planner thread error propagation.** Already works via `error: Arc<Mutex<Option<PlannerError>>>` slot in `PlannerHandle`. No change.

**Slot exhaustion mid-burst.** Treat identically to load failure.

## 6. Testing

**Unit tests in `curve_chunker.rs`:**

1. `single_piece_curve_returns_one_chunk` — fast path.
2. `degree_9_27_piece_curve_chunks_into_six` — assert chunks.len() == 6 (5+5+5+5+5+2) and every chunk's encoded knot count ≤ 63.
3. `chunk_boundaries_are_c0_continuous` — exact f32 equality across boundary.
4. `chunk_breakpoints_match_source_piece_breakpoints` — no spurious break-points.
5. `axis_breakpoint_union_aligns_chunk_count` — two axes, different piece counts, aligned post-union.
6. `max_pieces_per_chunk_for_degree_9_is_5` — direct numerical assertion.

**Unit tests in `dispatch.rs`:**

7. `build_chunked_push_plans_single_chunk_matches_today` — single-piece linear-cubic move, byte-identical output to today.

**Unit tests in `slot_pool.rs`:**

8. `release_after_failed_push_does_not_leak` — alloc 5, register all, release each, free count returns to capacity.

**Integration tests in `motion-bridge/tests/sim_motion.rs`:**

9. `single_axis_x_move` (existing) — must still pass. Critical regression gate.
10. `multi_chunk_x_move` (new) — submit a move whose post-shape NURBS exceeds 5 Bézier pieces. Assert: K consecutive load_curves + K push_segments, K > 1.
11. `mid_burst_load_failure_releases_slots_and_aborts` (new) — inject a failing 3rd `load_curve`. Assert planner stores Dispatch error; slot pool free count returns; chunks 4..K never pushed.

## 7. Open questions

1. **Single-axis vs aligned-axis chunking under non-trivial Y motion.** Defaulting to the safe (general) implementation with breakpoint-union splitting. If trajectory layer guarantees X/Y share breakpoints, can simplify later.

2. **MCU SPSC queue depth (`Q_N - 1 = 7`).** With chunking, K segments per logical move means a single 5-chunk move uses 5 of the 7 slots. Confirm before landing: `CREDIT_SEED_CAPACITY ≤ Q_N - 1` (i.e. ≤ 7) so the SPSC queue is not the binding bottleneck.

3. **`compute_ack_clock` / lead time per chunk vs per move.** The 100 ms `lead_cycles` is the safety margin on the *first* chunk only. Subsequent chunks have `t_start_clock = previous chunk's t_end_clock` — no additional lead, by design. Confirmed from source-reading; add a one-line comment in the new dispatch code making this explicit.

4. **Trajectory-layer awareness of the 5-piece bound.** Out of scope per the brief, but worth flagging: trajectory has no current incentive to keep its post-shape piece count low. If shaped output gets pathological (e.g. >50 pieces per axis), we'll churn slots and stress retirement. A non-blocking follow-up could be a soft warning when chunk count exceeds, say, 8 per axis.

---

**Files referenced** (all absolute):
- `rust/motion-bridge/src/bridge.rs` (lines 982–1115 are the dispatch closure to be modified)
- `rust/motion-bridge/src/dispatch.rs` (extend with `build_chunked_push_plans`)
- `rust/motion-bridge/src/curve_chunker.rs` (new file)
- `rust/motion-bridge/src/slot_pool.rs` (no API change; one new test)
- `rust/motion-bridge/src/homing.rs` (verified unchanged)
- `rust/motion-bridge/src/planner.rs` (unchanged)
- `rust/motion-bridge/tests/sim_motion.rs` (regression gate; new tests)
- `rust/kalico-host-rt/src/producer.rs` (lines 187–211 — already takes a window)
- `rust/kalico-host-rt/src/host_io/parser.rs:481` (the 255-byte buffer cap)
- `rust/nurbs/src/bezier.rs` (lines 493–569 — `extract_bezier_pieces` / `bezier_pieces_to_nurbs`)
- `rust/runtime/src/curve_pool.rs:24-25` (MCU caps)
- `rust/runtime/src/engine.rs:400-485` (segment-transition logic)
- `rust/runtime/src/queue.rs:16` (Q_N=8 → 7 in-flight cap)
- `src/runtime_tick.c:482-516` (push_segment wire format — unchanged)
