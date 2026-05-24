# Dispatch-Level Move Splitting

## Problem

Long moves produce more Bézier pieces than the MCU's `max_pieces_per_curve` limit (128 on H723). The host-side producer also has a stale backstop constant (`MAX_PIECES_PER_CURVE = 96`) that rejects curves the MCU could actually handle. Both limits cause `RuntimeError` crashes on moves that generate too many pieces.

Piece count is driven by move **duration**, not distance — the shaper discretizes at ~63ms/piece. A 140mm diagonal at 25mm/s (~8s) produces ~119 pieces; a 342mm Z probe at 8mm/s (~43s) produces ~672.

## Design

### Approach: Dispatch-level curve chunking

When the dispatch closure encounters a plan where any axis curve exceeds `max_pieces_per_curve`, it splits the plan into sub-plans — each carrying ≤ `max_pieces` pieces per axis — and emits each as its own `load_curve` + `push_segment` pair.

### Where in the pipeline

In the dispatch closure (`bridge.rs`), after timing is computed (`t_start_clock`, `t_end_clock`) but before segment ID allocation, slot allocation, and `load_curve` calls. Replace the `CapsExceeded` error with a `split_plan_if_needed()` call, then iterate over the resulting sub-plans.

### Time-domain splitting

Axes in a single `McuPushPlan` can have different piece counts. CoreXY motor curves share a knot union and always have equal piece counts, but a Z axis on the same MCU (or any non-CoreXY config) may have a different piece count from its shaping/passthrough knot structure. The MCU runtime (`engine.rs:724-748`) faults with `PieceAdvanceUnderflow` if one axis exhausts before another, so all axes within a sub-segment must cover the same time window with no axis omitted.

Splitting is therefore based on **time boundaries**, not piece indices:

1. Identify the **bottleneck axis** — the axis with the maximum piece count `N_max` in the plan.
2. Determine split times from the bottleneck axis's piece boundaries, every `max_pieces - 2` pieces. The `-2` headroom reserves capacity for up to 2 boundary-straddling pieces that de Casteljau subdivision may add to non-bottleneck axes (one at each chunk boundary).
3. For each time window between consecutive split times, extract each axis's pieces:
   - Pieces that fall entirely within the window → include as-is.
   - Pieces that straddle a window boundary → subdivide using de Casteljau at the boundary parameter, include the appropriate half.
4. Each chunk becomes a sub-plan with its own timing, segment ID, and per-axis curves.
5. **Post-split validation**: after splitting, verify every axis in every chunk has `piece_count() ≤ max_pieces` (runtime check, returns `DispatchError` on violation). If any chunk fails, recursively re-split that chunk at its own bottleneck axis's boundaries until all chunks pass. In the current architecture, piece density is uniform across axes (~63ms/piece from the shaper), so the initial split based on the global bottleneck is sufficient; recursive re-splitting is a safety net for pathological piece distributions.

**Minimum cap requirement**: `effective_max_pieces ≥ 3` is required only when splitting is actually needed (a curve exceeds the cap). Firmware caps of 1 or 2 are wire-valid and runtime-valid for moves that don't need splitting; the check gates the split path, not the general dispatch.

De Casteljau subdivision on cubic Bernstein control points `[b0, b1, b2, b3]` at parameter `t`:
- Left half: `[b0, lerp(b0,b1,t), lerp(lerp(b0,b1,t), lerp(b1,b2,t), t), eval(t)]`
- Right half: `[eval(t), lerp(lerp(b1,b2,t), lerp(b2,b3,t), t), lerp(b2,b3,t), b3]`
- Duration splits proportionally: left = `t × d`, right = `(1-t) × d`.

`CurveLoadParams` stores per-piece Bernstein control points (`bp_per_piece: Vec<[f32; 4]>`) and per-piece durations (`duration_per_piece: Vec<f32>`). Each sub-plan gets a new `CurveLoadParams` built from the sliced + subdivided pieces.

Timing precision: accumulate `duration_per_piece` sums in `f64` (the values are `f32`) to avoid precision loss over many pieces. The last sub-plan's `t_end_clock` snaps to the original segment's `t_end_clock` rather than computing from durations, eliminating cumulative rounding drift.

### Per sub-plan dispatch

**Segment ID pre-allocation**: before dispatching any sub-plan, pre-allocate ALL segment IDs for the split (one per sub-plan, monotonic from the per-MCU counter). This is required for correct homing lifecycle: `mark_dispatched_segment` overwrites the stored active segment ID on each call (homing.rs:70-75), and `complete_if_retired` fires when `retired_through >= active_segment_id` (homing.rs:77-86). If IDs were allocated on-the-fly and slot allocation blocked mid-split, earlier sub-segments could retire and trigger premature homing completion. Pre-allocating and marking only the **last** sub-segment's ID as the homing terminal prevents this race.

Each sub-plan then follows the existing dispatch flow:
1. Set segment ID from pre-allocated pool
2. For the last sub-plan only: `homing.mark_dispatched_segment(seg_id)`
3. Allocate slots from pool (one per axis), load curves, set handles
4. Register slots to segment ID for retirement (**before** push — `SlotPool` requires registration before `push_segment` so that `retire_through_segment()` can find them; see `slot_pool.rs:126`)
5. `push_segment` with the sub-plan's timing window and handles
6. On push failure: the error is **fatal for the split** — the dispatch closure returns `DispatchError::PushSegment`, which propagates to the planner error latch and surfaces on the next Python call. The existing `pool.release()` cleanup (bridge.rs:2552-2563) runs for the failed sub-plan's slots. Curves already loaded on the MCU for prior sub-plans are orphaned until the firmware restart that the error triggers. No partial recovery is attempted — this matches the current unsplit behavior.

### Cross-MCU split boundaries

Split boundaries are **per-MCU**, not globally unioned. Each MCU independently splits based on its own `caps.max_pieces_per_curve` and its own bottleneck axis. Different MCUs may produce different numbers of sub-segments for the same original `ShapedSegment`. This is correct: each MCU plays the same total time window `[t_start, t_end]` regardless of sub-segment count. Per-MCU segment ID counters are already independent. Homing, wait/barrier, and retirement semantics are all per-MCU.

### Extrusion semantics across sub-segments

Each sub-plan copies `e_mode` and `extrusion_ratio` from the original plan. The MCU resets `ds_xy_segment` at each segment arm and rolls E forward at retirement (`engine.rs:768`). Split sub-segments accumulate `ds_xy` independently and roll forward independently — the total E across all sub-segments = `extrusion_ratio × total_ds_xy`, identical to the unsplit case. When independent E curves exist in future, they are just another axis in `curves_to_load` and receive the same time-domain split treatment.

### Slot pool limits and retirement dependency

**Current state**: slot retirement via `on_credit_freed` → `SlotPool::retire_through_segment` is functionally unreachable at runtime (`slot_pool.rs:37-45`). The `EventDispatcher` that lifts `kalico_credit_freed` events from the serial path is not yet wired up (Task 10 dependency). The current bridge works only because short moves never exhaust the pool.

**Implication for splitting**: until Task 10 lands, the split depth is hard-limited by the pool capacity. With `CURVE_POOL_N = 16` and 2 axes per segment (CoreXY motor curves), at most 8 sub-segments can be in-flight. This covers the 119-piece crash (2 sub-segments) but not the 672-piece homing crash (6 sub-segments × 2 axes = 12 slots). The 672-piece homing move additionally requires retirement to work.

**Post-Task-10**: `on_credit_freed` is called from the Python reactor thread (klippy event loop), which runs concurrently with the planner thread. Both access `SlotPool` through `Arc<Mutex<SlotPool>>` (bridge.rs:1977). Once wired, the retry loop works: the planner thread spins on `try_alloc()` while the reactor thread processes `kalico_credit_freed` events and calls `retire_through_segment()` to free slots.

**Retry loop** (post-Task-10): replace bare `try_alloc()` with a retry loop that sleeps 1ms between attempts with a **60-second** timeout (matching the existing credit-acquire timeout in `push_segment_with_timeout`). If the timeout expires, return `SlotPoolExhausted`.

**Pre-Task-10 behavior**: if the split requires more slots than the pool holds, the first `try_alloc()` failure returns `SlotPoolExhausted` immediately (no retry, since retirement can't fire). Partial dispatch: any already-pushed sub-segments execute correctly on the MCU; the remaining sub-segments are lost, and the error propagates to Python. This is no worse than the current `CapsExceeded` crash — instead of crashing on the whole move, the first few sub-segments execute and then the error fires.

### Producer backstop and wire-format safety

Bump `producer.rs::MAX_PIECES_PER_CURVE` from 96 to **255** (max `u8`). The wire format encodes `piece_count` as `u8` (`LoadCurveCubic.piece_count`, producer.rs:320 casts `piece_count as u8`). A value of 256 overflows to 0.

The dispatch clamps the effective split limit to `min(caps.max_pieces_per_curve, 255)` when reading the MCU-reported cap. If `caps.max_pieces_per_curve > 255`, log a warning about the wire-format ceiling. This handles the current `src/Kconfig:417` range of `1 256` without requiring a firmware rebuild — the host silently caps at 255.

### What doesn't change

- Planner, shaper, TOPP-RA — untouched
- `build_push_params()` — still produces one McuPushPlan per MCU per segment
- Homing state machine — sub-segments are normal segments; the only change is which segment ID is registered as the homing terminal
- Credit/retirement semantics — `retired_through_segment_id` is monotonic, sequential sub-segments retire in order

## Files to modify

1. `rust/motion-bridge/src/dispatch.rs` — add `split_plan_if_needed()` with time-domain splitting, de Casteljau subdivision, post-split validation with recursive re-split fallback, and minimum-cap check
2. `rust/motion-bridge/src/bridge.rs` — replace `CapsExceeded` error with split + iterate; remove the pre-dispatch cap check; pre-allocate segment IDs for split and mark only the last for homing; clamp effective max_pieces to `min(caps, 255)`; add retry loop for slot allocation (post-Task-10, immediate failure pre-Task-10)
3. `rust/kalico-host-rt/src/producer.rs` — bump `MAX_PIECES_PER_CURVE` from 96 to 255

## Tests

- Splitting with equal piece counts across axes (CoreXY motor curves) — should degenerate to piece-boundary slicing with no de Casteljau
- Splitting with unequal piece counts (e.g. XY at 119 + Z at 80) — verify all chunks ≤ max_pieces, time windows consistent, de Casteljau correct at boundaries
- Recursive re-split: synthetic case with clustered non-bottleneck density — verify convergence
- Homing terminal ID: verify homing completes only when the last sub-segment retires, not earlier ones
- Slot pool exhaustion (pre-Task-10): verify immediate `SlotPoolExhausted` when pool is full
- Slot pool exhaustion (post-Task-10): verify retry loop waits for retirement and succeeds
- Piece count at u8 boundary (255 pieces) — verify wire encoding correctness
- MCU reporting max_pieces > 255 — verify host clamps to 255 with warning
- MCU reporting max_pieces < 3 with oversized curve — verify clear error
- MCU reporting max_pieces < 3 with fitting curve — verify no error (no split needed)
- E semantics: verify extrusion_ratio and e_mode propagated to all sub-plans
- Cross-MCU: verify different MCUs can produce different sub-segment counts for the same shaped segment
