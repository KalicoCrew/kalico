# Dispatch-Level Move Splitting Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Split oversized Bézier curve segments at dispatch time so moves that exceed the MCU's `max_pieces_per_curve` limit are chunked into multiple sub-segments instead of crashing.

**Architecture:** Time-domain splitting in `dispatch.rs`, invoked from the dispatch closure in `bridge.rs`. De Casteljau subdivision handles pieces that straddle chunk boundaries. The planner, shaper, and TOPP-RA are untouched — splitting operates on the already-built `McuPushPlan` output of `build_push_params()`.

**Tech Stack:** Rust, `motion-bridge` crate, `kalico-host-rt` crate. Tests via `cargo test -p motion-bridge`.

**Spec:** `docs/superpowers/specs/2026-05-23-dispatch-level-move-splitting-design.md`

**Build/test command:** `cd /Users/daniladergachev/Developer/kalico/rust && cargo test -p motion-bridge`

---

### Task 1: Bump producer backstop constant

**Files:**
- Modify: `rust/kalico-host-rt/src/producer.rs:29`

- [ ] **Step 1: Change `MAX_PIECES_PER_CURVE` from 96 to 255**

```rust
pub const MAX_PIECES_PER_CURVE: usize = 255;
```

Update the doc comment above it (lines 20-28) to reflect the new value and rationale:

```rust
/// Wire-format safety ceiling for `load_curve`'s piece_count argument.
/// The authoritative per-MCU cap is `RuntimeCapsResponse.max_pieces_per_curve`.
/// This guard short-circuits obviously-malformed uploads before they hit the
/// wire. Set to 255 (max u8) because `LoadCurveCubic.piece_count` is encoded
/// as `u8` on the wire — a value of 256 would overflow to 0.
/// Callers should validate against `caps.max_pieces_per_curve` (clamped to
/// 255 by the dispatch layer) rather than this constant.
pub const MAX_PIECES_PER_CURVE: usize = 255;
```

- [ ] **Step 2: Verify build**

Run: `cd /Users/daniladergachev/Developer/kalico/rust && cargo check -p kalico-host-rt`
Expected: compiles cleanly.

- [ ] **Step 3: Commit**

```bash
cd /Users/daniladergachev/Developer/kalico/rust && git add ../rust/kalico-host-rt/src/producer.rs && git commit -m "fix: bump MAX_PIECES_PER_CURVE from 96 to 255 (max u8 wire ceiling)"
```

---

### Task 2: De Casteljau subdivision helper

**Files:**
- Modify: `rust/motion-bridge/src/dispatch.rs`

- [ ] **Step 1: Write the test for de Casteljau split**

Add to the `#[cfg(test)] mod tests` block at the bottom of `dispatch.rs`:

```rust
#[test]
fn de_casteljau_split_at_midpoint() {
    // Linear curve b0=0, b1=1, b2=2, b3=3 split at t=0.5
    let bp: [f32; 4] = [0.0, 1.0, 2.0, 3.0];
    let (left, right) = super::de_casteljau_split(bp, 0.5);
    // At t=0.5 of a linear curve [0,1,2,3], eval = 1.5
    // Left half endpoints: start=0.0, end=1.5
    assert!((left[0] - 0.0).abs() < 1e-6);
    assert!((left[3] - 1.5).abs() < 1e-6);
    // Right half endpoints: start=1.5, end=3.0
    assert!((right[0] - 1.5).abs() < 1e-6);
    assert!((right[3] - 3.0).abs() < 1e-6);
    // Continuity: left end == right start
    assert!((left[3] - right[0]).abs() < 1e-6);
}

#[test]
fn de_casteljau_split_at_quarter() {
    // Quadratic-ish curve: [0, 0, 0, 12] (all acceleration at end)
    let bp: [f32; 4] = [0.0, 0.0, 0.0, 12.0];
    let (left, right) = super::de_casteljau_split(bp, 0.25);
    // Bernstein eval at t=0.25: 3*0*(1-t)^2*t + 3*0*(1-t)*t^2 + 12*t^3
    // = 12 * 0.015625 = 0.1875
    assert!((left[0] - 0.0).abs() < 1e-5, "left start");
    assert!((left[3] - 0.1875).abs() < 1e-4, "left end = eval(0.25) got {}", left[3]);
    assert!((right[0] - 0.1875).abs() < 1e-4, "right start");
    assert!((right[3] - 12.0).abs() < 1e-5, "right end");
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /Users/daniladergachev/Developer/kalico/rust && cargo test -p motion-bridge --lib -- dispatch::tests::de_casteljau_split 2>&1 | tail -5`
Expected: FAIL — `de_casteljau_split` not found.

- [ ] **Step 3: Implement de_casteljau_split**

Add above the `#[cfg(test)]` block in `dispatch.rs`:

```rust
/// De Casteljau subdivision of a cubic Bernstein polynomial at parameter `t`.
/// Returns `(left_half, right_half)` — two sets of cubic Bernstein control
/// points covering `[0, t]` and `[t, 1]` respectively.
pub fn de_casteljau_split(bp: [f32; 4], t: f32) -> ([f32; 4], [f32; 4]) {
    let [b0, b1, b2, b3] = bp;
    let p01 = b0 + t * (b1 - b0);
    let p12 = b1 + t * (b2 - b1);
    let p23 = b2 + t * (b3 - b2);
    let p012 = p01 + t * (p12 - p01);
    let p123 = p12 + t * (p23 - p12);
    let p0123 = p012 + t * (p123 - p012);
    ([b0, p01, p012, p0123], [p0123, p123, p23, b3])
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /Users/daniladergachev/Developer/kalico/rust && cargo test -p motion-bridge --lib -- dispatch::tests::de_casteljau_split 2>&1 | tail -5`
Expected: 2 tests PASS.

- [ ] **Step 5: Commit**

```bash
cd /Users/daniladergachev/Developer/kalico/rust && git add ../rust/motion-bridge/src/dispatch.rs && git commit -m "feat: add de_casteljau_split helper for cubic Bernstein subdivision"
```

---

### Task 3: Time-window extraction helper

**Files:**
- Modify: `rust/motion-bridge/src/dispatch.rs`

- [ ] **Step 1: Write the tests**

Add to `mod tests` in `dispatch.rs`:

```rust
fn make_curve(n_pieces: usize, piece_dur: f32, slope: f32) -> CurveLoadParams {
    let mut bp = Vec::with_capacity(n_pieces);
    let mut dur = Vec::with_capacity(n_pieces);
    for i in 0..n_pieces {
        let v0 = slope * i as f32;
        let v1 = slope * (i as f32 + 1.0 / 3.0);
        let v2 = slope * (i as f32 + 2.0 / 3.0);
        let v3 = slope * (i as f32 + 1.0);
        bp.push([v0, v1, v2, v3]);
        dur.push(piece_dur);
    }
    CurveLoadParams {
        bp_per_piece: bp,
        duration_per_piece: dur,
    }
}

#[test]
fn extract_time_window_full_range_is_identity() {
    let curve = make_curve(5, 0.1, 1.0);
    let result = super::extract_time_window(&curve, 0.0, 0.5);
    assert_eq!(result.piece_count(), 5);
    assert_eq!(result.bp_per_piece, curve.bp_per_piece);
}

#[test]
fn extract_time_window_first_half() {
    // 10 pieces, each 0.1s. Extract [0.0, 0.5) = first 5 pieces.
    let curve = make_curve(10, 0.1, 1.0);
    let result = super::extract_time_window(&curve, 0.0, 0.5);
    assert_eq!(result.piece_count(), 5);
    for i in 0..5 {
        assert_eq!(result.bp_per_piece[i], curve.bp_per_piece[i]);
    }
}

#[test]
fn extract_time_window_mid_piece_boundary_uses_de_casteljau() {
    // 4 pieces, each 1.0s. Extract [0.0, 2.5) — first 2 whole + half of 3rd.
    let curve = make_curve(4, 1.0, 1.0);
    let result = super::extract_time_window(&curve, 0.0, 2.5);
    assert_eq!(result.piece_count(), 3, "2 whole + 1 subdivided");
    // First two pieces unchanged
    assert_eq!(result.bp_per_piece[0], curve.bp_per_piece[0]);
    assert_eq!(result.bp_per_piece[1], curve.bp_per_piece[1]);
    // Third piece is left half of de Casteljau split at t=0.5
    assert!((result.duration_per_piece[2] - 0.5).abs() < 1e-5);
    // Start of subdivided piece matches start of original piece 2
    assert!((result.bp_per_piece[2][0] - curve.bp_per_piece[2][0]).abs() < 1e-5);
}

#[test]
fn extract_time_window_second_half_starts_mid_piece() {
    // 4 pieces, each 1.0s. Extract [2.5, 4.0) — right half of piece 2 + piece 3.
    let curve = make_curve(4, 1.0, 1.0);
    let result = super::extract_time_window(&curve, 2.5, 4.0);
    assert_eq!(result.piece_count(), 2, "1 subdivided + 1 whole");
    assert!((result.duration_per_piece[0] - 0.5).abs() < 1e-5);
    // Last piece unchanged
    assert_eq!(result.bp_per_piece[1], curve.bp_per_piece[3]);
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /Users/daniladergachev/Developer/kalico/rust && cargo test -p motion-bridge --lib -- dispatch::tests::extract_time_window 2>&1 | tail -5`
Expected: FAIL — `extract_time_window` not found.

- [ ] **Step 3: Implement extract_time_window**

Add to `dispatch.rs` above `#[cfg(test)]`:

```rust
/// Extract the sub-curve covering time window `[win_start, win_end]` (seconds
/// relative to curve start). Pieces entirely within the window are included
/// as-is. Pieces straddling a boundary are subdivided via de Casteljau.
pub fn extract_time_window(
    curve: &CurveLoadParams,
    win_start: f64,
    win_end: f64,
) -> CurveLoadParams {
    let mut result_bp = Vec::new();
    let mut result_dur = Vec::new();
    let mut elapsed = 0.0_f64;

    for i in 0..curve.bp_per_piece.len() {
        let d = curve.duration_per_piece[i] as f64;
        let piece_start = elapsed;
        let piece_end = elapsed + d;
        elapsed = piece_end;

        if piece_end <= win_start + 1e-12 || piece_start >= win_end - 1e-12 {
            continue;
        }

        if piece_start >= win_start - 1e-12 && piece_end <= win_end + 1e-12 {
            result_bp.push(curve.bp_per_piece[i]);
            result_dur.push(curve.duration_per_piece[i]);
            continue;
        }

        let mut cur_bp = curve.bp_per_piece[i];
        let mut cur_dur = d;
        let mut cur_start = piece_start;

        if cur_start < win_start - 1e-12 {
            let t = ((win_start - cur_start) / cur_dur) as f32;
            let (_, right) = de_casteljau_split(cur_bp, t);
            cur_bp = right;
            cur_dur *= 1.0 - t as f64;
            cur_start = win_start;
        }

        if cur_start + cur_dur > win_end + 1e-12 {
            let t = ((win_end - cur_start) / cur_dur) as f32;
            let (left, _) = de_casteljau_split(cur_bp, t);
            cur_bp = left;
            cur_dur *= t as f64;
        }

        result_bp.push(cur_bp);
        result_dur.push(cur_dur as f32);
    }

    CurveLoadParams {
        bp_per_piece: result_bp,
        duration_per_piece: result_dur,
    }
}
```

- [ ] **Step 4: Run tests**

Run: `cd /Users/daniladergachev/Developer/kalico/rust && cargo test -p motion-bridge --lib -- dispatch::tests::extract_time_window 2>&1 | tail -10`
Expected: 4 tests PASS.

- [ ] **Step 5: Commit**

```bash
cd /Users/daniladergachev/Developer/kalico/rust && git add ../rust/motion-bridge/src/dispatch.rs && git commit -m "feat: add extract_time_window helper for time-domain curve slicing"
```

---

### Task 4: split_plan_if_needed function

**Files:**
- Modify: `rust/motion-bridge/src/dispatch.rs`

- [ ] **Step 1: Write the tests**

Add to `mod tests` in `dispatch.rs`:

```rust
#[test]
fn split_plan_no_split_needed() {
    let curve = make_curve(5, 0.1, 1.0);
    let plan = McuPushPlan {
        mcu_id: 0,
        curves_to_load: vec![(AXIS_X, curve)],
        params: SegmentPushParams {
            id: 0, x_handle_packed: UNUSED_HANDLE, y_handle_packed: UNUSED_HANDLE,
            z_handle_packed: UNUSED_HANDLE, e_handle_packed: UNUSED_HANDLE,
            t_start: 1000, t_end: 2000, kinematics: 0, e_mode: 2, extrusion_ratio: 0.0,
        },
    };
    let result = super::split_plan_if_needed(plan, 10, 1_000_000.0).unwrap();
    assert_eq!(result.len(), 1, "no split needed");
    assert_eq!(result[0].curves_to_load[0].1.piece_count(), 5);
}

#[test]
fn split_plan_equal_axes_splits_at_piece_boundaries() {
    // 10 pieces per axis, max_pieces=4 → stride=2, expect 5 chunks
    let curve_x = make_curve(10, 0.1, 1.0);
    let curve_y = make_curve(10, 0.1, 2.0);
    let plan = McuPushPlan {
        mcu_id: 0,
        curves_to_load: vec![(AXIS_X, curve_x), (AXIS_Y, curve_y)],
        params: SegmentPushParams {
            id: 0, x_handle_packed: UNUSED_HANDLE, y_handle_packed: UNUSED_HANDLE,
            z_handle_packed: UNUSED_HANDLE, e_handle_packed: UNUSED_HANDLE,
            t_start: 0, t_end: 1_000_000, kinematics: 0, e_mode: 2, extrusion_ratio: 0.0,
        },
    };
    let result = super::split_plan_if_needed(plan, 4, 1_000_000.0).unwrap();
    assert_eq!(result.len(), 5);
    for chunk in &result {
        for (_, curve) in &chunk.curves_to_load {
            assert!(curve.piece_count() <= 4, "chunk has {} pieces", curve.piece_count());
        }
    }
    // Timing: first chunk starts at original t_start, last ends at original t_end
    assert_eq!(result[0].params.t_start, 0);
    assert_eq!(result[4].params.t_end, 1_000_000);
    // Timing continuity: each chunk's t_start == prev chunk's t_end
    for i in 1..result.len() {
        assert_eq!(result[i].params.t_start, result[i - 1].params.t_end);
    }
}

#[test]
fn split_plan_unequal_axes_uses_de_casteljau() {
    // X has 10 pieces (each 0.1s), Z has 3 pieces (each 0.333s).
    // max_pieces=4 → stride=2 → splits at X piece boundaries.
    // Z pieces straddle X boundaries → de Casteljau fires.
    let curve_x = make_curve(10, 0.1, 1.0);
    let curve_z = make_curve(3, 1.0 / 3.0, 5.0);
    let plan = McuPushPlan {
        mcu_id: 0,
        curves_to_load: vec![(AXIS_X, curve_x), (AXIS_Z, curve_z)],
        params: SegmentPushParams {
            id: 0, x_handle_packed: UNUSED_HANDLE, y_handle_packed: UNUSED_HANDLE,
            z_handle_packed: UNUSED_HANDLE, e_handle_packed: UNUSED_HANDLE,
            t_start: 0, t_end: 1_000_000, kinematics: 2, e_mode: 2, extrusion_ratio: 0.0,
        },
    };
    let result = super::split_plan_if_needed(plan, 4, 1_000_000.0).unwrap();
    assert!(result.len() >= 3, "should produce multiple chunks");
    for chunk in &result {
        for (_, curve) in &chunk.curves_to_load {
            assert!(curve.piece_count() <= 4,
                "axis piece count {} exceeds max 4", curve.piece_count());
        }
        assert_eq!(chunk.curves_to_load.len(), 2, "both axes in every chunk");
    }
}

#[test]
fn split_plan_preserves_e_mode_and_extrusion_ratio() {
    let curve = make_curve(10, 0.1, 1.0);
    let plan = McuPushPlan {
        mcu_id: 7,
        curves_to_load: vec![(AXIS_X, curve)],
        params: SegmentPushParams {
            id: 0, x_handle_packed: UNUSED_HANDLE, y_handle_packed: UNUSED_HANDLE,
            z_handle_packed: UNUSED_HANDLE, e_handle_packed: UNUSED_HANDLE,
            t_start: 0, t_end: 1_000_000, kinematics: 0, e_mode: 1, extrusion_ratio: 0.042,
        },
    };
    let result = super::split_plan_if_needed(plan, 4, 1_000_000.0).unwrap();
    for chunk in &result {
        assert_eq!(chunk.params.e_mode, 1);
        assert!((chunk.params.extrusion_ratio - 0.042).abs() < 1e-6);
        assert_eq!(chunk.params.kinematics, 0);
        assert_eq!(chunk.mcu_id, 7);
    }
}

#[test]
fn split_plan_cap_below_3_errors_only_when_splitting_needed() {
    // 2 pieces, cap=2 → no split needed → should succeed
    let small = make_curve(2, 0.1, 1.0);
    let plan_ok = McuPushPlan {
        mcu_id: 0,
        curves_to_load: vec![(AXIS_X, small)],
        params: SegmentPushParams {
            id: 0, x_handle_packed: UNUSED_HANDLE, y_handle_packed: UNUSED_HANDLE,
            z_handle_packed: UNUSED_HANDLE, e_handle_packed: UNUSED_HANDLE,
            t_start: 0, t_end: 1000, kinematics: 0, e_mode: 2, extrusion_ratio: 0.0,
        },
    };
    assert!(super::split_plan_if_needed(plan_ok, 2, 1e6).is_ok());

    // 5 pieces, cap=2 → split needed, cap too low → error
    let big = make_curve(5, 0.1, 1.0);
    let plan_err = McuPushPlan {
        mcu_id: 0,
        curves_to_load: vec![(AXIS_X, big)],
        params: SegmentPushParams {
            id: 0, x_handle_packed: UNUSED_HANDLE, y_handle_packed: UNUSED_HANDLE,
            z_handle_packed: UNUSED_HANDLE, e_handle_packed: UNUSED_HANDLE,
            t_start: 0, t_end: 1000, kinematics: 0, e_mode: 2, extrusion_ratio: 0.0,
        },
    };
    assert!(super::split_plan_if_needed(plan_err, 2, 1e6).is_err());
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /Users/daniladergachev/Developer/kalico/rust && cargo test -p motion-bridge --lib -- dispatch::tests::split_plan 2>&1 | tail -5`
Expected: FAIL — `split_plan_if_needed` not found.

- [ ] **Step 3: Implement split_plan_if_needed**

Add to `dispatch.rs` above `#[cfg(test)]`:

```rust
use crate::planner::DispatchError;

/// Split an `McuPushPlan` into sub-plans where every axis has
/// `≤ max_pieces` pieces. Returns the plan unchanged if no axis exceeds
/// the limit. Uses time-domain splitting with de Casteljau subdivision
/// for pieces straddling chunk boundaries.
pub fn split_plan_if_needed(
    plan: McuPushPlan,
    max_pieces: usize,
    freq: f64,
) -> Result<Vec<McuPushPlan>, DispatchError> {
    split_recursive(plan, max_pieces, freq, 0)
}

fn split_recursive(
    plan: McuPushPlan,
    max_pieces: usize,
    freq: f64,
    depth: usize,
) -> Result<Vec<McuPushPlan>, DispatchError> {
    let max_pc = plan
        .curves_to_load
        .iter()
        .map(|(_, c)| c.piece_count())
        .max()
        .unwrap_or(0);

    if max_pc <= max_pieces {
        return Ok(vec![plan]);
    }

    if max_pieces < 3 {
        return Err(DispatchError::CapsExceeded {
            mcu_id: plan.mcu_id,
            pieces: max_pc,
            max_pieces,
        });
    }

    if depth > 8 {
        return Err(DispatchError::CapsExceeded {
            mcu_id: plan.mcu_id,
            pieces: max_pc,
            max_pieces,
        });
    }

    let stride = max_pieces - 2;

    let bottleneck_idx = plan
        .curves_to_load
        .iter()
        .enumerate()
        .max_by_key(|(_, (_, c))| c.piece_count())
        .map(|(i, _)| i)
        .unwrap();
    let bottleneck = &plan.curves_to_load[bottleneck_idx].1;

    let mut split_times = vec![0.0_f64];
    let mut elapsed = 0.0_f64;
    for (i, d) in bottleneck.duration_per_piece.iter().enumerate() {
        elapsed += *d as f64;
        if (i + 1) % stride == 0 && i + 1 < bottleneck.piece_count() {
            split_times.push(elapsed);
        }
    }
    split_times.push(elapsed);

    let t_start_clock = plan.params.t_start;
    let t_end_clock = plan.params.t_end;
    let n_chunks = split_times.len() - 1;
    let mut chunks = Vec::with_capacity(n_chunks);
    let mut chunk_start_clock = t_start_clock;

    for w in 0..n_chunks {
        let win_start = split_times[w];
        let win_end = split_times[w + 1];

        let chunk_end_clock = if w == n_chunks - 1 {
            t_end_clock
        } else {
            let dur_clocks = (win_end - win_start) * freq;
            chunk_start_clock + dur_clocks.round() as u64
        };

        let sub_curves: Vec<(usize, CurveLoadParams)> = plan
            .curves_to_load
            .iter()
            .map(|(axis_idx, curve)| (*axis_idx, extract_time_window(curve, win_start, win_end)))
            .collect();

        let mut sub_params = plan.params;
        sub_params.t_start = chunk_start_clock;
        sub_params.t_end = chunk_end_clock;
        sub_params.id = 0;
        sub_params.x_handle_packed = UNUSED_HANDLE;
        sub_params.y_handle_packed = UNUSED_HANDLE;
        sub_params.z_handle_packed = UNUSED_HANDLE;
        sub_params.e_handle_packed = UNUSED_HANDLE;

        chunks.push(McuPushPlan {
            mcu_id: plan.mcu_id,
            curves_to_load: sub_curves,
            params: sub_params,
        });

        chunk_start_clock = chunk_end_clock;
    }

    let mut result = Vec::new();
    for chunk in chunks {
        let sub = split_recursive(chunk, max_pieces, freq, depth + 1)?;
        result.extend(sub);
    }

    Ok(result)
}
```

- [ ] **Step 4: Run tests**

Run: `cd /Users/daniladergachev/Developer/kalico/rust && cargo test -p motion-bridge --lib -- dispatch::tests::split_plan 2>&1 | tail -15`
Expected: 5 tests PASS.

- [ ] **Step 5: Run all dispatch tests to check for regressions**

Run: `cd /Users/daniladergachev/Developer/kalico/rust && cargo test -p motion-bridge --lib -- dispatch::tests 2>&1 | tail -10`
Expected: all tests PASS.

- [ ] **Step 6: Commit**

```bash
cd /Users/daniladergachev/Developer/kalico/rust && git add ../rust/motion-bridge/src/dispatch.rs && git commit -m "feat: add split_plan_if_needed with time-domain splitting and recursive validation"
```

---

### Task 5: Integrate splitting into the dispatch closure

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs:2180-2402`

This is the most delicate task. The dispatch closure in `bridge.rs` needs three changes:
1. Remove the `CapsExceeded` pre-dispatch check (lines 2184-2210)
2. After timing is computed (line 2392), split the plan and iterate over sub-plans
3. Pre-allocate segment IDs for the split, mark only the last for homing
4. Clamp effective max_pieces to `min(caps.max_pieces_per_curve, 255)`

- [ ] **Step 1: Remove the CapsExceeded pre-dispatch check**

Delete lines 2180-2210 (the `for plan in &mcu_plans { ... caps check ... }` block). The entire block from the comment `// Cap-check each curve against the destination MCU's caps` through the closing `}` of that for loop.

- [ ] **Step 2: Add caps lookup + split after timing computation**

After `plan.params.t_end = t_end_clock;` (line 2392), add the splitting logic. Replace the existing segment-ID allocation block (lines 2394-2402) and the dispatch loop body with a sub-plan iteration loop.

Find the block that starts at line 2391 (`plan.params.t_end = t_end_clock;`) and replace everything from line 2394 through line 2563 (the end of the push_segment error handling) with:

```rust
                    // --- Move splitting ---
                    let caps = mcu_configs_for_cb
                        .iter()
                        .find(|c| c.mcu_id == plan.mcu_id)
                        .map(|c| c.caps)
                        .unwrap_or_default();
                    let effective_max_pieces =
                        (caps.max_pieces_per_curve as usize).min(255);
                    if caps.max_pieces_per_curve > 255 {
                        log::warn!(
                            "MCU {} reports max_pieces_per_curve={}, clamping to 255 (u8 wire ceiling)",
                            plan.mcu_id, caps.max_pieces_per_curve,
                        );
                    }

                    let sub_plans = dispatch::split_plan_if_needed(
                        plan, effective_max_pieces, freq,
                    )?;

                    let n_sub = sub_plans.len();
                    if n_sub > 1 {
                        log::info!(
                            "[bridge-trace] split mcu={}: {} sub-segments (max_pieces={})",
                            sub_plans[0].mcu_id, n_sub, effective_max_pieces,
                        );
                    }

                    // Pre-allocate segment IDs for homing correctness:
                    // mark_dispatched_segment overwrites on each call, so we
                    // must register only the LAST sub-segment's ID.
                    let pre_alloc_ids: Vec<u32> = {
                        let mut ids = next_seg_id.lock().unwrap_or_else(|p| p.into_inner());
                        let entry = ids.entry(sub_plans[0].mcu_id).or_insert(1);
                        let first = *entry;
                        *entry = entry.wrapping_add(n_sub as u32);
                        (0..n_sub as u32).map(|i| first.wrapping_add(i)).collect()
                    };
                    // Mark only the last sub-segment for homing completion
                    homing.mark_dispatched_segment(*pre_alloc_ids.last().unwrap());

                    for (sub_idx, mut sub_plan) in sub_plans.into_iter().enumerate() {
                        sub_plan.params.id = pre_alloc_ids[sub_idx];

                        // Allocate slots, load curves
                        let mut allocated_slots: Vec<u16> =
                            Vec::with_capacity(sub_plan.curves_to_load.len());
                        let mut seg_err: Option<DispatchError> = None;
                        for i in 0..sub_plan.curves_to_load.len() {
                            let axis_idx = sub_plan.curves_to_load[i].0;
                            let curve_params = sub_plan.curves_to_load[i].1.clone();
                            let alloc_result = {
                                let mut pool =
                                    slot_pool.lock().unwrap_or_else(|p| p.into_inner());
                                let cap = pool.capacity();
                                let in_flight = pool.in_flight_count();
                                pool.try_alloc()
                                    .ok_or(DispatchError::SlotPoolExhausted {
                                        mcu_id: sub_plan.mcu_id,
                                        capacity: cap,
                                        in_flight,
                                    })
                            };
                            let (slot, slot_gen) = match alloc_result {
                                Ok(v) => v,
                                Err(e) => {
                                    seg_err = Some(e);
                                    break;
                                }
                            };
                            log::debug!(
                                "[slot-trace] try_alloc mcu={} seg_id={} axis={} slot={} gen={}",
                                sub_plan.mcu_id, sub_plan.params.id, axis_idx, slot, slot_gen,
                            );
                            allocated_slots.push(slot);
                            match producer::load_curve(
                                io.as_ref(),
                                slot,
                                axis_idx as u8,
                                &curve_params,
                                producer::DEFAULT_LOAD_CURVE_TIMEOUT,
                            ) {
                                Ok(handle) => {
                                    sub_plan.set_handle(axis_idx, handle);
                                }
                                Err(e) => {
                                    seg_err = Some(DispatchError::LoadCurve {
                                        mcu_id: sub_plan.mcu_id,
                                        slot,
                                        seg_id: sub_plan.params.id,
                                        axis: axis_idx,
                                        host_gen: slot_gen,
                                        detail: e.to_string(),
                                    });
                                    break;
                                }
                            }
                        }

                        if let Some(err) = seg_err {
                            let mut pool =
                                slot_pool.lock().unwrap_or_else(|p| p.into_inner());
                            for s in &allocated_slots {
                                pool.release(*s);
                            }
                            return Err(err);
                        }

                        // Register slots BEFORE push (slot_pool.rs:126 requirement)
                        {
                            let mut pool =
                                slot_pool.lock().unwrap_or_else(|p| p.into_inner());
                            for slot in &allocated_slots {
                                pool.register_segment(*slot, sub_plan.params.id);
                            }
                        }

                        let push_result =
                            dispatch_push_segment(io.as_ref(), credit, &sub_plan.params);
                        match &push_result {
                            Ok(info) => log::info!(
                                "[bridge-trace] push_segment ok: mcu={} seg_id={} sub={}/{} accepted_id={}",
                                sub_plan.mcu_id, sub_plan.params.id, sub_idx + 1, n_sub,
                                info.accepted_segment_id,
                            ),
                            Err(e) => log::info!(
                                "[bridge-trace] push_segment err: mcu={} seg_id={} sub={}/{} err={:?}",
                                sub_plan.mcu_id, sub_plan.params.id, sub_idx + 1, n_sub, e,
                            ),
                        }
                        if let Err(e) = push_result {
                            let mut pool =
                                slot_pool.lock().unwrap_or_else(|p| p.into_inner());
                            for s in &allocated_slots {
                                pool.release(*s);
                            }
                            return Err(DispatchError::PushSegment {
                                mcu_id: sub_plan.mcu_id,
                                detail: e.to_string(),
                            });
                        }
                    } // end sub_plan loop
```

Also remove the diagnostic `pos_at` closure and its log (lines 2500-2534) — it references `seg.axes` which won't be available inside the sub-plan loop. This is diagnostic-only and can be restored later if needed.

- [ ] **Step 3: Add `use` for `dispatch` module**

At the top of the dispatch closure (or in the file's `use` block), ensure `dispatch::split_plan_if_needed` is importable. The dispatch closure is in `bridge.rs` which already imports from `dispatch`. If `split_plan_if_needed` is `pub`, add:

```rust
use crate::dispatch;
```

If `dispatch` is already imported, this step is a no-op. Check by searching for `use crate::dispatch` in `bridge.rs`.

- [ ] **Step 4: Verify it compiles**

Run: `cd /Users/daniladergachev/Developer/kalico/rust && cargo check -p motion-bridge 2>&1 | tail -10`
Expected: compiles cleanly. Fix any borrow/lifetime issues.

- [ ] **Step 5: Run all tests**

Run: `cd /Users/daniladergachev/Developer/kalico/rust && cargo test -p motion-bridge 2>&1 | tail -15`
Expected: all tests PASS.

- [ ] **Step 6: Commit**

```bash
cd /Users/daniladergachev/Developer/kalico/rust && git add ../rust/motion-bridge/src/bridge.rs && git commit -m "feat: replace CapsExceeded error with dispatch-level move splitting"
```

---

### Task 6: Verify the full change works end-to-end

**Files:** none — verification only.

- [ ] **Step 1: Run the full test suite**

Run: `cd /Users/daniladergachev/Developer/kalico/rust && cargo test 2>&1 | tail -20`
Expected: all workspace tests PASS.

- [ ] **Step 2: Verify the build for the Python extension**

Run: `cd /Users/daniladergachev/Developer/kalico/rust && cargo check -p motion-bridge --features pyo3 2>&1 | tail -5`
Expected: compiles cleanly. (The `pyo3` feature may not exist — if so, just `cargo check -p motion-bridge` is sufficient.)

- [ ] **Step 3: Review the diff**

Run: `cd /Users/daniladergachev/Developer/kalico && git diff --stat HEAD~4`
Expected: changes only in:
- `rust/kalico-host-rt/src/producer.rs` (constant bump)
- `rust/motion-bridge/src/dispatch.rs` (splitting helpers + tests)
- `rust/motion-bridge/src/bridge.rs` (dispatch closure integration)

---

## Summary of changes

| File | Change |
|------|--------|
| `rust/kalico-host-rt/src/producer.rs` | `MAX_PIECES_PER_CURVE`: 96 → 255 |
| `rust/motion-bridge/src/dispatch.rs` | Add `de_casteljau_split`, `extract_time_window`, `split_plan_if_needed` + tests |
| `rust/motion-bridge/src/bridge.rs` | Remove `CapsExceeded` check, add split+iterate with pre-allocated segment IDs and homing terminal marking, clamp caps to 255 |
