# Layer 1 — G5 / G5.1 Reduction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close the remaining gap in Layer 1 build-order Step 2: reduce + pipeline construct exact non-rational NURBS for G5 (degree-3, 4 control points) and G5.1 (degree-2, 3 control points) per the LinuxCNC RS274NGC convention, with the modal-chain implicit-tangent rule for G5 and active-plane (G17/G18/G19) tracking for G5.1.

**Architecture:** Refactor the existing `ReduceEvent::G1Move` and `ReduceEvent::Arc` arms onto a new `ReduceEvent::Curve(CurveGeom, common…)` shape (fixed-size-array variants for `Linear` / `Quadratic` / `RationalQuadratic` / `Cubic`); add `Curve(CurveGeom::Cubic …)` for G5 and `Curve(CurveGeom::Quadratic …)` for G5.1. ModalState gains `prev_g5_pq: Option<[f64; 2]>` (RS274NGC modal chain) and `active_plane: Plane` (XY/XZ/YZ tracked by G17/G18/G19; default G17). Pipeline builds the corresponding `VectorNurbs<f64, 3>` and emits `Segment::Fitted { degree: 3 | 2, max_residual_mm: 0.0 }`. Both G5 and G5.1 break the G1-tangent chain (no JD against their endpoints) — Layer 2 derives endpoint curvature directly from the NURBS per CLAUDE.md's curvature-continuity principle.

**Tech Stack:** Rust 2024 (`MSRV 1.85`), `nurbs` workspace crate (Layer 0), `gcode` workspace crate (Layer 1 lexer), `cargo test` for unit/integration tests. No new external dependencies.

---

## Build-order context and decisions log

This plan implements **build-order Step 3** from `CLAUDE.md`:

> *G5 / G5.1 reduction — closes the remaining gap in step 2. Lexer already tokenizes G5/G5.1 (Task 6 of the Phase 1 plan). Per LinuxCNC RS274NGC convention: G5 → degree-3 single-piece NURBS with 4 control points (P0=current, P1=current+I,J, P2=end+P,Q, P3=end); G5.1 → degree-2 single-piece NURBS with 3 control points (P0=current, P1=current+I,J, P2=end), restricted to the active plane (G17/G18/G19). Implement the RS274NGC modal-chain implicit-tangent rule for G5 […] Both G5 and G5.1 break the G1-tangent chain — Layer 2 derives endpoint curvature from the NURBS per the curvature-continuity principle.*

### Decisions settled during brainstorming (round 1, 2026-04-27)

| Decision | Source | Rationale |
|---|---|---|
| Adopt LinuxCNC G5/G5.1 semantics. G5.1 is a **degree-2 non-rational NURBS** (3 CPs), not a degenerate cubic. | `[DIRECTION]` Q1 + research on Marlin / RRF / grblHAL / Fanuc | Marlin doesn't implement G5.1; RRF doesn't implement G5/G5.1; grblHAL matches LinuxCNC; Fanuc's `G05.1 Q1` is an unrelated AICC mode toggle on a colliding number. LinuxCNC is the only meaningful spec in the open-source space. |
| Implement the RS274NGC G5 modal-chain implicit-tangent rule (`prev_g5_pq` carried in modal state; absent I,J on consecutive G5 → `−prev_pq`). Strict error if chain is broken and I,J still missing. Single I or single J also an error. | `[DIRECTION]` Q2 + RS274NGC §3.5.5 | Professional CAM output assumes the rule. Cost is one `Option<[f64; 2]>` in `ModalState`. Strict error on broken chain rather than silent default — fabricating tangents masks bugs. |
| G5 and G5.1 break the G1-tangent chain (`prev_g1_dir = None` after emission). No JD between G5/G5.1 endpoint and a following G1. | `[DIRECTION]` Q3 + CLAUDE.md curvature-continuity principle (added 2026-04-27) | Junction velocity at any boundary derives from curvature continuity, not per-source-g-code special cases. Layer 1's job is to preserve geometry exactly; Layer 2 evaluates end-tangents and end-curvatures from the NURBS itself. |
| Synthetic-only test corpus for Step 3. No real-world G5 corpus integration. | `[DIRECTION]` Q4 | No slicer or common CAM tool emits G5/G5.1 today. Real-world corpus integration belongs to Step 8 (spline fitter) and any future G5-emitting slicer. |
| Refactor `ReduceEvent` to `ReduceEvent::Curve(CurveGeom, …)` with fixed-size-array variants. G5 lands as `CurveGeom::Cubic`; G5.1 lands as `CurveGeom::Quadratic`; existing G1 → `Linear`; existing G2/G3 → `RationalQuadratic`. | `[DIRECTION]` Q5 | Fixed-size arrays per variant — zero heap allocation per segment. Exhaustive-match safety preserved. Clean ontology: `Quadratic` (non-rational) and `RationalQuadratic` are distinct variants; processing sites that handle them differently don't need to inspect `Option<weights>`. Future G6.2 NURBS slots in as one new `CurveGeom::Nurbs { … }` variant. |

### Risks flagged in CLAUDE.md and how this plan addresses them

CLAUDE.md does not flag G5 reduction as a high-risk item — Step 3 is explicitly described as a *small follow-up to Step 2*. The known risks for Layer 1 (the spline fitter, Step 8, being the highest-risk item by a meaningful margin) are out of scope here. Two minor risks specific to this plan:

1. **G5 modal-chain semantics rarely tested in practice.** Mitigation: the test plan covers the documented RS274NGC chain behavior (Tasks 13, 14, 16, 17) including broken-chain rejection.
2. **G5.1 active-plane handling is a new modal-state concern.** Mitigation: a minimal `Plane` enum tracking the most recent G17/G18/G19 (default G17 per RS274NGC); G5.1 outside the active plane errors out (Task 18). G2/G3 are *not* re-gated by plane in this plan — they remain XY-plane-only as Phase 1 already implements them; plane-aware G2/G3 is a deliberate non-goal of Step 3 (would touch existing arc tests).

### Acceptance criterion for the whole plan

- `cargo test --workspace --manifest-path rust/Cargo.toml` passes on a clean tree.
- `cargo clippy --workspace --manifest-path rust/Cargo.toml -- -D warnings` passes.
- All new tests in this plan pass; all existing tests continue to pass without modification beyond the explicit refactor in Tasks 4–10 (the `ReduceEvent` shape change; old test bodies are mechanically rewritten to the new variant shape, no semantic change).
- A G5 line `G5 X10 Y0 I3 J3 P-3 Q3` after `G1 X0 Y0` produces exactly one `Segment::Fitted { degree: 3, max_residual_mm: 0.0, … }` with control points `[(0,0,0), (3,3,0), (7,3,0), (10,0,0)]` and knot vector `[0,0,0,0,1,1,1,1]`, and *no* `Junction` segment after.
- A G5.1 line `G5.1 X10 Y0 I3 J3` after `G1 X0 Y0` produces exactly one `Segment::Fitted { degree: 2, max_residual_mm: 0.0, … }` with control points `[(0,0,0), (3,3,0), (10,0,0)]` and knot vector `[0,0,0,1,1,1]`.

---

## File structure

This plan adds **no new files**. All work is within existing modules:

| File | Role | Tasks |
|---|---|---|
| `rust/geometry/src/reduce.rs` | Modal state extensions; `CurveGeom` + new `ReduceEvent::Curve` variant; G5 / G5.1 reduction logic; G17/G18/G19 plane tracking; refactor existing G1/G2/G3 arms onto the new shape. | 1–9, 13–18 |
| `rust/geometry/src/pipeline.rs` | Refactor `handle_event` arms onto `Curve` shape; pipeline-side construction of degree-2 / degree-3 non-rational NURBS for G5.1 / G5; integration of new `Recovery::G5MissingTangent` / `Recovery::G5PlaneMismatch` variants. | 10–12, 19, 20 |
| `rust/geometry/src/error.rs` | Two new `Recovery` variants for G5-specific malformed-input cases. | 11 |
| `rust/geometry/tests/g5_reduction.rs` | New integration test file for end-to-end G5/G5.1 pipeline behavior. | 21 |

**Why no new modules:** the work is entirely additive within the existing `reduce` and `pipeline` modules and is small enough that adding a `g5.rs` submodule would scatter G5-specific code from its caller without buying compile-time isolation. If a future fitter-output type is introduced that justifies a `curve` submodule, that's a separate refactor.

---

## Task ordering rationale

Tasks 1–3 add the new modal-state types (`Plane`, `prev_g5_pq`). Task 4 defines `CurveGeom`. Tasks 5–9 perform the additive `ReduceEvent::Curve` refactor (introduce variant, migrate G1, migrate G2/G3 arcs, migrate `pipeline::handle_event`, delete legacy variants). Tasks 10–11 introduce the new `Recovery` variants and the pipeline error-path mapping. Tasks 12–16 implement G5 and G5.1 reduce-side logic (including the defensive G5 error-path clearing of `prev_g5_pq` in Task 12.5). Tasks 17–18 implement the pipeline-side NURBS construction and `Segment::Fitted` emission for `CurveGeom::Cubic` and `CurveGeom::Quadratic`. Task 19 is the integration test sweep. Task 20 ticks the CLAUDE.md build-order checkbox.

**Each task is independently committable.** The refactor block (4–10) ships in green-tested increments — at no point does the workspace have a broken `cargo test`.

---

## Task 1: Add `Plane` enum and active-plane tracking to `ModalState`

**Files:**
- Modify: `rust/geometry/src/reduce.rs`

**Why:** G5.1 is restricted to the active plane per LinuxCNC §G5.1. Plane tracking is otherwise absent from the codebase. We add the enum + state field but do not yet check it (that's Task 18).

- [ ] **Step 1: Write the failing test**

Add to the `tests` module in `rust/geometry/src/reduce.rs` (append, do not replace any existing test):
```rust
    #[test]
    fn modal_state_plane_defaults_to_xy() {
        let st = ModalState::new();
        assert_eq!(st.active_plane, Plane::XY);
    }

    #[test]
    fn g17_keeps_xy_plane() {
        let toks = vec![cmd(b'G', 17, 1, Params::default())];
        let _events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        // Plane is internal modal state; this test is reachable today only by
        // observing through downstream behavior, which lands in Task 18. Test
        // ordering: this scaffolds the type so Task 18's plane-mismatch test
        // can construct cases that change the plane. For now, assert the type
        // compiles and the variant set is what we expect.
        assert_eq!(Plane::default(), Plane::XY);
        assert_eq!(Plane::XY, Plane::XY);
        assert_ne!(Plane::XY, Plane::XZ);
        assert_ne!(Plane::XZ, Plane::YZ);
    }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests::modal_state_plane_defaults_to_xy reduce::tests::g17_keeps_xy_plane`
Expected: FAIL — `Plane` not defined.

- [ ] **Step 3: Add the `Plane` enum and field**

In `rust/geometry/src/reduce.rs`, add after the `ParseErrorKind` enum (around line 103):
```rust
/// Active machining plane per RS274NGC §3.5.1. Tracked across the gcode
/// stream by G17/G18/G19. Default G17 (XY) per spec. Used by G5.1 to validate
/// that the curve lies in a supported plane; G2/G3 are XY-only in Phase 1
/// regardless of plane state (deliberate non-goal of Step 3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) enum Plane {
    #[default]
    XY,
    XZ,
    YZ,
}
```

In the `ModalState` struct (currently at line 18), add a field:
```rust
pub(crate) struct ModalState {
    pub position: [f64; 3],
    pub e: f64,
    pub feedrate_mm_s: Option<f64>,
    pub tool: u32,
    pub active_plane: Plane,
}
```

In `ModalState::new`, initialize the field:
```rust
impl ModalState {
    #[allow(dead_code)]
    pub fn new() -> Self {
        Self {
            position: [0.0, 0.0, 0.0],
            e: 0.0,
            feedrate_mm_s: None,
            tool: 0,
            active_plane: Plane::XY,
        }
    }
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests::modal_state_plane_defaults_to_xy reduce::tests::g17_keeps_xy_plane`
Expected: PASS.

- [ ] **Step 5: Run the full reduce test suite to confirm no regressions**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests`
Expected: PASS — all existing reduce tests continue to pass (the new field has a default, no existing construction site changed).

- [ ] **Step 6: Commit**

```bash
git add rust/geometry/src/reduce.rs
git commit -m "geometry/reduce: add Plane enum and ModalState::active_plane (default XY)"
```

---

## Task 2: Wire G17/G18/G19 to update the active plane

**Files:**
- Modify: `rust/geometry/src/reduce.rs`

**Why:** Step 3 needs G5.1 plane checks against G17/G18/G19. Without consuming the plane-select g-codes, the field defaults to `XY` forever and the check is meaningless.

- [ ] **Step 1: Write the failing test**

Add to the `tests` module in `rust/geometry/src/reduce.rs`:
```rust
    #[test]
    fn g17_sets_xy_plane() {
        let mut st = ModalState::new();
        let toks = vec![cmd(b'G', 17, 1, Params::default())];
        // Drive the iterator to consume the token; we observe the side-effect
        // by re-running with a follow-up G18 and checking that G18 wins.
        let _events: Vec<_> = reduce_with_state(&mut st, toks.into_iter().map(Ok)).collect();
        assert_eq!(st.active_plane, Plane::XY);
    }

    #[test]
    fn g18_sets_xz_plane() {
        let mut st = ModalState::new();
        let toks = vec![cmd(b'G', 18, 1, Params::default())];
        let _events: Vec<_> = reduce_with_state(&mut st, toks.into_iter().map(Ok)).collect();
        assert_eq!(st.active_plane, Plane::XZ);
    }

    #[test]
    fn g19_sets_yz_plane() {
        let mut st = ModalState::new();
        let toks = vec![cmd(b'G', 19, 1, Params::default())];
        let _events: Vec<_> = reduce_with_state(&mut st, toks.into_iter().map(Ok)).collect();
        assert_eq!(st.active_plane, Plane::YZ);
    }

    #[test]
    fn plane_select_emits_no_event() {
        let toks = vec![cmd(b'G', 17, 1, Params::default())];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        // Plane selects update modal state silently — they're configuration,
        // not motion, and intentionally do not produce telemetry events.
        assert!(events.is_empty(), "expected no events, got {events:?}");
    }
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests::g17_sets_xy_plane reduce::tests::g18_sets_xz_plane reduce::tests::g19_sets_yz_plane reduce::tests::plane_select_emits_no_event`
Expected: FAIL — `reduce_with_state` does not exist; G17/G18/G19 do not currently update state.

- [ ] **Step 3: Add a `reduce_with_state` test helper that exposes mutable state**

In `rust/geometry/src/reduce.rs`, add after the existing `reduce` function (around line 121):
```rust
/// Test-only variant of `reduce` that takes a mutable `ModalState` reference,
/// allowing tests to inspect modal state after the iterator drains. Identical
/// to `reduce` otherwise; not exposed outside `#[cfg(test)]`.
#[cfg(test)]
#[allow(dead_code)]
pub(crate) fn reduce_with_state<'a, I>(
    state: &'a mut ModalState,
    tokens: I,
) -> impl Iterator<Item = ReduceEvent> + 'a
where
    I: IntoIterator<Item = Result<Token, ParseError>> + 'a,
    I::IntoIter: 'a,
{
    ReduceIterRef { tokens: tokens.into_iter(), state }
}

#[cfg(test)]
struct ReduceIterRef<'a, I>
where
    I: Iterator<Item = Result<Token, ParseError>>,
{
    tokens: I,
    state: &'a mut ModalState,
}

#[cfg(test)]
impl<I> Iterator for ReduceIterRef<'_, I>
where
    I: Iterator<Item = Result<Token, ParseError>>,
{
    type Item = ReduceEvent;

    fn next(&mut self) -> Option<Self::Item> {
        // Delegate to the same logic as ReduceIter::next via a function that
        // takes &mut ModalState, so test and production share one body.
        next_event(&mut self.tokens, self.state)
    }
}
```

This requires extracting the body of `ReduceIter::next` into a free function `next_event(tokens, state)`. Refactor `ReduceIter::next` accordingly:
```rust
impl<I> Iterator for ReduceIter<I>
where
    I: Iterator<Item = Result<Token, ParseError>>,
{
    type Item = ReduceEvent;

    fn next(&mut self) -> Option<Self::Item> {
        next_event(&mut self.tokens, &mut self.state)
    }
}

/// Pull the next reduce-output event from the token stream, mutating modal
/// state in place. Shared between `ReduceIter` (production) and
/// `ReduceIterRef` (tests). Logic is identical to the original
/// `ReduceIter::next` body.
#[allow(clippy::too_many_lines)]
fn next_event<I>(tokens: &mut I, state: &mut ModalState) -> Option<ReduceEvent>
where
    I: Iterator<Item = Result<Token, ParseError>>,
{
    loop {
        let tok = tokens.next()?;
        // (move the rest of the original loop body here, replacing every
        //  occurrence of `self.state` with `state` and every reference to
        //  `self.update_position(&params)` with the inline body.)
        // …
    }
}
```

The `update_position` helper currently takes `&mut self`; promote it to a free function `update_position_in(state: &mut ModalState, params: &gcode::Params)` and call sites change from `self.update_position(&params)` → `update_position_in(state, &params)`.

- [ ] **Step 4: Add G17/G18/G19 handling to `next_event`**

In `next_event`, immediately before the catch-all `_ => {}` arm, add:
```rust
            Token::Command { letter: b'G', major: 17, .. } => {
                state.active_plane = Plane::XY;
                continue;
            }
            Token::Command { letter: b'G', major: 18, .. } => {
                state.active_plane = Plane::XZ;
                continue;
            }
            Token::Command { letter: b'G', major: 19, .. } => {
                state.active_plane = Plane::YZ;
                continue;
            }
```

`continue` (not `return Some(...)`) because plane-select is silent — it updates modal state without emitting an event, per the test in Step 1 and per RS274NGC's classification of G17/G18/G19 as configuration commands.

- [ ] **Step 5: Run tests to verify they pass**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests`
Expected: PASS — new tests pass, existing tests continue to pass.

- [ ] **Step 6: Commit**

```bash
git add rust/geometry/src/reduce.rs
git commit -m "geometry/reduce: track active plane via G17/G18/G19 (silent modal updates)"
```

---

## Task 3: Add `prev_g5_pq` to `ModalState`

**Files:**
- Modify: `rust/geometry/src/reduce.rs`

**Why:** RS274NGC §3.5.5: when G5 immediately follows G5 with both I,J omitted, default I,J to `−(prev P, prev Q)`. We need to carry the previous G5's (P, Q) across the inter-token gap.

**Note on the G5 success arm's two state effects (per spec §3.5):** when the G5 reduce arm lands in Task 12, the success path must do **both** of the following before emitting `ReduceEvent::Curve { geom: CurveGeom::Cubic, … }`:
1. **Clear the G1-tangent chain** — set `state.prev_g1_end = None`, `state.prev_g1_dir = None`, `state.prev_g1_feedrate = None` (identical to the existing G2/G3 behavior; G5 endpoints break the G1 chain because Layer 2 derives end-tangents from the NURBS itself per the curvature-continuity principle).
2. **Set `state.prev_g5_pq = Some([P, Q])`** — extends the G5 chain forward for the implicit-tangent rule on the next G5.

Task 12 implements both effects. This Task 3 only adds the modal-state slot; it does not yet wire either effect.

- [ ] **Step 1: Write the failing test**

Add to the `tests` module in `rust/geometry/src/reduce.rs`:
```rust
    #[test]
    fn modal_state_prev_g5_pq_defaults_to_none() {
        let st = ModalState::new();
        assert_eq!(st.prev_g5_pq, None);
    }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests::modal_state_prev_g5_pq_defaults_to_none`
Expected: FAIL — field does not exist.

- [ ] **Step 3: Add the field**

In `rust/geometry/src/reduce.rs`, modify `ModalState`:
```rust
pub(crate) struct ModalState {
    pub position: [f64; 3],
    pub e: f64,
    pub feedrate_mm_s: Option<f64>,
    pub tool: u32,
    pub active_plane: Plane,
    /// (P, Q) of the previous G5 segment, or `None` if the previous motion
    /// was not G5 (or no motion has occurred). Carried across an
    /// uninterrupted G5→G5 chain to support the RS274NGC §3.5.5 implicit
    /// next-tangent rule (I, J default to `−prev_pq` componentwise).
    /// **Cleared by every motion-producing g-code other than G5** (G0, G1,
    /// G2, G3, G5.1). Plane selects (G17/G18/G19), M-codes, and T-codes do
    /// **not** clear it — they don't move the machine.
    pub prev_g5_pq: Option<[f64; 2]>,
}
```

In `ModalState::new`:
```rust
impl ModalState {
    pub fn new() -> Self {
        Self {
            position: [0.0, 0.0, 0.0],
            e: 0.0,
            feedrate_mm_s: None,
            tool: 0,
            active_plane: Plane::XY,
            prev_g5_pq: None,
        }
    }
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests::modal_state_prev_g5_pq_defaults_to_none`
Expected: PASS.

- [ ] **Step 5: Run the full reduce test suite to confirm no regressions**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add rust/geometry/src/reduce.rs
git commit -m "geometry/reduce: add ModalState::prev_g5_pq for G5 modal-chain tracking"
```

---

## Task 4: Define the `CurveGeom` enum

**Files:**
- Modify: `rust/geometry/src/reduce.rs`

**Why:** This is the unifying inner-enum the user specified. Defining it before adding the `Curve` variant lets us write the new tests against the type immediately.

- [ ] **Step 1: Write the failing test**

Add to the `tests` module in `rust/geometry/src/reduce.rs`:
```rust
    #[test]
    fn curve_geom_variants_construct() {
        let _linear = CurveGeom::Linear { cps: [[0.0; 3], [1.0, 0.0, 0.0]] };
        let _quad = CurveGeom::Quadratic {
            cps: [[0.0; 3], [1.0, 1.0, 0.0], [2.0, 0.0, 0.0]],
        };
        let _ratquad = CurveGeom::RationalQuadratic {
            cps: [[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]],
            weights: [1.0, std::f64::consts::FRAC_1_SQRT_2, 1.0],
        };
        let _cubic = CurveGeom::Cubic {
            cps: [[0.0; 3], [1.0, 1.0, 0.0], [2.0, 1.0, 0.0], [3.0, 0.0, 0.0]],
        };
    }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests::curve_geom_variants_construct`
Expected: FAIL — `CurveGeom` not defined.

- [ ] **Step 3: Define the enum**

In `rust/geometry/src/reduce.rs`, immediately above the `ReduceEvent` enum (around line 38):
```rust
/// Geometry payload of a `ReduceEvent::Curve`. Each variant carries its
/// control points as a fixed-size array — zero per-segment heap allocation,
/// type-level enforcement of the correct CP count for each variant.
///
/// **Variant choice is by source g-code semantics**, not by mathematical
/// class: G5.1 (`Quadratic`, non-rational) is distinct from G2/G3
/// (`RationalQuadratic`) at this layer, so consuming code that handles them
/// differently does not need to inspect `Option<weights>`.
///
/// Future G6.2 NURBS would add a single `Nurbs { cps: SmallVec<…>, weights:
/// Option<…>, knots: SmallVec<…>, degree: u8 }` variant; the outer
/// `ReduceEvent::Curve(_, _)` arm doesn't change.
#[derive(Debug, Clone, PartialEq)]
#[allow(dead_code)]
pub(crate) enum CurveGeom {
    /// Degree-1 line segment. G0 (when promoted) and G1 land here.
    Linear { cps: [[f64; 3]; 2] },
    /// Degree-2 non-rational Bézier. G5.1 lands here.
    Quadratic { cps: [[f64; 3]; 3] },
    /// Degree-2 rational Bézier (NURBS with weights). G2/G3 land here.
    RationalQuadratic {
        cps: [[f64; 3]; 3],
        weights: [f64; 3],
    },
    /// Degree-3 non-rational Bézier. G5 lands here.
    Cubic { cps: [[f64; 3]; 4] },
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests::curve_geom_variants_construct`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/geometry/src/reduce.rs
git commit -m "geometry/reduce: define CurveGeom inner enum (Linear/Quadratic/RationalQuadratic/Cubic)"
```

---

## Task 5: Add `ReduceEvent::Curve` variant alongside the legacy variants

**Files:**
- Modify: `rust/geometry/src/reduce.rs`

**Why:** Additive variant — does not break any existing match arms. Legacy `G1Move` and `Arc` continue to work; subsequent tasks migrate them off.

- [ ] **Step 1: Write the failing test**

Add to the `tests` module in `rust/geometry/src/reduce.rs`:
```rust
    #[test]
    fn reduce_event_curve_variant_constructs() {
        let _e = ReduceEvent::Curve {
            geom: CurveGeom::Linear { cps: [[0.0; 3], [1.0, 0.0, 0.0]] },
            e_delta: Some(0.1),
            feedrate_mm_s: 100.0,
            line_no: 1,
        };
    }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests::reduce_event_curve_variant_constructs`
Expected: FAIL — `ReduceEvent::Curve` does not exist.

- [ ] **Step 3: Add the variant**

In `rust/geometry/src/reduce.rs`, modify the `ReduceEvent` enum to add a `Curve` variant (place it before `G1Move`):
```rust
#[derive(Debug, Clone, PartialEq)]
#[allow(dead_code)]
pub(crate) enum ReduceEvent {
    /// Any curve segment (line, conic, cubic, future NURBS). The geometry
    /// payload is in the `CurveGeom`; common motion-event fields are inline.
    Curve {
        geom: CurveGeom,
        e_delta: Option<f64>,
        feedrate_mm_s: f64,
        line_no: u32,
    },
    G1Move { /* unchanged */ from: [f64; 3], to: [f64; 3], e_delta: Option<f64>, feedrate_mm_s: f64, line_no: u32 },
    Arc { /* unchanged */ start: [f64; 3], end: [f64; 3], center: [f64; 3], clockwise: bool, z_delta: f64, e_delta: Option<f64>, feedrate_mm_s: f64, line_no: u32 },
    Marker { /* unchanged */ kind: MotionMarkerKind, line_no: u32, tool: Option<u32>, e_delta_mm: Option<f64> },
    CommentMarker { /* unchanged */ kind: MarkerKind, line_no: u32 },
    ParseError { /* unchanged */ line_no: u32, kind: ParseErrorKind, text: String },
}
```

- [ ] **Step 4: Run test to verify it passes; run full test suite to confirm no regressions**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/geometry/src/reduce.rs
git commit -m "geometry/reduce: add ReduceEvent::Curve variant alongside legacy G1Move/Arc"
```

---

## Task 6: Migrate G1 reduction to emit `Curve(Linear)` instead of `G1Move`

**Files:**
- Modify: `rust/geometry/src/reduce.rs`

**Why:** Convert G1 to the new shape. We do this *before* migrating the pipeline (Task 8) by keeping `G1Move` as a temporary deprecated alias inside the test fixtures only — but cleaner is to migrate reduce + pipeline atomically per side. The two-step approach: (a) reduce now emits `Curve(Linear)`; (b) pipeline pattern-matches `Curve(Linear)` and falls back to `G1Move` only for any test stub that constructs `G1Move` directly. Since `G1Move` has no production constructors after this task, deleting the variant in Task 9 is safe.

- [ ] **Step 1: Update the existing G1 test to expect the new shape**

In `rust/geometry/src/reduce.rs`, modify `g1_xy_emits_g1move` (rename to `g1_xy_emits_curve_linear`):
```rust
    #[test]
    #[allow(clippy::float_cmp)]
    fn g1_xy_emits_curve_linear() {
        let toks = vec![
            cmd(b'G', 1, 1, p(&[(b'X', 1.0), (b'Y', 2.0), (b'F', 1500.0)])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 1);
        match &events[0] {
            ReduceEvent::Curve {
                geom: CurveGeom::Linear { cps },
                feedrate_mm_s,
                line_no: 1,
                ..
            } => {
                assert_eq!(cps[0], [0.0, 0.0, 0.0]);
                assert_eq!(cps[1], [1.0, 2.0, 0.0]);
                assert!((feedrate_mm_s - 25.0).abs() < 1e-9, "F1500 → 25 mm/s");
            }
            other => panic!("expected Curve(Linear), got {other:?}"),
        }
    }
```

Also modify `modal_position_persists_across_g1s` to match against `Curve(Linear)`:
```rust
    #[test]
    #[allow(clippy::float_cmp)]
    fn modal_position_persists_across_g1s() {
        let toks = vec![
            cmd(b'G', 1, 1, p(&[(b'X', 1.0), (b'Y', 0.0), (b'F', 1500.0)])),
            cmd(b'G', 1, 2, p(&[(b'X', 2.0)])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 2);
        match &events[1] {
            ReduceEvent::Curve { geom: CurveGeom::Linear { cps }, .. } => {
                assert_eq!(cps[0], [1.0, 0.0, 0.0]);
                assert_eq!(cps[1], [2.0, 0.0, 0.0]);
            }
            other => panic!("expected Curve(Linear), got {other:?}"),
        }
    }
```

Also update `reduce_event_variants_construct` to remove the `G1Move` and `Arc` example constructions (they will be deleted in Task 9). Replace with `Curve` constructions:
```rust
    #[test]
    #[allow(clippy::no_effect_underscore_binding)]
    fn reduce_event_variants_construct() {
        let _e1 = ReduceEvent::Curve {
            geom: CurveGeom::Linear { cps: [[0.0; 3], [1.0, 0.0, 0.0]] },
            e_delta: Some(0.05),
            feedrate_mm_s: 100.0,
            line_no: 1,
        };
        let _e2 = ReduceEvent::Curve {
            geom: CurveGeom::RationalQuadratic {
                cps: [[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]],
                weights: [1.0, std::f64::consts::FRAC_1_SQRT_2, 1.0],
            },
            e_delta: None,
            feedrate_mm_s: 100.0,
            line_no: 1,
        };
        let _e3 = ReduceEvent::Marker {
            kind: MotionMarkerKind::ZOnly,
            line_no: 5,
            tool: None,
            e_delta_mm: None,
        };
    }
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests::g1_xy_emits_curve_linear reduce::tests::modal_position_persists_across_g1s reduce::tests::reduce_event_variants_construct`
Expected: FAIL — reduce still emits `G1Move`, not `Curve(Linear)`.

- [ ] **Step 3: Migrate the G1 arm in `next_event`**

In `next_event`, replace the entire G1 arm body (the block under `Token::Command { letter: b'G', major: 1, params, line_no, .. }`). The marker sub-cases (Z-only, E-only, F-only no-op) remain unchanged; the "real move" case at the bottom changes:
```rust
                    // Real move: update position and E, emit Curve(Linear).
                    let from = state.position;
                    update_position_in(state, &params);
                    let e_delta = params.e().map(|new_e| {
                        let d = new_e - state.e;
                        state.e = new_e;
                        d
                    });
                    let to = state.position;
                    let feedrate_mm_s = state.feedrate_mm_s.unwrap_or(0.0);
                    // G1 clears the G5 modal-chain tangent — non-G5 motion.
                    state.prev_g5_pq = None;
                    return Some(ReduceEvent::Curve {
                        geom: CurveGeom::Linear { cps: [from, to] },
                        e_delta,
                        feedrate_mm_s,
                        line_no,
                    });
```

(`from` was previously captured at the top of the arm; the reordering here keeps it consistent with the position update happening before `to` is read. Verify the original arm captured `from` correctly — at line 185 of the current file `let from = self.state.position;` is the first line of the G1 arm; preserve that ordering.)

The G0 arm also clears `prev_g5_pq` (G0 is non-G5 motion) — add `state.prev_g5_pq = None;` immediately before the `return Some(ReduceEvent::Marker { kind: MotionMarkerKind::G0, … })`. Same for the Z-only and E-only G1 sub-arms inside the G1 handler — they are non-G5 motion (or pseudo-motion in E's case), so they break the chain. Concretely:

```rust
                if !xy_changed && z_changed && !e_present {
                    update_position_in(state, &params);
                    state.prev_g5_pq = None;
                    return Some(ReduceEvent::Marker {
                        kind: MotionMarkerKind::ZOnly,
                        line_no,
                        tool: None,
                        e_delta_mm: None,
                    });
                }
                if !xy_changed && !z_changed && e_present {
                    let new_e = params.e().unwrap();
                    let delta = new_e - state.e;
                    state.e = new_e;
                    state.prev_g5_pq = None;
                    return Some(ReduceEvent::Marker {
                        kind: MotionMarkerKind::EOnly,
                        line_no,
                        tool: None,
                        e_delta_mm: Some(delta),
                    });
                }
                if !xy_changed && !z_changed && !e_present {
                    // F-only no-op: no motion, no chain break.
                    continue;
                }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests`
Expected: All G1-related tests pass under the new shape. The `g1_xy_emits_curve_linear`, `g1_z_only_emits_zonly_marker`, `g1_e_only_emits_eonly_marker`, `g0_emits_g0_marker`, and `modal_position_persists_across_g1s` tests pass.

(The `pipeline.rs` tests will *fail* at this point because pipeline still pattern-matches `G1Move` — that's expected, fixed in Task 8.)

- [ ] **Step 5: Run only the reduce module tests; pipeline tests will fail until Task 8 — do not commit yet**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests`
Expected: PASS.

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml pipeline::tests`
Expected: FAIL — pipeline still expects `G1Move`. **Continue to Task 8 before committing the workspace state.** This task and Task 8 ship as one commit so the workspace is never red.

---

## Task 7: Migrate G2/G3 reduction to emit `Curve(RationalQuadratic)`

**Files:**
- Modify: `rust/geometry/src/reduce.rs`

**Why:** Same reasoning as Task 6 but for arcs. After this task, reduce emits the new shape exclusively for production paths; only the test stubs for `G1Move` / `Arc` still reference the legacy variants.

**Important geometric note:** the rational-quadratic Bézier control points and weights for an arc are computed inside the existing `pipeline::build_arc_nurbs`. The reduce stage emits a *center-form* arc description today and lets the pipeline construct the NURBS. To preserve the algebraic-closure principle we move arc-NURBS construction into reduce: the `Curve(RationalQuadratic)` event already carries the NURBS-form data. This is the correct location — `reduce` is a pure G-code-to-NURBS map; `pipeline` should consume NURBS, not center-form descriptions.

- [ ] **Step 1: Update the existing arc tests to expect the new shape**

In `rust/geometry/src/reduce.rs`, modify `g2_emits_arc_clockwise`:
```rust
    #[test]
    #[allow(clippy::float_cmp)]
    fn g2_emits_curve_rational_quadratic_clockwise() {
        // Quarter-circle from (1, 0, 0) to (0, 1, 0), center (0, 0, 0), CW (G2).
        let toks = vec![
            cmd(b'G', 1, 1, p(&[(b'X', 1.0), (b'Y', 0.0), (b'F', 1500.0)])),
            cmd(b'G', 2, 2, p(&[(b'X', 0.0), (b'Y', 1.0), (b'I', -1.0), (b'J', 0.0)])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 2);
        match &events[1] {
            ReduceEvent::Curve {
                geom: CurveGeom::RationalQuadratic { cps, weights },
                line_no: 2,
                ..
            } => {
                let approx = |a: f64, b: f64| (a - b).abs() < 1e-9;
                // Tangent intersection of (1,0)→(0,1) on unit circle = (1,1).
                assert!(approx(cps[0][0], 1.0) && approx(cps[0][1], 0.0));
                assert!(approx(cps[1][0], 1.0) && approx(cps[1][1], 1.0));
                assert!(approx(cps[2][0], 0.0) && approx(cps[2][1], 1.0));
                // Z constant.
                for cp in cps { assert!(approx(cp[2], 0.0)); }
                // Weight middle = cos(π/4) = √½.
                assert!(approx(weights[0], 1.0));
                assert!(approx(weights[1], std::f64::consts::FRAC_1_SQRT_2));
                assert!(approx(weights[2], 1.0));
            }
            other => panic!("expected Curve(RationalQuadratic), got {other:?}"),
        }
    }
```

Modify `g3_emits_arc_counter_clockwise` similarly: match against `ReduceEvent::Curve { geom: CurveGeom::RationalQuadratic { .. }, .. }` and additionally assert that the middle control point's coordinates differ from the G2 case (CCW takes the long way around, which for a 90° from (1,0) to (0,1) is 270° — but for tests we keep the same geometry with G3 going the *short* way from a different start).

Replace `g3_emits_arc_counter_clockwise`:
```rust
    #[test]
    fn g3_emits_curve_rational_quadratic_counter_clockwise() {
        // CCW 90° from (0, 1) to (1, 0) around (0, 0). I = -0, J = -1 makes the
        // center at (0, 0) starting from (0, 1).
        let toks = vec![
            cmd(b'G', 1, 1, p(&[(b'X', 0.0), (b'Y', 1.0), (b'F', 1500.0)])),
            cmd(b'G', 3, 2, p(&[(b'X', 1.0), (b'Y', 0.0), (b'I', 0.0), (b'J', -1.0)])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[1] {
            ReduceEvent::Curve {
                geom: CurveGeom::RationalQuadratic { cps, weights },
                ..
            } => {
                let approx = |a: f64, b: f64| (a - b).abs() < 1e-9;
                // CCW short way from (0,1) to (1,0): tangent intersection at (1,1).
                assert!(approx(cps[0][0], 0.0) && approx(cps[0][1], 1.0));
                assert!(approx(cps[1][0], 1.0) && approx(cps[1][1], 1.0));
                assert!(approx(cps[2][0], 1.0) && approx(cps[2][1], 0.0));
                assert!(approx(weights[1], std::f64::consts::FRAC_1_SQRT_2));
            }
            other => panic!("expected Curve(RationalQuadratic), got {other:?}"),
        }
    }
```

Modify `g2_with_z_delta_yields_z_delta_field` to match against `Curve(RationalQuadratic)` and assert the middle control point's Z is the midpoint of start.z and end.z (this matches what `build_arc_nurbs` does today and what the `g2_helical_yields_z_linear_control_points` pipeline test asserts):
```rust
    #[test]
    #[allow(clippy::float_cmp)]
    fn g2_with_z_delta_yields_z_linear_control_points() {
        // Helical arc: end Z differs from start Z.
        let toks = vec![
            cmd(b'G', 1, 1, p(&[(b'X', 1.0), (b'Z', 0.0), (b'F', 1500.0)])),
            cmd(b'G', 2, 2, p(&[
                (b'X', 0.0), (b'Y', 1.0), (b'Z', 0.5),
                (b'I', -1.0), (b'J', 0.0),
            ])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[1] {
            ReduceEvent::Curve {
                geom: CurveGeom::RationalQuadratic { cps, .. },
                ..
            } => {
                // Z linear across CPs: 0.0, 0.25, 0.5
                let approx = |a: f64, b: f64| (a - b).abs() < 1e-9;
                assert!(approx(cps[0][2], 0.0));
                assert!(approx(cps[1][2], 0.25));
                assert!(approx(cps[2][2], 0.5));
            }
            other => panic!("expected Curve(RationalQuadratic), got {other:?}"),
        }
    }
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests::g2_emits_curve_rational_quadratic_clockwise reduce::tests::g3_emits_curve_rational_quadratic_counter_clockwise reduce::tests::g2_with_z_delta_yields_z_linear_control_points`
Expected: FAIL — reduce still emits `Arc { … }`.

- [ ] **Step 3: Move arc NURBS construction into reduce**

Add helpers to `rust/geometry/src/reduce.rs` (place after the `next_event` function, before `#[cfg(test)]`):
```rust
/// Build the rational-quadratic-Bézier control points and weights for an arc
/// in 3D. Z is interpolated linearly across the 3 control points (helical
/// support); the rational-quadratic geometry follows Piegl & Tiller §7.2.
///
/// **Phase 1 limitation** (preserved from the original `pipeline::build_arc_nurbs`):
/// |sweep| < π required; sweeps ≥ π are clamped to (π − ε) so `cos(half_sweep)`
/// stays positive. Multi-piece exact representation for full circles is a
/// Phase 2 item.
fn build_arc_curve(
    start: [f64; 3],
    end: [f64; 3],
    center: [f64; 3],
    clockwise: bool,
) -> CurveGeom {
    const MAX_SWEEP: f64 = std::f64::consts::PI * (1.0 - 1e-9);

    let r_start = [start[0] - center[0], start[1] - center[1]];
    let radius = (r_start[0] * r_start[0] + r_start[1] * r_start[1]).sqrt();
    let start_angle = r_start[1].atan2(r_start[0]);
    let r_end = [end[0] - center[0], end[1] - center[1]];
    let end_angle = r_end[1].atan2(r_end[0]);

    let sweep = if clockwise {
        let mut s = end_angle - start_angle;
        if s < 0.0 { s += 2.0 * std::f64::consts::PI; }
        s
    } else {
        let mut s = start_angle - end_angle;
        if s < 0.0 { s += 2.0 * std::f64::consts::PI; }
        -s
    };
    let sweep = sweep.clamp(-MAX_SWEEP, MAX_SWEEP);

    let half = sweep / 2.0;
    let cos_half = half.cos();
    let mid_x = center[0] + radius * (start_angle + half).cos() / cos_half;
    let mid_y = center[1] + radius * (start_angle + half).sin() / cos_half;

    let z0 = start[2];
    let z2 = end[2];
    let z1 = f64::midpoint(z0, z2);

    CurveGeom::RationalQuadratic {
        cps: [start, [mid_x, mid_y, z1], end],
        weights: [1.0, cos_half, 1.0],
    }
}
```

In `next_event`, replace the entire G2/G3 arm body with:
```rust
            Token::Command {
                letter: b'G', major: g, params, line_no, ..
            } if g == 2 || g == 3 => {
                let start = state.position;
                let i = params.i().unwrap_or(0.0);
                let j = params.j().unwrap_or(0.0);
                let center = [start[0] + i, start[1] + j, start[2]];
                let new_x = params.x().unwrap_or(start[0]);
                let new_y = params.y().unwrap_or(start[1]);
                let new_z = params.z().unwrap_or(start[2]);
                let end = [new_x, new_y, new_z];
                let clockwise = g == 2;
                if let Some(f) = params.f() {
                    state.feedrate_mm_s = Some(f_to_mm_s(f));
                }
                let e_delta = params.e().map(|new_e| {
                    let d = new_e - state.e;
                    state.e = new_e;
                    d
                });
                state.position = end;
                state.prev_g5_pq = None; // arcs are non-G5 motion.
                let feedrate_mm_s = state.feedrate_mm_s.unwrap_or(0.0);
                return Some(ReduceEvent::Curve {
                    geom: build_arc_curve(start, end, center, clockwise),
                    e_delta,
                    feedrate_mm_s,
                    line_no,
                });
            }
```

- [ ] **Step 4: Run reduce tests to verify they pass**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests`
Expected: PASS.

(Pipeline tests will fail until Task 8.)

---

## Task 8: Migrate `pipeline::handle_event` to consume `Curve` instead of `G1Move` / `Arc`

**Files:**
- Modify: `rust/geometry/src/pipeline.rs`

**Why:** Pipeline's `handle_event` matches against the legacy variants. After Tasks 6 and 7, reduce no longer produces those — pipeline must move to the new shape.

- [ ] **Step 1: Run pipeline tests to confirm they currently fail**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml pipeline::tests`
Expected: FAIL (in particular `two_g1s_emit_fitted_junction_fitted`, `single_g1_emits_degree_1_fitted`, `g2_emits_arc_segment_with_3d_control_points`, `g2_helical_yields_z_linear_control_points`) — `handle_event` no longer receives `G1Move` / `Arc`.

This is the diagnostic that drives the work in this task.

- [ ] **Step 2: Refactor `handle_event` to a single `Curve` arm with `CurveGeom` dispatch**

In `rust/geometry/src/pipeline.rs`, replace the `G1Move` and `Arc` arms in `handle_event` with one `Curve` arm. Update the `use` line to import `CurveGeom`:
```rust
use crate::{
    reduce::{reduce, CurveGeom, MotionMarkerKind, ParseErrorKind, ReduceEvent},
    ArcSegment, Fatal, FittedSegment, FitterParams, JunctionDeviation, Recovery, Segment,
    SourceRange, TelemetryEvent,
};
```

Replace the `match event { ReduceEvent::G1Move { … } => { … } ReduceEvent::Arc { … } => { … } … }` opening with:
```rust
    fn handle_event(&mut self, event: ReduceEvent) {
        match event {
            ReduceEvent::Curve { geom, e_delta: _, feedrate_mm_s, line_no } => {
                self.handle_curve(geom, feedrate_mm_s, line_no);
            }
            ReduceEvent::CommentMarker { kind, line_no } => {
                // (unchanged)
                if let gcode::MarkerKind::LayerChange { layer } = kind {
                    (self.sink)(TelemetryEvent::LayerChange { layer, line_no });
                }
                self.prev_g1_end = None;
                self.prev_g1_feedrate = None;
                self.prev_g1_dir = None;
            }
            ReduceEvent::Marker { kind, line_no, tool, e_delta_mm } => {
                // (unchanged — body identical to today)
                match kind {
                    MotionMarkerKind::T => {
                        if let Some(tool) = tool {
                            (self.sink)(TelemetryEvent::ToolChange { tool, line_no });
                        }
                    }
                    MotionMarkerKind::EOnly => {
                        if let Some(e_delta_mm) = e_delta_mm {
                            (self.sink)(TelemetryEvent::Retraction { e_delta_mm, line_no });
                        }
                    }
                    _ => {}
                }
                self.prev_g1_end = None;
                self.prev_g1_feedrate = None;
                self.prev_g1_dir = None;
            }
            ReduceEvent::ParseError { line_no, kind, text } => {
                // (unchanged — body identical to today)
                let recovery = match kind {
                    ParseErrorKind::MalformedNumber
                    | ParseErrorKind::DuplicateParam
                    | ParseErrorKind::EmptyCommand => {
                        Recovery::MalformedParams { line_no, raw: text }
                    }
                    ParseErrorKind::UnrecognizedHead => {
                        Recovery::UnrecognizedCommand { line_no, head: text }
                    }
                };
                (self.sink)(TelemetryEvent::Recovery(recovery.clone()));
                let pos = self.prev_g1_end.unwrap_or([0.0, 0.0, 0.0]);
                let jd = JunctionDeviation {
                    position: pos,
                    angle_deg: 0.0,
                    feedrate_mm_s: self.prev_g1_feedrate.unwrap_or(0.0),
                    source: SourceRange { start_line: line_no, end_line: line_no },
                };
                self.queue.push_back(Item::Recovered(Segment::Junction(jd), recovery));
            }
        }
    }
```

Add the `handle_curve` method on `Segments`:
```rust
    fn handle_curve(&mut self, geom: CurveGeom, feedrate_mm_s: f64, line_no: u32) {
        match geom {
            CurveGeom::Linear { cps } => {
                let from = cps[0];
                let to = cps[1];
                // Junction-deviation against previous G1 (if any).
                if let (Some(prev_dir), Some(prev_f)) =
                    (self.prev_g1_dir, self.prev_g1_feedrate)
                {
                    let cur_dir = unit([
                        to[0] - from[0], to[1] - from[1], to[2] - from[2],
                    ]);
                    let angle_deg = angle_between_deg(prev_dir, cur_dir);
                    let jd = JunctionDeviation {
                        position: from,
                        angle_deg,
                        feedrate_mm_s: prev_f.min(feedrate_mm_s),
                        source: SourceRange { start_line: line_no, end_line: line_no },
                    };
                    self.queue.push_back(Item::Segment(Segment::Junction(jd)));
                }
                let xyz = nurbs_from_linear(cps);
                let seg = FittedSegment {
                    xyz,
                    e: None,
                    feedrate_mm_s,
                    degree: 1,
                    max_residual_mm: 0.0,
                    source: SourceRange { start_line: line_no, end_line: line_no },
                };
                self.queue.push_back(Item::Segment(Segment::Fitted(seg)));
                self.prev_g1_end = Some(to);
                self.prev_g1_feedrate = Some(feedrate_mm_s);
                self.prev_g1_dir = Some(unit([
                    to[0] - from[0], to[1] - from[1], to[2] - from[2],
                ]));
            }
            CurveGeom::RationalQuadratic { cps, weights } => {
                let xyz = nurbs_from_rational_quadratic(cps, weights);
                let seg = ArcSegment {
                    xyz,
                    e: None,
                    feedrate_mm_s,
                    source: SourceRange { start_line: line_no, end_line: line_no },
                };
                self.queue.push_back(Item::Segment(Segment::Arc(seg)));
                // Arcs break the G1-junction chain (curvature-continuity principle).
                self.prev_g1_end = None;
                self.prev_g1_feedrate = None;
                self.prev_g1_dir = None;
            }
            CurveGeom::Quadratic { cps } => {
                // Implemented in Task 19. For now, surface as Fatal so the
                // workspace compiles and any inadvertent emission is loud.
                // After Task 19 this arm constructs a degree-2 NURBS and emits
                // Segment::Fitted { degree: 2 }.
                self.emit_unimplemented_curve("Quadratic", line_no);
                let _ = cps;
            }
            CurveGeom::Cubic { cps } => {
                self.emit_unimplemented_curve("Cubic", line_no);
                let _ = cps;
            }
        }
    }

    fn emit_unimplemented_curve(&mut self, kind: &'static str, _line_no: u32) {
        // Stub for Tasks 19 & 20. Production code never reaches here in
        // Tasks 6-12 because reduce does not yet emit Quadratic / Cubic.
        // debug_assert! lets tests catch a stray emission at developer time.
        debug_assert!(false, "CurveGeom::{kind} reached pipeline before Task 19/20 implementation");
    }
```

Replace the existing `degree_1_nurbs` helper with `nurbs_from_linear`, and `build_arc_nurbs` with `nurbs_from_rational_quadratic` (these are renames + signature simplifications — the geometry is now passed in directly):
```rust
fn nurbs_from_linear(cps: [[f64; 3]; 2]) -> nurbs::VectorNurbs<f64, 3> {
    nurbs::VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        cps.to_vec(),
        None,
    )
    .expect("degree-1 NURBS with 2 CPs is always valid")
}

fn nurbs_from_rational_quadratic(
    cps: [[f64; 3]; 3],
    weights: [f64; 3],
) -> nurbs::VectorNurbs<f64, 3> {
    nurbs::VectorNurbs::<f64, 3>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        cps.to_vec(),
        Some(weights.to_vec()),
    )
    .expect("rational quadratic from reduce is always valid")
}
```

Delete the old `degree_1_nurbs` and `build_arc_nurbs` functions (the geometry that used to live in `build_arc_nurbs` now lives in `reduce::build_arc_curve`).

- [ ] **Step 3: Run all geometry tests**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml`
Expected: PASS — reduce tests, pipeline tests, all integration tests pass.

- [ ] **Step 4: Run clippy on the geometry crate**

Run: `cargo clippy -p geometry --manifest-path rust/Cargo.toml -- -D warnings`
Expected: PASS.

- [ ] **Step 5: Commit Tasks 6, 7, and 8 together**

This commit completes the additive refactor to the new `Curve` shape across reduce + pipeline:
```bash
git add rust/geometry/src/reduce.rs rust/geometry/src/pipeline.rs
git commit -m "$(cat <<'EOF'
geometry: refactor reduce/pipeline to ReduceEvent::Curve(CurveGeom, …)

Migrates the production paths off ReduceEvent::G1Move (now Curve(Linear))
and ReduceEvent::Arc (now Curve(RationalQuadratic)). Arc NURBS construction
moves into reduce::build_arc_curve so reduce's contract is gcode → NURBS;
pipeline consumes NURBS, not center-form descriptions.

This is the structural prerequisite for G5 (Curve(Cubic)) and G5.1
(Curve(Quadratic)) reductions in subsequent commits.
EOF
)"
```

---

## Task 9: Delete the now-unused `ReduceEvent::G1Move` and `ReduceEvent::Arc` variants

**Files:**
- Modify: `rust/geometry/src/reduce.rs`

**Why:** No production code emits or consumes `G1Move` / `Arc` anymore. Leaving them invites stale-pattern-match bugs.

- [ ] **Step 1: Search for any remaining references**

Run: `grep -rn "G1Move\|ReduceEvent::Arc" rust/geometry rust/gcode 2>&1`
Expected output: only inside `rust/geometry/src/reduce.rs` (the variants themselves) and possibly inside the `reduce_event_variants_construct` test stub (which Task 6 already updated to use `Curve`). No production consumers remain.

If grep finds an unexpected reference, audit and migrate it before continuing.

- [ ] **Step 2: Delete the variants**

In `rust/geometry/src/reduce.rs`, remove the `G1Move` and `Arc` variants from the `ReduceEvent` enum. Final shape:
```rust
#[derive(Debug, Clone, PartialEq)]
#[allow(dead_code)]
pub(crate) enum ReduceEvent {
    Curve {
        geom: CurveGeom,
        e_delta: Option<f64>,
        feedrate_mm_s: f64,
        line_no: u32,
    },
    Marker {
        kind: MotionMarkerKind,
        line_no: u32,
        tool: Option<u32>,
        e_delta_mm: Option<f64>,
    },
    CommentMarker {
        kind: MarkerKind,
        line_no: u32,
    },
    ParseError {
        line_no: u32,
        kind: ParseErrorKind,
        text: String,
    },
}
```

- [ ] **Step 3: Run the workspace test suite**

Run: `cargo test --workspace --manifest-path rust/Cargo.toml`
Expected: PASS.

- [ ] **Step 4: Run clippy on the workspace**

Run: `cargo clippy --workspace --manifest-path rust/Cargo.toml -- -D warnings`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/geometry/src/reduce.rs
git commit -m "geometry/reduce: drop legacy G1Move/Arc variants (replaced by Curve)"
```

---

## Task 10: Add `Recovery::G5MissingTangent` variant

**Files:**
- Modify: `rust/geometry/src/error.rs`

**Why:** RS274NGC §3.5.5: G5 with both I,J omitted requires a previous G5 in the modal chain. If the chain is broken (any non-G5 motion intervened), reject with a specific recovery so consumers can distinguish "missing tangent" from generic malformed-params.

- [ ] **Step 1: Write the failing test**

Add to a new tests module at the bottom of `rust/geometry/src/error.rs`:
```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn g5_missing_tangent_constructs() {
        let _r = Recovery::G5MissingTangent { line_no: 42 };
    }

    #[test]
    fn g5_plane_mismatch_constructs() {
        let _r = Recovery::G5PlaneMismatch {
            line_no: 42,
            active_plane_g_code: 18,
        };
    }
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml error::tests`
Expected: FAIL — variants do not exist.

- [ ] **Step 3: Add the variants**

In `rust/geometry/src/error.rs`, add to the `Recovery` enum:
```rust
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum Recovery {
    UnrecognizedCommand { line_no: u32, head: String },
    MalformedParams { line_no: u32, raw: String },
    WindowCapHit { source: SourceRange, run_vertex_count: u32 },
    DegenerateSlotFallback { line_no: u32, reason: SlotDegeneracy },
    ToleranceExceeded { source: SourceRange, actual_mm: f64, budget_mm: f64 },
    LspiaNotConverged { source: SourceRange, last_update_mm: f64 },
    /// G5 with both I,J omitted but no previous G5 in modal chain (chain
    /// broken by intervening non-G5 motion). Per RS274NGC §3.5.5, the
    /// implicit-tangent rule requires `prev_g5_pq` to be set; when it is
    /// not, we reject the line rather than fabricate a tangent.
    G5MissingTangent { line_no: u32 },
    /// G5.1 issued while the active plane (G17/G18/G19) is not the only
    /// supported plane (XY in Phase 1). The G-code number of the active
    /// plane is included for diagnostic clarity (17 = XY, 18 = XZ, 19 = YZ).
    G5PlaneMismatch { line_no: u32, active_plane_g_code: u32 },
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml error::tests`
Expected: PASS.

- [ ] **Step 5: Run full workspace tests to confirm no regressions**

Run: `cargo test --workspace --manifest-path rust/Cargo.toml`
Expected: PASS — all match arms over `Recovery` use either explicit variants or `_ =>` because of `#[non_exhaustive]`.

- [ ] **Step 6: Commit**

```bash
git add rust/geometry/src/error.rs
git commit -m "geometry/error: add Recovery::G5MissingTangent and Recovery::G5PlaneMismatch"
```

---

## Task 11: Map `Recovery::G5MissingTangent` and `Recovery::G5PlaneMismatch` through the pipeline error path

**Files:**
- Modify: `rust/geometry/src/pipeline.rs`
- Modify: `rust/geometry/src/reduce.rs`

**Why:** When reduce identifies a G5 modal-chain break or a G5.1 plane mismatch, it surfaces the condition through a new `ReduceEvent::ParseError` sub-kind. Pipeline maps that to the new `Recovery` variant. We extend `ParseErrorKind` (defined in `reduce.rs`) with two new variants and the pipeline's `handle_event` `ParseError` arm gains two new mapping cases.

- [ ] **Step 1: Write the failing test**

Add to the `tests` module in `rust/geometry/src/pipeline.rs`:
```rust
    #[test]
    fn g5_missing_tangent_yields_recovered() {
        // G1 followed directly by G5 with no I,J — chain has no prev G5.
        let mut events = vec![];
        let mut p = GeometryPipeline::new(FitterParams::default());
        let items: Vec<_> = {
            let mut sink = |e: TelemetryEvent| events.push(e);
            p.process("G1 X1 Y0 F1500\nG5 X10 Y0 P-1 Q-1\n", &mut sink).collect()
        };
        let recovered = items.iter().find_map(|it| match it {
            Item::Recovered(_, Recovery::G5MissingTangent { line_no: 2 }) => Some(()),
            _ => None,
        });
        assert!(recovered.is_some(), "expected G5MissingTangent recovery, got {items:#?}");
        assert!(matches!(
            events.last(),
            Some(TelemetryEvent::Recovery(Recovery::G5MissingTangent { line_no: 2 }))
        ));
    }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml pipeline::tests::g5_missing_tangent_yields_recovered`
Expected: FAIL — `G5` token is currently ignored by `next_event` (matches no arm), so no events fire.

- [ ] **Step 3: Extend `ParseErrorKind` with the new sub-kinds**

In `rust/geometry/src/reduce.rs`, modify `ParseErrorKind`:
```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ParseErrorKind {
    MalformedNumber,
    UnrecognizedHead,
    EmptyCommand,
    DuplicateParam,
    /// G5 line missing both I,J with no previous G5 in modal chain.
    G5MissingTangent,
    /// G5.1 line with the active plane (G17/G18/G19) different from G17 (XY).
    /// The active plane is encoded in `text` as the literal G-code number
    /// ("18" or "19"); pipeline parses it back to populate Recovery.
    G5PlaneMismatch,
    /// G5/G5.1 with malformed I,J,P,Q (e.g. only I but not J, both zero on
    /// G5.1, etc.). Surfaced as MalformedParams equivalent but with G5
    /// context; pipeline maps to Recovery::MalformedParams.
    G5MalformedTangent,
}
```

- [ ] **Step 4: Map the new sub-kinds in pipeline's `ParseError` arm**

In `rust/geometry/src/pipeline.rs`, modify the `ReduceEvent::ParseError` arm in `handle_event`:
```rust
            ReduceEvent::ParseError { line_no, kind, text } => {
                let recovery = match kind {
                    ParseErrorKind::MalformedNumber
                    | ParseErrorKind::DuplicateParam
                    | ParseErrorKind::EmptyCommand
                    | ParseErrorKind::G5MalformedTangent => {
                        Recovery::MalformedParams { line_no, raw: text }
                    }
                    ParseErrorKind::UnrecognizedHead => {
                        Recovery::UnrecognizedCommand { line_no, head: text }
                    }
                    ParseErrorKind::G5MissingTangent => {
                        Recovery::G5MissingTangent { line_no }
                    }
                    ParseErrorKind::G5PlaneMismatch => {
                        let active_plane_g_code = text.parse::<u32>().unwrap_or(17);
                        Recovery::G5PlaneMismatch { line_no, active_plane_g_code }
                    }
                };
                (self.sink)(TelemetryEvent::Recovery(recovery.clone()));
                let pos = self.prev_g1_end.unwrap_or([0.0, 0.0, 0.0]);
                let jd = JunctionDeviation {
                    position: pos,
                    angle_deg: 0.0,
                    feedrate_mm_s: self.prev_g1_feedrate.unwrap_or(0.0),
                    source: SourceRange { start_line: line_no, end_line: line_no },
                };
                self.queue.push_back(Item::Recovered(Segment::Junction(jd), recovery));
            }
```

- [ ] **Step 5: Run the test to verify it still fails (because reduce doesn't emit G5 yet)**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml pipeline::tests::g5_missing_tangent_yields_recovered`
Expected: FAIL still — reduce's `next_event` has no G5 arm yet. **The mapping is now in place; the next tasks add the reduce arm that triggers it.**

- [ ] **Step 6: Run the rest of the test suite to confirm no regressions**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml --lib`
Expected: PASS — only the new G5 tests fail; everything else (G1, G2/G3, telemetry, plane tracking, modal state) passes.

- [ ] **Step 7: Commit**

```bash
git add rust/geometry/src/error.rs rust/geometry/src/reduce.rs rust/geometry/src/pipeline.rs
git commit -m "geometry: add ParseErrorKind G5* sub-kinds and pipeline Recovery mapping"
```

---

## Task 12: G5 reduction — single G5 with explicit I, J, P, Q

**Files:**
- Modify: `rust/geometry/src/reduce.rs`

**Why:** Implement the cubic case with all four tangent params explicit. This is the simplest G5 path; the modal-chain rule and validation come in subsequent tasks.

- [ ] **Step 1: Write the failing test**

Add to the `tests` module in `rust/geometry/src/reduce.rs`:
```rust
    #[test]
    #[allow(clippy::float_cmp)]
    fn g5_with_explicit_ijpq_emits_curve_cubic() {
        // Position at origin, G5 to (10, 0) with tangent params I=3, J=3, P=-3, Q=3.
        // Expected control points:
        //   P0 = (0, 0, 0)
        //   P1 = (0+3, 0+3, 0) = (3, 3, 0)
        //   P2 = (10+(-3), 0+3, 0) = (7, 3, 0)
        //   P3 = (10, 0, 0)
        let toks = vec![
            cmd_with_minor(b'G', 5, None, 1, p(&[
                (b'X', 10.0), (b'Y', 0.0),
                (b'I', 3.0), (b'J', 3.0),
                (b'P', -3.0), (b'Q', 3.0),
                (b'F', 1500.0),
            ])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 1);
        match &events[0] {
            ReduceEvent::Curve {
                geom: CurveGeom::Cubic { cps },
                feedrate_mm_s,
                line_no: 1,
                ..
            } => {
                assert_eq!(cps[0], [0.0, 0.0, 0.0]);
                assert_eq!(cps[1], [3.0, 3.0, 0.0]);
                assert_eq!(cps[2], [7.0, 3.0, 0.0]);
                assert_eq!(cps[3], [10.0, 0.0, 0.0]);
                assert!((feedrate_mm_s - 25.0).abs() < 1e-9);
            }
            other => panic!("expected Curve(Cubic), got {other:?}"),
        }
    }
```

The test uses a new helper `cmd_with_minor`. Add to the `tests` module:
```rust
    fn cmd_with_minor(letter: u8, major: u32, minor: Option<u32>, line_no: u32, params: Params) -> Token {
        Token::Command { letter, major, minor, params, line_no }
    }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests::g5_with_explicit_ijpq_emits_curve_cubic`
Expected: FAIL — G5 is silently dropped by `next_event`.

- [ ] **Step 3: Add the G5 arm to `next_event`**

In `next_event`, immediately above the G2/G3 arm (so G5 takes priority over the catch-all arc handler if it ever overlapped — they don't, but ordering matches the source-code reading flow):

```rust
            // G5: cubic Bézier with control points P0=current, P1=current+(I,J),
            // P2=end+(P,Q), P3=end. Per LinuxCNC RS274NGC §3.5.5.
            // Distinguished from G5.1 by the absence of `minor`.
            Token::Command {
                letter: b'G', major: 5, minor: None, params, line_no, ..
            } => {
                let p0 = state.position;

                let i_present = params.i().is_some();
                let j_present = params.j().is_some();

                // Resolve I,J: explicit if present, modal-chain rule if both
                // absent and prev_g5_pq is set, error otherwise.
                let (i, j) = match (params.i(), params.j(), state.prev_g5_pq) {
                    (Some(i), Some(j), _) => (i, j),
                    (None, None, Some([prev_p, prev_q])) => (-prev_p, -prev_q),
                    (None, None, None) => {
                        return Some(ReduceEvent::ParseError {
                            line_no,
                            kind: ParseErrorKind::G5MissingTangent,
                            text: String::new(),
                        });
                    }
                    _ => {
                        // Single I or single J specified — invalid per LinuxCNC.
                        return Some(ReduceEvent::ParseError {
                            line_no,
                            kind: ParseErrorKind::G5MalformedTangent,
                            text: format!("G5: I and J must both be specified or both omitted (i_present={i_present}, j_present={j_present})"),
                        });
                    }
                };

                // P, Q are required and explicit on every G5.
                let (pp, qq) = match (params.p(), params.q()) {
                    (Some(p_val), Some(q_val)) => (p_val, q_val),
                    _ => {
                        return Some(ReduceEvent::ParseError {
                            line_no,
                            kind: ParseErrorKind::G5MalformedTangent,
                            text: format!(
                                "G5: P and Q are required (got p={:?}, q={:?})",
                                params.p(), params.q()
                            ),
                        });
                    }
                };

                // End position: X/Y/Z modal — inherit from current position
                // for any axis not specified.
                let new_x = params.x().unwrap_or(p0[0]);
                let new_y = params.y().unwrap_or(p0[1]);
                let new_z = params.z().unwrap_or(p0[2]);
                let p3 = [new_x, new_y, new_z];

                // Z linearly interpolated across the four control points so
                // the curve remains exactly the planar cubic Bézier in XY
                // and linear in Z. Spacing 0, ⅓, ⅔, 1 along the parameter.
                let dz = p3[2] - p0[2];
                let p1 = [p0[0] + i, p0[1] + j, p0[2] + dz / 3.0];
                let p2 = [p3[0] + pp, p3[1] + qq, p0[2] + 2.0 * dz / 3.0];

                // Feedrate update.
                if let Some(f) = params.f() {
                    state.feedrate_mm_s = Some(f_to_mm_s(f));
                }
                let e_delta = params.e().map(|new_e| {
                    let d = new_e - state.e;
                    state.e = new_e;
                    d
                });

                // State updates: position, prev_g5_pq for the next link.
                state.position = p3;
                state.prev_g5_pq = Some([pp, qq]);

                let feedrate_mm_s = state.feedrate_mm_s.unwrap_or(0.0);
                return Some(ReduceEvent::Curve {
                    geom: CurveGeom::Cubic { cps: [p0, p1, p2, p3] },
                    e_delta,
                    feedrate_mm_s,
                    line_no,
                });
            }
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests::g5_with_explicit_ijpq_emits_curve_cubic`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/geometry/src/reduce.rs
git commit -m "geometry/reduce: G5 with explicit I/J/P/Q emits Curve(Cubic)"
```

---

## Task 12.5: Defensive — G5 error paths clear `prev_g5_pq`

**Files:**
- Modify: `rust/geometry/src/reduce.rs`

**Why:** Spec §7 risk #2 calls out a latent-bug class: every G5-arm error-return path must explicitly set `state.prev_g5_pq = None` immediately before returning. Without this, an erroring G5 (e.g. missing P, single-I-only) leaves stale `(P, Q)` from the previous successful G5 in modal state. A subsequent G5 with omitted I, J would then silently link to the *pre-error* G5 — masking the bad input by producing a geometrically-bogus continuation rather than the strict `Recovery::G5MissingTangent` the spec requires. This is split out as a named subtask (rather than buried in Task 12) because it has its own behavioral guarantee and its own failing-test-first cycle.

**Acceptance criterion:** A `G5(success) → G5(missing P, error) → G5(no IJ)` sequence produces `Recovery::G5MissingTangent` on the third G5 — proving the erroring (second) G5 cleared `prev_g5_pq` and the third G5 did **not** silently link to the first.

- [ ] **Step 1: Write the failing test**

Add to the `tests` module in `rust/geometry/src/reduce.rs`:
```rust
    #[test]
    fn g5_error_path_clears_prev_g5_pq() {
        // First G5 succeeds and would normally extend the chain.
        // Second G5 errors (missing P) — must clear prev_g5_pq.
        // Third G5 has no I,J — must produce G5MissingTangent
        // (proves the second G5's error cleared the chain).
        let toks = vec![
            cmd_with_minor(b'G', 5, None, 1, p(&[
                (b'X', 10.0), (b'Y', 0.0),
                (b'I', 3.0), (b'J', 3.0),
                (b'P', -3.0), (b'Q', 3.0),
                (b'F', 1500.0),
            ])),
            // Second G5: P omitted -> G5MalformedTangent.
            cmd_with_minor(b'G', 5, None, 2, p(&[
                (b'X', 20.0), (b'Y', 0.0),
                (b'I', 3.0), (b'J', 3.0),
                (b'Q', 3.0),
            ])),
            // Third G5: no I,J. If the second G5 didn't clear, this would
            // silently link to the *first* G5's (P, Q) — wrong. Must error.
            cmd_with_minor(b'G', 5, None, 3, p(&[
                (b'X', 30.0), (b'Y', 0.0),
                (b'P', -2.0), (b'Q', 2.0),
            ])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 3);
        match &events[1] {
            ReduceEvent::ParseError { line_no: 2, kind: ParseErrorKind::G5MalformedTangent, .. } => {}
            other => panic!("[1] expected G5MalformedTangent, got {other:?}"),
        }
        match &events[2] {
            ReduceEvent::ParseError { line_no: 3, kind: ParseErrorKind::G5MissingTangent, .. } => {}
            other => panic!("[2] expected G5MissingTangent (error path must clear chain), got {other:?}"),
        }
    }
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests::g5_error_path_clears_prev_g5_pq`
Expected: **FAIL** — Task 12 as written does not clear `prev_g5_pq` on the error-return paths, so the third G5 silently links to the first G5's (P, Q) = (−3, 3) and produces a `Curve(Cubic)` event rather than a `ParseError`.

- [ ] **Step 3: Add `state.prev_g5_pq = None;` before each `return` in the G5 arm's error paths**

In `rust/geometry/src/reduce.rs`, in the G5 arm of `next_event` (added in Task 12), amend each error-return so the chain pointer is cleared first. Concretely, the three error-return sites in Task 12's body become:

```rust
                let (i, j) = match (params.i(), params.j(), state.prev_g5_pq) {
                    (Some(i), Some(j), _) => (i, j),
                    (None, None, Some([prev_p, prev_q])) => (-prev_p, -prev_q),
                    (None, None, None) => {
                        state.prev_g5_pq = None; // already None, but explicit for symmetry
                        return Some(ReduceEvent::ParseError {
                            line_no,
                            kind: ParseErrorKind::G5MissingTangent,
                            text: String::new(),
                        });
                    }
                    _ => {
                        state.prev_g5_pq = None;
                        return Some(ReduceEvent::ParseError {
                            line_no,
                            kind: ParseErrorKind::G5MalformedTangent,
                            text: format!("G5: I and J must both be specified or both omitted (i_present={i_present}, j_present={j_present})"),
                        });
                    }
                };

                let (pp, qq) = match (params.p(), params.q()) {
                    (Some(p_val), Some(q_val)) => (p_val, q_val),
                    _ => {
                        state.prev_g5_pq = None;
                        return Some(ReduceEvent::ParseError {
                            line_no,
                            kind: ParseErrorKind::G5MalformedTangent,
                            text: format!(
                                "G5: P and Q are required (got p={:?}, q={:?})",
                                params.p(), params.q()
                            ),
                        });
                    }
                };
```

The successful path's `state.prev_g5_pq = Some([pp, qq]);` is correct as-is and unchanged.

- [ ] **Step 4: Run the test to verify it passes**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests::g5_error_path_clears_prev_g5_pq`
Expected: **PASS**.

- [ ] **Step 5: Run the full reduce test suite to confirm no regressions**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests`
Expected: **PASS** — all existing G5 tests (Task 12, plus Task 13's chain tests when they land) continue to pass; the defensive clearing only changes behavior on error paths.

- [ ] **Step 6: Commit**

```bash
git add rust/geometry/src/reduce.rs
git commit -m "geometry/reduce: G5 error paths clear prev_g5_pq (defensive)"
```

---

## Task 13: G5 reduction — modal-chain implicit-tangent rule (G5 follows G5)

**Files:**
- Modify: `rust/geometry/src/reduce.rs`

**Why:** Tests of the modal-chain behavior. Logic is in Task 12 already; this task is purely test additions to lock the behavior down.

**Acceptance criteria for this task:**

1. A three-G5 chain with implicit I, J on the second and third lines produces three `Curve(Cubic)` events whose P1 control points equal `−(prev P, prev Q)` of the previous G5.
2. A G5 → G1 → G5(no I, J) sequence produces `Recovery::G5MissingTangent` on the trailing G5 (G1 clears the chain).
3. A G5 → G17 → G5(no I, J) sequence succeeds — G17 is a non-motion plane select and must not clear the chain.
4. A G5 → M104 → T0 → G5(no I, J) sequence succeeds — M and T are non-motion and must not clear the chain.
5. **A G5 → G92 → G5(no I, J) sequence produces `Recovery::G5MissingTangent` on the trailing G5** — G92 redefines the coordinate frame, so the spec §3.5 clearing-discipline table requires `prev_g5_pq` to be cleared. Locks the derived behavior per spec §6.2 ("G5 → G92 → G5(no IJ) → Recovery::G5MissingTangent (G92 clears chain — derived behavior per §3.5)").
6. Single I or single J on a G5 → `Recovery::MalformedParams` (mapped from `ParseErrorKind::G5MalformedTangent`).
7. G5 missing P or Q (or both) → `Recovery::MalformedParams`.

- [ ] **Step 1: Write the failing test**

Add to the `tests` module in `rust/geometry/src/reduce.rs`:
```rust
    #[test]
    #[allow(clippy::float_cmp)]
    fn g5_chain_implicit_tangent_from_prev_pq() {
        // Three-G5 chain. Second and third have no I,J — should default to
        // -(prev P, prev Q).
        let toks = vec![
            cmd_with_minor(b'G', 5, None, 1, p(&[
                (b'X', 10.0), (b'Y', 0.0),
                (b'I', 3.0), (b'J', 3.0),
                (b'P', -3.0), (b'Q', 3.0),
                (b'F', 1500.0),
            ])),
            // Second G5: I,J implicit. Should be -(P,Q) of prev = (3, -3).
            cmd_with_minor(b'G', 5, None, 2, p(&[
                (b'X', 20.0), (b'Y', 0.0),
                (b'P', -2.0), (b'Q', 2.0),
            ])),
            // Third G5: I,J implicit. Should be -(P,Q) of second = (2, -2).
            cmd_with_minor(b'G', 5, None, 3, p(&[
                (b'X', 30.0), (b'Y', 0.0),
                (b'P', 0.0), (b'Q', 0.0),
            ])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 3);

        // Second G5: P0=(10,0,0), P1=(10+3, 0+(-3), 0)=(13, -3, 0).
        match &events[1] {
            ReduceEvent::Curve { geom: CurveGeom::Cubic { cps }, .. } => {
                assert_eq!(cps[0], [10.0, 0.0, 0.0]);
                assert_eq!(cps[1], [13.0, -3.0, 0.0]);
                assert_eq!(cps[2], [20.0 + (-2.0), 0.0 + 2.0, 0.0]);
                assert_eq!(cps[3], [20.0, 0.0, 0.0]);
            }
            other => panic!("[1] expected Curve(Cubic), got {other:?}"),
        }

        // Third G5: P0=(20,0,0), P1=(20+2, 0+(-2), 0)=(22, -2, 0).
        match &events[2] {
            ReduceEvent::Curve { geom: CurveGeom::Cubic { cps }, .. } => {
                assert_eq!(cps[0], [20.0, 0.0, 0.0]);
                assert_eq!(cps[1], [22.0, -2.0, 0.0]);
                assert_eq!(cps[2], [30.0 + 0.0, 0.0 + 0.0, 0.0]);
                assert_eq!(cps[3], [30.0, 0.0, 0.0]);
            }
            other => panic!("[2] expected Curve(Cubic), got {other:?}"),
        }
    }

    #[test]
    fn g5_chain_broken_by_g1_emits_recovery() {
        // G5 → G1 (breaks chain) → G5 with no I,J → expect ParseError.
        let toks = vec![
            cmd_with_minor(b'G', 5, None, 1, p(&[
                (b'X', 10.0), (b'Y', 0.0),
                (b'I', 3.0), (b'J', 3.0),
                (b'P', -3.0), (b'Q', 3.0),
                (b'F', 1500.0),
            ])),
            cmd(b'G', 1, 2, p(&[(b'X', 11.0), (b'Y', 0.0)])),
            cmd_with_minor(b'G', 5, None, 3, p(&[
                (b'X', 20.0), (b'Y', 0.0),
                (b'P', -2.0), (b'Q', 2.0),
            ])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 3);
        match &events[2] {
            ReduceEvent::ParseError { line_no: 3, kind: ParseErrorKind::G5MissingTangent, .. } => {}
            other => panic!("[2] expected G5MissingTangent ParseError, got {other:?}"),
        }
    }

    #[test]
    fn g5_chain_preserved_by_plane_select() {
        // G5 → G17 (no motion, doesn't break chain) → G5 with no I,J → uses prev_g5_pq.
        let toks = vec![
            cmd_with_minor(b'G', 5, None, 1, p(&[
                (b'X', 10.0), (b'Y', 0.0),
                (b'I', 3.0), (b'J', 3.0),
                (b'P', -3.0), (b'Q', 3.0),
                (b'F', 1500.0),
            ])),
            cmd(b'G', 17, 2, Params::default()),
            cmd_with_minor(b'G', 5, None, 3, p(&[
                (b'X', 20.0), (b'Y', 0.0),
                (b'P', -2.0), (b'Q', 2.0),
            ])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        // G17 emits no event, so we have 2 events total (the two G5s).
        assert_eq!(events.len(), 2);
        match &events[1] {
            ReduceEvent::Curve { geom: CurveGeom::Cubic { cps }, .. } => {
                // Modal-chain implicit I,J = -(prev P, prev Q) = (3, -3).
                assert_eq!(cps[1], [13.0, -3.0, 0.0]);
            }
            other => panic!("[1] expected Curve(Cubic), got {other:?}"),
        }
    }

    #[test]
    fn g5_chain_preserved_by_m_and_t_codes() {
        // G5 → M104 → T0 → G5 with no I,J. M and T don't move; chain intact.
        let toks = vec![
            cmd_with_minor(b'G', 5, None, 1, p(&[
                (b'X', 10.0), (b'Y', 0.0),
                (b'I', 3.0), (b'J', 3.0),
                (b'P', -3.0), (b'Q', 3.0),
                (b'F', 1500.0),
            ])),
            cmd(b'M', 104, 2, p(&[(b'S', 210.0)])),
            cmd(b'T', 0, 3, Params::default()),
            cmd_with_minor(b'G', 5, None, 4, p(&[
                (b'X', 20.0), (b'Y', 0.0),
                (b'P', -2.0), (b'Q', 2.0),
            ])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        // M and T emit Marker events; G5s emit Curve events; total = 4.
        assert_eq!(events.len(), 4);
        match &events[3] {
            ReduceEvent::Curve { geom: CurveGeom::Cubic { cps }, .. } => {
                assert_eq!(cps[1], [13.0, -3.0, 0.0]);
            }
            other => panic!("[3] expected Curve(Cubic), got {other:?}"),
        }
    }

    #[test]
    fn g5_chain_broken_by_g92_emits_recovery() {
        // G5 → G92 (redefines coordinate frame; clears chain per spec §3.5)
        // → G5 with no I,J → expect ParseError::G5MissingTangent.
        // (G5 → G92 → G5(no IJ) → Recovery::G5MissingTangent — derived behavior.)
        let toks = vec![
            cmd_with_minor(b'G', 5, None, 1, p(&[
                (b'X', 10.0), (b'Y', 0.0),
                (b'I', 3.0), (b'J', 3.0),
                (b'P', -3.0), (b'Q', 3.0),
                (b'F', 1500.0),
            ])),
            // G92 redefines the current position / coordinate frame; (P, Q)
            // become semantically stale because they are deltas in the prior
            // frame. Spec §3.5 chooses to clear conservatively.
            cmd(b'G', 92, 2, p(&[(b'X', 0.0), (b'Y', 0.0)])),
            cmd_with_minor(b'G', 5, None, 3, p(&[
                (b'X', 20.0), (b'Y', 0.0),
                (b'P', -2.0), (b'Q', 2.0),
            ])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        // The trailing G5 must produce a ParseError, not silently link to
        // the pre-G92 G5's (P, Q).
        let last = events.last().expect("expected at least one event");
        match last {
            ReduceEvent::ParseError { line_no: 3, kind: ParseErrorKind::G5MissingTangent, .. } => {}
            other => panic!("expected G5MissingTangent on trailing G5 (G92 must clear chain), got {other:?}"),
        }
    }

    #[test]
    fn g5_single_i_only_is_malformed() {
        // I given but J omitted — invalid.
        let toks = vec![
            cmd_with_minor(b'G', 5, None, 1, p(&[
                (b'X', 10.0), (b'Y', 0.0),
                (b'I', 3.0),
                (b'P', -3.0), (b'Q', 3.0),
                (b'F', 1500.0),
            ])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::ParseError { line_no: 1, kind: ParseErrorKind::G5MalformedTangent, .. } => {}
            other => panic!("expected G5MalformedTangent, got {other:?}"),
        }
    }

    #[test]
    fn g5_missing_pq_is_malformed() {
        // P,Q absent on G5 — invalid (P,Q are required on every G5 line).
        let toks = vec![
            cmd_with_minor(b'G', 5, None, 1, p(&[
                (b'X', 10.0), (b'Y', 0.0),
                (b'I', 3.0), (b'J', 3.0),
                (b'F', 1500.0),
            ])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::ParseError { line_no: 1, kind: ParseErrorKind::G5MalformedTangent, .. } => {}
            other => panic!("expected G5MalformedTangent, got {other:?}"),
        }
    }
```

- [ ] **Step 2: Run tests**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests::g5_chain reduce::tests::g5_single_i_only_is_malformed reduce::tests::g5_missing_pq_is_malformed`
Expected: PASS — Task 12's logic plus Task 12.5's defensive clearing implements all of this; these tests lock the behavior down.

**Note on `g5_chain_broken_by_g92_emits_recovery`:** for the G92 case to produce `G5MissingTangent`, the reduce-stage G92 handling (Marker arm) must `state.prev_g5_pq = None;` per spec §3.5 clearing-discipline table. If the existing G92 arm in `reduce.rs` does not yet clear the field, add `state.prev_g5_pq = None;` immediately before the G92 marker emission as part of executing this task's Step 3.

- [ ] **Step 3: Verify and (if needed) wire G92's chain-clear**

If `g5_chain_broken_by_g92_emits_recovery` fails after Step 2, locate the G92 handling arm in `next_event` and add `state.prev_g5_pq = None;` before its `return Some(ReduceEvent::Marker { … })`. Re-run the test. The other clearing rules from spec §3.5 (G0/G1/G2/G3/G5.1 clear; M/T/G17–G19 preserve) should already be satisfied by Tasks 6–7 and the unconditional preservation behavior of plane / M / T arms; the G92 case is the remaining derived clearing the table specifies.

- [ ] **Step 4: Commit**

```bash
git add rust/geometry/src/reduce.rs
git commit -m "geometry/reduce: lock G5 modal-chain behavior with regression tests"
```

---

## Task 14: G5 / G5.1 with Z delta (helical-like)

**Files:**
- Modify: `rust/geometry/src/reduce.rs`

**Why:** Lock down the linear-Z interpolation across control points for both curve orders: G5 places Z at thirds (0, dz/3, 2·dz/3, dz) across its four CPs; G5.1 places Z at the midpoint (0, dz/2, dz) across its three CPs. The math is in Tasks 12 and 15; this task tests both explicitly so the unit-test layer matches spec §6.2's Z-handling coverage.

- [ ] **Step 1: Write the failing tests**

Add to the `tests` module in `rust/geometry/src/reduce.rs`:
```rust
    #[test]
    #[allow(clippy::float_cmp)]
    fn g5_with_z_delta_interpolates_z_at_thirds() {
        // From (0,0,0) to (10, 0, 0.3). Expected Z at CPs: 0, 0.1, 0.2, 0.3.
        let toks = vec![
            cmd_with_minor(b'G', 5, None, 1, p(&[
                (b'X', 10.0), (b'Y', 0.0), (b'Z', 0.3),
                (b'I', 3.0), (b'J', 3.0),
                (b'P', -3.0), (b'Q', 3.0),
                (b'F', 1500.0),
            ])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::Curve { geom: CurveGeom::Cubic { cps }, .. } => {
                let approx = |a: f64, b: f64| (a - b).abs() < 1e-12;
                assert!(approx(cps[0][2], 0.0));
                assert!(approx(cps[1][2], 0.1));
                assert!(approx(cps[2][2], 0.2));
                assert!(approx(cps[3][2], 0.3));
            }
            other => panic!("expected Curve(Cubic), got {other:?}"),
        }
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn g5_1_with_z_delta_interpolates_z_at_midpoint() {
        // From (0,0,0) to (10, 0, 0.4). Expected Z at the three CPs:
        //   P0.z = 0, P1.z = 0.2 (midpoint), P2.z = 0.4.
        // Spec §6.2: "G5.1 with Z delta → control-point Z values at midpoint
        // (0, dz/2, dz)." Mirrors the cubic-at-thirds test above for G5.
        let toks = vec![
            cmd_with_minor(b'G', 5, Some(1), 1, p(&[
                (b'X', 10.0), (b'Y', 0.0), (b'Z', 0.4),
                (b'I', 3.0), (b'J', 3.0),
                (b'F', 1500.0),
            ])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::Curve { geom: CurveGeom::Quadratic { cps }, .. } => {
                let approx = |a: f64, b: f64| (a - b).abs() < 1e-12;
                assert!(approx(cps[0][2], 0.0));
                assert!(approx(cps[1][2], 0.2));
                assert!(approx(cps[2][2], 0.4));
            }
            other => panic!("expected Curve(Quadratic), got {other:?}"),
        }
    }
```

- [ ] **Step 2: Run the tests**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests::g5_with_z_delta_interpolates_z_at_thirds reduce::tests::g5_1_with_z_delta_interpolates_z_at_midpoint`
Expected: PASS — Task 12's `dz / 3.0` and `2.0 * dz / 3.0` math implements the cubic case; Task 15's `dz / 2.0` (or equivalent `f64::midpoint(p0.z, p3.z)`) implements the quadratic case.

- [ ] **Step 3: Commit**

```bash
git add rust/geometry/src/reduce.rs
git commit -m "geometry/reduce: lock G5/G5.1 Z-linear interpolation behavior with regression tests"
```

---

## Task 15: G5.1 reduction — base case in active XY plane

**Files:**
- Modify: `rust/geometry/src/reduce.rs`

**Why:** G5.1 is structurally simpler than G5 (no P,Q, no modal chain). It is a degree-2 non-rational Bézier with three control points.

- [ ] **Step 1: Write the failing test**

Add to the `tests` module in `rust/geometry/src/reduce.rs`:
```rust
    #[test]
    #[allow(clippy::float_cmp)]
    fn g5_1_with_explicit_ij_emits_curve_quadratic() {
        // From (0,0,0) to (10,0). I=3, J=3. Expected:
        //   P0 = (0, 0, 0), P1 = (3, 3, 0), P2 = (10, 0, 0).
        let toks = vec![
            cmd_with_minor(b'G', 5, Some(1), 1, p(&[
                (b'X', 10.0), (b'Y', 0.0),
                (b'I', 3.0), (b'J', 3.0),
                (b'F', 1500.0),
            ])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 1);
        match &events[0] {
            ReduceEvent::Curve {
                geom: CurveGeom::Quadratic { cps },
                feedrate_mm_s,
                line_no: 1,
                ..
            } => {
                assert_eq!(cps[0], [0.0, 0.0, 0.0]);
                assert_eq!(cps[1], [3.0, 3.0, 0.0]);
                assert_eq!(cps[2], [10.0, 0.0, 0.0]);
                assert!((feedrate_mm_s - 25.0).abs() < 1e-9);
            }
            other => panic!("expected Curve(Quadratic), got {other:?}"),
        }
    }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests::g5_1_with_explicit_ij_emits_curve_quadratic`
Expected: FAIL — G5.1 is silently dropped.

- [ ] **Step 3: Add the G5.1 arm to `next_event`**

In `next_event`, immediately after the G5 arm:
```rust
            // G5.1: quadratic Bézier with control points P0=current,
            // P1=current+(I,J), P2=end. Per LinuxCNC RS274NGC §G5.1.
            // Restricted to the active plane (G17/G18/G19); Phase 1 supports
            // only XY (G17). Both I and J must be specified and at least one
            // must be non-zero (a fully-zero tangent collapses to G1).
            Token::Command {
                letter: b'G', major: 5, minor: Some(1), params, line_no, ..
            } => {
                // Plane check (Phase 1: XY only).
                if state.active_plane != Plane::XY {
                    let plane_g_code = match state.active_plane {
                        Plane::XY => 17,
                        Plane::XZ => 18,
                        Plane::YZ => 19,
                    };
                    state.prev_g5_pq = None;
                    return Some(ReduceEvent::ParseError {
                        line_no,
                        kind: ParseErrorKind::G5PlaneMismatch,
                        text: plane_g_code.to_string(),
                    });
                }

                // I,J both required and at least one non-zero.
                let (i, j) = match (params.i(), params.j()) {
                    (Some(i), Some(j)) if i != 0.0 || j != 0.0 => (i, j),
                    (Some(_), Some(_)) => {
                        // Both zero — degenerate, equivalent to G1.
                        state.prev_g5_pq = None;
                        return Some(ReduceEvent::ParseError {
                            line_no,
                            kind: ParseErrorKind::G5MalformedTangent,
                            text: "G5.1: I and J both zero".to_string(),
                        });
                    }
                    _ => {
                        state.prev_g5_pq = None;
                        return Some(ReduceEvent::ParseError {
                            line_no,
                            kind: ParseErrorKind::G5MalformedTangent,
                            text: format!(
                                "G5.1: both I and J required (got i={:?}, j={:?})",
                                params.i(), params.j()
                            ),
                        });
                    }
                };

                let p0 = state.position;
                let new_x = params.x().unwrap_or(p0[0]);
                let new_y = params.y().unwrap_or(p0[1]);
                let new_z = params.z().unwrap_or(p0[2]);
                let p2 = [new_x, new_y, new_z];

                // Z linearly interpolated across 3 control points: 0, ½, 1.
                let z1 = f64::midpoint(p0[2], p2[2]);
                let p1 = [p0[0] + i, p0[1] + j, z1];

                if let Some(f) = params.f() {
                    state.feedrate_mm_s = Some(f_to_mm_s(f));
                }
                let e_delta = params.e().map(|new_e| {
                    let d = new_e - state.e;
                    state.e = new_e;
                    d
                });

                state.position = p2;
                // G5.1 is non-G5 motion: clear the modal chain.
                state.prev_g5_pq = None;

                let feedrate_mm_s = state.feedrate_mm_s.unwrap_or(0.0);
                return Some(ReduceEvent::Curve {
                    geom: CurveGeom::Quadratic { cps: [p0, p1, p2] },
                    e_delta,
                    feedrate_mm_s,
                    line_no,
                });
            }
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests::g5_1_with_explicit_ij_emits_curve_quadratic`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/geometry/src/reduce.rs
git commit -m "geometry/reduce: G5.1 with explicit I/J emits Curve(Quadratic)"
```

---

## Task 16: G5.1 — plane mismatch and degenerate-input rejection

**Files:**
- Modify: `rust/geometry/src/reduce.rs`

**Why:** Lock down the validation paths added in Task 15.

- [ ] **Step 1: Write the failing tests**

Add to the `tests` module in `rust/geometry/src/reduce.rs`:
```rust
    #[test]
    fn g5_1_outside_xy_plane_emits_recovery() {
        // G18 sets XZ plane; G5.1 should error.
        let toks = vec![
            cmd(b'G', 18, 1, Params::default()),
            cmd_with_minor(b'G', 5, Some(1), 2, p(&[
                (b'X', 10.0), (b'Z', 1.0),
                (b'I', 3.0), (b'J', 3.0),
            ])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        // G18 emits no event, so we have 1 event total (the G5.1 ParseError).
        assert_eq!(events.len(), 1);
        match &events[0] {
            ReduceEvent::ParseError {
                line_no: 2,
                kind: ParseErrorKind::G5PlaneMismatch,
                text,
            } => {
                assert_eq!(text, "18", "expected active plane G-code 18, got {text:?}");
            }
            other => panic!("expected G5PlaneMismatch, got {other:?}"),
        }
    }

    #[test]
    fn g5_1_with_both_ij_zero_is_malformed() {
        let toks = vec![
            cmd_with_minor(b'G', 5, Some(1), 1, p(&[
                (b'X', 10.0), (b'Y', 0.0),
                (b'I', 0.0), (b'J', 0.0),
            ])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::ParseError { kind: ParseErrorKind::G5MalformedTangent, .. } => {}
            other => panic!("expected G5MalformedTangent, got {other:?}"),
        }
    }

    #[test]
    fn g5_1_missing_j_is_malformed() {
        let toks = vec![
            cmd_with_minor(b'G', 5, Some(1), 1, p(&[
                (b'X', 10.0), (b'Y', 0.0),
                (b'I', 3.0),
            ])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::ParseError { kind: ParseErrorKind::G5MalformedTangent, .. } => {}
            other => panic!("expected G5MalformedTangent, got {other:?}"),
        }
    }

    #[test]
    fn g5_1_missing_i_is_malformed() {
        // J specified but I omitted — invalid (G5.1 has no modal-chain rule;
        // both I and J are required). Symmetric to g5_1_missing_j_is_malformed.
        let toks = vec![
            cmd_with_minor(b'G', 5, Some(1), 1, p(&[
                (b'X', 10.0), (b'Y', 0.0),
                (b'J', 3.0),
            ])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::ParseError { kind: ParseErrorKind::G5MalformedTangent, .. } => {}
            other => panic!("expected G5MalformedTangent, got {other:?}"),
        }
    }

    #[test]
    fn g5_1_no_ij_is_malformed() {
        // Neither I nor J — G5.1 has no modal-chain rule, so this is invalid.
        // (Per spec §6.2: "G5.1 with no I, J → Recovery::MalformedParams.
        // No modal-chain rule for G5.1.")
        let toks = vec![
            cmd_with_minor(b'G', 5, Some(1), 1, p(&[
                (b'X', 10.0), (b'Y', 0.0),
            ])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::ParseError { kind: ParseErrorKind::G5MalformedTangent, .. } => {}
            other => panic!("expected G5MalformedTangent, got {other:?}"),
        }
    }

    #[test]
    fn g5_1_outside_g19_plane_emits_recovery() {
        // G19 sets YZ plane; G5.1 should error.
        // Symmetric to g5_1_outside_xy_plane_emits_recovery (which uses G18).
        let toks = vec![
            cmd(b'G', 19, 1, Params::default()),
            cmd_with_minor(b'G', 5, Some(1), 2, p(&[
                (b'Y', 10.0), (b'Z', 1.0),
                (b'I', 3.0), (b'J', 3.0),
            ])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        // G19 emits no event, so we have 1 event total (the G5.1 ParseError).
        assert_eq!(events.len(), 1);
        match &events[0] {
            ReduceEvent::ParseError {
                line_no: 2,
                kind: ParseErrorKind::G5PlaneMismatch,
                text,
            } => {
                assert_eq!(text, "19", "expected active plane G-code 19, got {text:?}");
            }
            other => panic!("expected G5PlaneMismatch, got {other:?}"),
        }
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn g5_1_after_g18_then_g17_succeeds() {
        // G18 (sets XZ — would error if G5.1 followed) → G17 (resets to XY)
        // → G5.1 should now succeed. Asserts the plane-mismatch error path
        // is not sticky and that G17 properly resets the active plane.
        let toks = vec![
            cmd(b'G', 18, 1, Params::default()),
            cmd(b'G', 17, 2, Params::default()),
            cmd_with_minor(b'G', 5, Some(1), 3, p(&[
                (b'X', 10.0), (b'Y', 0.0),
                (b'I', 3.0), (b'J', 3.0),
                (b'F', 1500.0),
            ])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        // G18 and G17 emit no events; G5.1 emits one Curve(Quadratic) event.
        assert_eq!(events.len(), 1);
        match &events[0] {
            ReduceEvent::Curve {
                geom: CurveGeom::Quadratic { cps },
                line_no: 3,
                ..
            } => {
                assert_eq!(cps[0], [0.0, 0.0, 0.0]);
                assert_eq!(cps[1], [3.0, 3.0, 0.0]);
                assert_eq!(cps[2], [10.0, 0.0, 0.0]);
            }
            other => panic!("expected Curve(Quadratic) after G18→G17 reset, got {other:?}"),
        }
    }
```

- [ ] **Step 2: Run tests**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests::g5_1_outside_xy_plane_emits_recovery reduce::tests::g5_1_outside_g19_plane_emits_recovery reduce::tests::g5_1_after_g18_then_g17_succeeds reduce::tests::g5_1_with_both_ij_zero_is_malformed reduce::tests::g5_1_missing_j_is_malformed reduce::tests::g5_1_missing_i_is_malformed reduce::tests::g5_1_no_ij_is_malformed`
Expected: PASS — Task 15's logic already implements all of this. The G19-mismatch test exercises the same `active_plane != Plane::XY` branch as the G18 test with a different active-plane value; the G17-reset test exercises the happy path after a non-XY plane was set; the missing-I and no-I-J tests exercise the same `(params.i(), params.j())` exhaustive-match used for the missing-J case.

- [ ] **Step 3: Commit**

```bash
git add rust/geometry/src/reduce.rs
git commit -m "geometry/reduce: lock G5.1 plane-mismatch and degenerate-input rejection"
```

---

## Task 17: Pipeline — implement `Curve(Cubic)` to emit `Segment::Fitted { degree: 3 }`

**Files:**
- Modify: `rust/geometry/src/pipeline.rs`

**Why:** Wire the cubic Bézier from reduce into a `VectorNurbs<f64, 3>` and into `Segment::Fitted`. Replace the `emit_unimplemented_curve("Cubic")` stub from Task 8.

- [ ] **Step 1: Write the failing test**

Add to the `tests` module in `rust/geometry/src/pipeline.rs`:
```rust
    #[test]
    fn g5_emits_fitted_degree_3() {
        let items = collect("G1 X0 Y0 F1500\nG5 X10 Y0 I3 J3 P-3 Q3\n");
        // Expect: G1 fitted (degree 1) + G5 fitted (degree 3). No Junction
        // between them because G5 breaks the G1-tangent chain.
        let g5 = items.iter().find_map(|it| match it {
            Item::Segment(Segment::Fitted(f)) if f.degree == 3 => Some(f),
            _ => None,
        });
        let f = g5.expect("expected a degree-3 FittedSegment");
        assert_eq!(f.xyz.degree(), 3);
        let cps = f.xyz.control_points();
        assert_eq!(cps.len(), 4);
        let approx = |a: f64, b: f64| (a - b).abs() < 1e-12;
        assert!(approx(cps[0][0], 0.0) && approx(cps[0][1], 0.0));
        assert!(approx(cps[1][0], 3.0) && approx(cps[1][1], 3.0));
        assert!(approx(cps[2][0], 7.0) && approx(cps[2][1], 3.0));
        assert!(approx(cps[3][0], 10.0) && approx(cps[3][1], 0.0));
        // Knot vector [0,0,0,0,1,1,1,1].
        let knots = f.xyz.knots();
        assert_eq!(knots, &[0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]);
        // Non-rational.
        assert!(f.xyz.weights().is_none(), "G5 cubic must be non-rational");
        // Quality metric: exact construction → zero residual.
        assert_eq!(f.max_residual_mm, 0.0);
        // No JD between the G1 and the G5.
        assert!(
            !items.iter().any(|it| matches!(it, Item::Segment(Segment::Junction(_)))),
            "G5 must break the G1-tangent chain — no Junction expected, got {items:#?}"
        );
    }

    #[test]
    fn g5_is_followed_by_g1_with_no_junction() {
        // After a G5 endpoint, a G1 should not produce a JunctionDeviation
        // because G5 broke the chain.
        let items = collect("G1 X0 Y0 F1500\nG5 X10 Y0 I3 J3 P-3 Q3\nG1 X20 Y0\n");
        let junctions: Vec<_> = items.iter().filter_map(|it| match it {
            Item::Segment(Segment::Junction(_)) => Some(()),
            _ => None,
        }).collect();
        assert!(junctions.is_empty(), "expected no junctions, got {} in {items:#?}", junctions.len());
    }
```

You may need to verify `nurbs::VectorNurbs::knots()` exists; if the Layer 0 API exposes it under a different name (e.g. `knot_vector()`), substitute. The check is "knots are `[0,0,0,0,1,1,1,1]`".

If `knots()` does not exist on the public API, drop the explicit knot assertion (the curve construction has it baked in; downstream evaluation tests cover correctness implicitly).

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml pipeline::tests::g5_emits_fitted_degree_3 pipeline::tests::g5_is_followed_by_g1_with_no_junction`
Expected: FAIL — `handle_curve` for `CurveGeom::Cubic` is the `emit_unimplemented_curve` stub.

- [ ] **Step 3: Implement the `Cubic` arm of `handle_curve`**

In `rust/geometry/src/pipeline.rs`, replace the `CurveGeom::Cubic { cps }` arm of `handle_curve` (and remove the corresponding `emit_unimplemented_curve("Cubic", …)` call):
```rust
            CurveGeom::Cubic { cps } => {
                let xyz = nurbs_from_cubic(cps);
                let seg = FittedSegment {
                    xyz,
                    e: None,
                    feedrate_mm_s,
                    degree: 3,
                    max_residual_mm: 0.0,
                    source: SourceRange { start_line: line_no, end_line: line_no },
                };
                self.queue.push_back(Item::Segment(Segment::Fitted(seg)));
                // G5 breaks the G1-tangent chain (curvature-continuity principle).
                self.prev_g1_end = None;
                self.prev_g1_feedrate = None;
                self.prev_g1_dir = None;
            }
```

Add the helper near the other NURBS-construction helpers:
```rust
fn nurbs_from_cubic(cps: [[f64; 3]; 4]) -> nurbs::VectorNurbs<f64, 3> {
    nurbs::VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        cps.to_vec(),
        None,
    )
    .expect("non-rational cubic Bézier with 4 CPs and clamped knots is always valid")
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml pipeline::tests::g5_emits_fitted_degree_3 pipeline::tests::g5_is_followed_by_g1_with_no_junction`
Expected: PASS.

- [ ] **Step 5: Run full workspace tests**

Run: `cargo test --workspace --manifest-path rust/Cargo.toml`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add rust/geometry/src/pipeline.rs
git commit -m "geometry/pipeline: emit Segment::Fitted { degree: 3 } for G5 (Curve(Cubic))"
```

---

## Task 18: Pipeline — implement `Curve(Quadratic)` to emit `Segment::Fitted { degree: 2 }`

**Files:**
- Modify: `rust/geometry/src/pipeline.rs`

**Why:** Wire the quadratic non-rational Bézier from reduce (G5.1) into a degree-2 `Segment::Fitted`. Replace the `emit_unimplemented_curve("Quadratic")` stub.

Note: `Segment::Fitted` already supports any non-1 / non-3 degree per the spec (`pub degree: u8`). Degree 2 (non-rational) is distinct from `Segment::Arc` (degree 2 *rational*); both are valid in the segment enum.

- [ ] **Step 1: Write the failing test**

Add to the `tests` module in `rust/geometry/src/pipeline.rs`:
```rust
    #[test]
    fn g5_1_emits_fitted_degree_2_non_rational() {
        let items = collect("G1 X0 Y0 F1500\nG5.1 X10 Y0 I3 J3\n");
        let g5_1 = items.iter().find_map(|it| match it {
            Item::Segment(Segment::Fitted(f)) if f.degree == 2 => Some(f),
            _ => None,
        });
        let f = g5_1.expect("expected a degree-2 FittedSegment");
        assert_eq!(f.xyz.degree(), 2);
        let cps = f.xyz.control_points();
        assert_eq!(cps.len(), 3);
        let approx = |a: f64, b: f64| (a - b).abs() < 1e-12;
        assert!(approx(cps[0][0], 0.0) && approx(cps[0][1], 0.0));
        assert!(approx(cps[1][0], 3.0) && approx(cps[1][1], 3.0));
        assert!(approx(cps[2][0], 10.0) && approx(cps[2][1], 0.0));
        // Knot vector [0,0,0,1,1,1].
        let knots = f.xyz.knots();
        assert_eq!(knots, &[0.0, 0.0, 0.0, 1.0, 1.0, 1.0]);
        // Non-rational — distinguishes from rational-quadratic Arc.
        assert!(f.xyz.weights().is_none(), "G5.1 quadratic must be non-rational");
        assert_eq!(f.max_residual_mm, 0.0);
        // G5.1 also breaks the G1-tangent chain — no Junction in the output.
        assert!(
            !items.iter().any(|it| matches!(it, Item::Segment(Segment::Junction(_)))),
            "G5.1 must break the G1-tangent chain"
        );
    }

    #[test]
    fn g5_1_outside_xy_plane_yields_recovered() {
        let mut events = vec![];
        let mut p = GeometryPipeline::new(FitterParams::default());
        let items: Vec<_> = {
            let mut sink = |e: TelemetryEvent| events.push(e);
            p.process("G18\nG5.1 X10 Z1 I3 J3\n", &mut sink).collect()
        };
        let recovered = items.iter().find_map(|it| match it {
            Item::Recovered(_, Recovery::G5PlaneMismatch { line_no: 2, active_plane_g_code: 18 }) => Some(()),
            _ => None,
        });
        assert!(recovered.is_some(), "expected G5PlaneMismatch, got {items:#?}");
        assert!(matches!(
            events.last(),
            Some(TelemetryEvent::Recovery(Recovery::G5PlaneMismatch { line_no: 2, active_plane_g_code: 18 }))
        ));
    }
```

(If `knots()` does not exist on the public Layer 0 API, drop the knot-vector assertion as in Task 17.)

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml pipeline::tests::g5_1_emits_fitted_degree_2_non_rational pipeline::tests::g5_1_outside_xy_plane_yields_recovered`
Expected: FAIL — `Quadratic` arm is the stub; lexer must emit minor=Some(1) for `G5.1`.

- [ ] **Step 3: Implement the `Quadratic` arm of `handle_curve`**

In `rust/geometry/src/pipeline.rs`, replace the `CurveGeom::Quadratic { cps }` arm:
```rust
            CurveGeom::Quadratic { cps } => {
                let xyz = nurbs_from_quadratic(cps);
                let seg = FittedSegment {
                    xyz,
                    e: None,
                    feedrate_mm_s,
                    degree: 2,
                    max_residual_mm: 0.0,
                    source: SourceRange { start_line: line_no, end_line: line_no },
                };
                self.queue.push_back(Item::Segment(Segment::Fitted(seg)));
                // G5.1 also breaks the G1-tangent chain (curvature-continuity principle).
                self.prev_g1_end = None;
                self.prev_g1_feedrate = None;
                self.prev_g1_dir = None;
            }
```

Add the helper:
```rust
fn nurbs_from_quadratic(cps: [[f64; 3]; 3]) -> nurbs::VectorNurbs<f64, 3> {
    nurbs::VectorNurbs::<f64, 3>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        cps.to_vec(),
        None,
    )
    .expect("non-rational quadratic Bézier with 3 CPs and clamped knots is always valid")
}
```

Delete the `emit_unimplemented_curve` helper (no longer reachable).

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml pipeline::tests::g5_1_emits_fitted_degree_2_non_rational pipeline::tests::g5_1_outside_xy_plane_yields_recovered`
Expected: PASS.

- [ ] **Step 5: Run full workspace tests**

Run: `cargo test --workspace --manifest-path rust/Cargo.toml`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add rust/geometry/src/pipeline.rs
git commit -m "geometry/pipeline: emit Segment::Fitted { degree: 2 } for G5.1 (Curve(Quadratic))"
```

---

## Task 19: Integration test file for end-to-end G5/G5.1 pipeline behavior

**Files:**
- Create: `rust/geometry/tests/g5_reduction.rs`

**Why:** Black-box tests at the public API boundary. Run alongside the existing integration tests (`integration_orca`, etc.). These tests are independent of internal type names and exercise exactly what an external consumer would observe.

- [ ] **Step 1: Create the test file**

Create `rust/geometry/tests/g5_reduction.rs`:
```rust
//! End-to-end integration tests for G5 / G5.1 reduction (build-order Step 3).
//! Black-box: drives `GeometryPipeline::process` against synthetic G-code
//! strings and asserts on the public `Item` / `Segment` / `Recovery` /
//! `TelemetryEvent` surface.

use geometry::{
    FittedSegment, FitterParams, GeometryPipeline, Item, Recovery, Segment,
    TelemetryEvent,
};

fn process(text: &str) -> (Vec<Item>, Vec<TelemetryEvent>) {
    let mut p = GeometryPipeline::new(FitterParams::default());
    let mut events = vec![];
    let items: Vec<_> = {
        let mut sink = |e: TelemetryEvent| events.push(e);
        p.process(text, &mut sink).collect()
    };
    (items, events)
}

fn approx(a: f64, b: f64) -> bool { (a - b).abs() < 1e-12 }

#[test]
fn single_g5_emits_one_cubic_fitted_segment() {
    let (items, _events) = process("G1 X0 Y0 F1500\nG5 X10 Y0 I3 J3 P-3 Q3\n");
    let cubics: Vec<&FittedSegment> = items
        .iter()
        .filter_map(|it| match it {
            Item::Segment(Segment::Fitted(f)) if f.degree == 3 => Some(f),
            _ => None,
        })
        .collect();
    assert_eq!(cubics.len(), 1, "expected exactly one degree-3 Fitted, got {} in {items:#?}", cubics.len());
    let f = cubics[0];
    let cps = f.xyz.control_points();
    assert!(approx(cps[1][0], 3.0) && approx(cps[1][1], 3.0));
    assert!(approx(cps[2][0], 7.0) && approx(cps[2][1], 3.0));
    assert!(f.xyz.weights().is_none());
    assert!(approx(f.max_residual_mm, 0.0));
}

#[test]
fn single_g5_1_emits_one_quadratic_non_rational_fitted_segment() {
    let (items, _events) = process("G1 X0 Y0 F1500\nG5.1 X10 Y0 I3 J3\n");
    let quads: Vec<&FittedSegment> = items
        .iter()
        .filter_map(|it| match it {
            Item::Segment(Segment::Fitted(f)) if f.degree == 2 => Some(f),
            _ => None,
        })
        .collect();
    assert_eq!(quads.len(), 1, "expected exactly one degree-2 Fitted, got {} in {items:#?}", quads.len());
    let f = quads[0];
    let cps = f.xyz.control_points();
    assert!(approx(cps[1][0], 3.0) && approx(cps[1][1], 3.0));
    assert!(f.xyz.weights().is_none(), "G5.1 must be non-rational (distinguishes from Arc)");
}

#[test]
fn g5_chain_three_lines_no_junctions_between() {
    let (items, _events) = process(
        "G1 X0 Y0 F1500\n\
         G5 X10 Y0 I3 J3 P-3 Q3\n\
         G5 X20 Y0 P-2 Q2\n\
         G5 X30 Y0 P0 Q0\n",
    );
    let cubics_count = items.iter().filter(|it| matches!(it, Item::Segment(Segment::Fitted(f)) if f.degree == 3)).count();
    let junctions_count = items.iter().filter(|it| matches!(it, Item::Segment(Segment::Junction(_)))).count();
    assert_eq!(cubics_count, 3, "expected 3 cubic G5 segments");
    assert_eq!(junctions_count, 0, "G5↔G5 boundaries should produce no junctions");
}

#[test]
fn g5_followed_by_g1_breaks_chain_no_junction() {
    let (items, _events) = process(
        "G1 X0 Y0 F1500\n\
         G5 X10 Y0 I3 J3 P-3 Q3\n\
         G1 X20 Y0\n",
    );
    let junctions_count = items.iter().filter(|it| matches!(it, Item::Segment(Segment::Junction(_)))).count();
    assert_eq!(junctions_count, 0, "G5→G1 boundary should not produce a junction");
}

#[test]
fn g5_chain_break_then_implicit_tangent_emits_recovery() {
    let (items, events) = process(
        "G1 X0 Y0 F1500\n\
         G5 X10 Y0 I3 J3 P-3 Q3\n\
         G1 X11 Y0\n\
         G5 X20 Y0 P-2 Q2\n",
    );
    let recoveries: Vec<_> = items.iter().filter_map(|it| match it {
        Item::Recovered(_, r @ Recovery::G5MissingTangent { .. }) => Some(r.clone()),
        _ => None,
    }).collect();
    assert_eq!(recoveries.len(), 1, "expected one G5MissingTangent recovery, got {items:#?}");
    let recovery_in_sink = events.iter().any(|e| matches!(e, TelemetryEvent::Recovery(Recovery::G5MissingTangent { .. })));
    assert!(recovery_in_sink, "Recovery should also appear in sink (dual-emit)");
}

#[test]
fn g5_1_outside_g17_plane_emits_recovery() {
    let (items, _events) = process("G18\nG5.1 X10 Z1 I3 J3\n");
    let recoveries: Vec<_> = items.iter().filter_map(|it| match it {
        Item::Recovered(_, r @ Recovery::G5PlaneMismatch { .. }) => Some(r.clone()),
        _ => None,
    }).collect();
    assert_eq!(recoveries.len(), 1, "expected one G5PlaneMismatch recovery");
    match &recoveries[0] {
        Recovery::G5PlaneMismatch { active_plane_g_code: 18, line_no: 2 } => {}
        other => panic!("expected G5PlaneMismatch with active_plane_g_code=18, got {other:?}"),
    }
}

#[test]
fn g5_with_z_motion_interpolates_z_at_thirds() {
    let (items, _events) = process(
        "G1 X0 Y0 Z0 F1500\nG5 X10 Y0 Z0.3 I3 J3 P-3 Q3\n",
    );
    let f = items.iter().find_map(|it| match it {
        Item::Segment(Segment::Fitted(f)) if f.degree == 3 => Some(f),
        _ => None,
    }).expect("expected a degree-3 Fitted");
    let cps = f.xyz.control_points();
    assert!(approx(cps[0][2], 0.0));
    assert!(approx(cps[1][2], 0.1));
    assert!(approx(cps[2][2], 0.2));
    assert!(approx(cps[3][2], 0.3));
}

#[test]
fn g5_1_with_z_motion_interpolates_z_at_midpoint() {
    let (items, _events) = process(
        "G1 X0 Y0 Z0 F1500\nG5.1 X10 Y0 Z0.4 I3 J3\n",
    );
    let f = items.iter().find_map(|it| match it {
        Item::Segment(Segment::Fitted(f)) if f.degree == 2 => Some(f),
        _ => None,
    }).expect("expected a degree-2 Fitted");
    let cps = f.xyz.control_points();
    assert!(approx(cps[0][2], 0.0));
    assert!(approx(cps[1][2], 0.2));
    assert!(approx(cps[2][2], 0.4));
}

#[test]
fn g5_chain_preserved_by_m_codes_and_t_codes() {
    let (items, _events) = process(
        "G1 X0 Y0 F1500\n\
         G5 X10 Y0 I3 J3 P-3 Q3\n\
         M104 S210\n\
         T0\n\
         G5 X20 Y0 P-2 Q2\n",
    );
    let cubics_count = items.iter().filter(|it| matches!(it, Item::Segment(Segment::Fitted(f)) if f.degree == 3)).count();
    assert_eq!(cubics_count, 2, "expected 2 cubics — modal chain should survive M and T");
    let recoveries_count = items.iter().filter(|it| matches!(it, Item::Recovered(_, Recovery::G5MissingTangent { .. }))).count();
    assert_eq!(recoveries_count, 0, "expected no missing-tangent recoveries");
}
```

- [ ] **Step 2: Run the integration tests**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml --test g5_reduction`
Expected: PASS — all 9 tests.

- [ ] **Step 3: Run the full workspace test suite**

Run: `cargo test --workspace --manifest-path rust/Cargo.toml`
Expected: PASS.

- [ ] **Step 4: Run clippy on the workspace**

Run: `cargo clippy --workspace --manifest-path rust/Cargo.toml --all-targets -- -D warnings`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/geometry/tests/g5_reduction.rs
git commit -m "geometry: integration tests for G5/G5.1 reduction (Step 3)"
```

---

## Task 20: Mark CLAUDE.md build-order Step 3 as done

**Files:**
- Modify: `CLAUDE.md`

**Why:** The build-order checklist is the single source of truth for "what's shipped." Mark Step 3 complete and capture an evidence link in the plan-changes log.

- [ ] **Step 1: Tick the checkbox**

In `CLAUDE.md`, find the build-order list (around line 200) and change:
```
3. [ ] **G5 / G5.1 reduction** — closes the remaining gap in step 2. […]
```
to:
```
3. [x] **G5 / G5.1 reduction** — closes the remaining gap in step 2. […]
```

(Leave the body text unchanged.)

- [ ] **Step 2: Append a 2026-04-27 entry to the plan-changes log**

Under the existing 2026-04-27 entry, add (or append a new entry if the existing one has already been "closed"):
```
- **Build-order Step 3 (G5 / G5.1 reduction): completed.** Implementation per `docs/superpowers/plans/2026-04-27-layer-1-g5-reduction.md`. Reduce + pipeline now construct exact non-rational NURBS for G5 (degree-3, 4 CPs) and G5.1 (degree-2, 3 CPs); G5 modal-chain implicit-tangent rule, G5.1 active-plane validation, and curvature-continuity G1-chain break all in place. ReduceEvent shape refactored to ReduceEvent::Curve(CurveGeom, …) with fixed-size-array variants per the Q5 brainstorm decision.

**Evidence:** Plan + commits on this branch. Integration tests at `rust/geometry/tests/g5_reduction.rs`.
```

- [ ] **Step 3: Run the workspace tests one more time as a smoke check**

Run: `cargo test --workspace --manifest-path rust/Cargo.toml`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md
git commit -m "CLAUDE.md: mark Step 3 (G5/G5.1 reduction) complete; log evidence link"
```

---

## Self-review checklist

The plan-writer ran the following checks before finalizing:

**Spec coverage** (cross-referencing the brainstormer task description):

| Required topic | Plan task |
|---|---|
| G5/G5.1 RS274NGC semantics with LinuxCNC convention | Task 12 (G5 explicit), Task 15 (G5.1 base case), explanatory headers in Task 4 |
| Reduce-stage construction with knot vector / weights / Z handling | Task 12 (G5 control-point recipe + Z-at-thirds), Task 15 (G5.1 + Z-at-half) |
| Modal G5 chain (`prev_g5_pq` carried; cleared by non-G5 motion; preserved by M / T / plane select; cleared by G92) | Task 3 (state field + note on G5 success-arm dual effect), Task 6 (G1 clears), Task 7 (arc clears), Task 12 (G5 sets + clears G1 chain), Task 12.5 (defensive: G5 error paths clear), Task 13 (regression tests for chain preservation, plus G92 broken-chain test) |
| `Segment::Fitted { degree: 3 }` per CLAUDE.md, plus G5.1 → degree 2 | Task 17, Task 18 |
| Validation rules (degenerate / missing tangents) | Task 12 (G5), Task 15 (G5.1), Task 13 + Task 16 (regression tests) |
| Test plan, synthetic-only | Tasks 12–18 unit tests, Task 19 integration tests |
| Telemetry — no new event types | Confirmed in plan header; Recovery dual-emits via the existing `TelemetryEvent::Recovery` mechanism — no schema additions required |
| `ReduceEvent` refactor to `Curve(CurveGeom, …)` per Q5 | Tasks 4–9 |
| Active-plane tracking for G5.1 | Tasks 1–2 |
| Curvature-continuity break of G1 chain on G5/G5.1 | Tasks 17 / 18 (`prev_g1_dir = None`); Task 19 black-box assertion |
| Acceptance criteria runnable | `cargo test --workspace` and `cargo clippy --workspace -- -D warnings` invoked at every commit boundary; integration tests in Task 19 directly assert the verbatim acceptance examples in the plan header |

**Placeholder scan:** No "TBD"/"TODO"/"implement later" outside the `emit_unimplemented_curve` stub explicitly resolved in Tasks 17 and 18.

**Type consistency:** `CurveGeom` is defined once in Task 4; `nurbs_from_linear` / `nurbs_from_rational_quadratic` / `nurbs_from_quadratic` / `nurbs_from_cubic` are the four NURBS-construction helpers (Tasks 8, 17, 18). `Recovery::G5MissingTangent` and `Recovery::G5PlaneMismatch` are introduced in Task 10 and consumed in Task 11.

**Risk / known-uncertain items flagged for the executor:**

1. **Layer 0 `VectorNurbs` API surface — `knots()` accessor.** Tasks 17 and 18's tests assert against `xyz.knots()`. If the public accessor is named differently (e.g. `knot_vector()`), the executor should adapt the assertion to match the actual API and drop the literal-vector check if no accessor is exposed; the construction is correct regardless.
2. **`Token::Marker` line numbers from comment-only lines.** Existing `g5_1_outside_xy_plane_yields_recovery` integration test (Task 19) assumes that `G18\nG5.1 …\n` has the G18 on line 1 and G5.1 on line 2. The lexer's line numbering is verified for command-only lines in existing tests; a quick grep confirms line numbers are 1-indexed. If they aren't, adapt the `line_no: 2` assertion.
3. **G5 error paths must clear `prev_g5_pq`.** Covered by Task 12.5 (defensive subtask with its own failing-test-first cycle and acceptance criterion). The G5.1 arm's error paths (Task 15) clear the chain inline as part of that task.

---

## Plan complete

The plan saves and commits at the end of every task. Test runs are mandatory before commit; clippy runs at workspace boundaries (Tasks 9, 19). No task is permitted to commit a workspace with failing tests.
