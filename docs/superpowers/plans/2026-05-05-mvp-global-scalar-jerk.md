# MVP global scalar jerk — implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the bridge config layer's per-axis jerk default `[max_accel*2, max_accel*2, max_z_accel*2]` with a single global scalar `max_jerk` (default `max_accel*2`), exposed as a `[printer]` config key and a `SET_VELOCITY_LIMIT JERK=` g-code parameter, to unblock the Phase 4 G28 X homing stall.

**Architecture:** Bridge-config-layer-only change. `temporal::Limits.j_max: [f64; 3]` API and the planner's two-stage SLP machinery are untouched; the bridge populates `j_max` uniformly from a single `max_jerk` field. PyO3 boundary uses `Option<f64>` so klippy can express "use default." Runtime updates via `SET_VELOCITY_LIMIT` preserve `max_jerk` unless `JERK=` is explicitly passed.

**Tech Stack:** Rust (workspace crates `motion-bridge`, `trajectory`, `temporal`), PyO3 boundary, Klippy Python, Docker-based klippy-in-loop sim.

**Spec:** [`docs/superpowers/specs/2026-05-05-mvp-global-scalar-jerk-design.md`](../specs/2026-05-05-mvp-global-scalar-jerk-design.md)

---

## File map

**Create:**
- `rust/trajectory/tests/homing_300mm_pure_x.rs` — Rust-layer regression test proving Stage 1 SLP convergence at uniform `j_max=[6000;3]` on the 300 mm pure-X homing fixture. Architectural correctness gate.

**Modify:**
- `rust/motion-bridge/src/config.rs` — add `max_jerk: f64` field to `PlannerLimits`; update `PlannerConfig::default()`; rewrite `to_temporal_limits` to emit uniform `j_max`; add unit tests.
- `rust/motion-bridge/src/bridge.rs:901` — extend `init_planner` PyO3 signature with `max_jerk: Option<f64>` (None → derive from `max_accel`).
- `rust/motion-bridge/src/bridge.rs:1437` — extend `update_limits` PyO3 signature with `max_jerk: Option<f64>` (None → preserve stored value).
- `klippy/motion_bridge.py:179` — extend `MotionBridgeWrapper.init_planner` with `max_jerk=None`.
- `klippy/motion_bridge.py:223` — extend `MotionBridgeWrapper.update_limits` with `max_jerk=None`.
- `klippy/motion_toolhead.py:140` — parse optional `[printer] max_jerk` key.
- `klippy/motion_toolhead.py:444` — thread `self.max_jerk` through `update_limits` callsite (limit-clamp recompute path).
- `klippy/motion_toolhead.py:469` — extend `cmd_SET_VELOCITY_LIMIT` to parse `JERK=` and forward.
- `klippy/motion_toolhead.py:635` — thread `self.max_jerk` into the `init_planner` callsite.
- `tools/sim_klippy/test_home_x.py` — fix the variable-shadowing bug (existing test exits 0 even on G28 failure); add stricter assertions on M114 X position and absence of stall string in `klippy.log`.
- `rust/temporal/src/topp/constraints.rs:236-247` — append maintainer-note paragraph about config-layer collapse.
- `docs/superpowers/plan-changes-log.md` — add 2026-05-05 entry.

---

## Task 1: Rust-layer regression test — proves Stage 1 SLP converges at the new default

**Files:**
- Create: `rust/trajectory/tests/homing_300mm_pure_x.rs`

This test is the architectural gate. It must pass on `sota-motion` HEAD *before* any other change, demonstrating that the Rust trajectory layer already converges at uniform `j_max=[6000;3]` on the homing fixture. The bug isn't in the Rust layer — it's in the bridge config layer feeding non-uniform `[6000, 6000, 200]`.

- [ ] **Step 1: Write the test file**

Create `rust/trajectory/tests/homing_300mm_pure_x.rs`:

```rust
//! Regression test: 300 mm pure-X collinear cubic at 50 mm/s with uniform
//! `j_max = [6000; 3]` must converge at the trajectory layer.
//!
//! This pins the architectural correctness of the MVP global-scalar-jerk
//! change at the bridge config layer (see
//! `docs/superpowers/specs/2026-05-05-mvp-global-scalar-jerk-design.md`).
//! The bug being fixed is in `rust/motion-bridge/src/config.rs` (non-uniform
//! `j_max` defaults causing `J_path = min(j_max) = 200` to dominate); this
//! test proves the trajectory layer is healthy when fed the uniform value
//! the new config produces.

use geometry::segment::EMode;
use nurbs::VectorNurbs;
use temporal::multi::{GridStrategy, JoiningStatus, SegmentInput};
use trajectory::{
    AxisShaper, ELimits, RequiredShaper, ShapeBatchInput, ShapeError, ShapeSegmentInput,
    ShaperConfig,
};

fn pure_x_300mm_collinear_cubic() -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [-300.0, 0.0, 0.0],
            [-200.0, 0.0, 0.0],
            [-100.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ],
        None,
    )
    .unwrap()
}

fn sim_homing_limits() -> temporal::Limits {
    // Matches sim's printer.cfg: max_velocity=300, max_accel=3000,
    // max_z_velocity=15, max_z_accel=100, square_corner_velocity=5.
    // The MVP change makes j_max uniform at max_accel * 2.0 = 6000.
    temporal::Limits::new(
        [300.0, 300.0, 15.0],
        [3000.0, 3000.0, 100.0],
        [6000.0, 6000.0, 6000.0],
        5.0_f64.powi(2) / (3000.0 * 0.5),
    )
}

#[test]
fn homing_300mm_pure_x_at_uniform_jerk_converges() {
    let curve = pure_x_300mm_collinear_cubic();

    let segments = [ShapeSegmentInput {
        temporal: SegmentInput {
            curve: &curve,
            limits: sim_homing_limits(),
            trailing_junction_chord_tolerance_mm: 0.05,
        },
        e_mode: EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        feedrate_mm_s: 50.0,
    }];

    let input = ShapeBatchInput {
        segments: &segments,
        grid_strategy: GridStrategy::Adaptive {
            min_n: 20,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        },
        worker_threads: 1,
        shaper: ShaperConfig {
            x: RequiredShaper::SmoothMzv { frequency_hz: 50.0 },
            y: RequiredShaper::SmoothMzv { frequency_hz: 50.0 },
            z: AxisShaper::Passthrough,
        },
        fit_tolerance_mm: 0.005,
        beta_max_iters: 10,
        beta_convergence_ratio: 0.05,
        e_limits: ELimits {
            v_max: 50.0,
            a_max: 5000.0,
        },
    };

    let result = trajectory::shape_batch(&input);

    match result {
        Ok(output) => {
            assert!(
                matches!(output.temporal_status, JoiningStatus::Converged),
                "expected JoiningStatus::Converged, got {:?}",
                output.temporal_status
            );
            assert_eq!(output.segments.len(), 1);
            assert!(
                output.beta_warning.is_none(),
                "unexpected beta warning: {:?}",
                output.beta_warning
            );
        }
        Err(ShapeError::TemporalJoining(status)) => {
            panic!(
                "regression: 300 mm pure-X at j_max=[6000;3] failed temporal joining: {status:?}"
            );
        }
        Err(err) => panic!("unexpected shape_batch error: {err:?}"),
    }
}
```

- [ ] **Step 2: Run the test — must pass on current HEAD**

```bash
cargo test -p trajectory --test homing_300mm_pure_x -- --nocapture
```

Expected: `test result: ok. 1 passed`. If this fails, the architectural premise of the MVP change is wrong and we stop here to investigate before any config-layer work.

- [ ] **Step 3: Commit**

```bash
git add rust/trajectory/tests/homing_300mm_pure_x.rs
git commit -m "test(trajectory): pin Stage 1 SLP convergence at uniform j_max=[6000;3]

Architectural gate for MVP global-scalar-jerk change. Test runs the
captured Phase 4 homing fixture (300 mm pure-X collinear cubic at
50 mm/s with smooth-MZV@50Hz) through trajectory::shape_batch with
the uniform j_max the new bridge config will produce, asserting
JoiningStatus::Converged. Spec:
docs/superpowers/specs/2026-05-05-mvp-global-scalar-jerk-design.md"
```

---

## Task 2: Add `max_jerk` field to `PlannerLimits`

**Files:**
- Modify: `rust/motion-bridge/src/config.rs`

- [ ] **Step 1: Write failing tests first**

Add at the bottom of `rust/motion-bridge/src/config.rs` (or in a new `#[cfg(test)] mod tests { ... }` if one doesn't exist; check the file for an existing test module before creating a new one):

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_max_jerk_is_2x_max_accel() {
        let cfg = PlannerConfig::default();
        let expected = cfg.limits.max_accel * 2.0;
        assert_eq!(cfg.limits.max_jerk, expected);
    }

    #[test]
    fn to_temporal_limits_emits_uniform_jerk() {
        let limits = PlannerLimits {
            max_velocity: 300.0,
            max_accel: 3000.0,
            max_z_velocity: 15.0,
            max_z_accel: 100.0,
            square_corner_velocity: 5.0,
            max_jerk: 7500.0,
        };
        let temp = limits.to_temporal_limits();
        assert_eq!(temp.j_max, [7500.0, 7500.0, 7500.0]);
    }

    #[test]
    fn to_temporal_limits_preserves_per_axis_velocity_and_accel() {
        let limits = PlannerLimits {
            max_velocity: 250.0,
            max_accel: 2500.0,
            max_z_velocity: 10.0,
            max_z_accel: 80.0,
            square_corner_velocity: 4.0,
            max_jerk: 5000.0,
        };
        let temp = limits.to_temporal_limits();
        assert_eq!(temp.v_max, [250.0, 250.0, 10.0]);
        assert_eq!(temp.a_max, [2500.0, 2500.0, 80.0]);
    }
}
```

- [ ] **Step 2: Run tests — must fail with "no field `max_jerk`"**

```bash
cargo test -p motion-bridge --lib config::tests
```

Expected: build error `no field 'max_jerk' on type 'PlannerLimits'`.

- [ ] **Step 3: Add the field, default, and rewrite `to_temporal_limits`**

In `rust/motion-bridge/src/config.rs`, modify the struct, default, and `to_temporal_limits`:

```rust
#[derive(Debug, Clone, Copy)]
pub struct PlannerLimits {
    pub max_velocity: f64,
    pub max_accel: f64,
    pub max_z_velocity: f64,
    pub max_z_accel: f64,
    pub square_corner_velocity: f64,
    /// Global scalar jerk limit (mm/s³). Populates all three axes of
    /// `temporal::Limits.j_max` uniformly. See
    /// `docs/superpowers/specs/2026-05-05-mvp-global-scalar-jerk-design.md`
    /// for why per-axis jerk distinction is deferred.
    pub max_jerk: f64,
}

impl PlannerLimits {
    /// Convert to temporal's `Limits` struct.
    ///
    /// Jerk is uniform across X/Y/Z by design (MVP scope). Per-axis jerk
    /// distinction is deferred alongside the per-axis Cartesian-jerk SOCP
    /// relaxation work; see the maintainer warning at
    /// `rust/temporal/src/topp/constraints.rs:236-247`.
    pub fn to_temporal_limits(&self) -> Limits {
        Limits::new(
            [self.max_velocity, self.max_velocity, self.max_z_velocity],
            [self.max_accel, self.max_accel, self.max_z_accel],
            [self.max_jerk, self.max_jerk, self.max_jerk],
            self.square_corner_velocity.powi(2) / (self.max_accel * 0.5),
        )
    }
}
```

In `impl Default for PlannerConfig`, add the field:

```rust
impl Default for PlannerConfig {
    fn default() -> Self {
        Self {
            limits: PlannerLimits {
                max_velocity: 300.0,
                max_accel: 3000.0,
                max_z_velocity: 15.0,
                max_z_accel: 100.0,
                square_corner_velocity: 5.0,
                max_jerk: 6000.0, // = max_accel * 2.0
            },
            shaper: ShaperConfig {
                x: RequiredShaper::SmoothMzv { frequency_hz: 50.0 },
                y: RequiredShaper::SmoothMzv { frequency_hz: 50.0 },
                z: AxisShaper::Passthrough,
            },
            e_limits: ELimits {
                v_max: 50.0,
                a_max: 5000.0,
            },
            window_capacity: 32,
            beta_max_iters: 10,
            beta_convergence_ratio: 0.05,
            fit_tolerance_mm: 0.005,
            worker_threads: 3,
        }
    }
}
```

- [ ] **Step 4: Run tests — must pass**

```bash
cargo test -p motion-bridge --lib config::tests
```

Expected: `test result: ok. 3 passed`.

- [ ] **Step 5: Run full motion-bridge test suite to catch breakage**

```bash
cargo test -p motion-bridge
```

Expected: green. If `PlannerLimits { ... }` struct literals elsewhere in the crate (e.g., bridge tests) trip on the missing field, fix them inline by adding `max_jerk: <value>` (matching the existing `max_accel * 2.0` derivation per existing test fixture).

- [ ] **Step 6: Commit**

```bash
git add rust/motion-bridge/src/config.rs
git commit -m "feat(motion-bridge): add max_jerk to PlannerLimits

PlannerLimits gains a max_jerk: f64 field, defaulted to max_accel*2.0
(6000 mm/s³ at the standard sim defaults). to_temporal_limits emits
uniform j_max=[max_jerk; 3], replacing the prior per-axis derivation
that was making j_max[Z] = 2*max_z_accel = 200 dominate J_path on
pure-X moves and stalling Phase 4 homing.

Spec: docs/superpowers/specs/2026-05-05-mvp-global-scalar-jerk-design.md"
```

---

## Task 3: Extend `init_planner` PyO3 signature with `max_jerk: Option<f64>`

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs:901-944`

- [ ] **Step 1: Write failing test for the new signature**

In `rust/motion-bridge/src/config.rs` test module (Task 2's `mod tests`), add a test for the resolution helper that the new code will need:

```rust
    #[test]
    fn resolve_max_jerk_with_some_uses_provided_value() {
        let cfg = PlannerLimits::resolve_max_jerk(Some(8000.0), /*max_accel*/ 3000.0);
        assert_eq!(cfg, 8000.0);
    }

    #[test]
    fn resolve_max_jerk_with_none_derives_from_max_accel() {
        let cfg = PlannerLimits::resolve_max_jerk(None, /*max_accel*/ 2500.0);
        assert_eq!(cfg, 5000.0);
    }
```

- [ ] **Step 2: Run tests — must fail with "no associated function `resolve_max_jerk`"**

```bash
cargo test -p motion-bridge --lib config::tests::resolve
```

Expected: build error `no associated function 'resolve_max_jerk'`.

- [ ] **Step 3: Add the resolution helper to `PlannerLimits`**

In `rust/motion-bridge/src/config.rs`, inside `impl PlannerLimits`:

```rust
    /// Resolve a caller-supplied optional `max_jerk` into a concrete value.
    /// `None` means "use the default tied to the caller-supplied
    /// `max_accel`," which is `max_accel * 2.0`. This is the canonical
    /// init-time and runtime-update default rule.
    pub fn resolve_max_jerk(supplied: Option<f64>, max_accel: f64) -> f64 {
        supplied.unwrap_or(max_accel * 2.0)
    }
```

- [ ] **Step 4: Run helper tests — must pass**

```bash
cargo test -p motion-bridge --lib config::tests::resolve
```

Expected: `test result: ok. 2 passed`.

- [ ] **Step 5: Update `init_planner` PyO3 signature in `bridge.rs`**

In `rust/motion-bridge/src/bridge.rs:892-916`, update both the `#[pyo3(signature = ...)]` block and the function signature:

```rust
    #[pyo3(signature = (
        max_velocity,
        max_accel,
        max_z_velocity,
        max_z_accel,
        square_corner_velocity,
        shaper_type_x,
        shaper_freq_x,
        shaper_type_y,
        shaper_freq_y,
        octopus_handle,
        f446_handle,
        window_capacity = 32,
        beta_max_iters = 10,
        max_jerk = None,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn init_planner(
        &self,
        max_velocity: f64,
        max_accel: f64,
        max_z_velocity: f64,
        max_z_accel: f64,
        square_corner_velocity: f64,
        shaper_type_x: &str,
        shaper_freq_x: f64,
        shaper_type_y: &str,
        shaper_freq_y: f64,
        octopus_handle: u32,
        f446_handle: u32,
        window_capacity: usize,
        beta_max_iters: u8,
        max_jerk: Option<f64>,
    ) -> PyResult<()> {
```

Then in the body at `bridge.rs:932-938`, update `PlannerLimits` construction:

```rust
        let limits = PlannerLimits {
            max_velocity,
            max_accel,
            max_z_velocity,
            max_z_accel,
            square_corner_velocity,
            max_jerk: PlannerLimits::resolve_max_jerk(max_jerk, max_accel),
        };
```

- [ ] **Step 6: Run motion-bridge test suite**

```bash
cargo test -p motion-bridge
```

Expected: green. The PyO3 signature change is additive (new keyword arg defaulting to `None`); no Rust caller breaks. Python side is updated in Task 5.

- [ ] **Step 7: Commit**

```bash
git add rust/motion-bridge/src/config.rs rust/motion-bridge/src/bridge.rs
git commit -m "feat(motion-bridge): init_planner accepts max_jerk: Option<f64>

PyO3 boundary now accepts max_jerk as an optional keyword argument.
Python None maps to Rust None, which PlannerLimits::resolve_max_jerk
resolves to max_accel*2.0 (the canonical default). When Some, the
value is stored as-is on cfg.limits.max_jerk."
```

---

## Task 4: Extend `update_limits` with optional `max_jerk` (preserve-on-absence)

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs:1437-1455`

- [ ] **Step 1: Write failing tests for preserve-vs-override semantics**

The cleanest place is a new file `rust/motion-bridge/tests/update_limits_max_jerk.rs`. Check first whether bridge has a tests/ directory; if not, create one:

```bash
ls rust/motion-bridge/tests/
```

If the directory exists, add the test there. If not, create:

```rust
//! `update_limits` max_jerk preserve-vs-override behavior.

// NOTE: This is an integration test against the public crate API, not the
// PyO3 wrapper. We exercise PlannerLimits directly because the PyO3
// `update_limits` body is a thin shim around field mutation; the contract
// we want to pin is "None preserves, Some overrides."

use motion_bridge::config::PlannerLimits;

fn baseline_limits(max_jerk: f64) -> PlannerLimits {
    PlannerLimits {
        max_velocity: 300.0,
        max_accel: 3000.0,
        max_z_velocity: 15.0,
        max_z_accel: 100.0,
        square_corner_velocity: 5.0,
        max_jerk,
    }
}

#[test]
fn apply_optional_max_jerk_none_preserves() {
    let mut limits = baseline_limits(6000.0);
    limits.apply_optional_max_jerk(None);
    assert_eq!(limits.max_jerk, 6000.0);
}

#[test]
fn apply_optional_max_jerk_some_overrides() {
    let mut limits = baseline_limits(6000.0);
    limits.apply_optional_max_jerk(Some(9000.0));
    assert_eq!(limits.max_jerk, 9000.0);
}
```

If `motion_bridge::config::PlannerLimits` is not currently `pub` from the crate root, also add `pub mod config;` to `rust/motion-bridge/src/lib.rs` if missing — check first with `grep -n "pub mod config" rust/motion-bridge/src/lib.rs`. (Existing visibility is fine if `cargo check -p motion-bridge --tests` succeeds at Step 2 before the helper is added.)

- [ ] **Step 2: Run tests — must fail with "no method `apply_optional_max_jerk`"**

```bash
cargo test -p motion-bridge --test update_limits_max_jerk
```

Expected: build error.

- [ ] **Step 3: Add the helper to `PlannerLimits`**

In `rust/motion-bridge/src/config.rs`:

```rust
    /// Apply an optional `max_jerk` update with preserve-on-`None` semantics.
    /// This is the canonical runtime-update rule: `SET_VELOCITY_LIMIT
    /// ACCEL=...` without a `JERK=` parameter must NOT recompute jerk from
    /// the new accel — `max_jerk` is a real config knob, not a derived
    /// quantity.
    pub fn apply_optional_max_jerk(&mut self, supplied: Option<f64>) {
        if let Some(j) = supplied {
            self.max_jerk = j;
        }
    }
```

- [ ] **Step 4: Run tests — must pass**

```bash
cargo test -p motion-bridge --test update_limits_max_jerk
```

Expected: `test result: ok. 2 passed`.

- [ ] **Step 5: Update `update_limits` PyO3 signature in `bridge.rs`**

In `rust/motion-bridge/src/bridge.rs:1437-1455`:

```rust
    /// Update velocity / acceleration / jerk limits at runtime
    /// (klippy `SET_VELOCITY_LIMIT`).
    ///
    /// `max_jerk = None` preserves the stored jerk; `Some(j)` overrides.
    /// Per spec, changing accel does NOT implicitly recompute jerk —
    /// `max_jerk` is a real config knob.
    #[pyo3(signature = (max_velocity, max_accel, max_jerk = None))]
    fn update_limits(
        &self,
        max_velocity: f64,
        max_accel: f64,
        max_jerk: Option<f64>,
    ) -> PyResult<()> {
        let mut cfg = self.planner_config.lock().unwrap();
        cfg.limits.max_velocity = max_velocity;
        cfg.limits.max_accel = max_accel;
        cfg.limits.apply_optional_max_jerk(max_jerk);
        let new_limits = cfg.limits;
        drop(cfg);

        let planner_guard = self.planner.lock().unwrap();
        let planner = planner_guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err(
                "planner not initialized — call init_planner first",
            )
        })?;
        planner.update_limits(new_limits).map_err(planner_err)
    }
```

- [ ] **Step 6: Run motion-bridge test suite**

```bash
cargo test -p motion-bridge
```

Expected: green.

- [ ] **Step 7: Commit**

```bash
git add rust/motion-bridge/src/config.rs rust/motion-bridge/src/bridge.rs rust/motion-bridge/tests/update_limits_max_jerk.rs
git commit -m "feat(motion-bridge): update_limits accepts optional max_jerk

PyO3 update_limits gains a third arg max_jerk: Option<f64>. None
preserves the currently stored value (so SET_VELOCITY_LIMIT
ACCEL=... does NOT implicitly retune jerk); Some overrides. Helper
PlannerLimits::apply_optional_max_jerk encapsulates the semantics."
```

---

## Task 5: Extend Klippy `MotionBridgeWrapper` to forward optional `max_jerk`

**Files:**
- Modify: `klippy/motion_bridge.py:179` (init_planner)
- Modify: `klippy/motion_bridge.py:223` (update_limits)

- [ ] **Step 1: Update `MotionBridgeWrapper.init_planner`**

In `klippy/motion_bridge.py:179-209`:

```python
    def init_planner(
        self,
        max_velocity,
        max_accel,
        max_z_velocity,
        max_z_accel,
        square_corner_velocity,
        shaper_type_x,
        shaper_freq_x,
        shaper_type_y,
        shaper_freq_y,
        octopus_handle,
        f446_handle,
        window_capacity=32,
        beta_max_iters=10,
        max_jerk=None,
    ):
        return self._bridge.init_planner(
            max_velocity,
            max_accel,
            max_z_velocity,
            max_z_accel,
            square_corner_velocity,
            shaper_type_x,
            shaper_freq_x,
            shaper_type_y,
            shaper_freq_y,
            octopus_handle,
            f446_handle,
            window_capacity,
            beta_max_iters,
            max_jerk,
        )
```

- [ ] **Step 2: Update `MotionBridgeWrapper.update_limits`**

In `klippy/motion_bridge.py:223-224`:

```python
    def update_limits(self, max_velocity, max_accel, max_jerk=None):
        return self._bridge.update_limits(max_velocity, max_accel, max_jerk)
```

- [ ] **Step 3: Verify klippy import doesn't crash**

```bash
cd /Users/daniladergachev/Developer/kalico
python3 -c "import sys; sys.path.insert(0, 'klippy'); import motion_bridge; print('ok')"
```

Expected: `ok` printed.

- [ ] **Step 4: Commit**

```bash
git add klippy/motion_bridge.py
git commit -m "feat(klippy): MotionBridgeWrapper forwards optional max_jerk

init_planner and update_limits gain a max_jerk=None keyword argument
that forwards to the PyO3 bridge as Option<f64>. Existing call sites
that don't pass max_jerk continue to work; the Rust default applies."
```

---

## Task 6: Klippy parses `[printer] max_jerk` and `SET_VELOCITY_LIMIT JERK=`

**Files:**
- Modify: `klippy/motion_toolhead.py:140` area (config parsing)
- Modify: `klippy/motion_toolhead.py:444` area (`update_limits` callsite — limit clamp recompute path)
- Modify: `klippy/motion_toolhead.py:469` area (`cmd_SET_VELOCITY_LIMIT`)
- Modify: `klippy/motion_toolhead.py:635` area (`init_planner` callsite)

- [ ] **Step 1: Parse `[printer] max_jerk` in `__init__`**

In `klippy/motion_toolhead.py`, after `self.max_z_accel = config.getfloat(...)` at line ~140, add:

```python
        self.max_jerk = config.getfloat("max_jerk", default=None, above=0.0)
```

This deliberately uses `default=None` (not a numeric default). The bridge resolves `None` to `max_accel * 2.0` on the Rust side at init time, so the default tracks the actual init-time accel value. Read `motion_toolhead.py:140` first to confirm exact context.

- [ ] **Step 2: Forward `self.max_jerk` to `init_planner`**

In `klippy/motion_toolhead.py` around line 635 (the existing `self.bridge.init_planner(...)` call), add `max_jerk=self.max_jerk` as a keyword argument:

```python
            self.bridge.init_planner(
                self.max_velocity,
                self.max_accel,
                self.max_z_velocity,
                self.max_z_accel,
                self.square_corner_velocity,
                shaper_type_x,
                shaper_freq_x,
                shaper_type_y,
                shaper_freq_y,
                octopus,
                f446,
                max_jerk=self.max_jerk,
            )
```

- [ ] **Step 3: Forward `self.max_jerk` through the limit-clamp recompute path**

`klippy/motion_toolhead.py:444-453` is a path that calls `update_limits(self.max_velocity, self.max_accel)` after a limit-clamp recompute. Read the existing two callsites (around lines 444-445 and 452-453) and update both to:

```python
                self.bridge.update_limits(self.max_velocity, self.max_accel, self.max_jerk)
```

This is the **preserve-on-recompute** path: when accel-derived clamps recompute, we still pass `self.max_jerk` so the stored value tracks. (`self.max_jerk` is `None` if user didn't configure it; the Rust side preserves the stored value when handed `None`.)

- [ ] **Step 4: Extend `cmd_SET_VELOCITY_LIMIT`**

In `klippy/motion_toolhead.py:469-479`:

```python
    def cmd_SET_VELOCITY_LIMIT(self, gcmd):
        max_velocity = gcmd.get_float("VELOCITY", None, above=0.0)
        max_accel = gcmd.get_float("ACCEL", None, above=0.0)
        max_jerk = gcmd.get_float("JERK", None, above=0.0)
        if max_velocity is not None:
            self.max_velocity = max_velocity
        if max_accel is not None:
            self.max_accel = max_accel
        if max_jerk is not None:
            self.max_jerk = max_jerk
        if self.bridge is not None and (
            max_velocity is not None or max_accel is not None or max_jerk is not None
        ):
            self.bridge.update_limits(self.max_velocity, self.max_accel, self.max_jerk)
```

- [ ] **Step 5: Verify klippy still imports**

```bash
cd /Users/daniladergachev/Developer/kalico
python3 -c "import sys; sys.path.insert(0, 'klippy'); import motion_toolhead; print('ok')"
```

Expected: `ok` printed.

- [ ] **Step 6: Commit**

```bash
git add klippy/motion_toolhead.py
git commit -m "feat(klippy): [printer] max_jerk + SET_VELOCITY_LIMIT JERK= surface

[printer] section accepts an optional max_jerk float key (default None
means 'use Rust default = max_accel * 2.0'). SET_VELOCITY_LIMIT gains
an optional JERK= parameter for runtime override. update_limits
callsites all forward self.max_jerk; the preserve-on-None semantics on
the Rust side mean accel-only changes don't retune jerk implicitly."
```

---

## Task 7: Fix `test_home_x.py` shadowing bug + strengthen assertions

**Files:**
- Modify: `tools/sim_klippy/test_home_x.py:62-100`

The current test exits 0 even when G28 fails (the `r` variable is overwritten by the M114 response before the return statement evaluates). This task fixes the bug and adds the spec-required assertions.

- [ ] **Step 1: Rewrite the test body**

Replace `def main()` and the file's lower half with:

```python
def main():
    cleanup_prior()
    elf = spawn_elf()
    klippy = spawn_klippy()
    failures = []
    try:
        # Step 1: leave endstop LOW (not triggered) initially.
        print("[home] forcing endstop gpio20 LOW")
        r = send_gcode("KALICO_SIM_ENDSTOP_SET_PIN GPIO=20 LEVEL=0")
        print(f"  -> {r}")

        # Step 2: arm a background thread to flip it HIGH after 600ms.
        def trip_after_delay():
            time.sleep(0.6)
            try:
                rr = send_gcode("KALICO_SIM_ENDSTOP_SET_PIN GPIO=20 LEVEL=1", timeout=5.0)
                print(f"[home] late-trip set gpio20=1 -> {rr}")
            except Exception as e:
                print(f"[home] late-trip set failed: {e}")
        t = threading.Thread(target=trip_after_delay, daemon=True)
        t.start()

        # Step 3: send G28 X. Capture its result separately from M114.
        print("[home] sending G28 X")
        g28_resp = send_gcode("G28 X", timeout=30.0)
        print(f"[home] G28 X result: {g28_resp}")

        if "error" in g28_resp:
            failures.append(f"G28 X returned error: {g28_resp['error']}")

        # Step 4: verify position via M114.
        m114_resp = send_gcode("M114", timeout=5.0)
        print(f"[home] M114: {m114_resp}")

        m114_result = m114_resp.get("result", {}) if isinstance(m114_resp, dict) else {}
        # M114 result format depends on klippy plumbing; accept either a
        # raw dict with 'X' or a textual response containing 'X:0.0'.
        x_pos = None
        if isinstance(m114_result, dict) and "X" in m114_result:
            x_pos = float(m114_result["X"])
        else:
            text = str(m114_result)
            for token in text.split():
                if token.startswith("X:"):
                    try:
                        x_pos = float(token[2:])
                    except ValueError:
                        pass
        if x_pos is None:
            failures.append(f"M114 result missing X position: {m114_resp}")
        elif abs(x_pos) > 1e-6:
            failures.append(f"M114 reports X = {x_pos}, expected 0.0")

        # Step 5: scan klippy.log for stall string.
        try:
            log_text = KLIPPY_LOG.read_text(errors="replace")
        except FileNotFoundError:
            log_text = ""
        if "StalledOnInfeasibleSegment" in log_text:
            failures.append("klippy.log contains StalledOnInfeasibleSegment")

        if failures:
            print("[home] FAILURES:")
            for f in failures:
                print(f"  - {f}")
            return 1
        print("[home] PASS")
        return 0
    finally:
        for p in (klippy, elf):
            try: p.terminate(); p.wait(timeout=3)
            except Exception: p.kill()

if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Run the test against current `sota-motion` HEAD — must FAIL with the strengthened assertions**

```bash
./tools/sim_klippy/run_local.sh
```

Expected: exit code 1; output should report `G28 X returned error` (and possibly `klippy.log contains StalledOnInfeasibleSegment`). This proves the test now actually catches the failure mode it claims to test.

(If the user has not run a recent build, the Docker step takes a few minutes; the test failure surfaces in the harness output.)

- [ ] **Step 3: Commit (test bug fix)**

```bash
git add tools/sim_klippy/test_home_x.py
git commit -m "fix(sim): test_home_x.py actually checks homing succeeded

The prior test had a variable-shadowing bug: 'r' was reassigned to
the M114 response before the return statement evaluated, so the test
exited 0 even when G28 produced StalledOnInfeasibleSegment. The new
test captures G28 and M114 responses separately, asserts X=0.0 from
M114, and greps klippy.log for the stall substring. With this fix,
the test correctly fails on current HEAD and will pass once the
config-layer change lands."
```

---

## Task 8: Append maintainer-note paragraph to `constraints.rs`

**Files:**
- Modify: `rust/temporal/src/topp/constraints.rs:236-247`

- [ ] **Step 1: Read current state of the warning**

```bash
sed -n '236,250p' /Users/daniladergachev/Developer/kalico/rust/temporal/src/topp/constraints.rs
```

Confirm the comment block is unchanged from the spec's quoted form.

- [ ] **Step 2: Append the new paragraph**

In `rust/temporal/src/topp/constraints.rs`, immediately after the existing 12-line MAINTAINER WARNING block (preserving its content), insert:

```rust
    //
    // BRIDGE-CONFIG NOTE (2026-05-05): the bridge config layer at
    // `rust/motion-bridge/src/config.rs::PlannerLimits::to_temporal_limits`
    // populates `j_max = [J, J, J]` uniformly from a single `max_jerk`
    // scalar, so under the MVP this `j_path = min(j_max)` reduces to
    // `j_path = J` regardless of axis. Future per-axis jerk *configuration*
    // at the bridge layer must land alongside the per-axis Cartesian-jerk
    // SOCP relaxation work the warning above defers — the two are coupled.
    // See `docs/superpowers/specs/2026-05-05-mvp-global-scalar-jerk-design.md`.
```

- [ ] **Step 3: Verify temporal builds**

```bash
cargo build -p temporal
```

Expected: clean build (the change is comment-only).

- [ ] **Step 4: Commit**

```bash
git add rust/temporal/src/topp/constraints.rs
git commit -m "doc(temporal): note bridge-layer global-scalar-jerk MVP coupling

Append a paragraph to the existing MAINTAINER WARNING noting that the
bridge config layer also collapses jerk to a single scalar for MVP, so
under this configuration j_path = min(j_max) trivially equals the
single max_jerk value regardless of axis. Future per-axis jerk
configuration at the bridge layer must land alongside the per-axis
Cartesian-jerk SOCP work the original warning defers."
```

---

## Task 9: Plan-changes-log entry

**Files:**
- Modify: `docs/superpowers/plan-changes-log.md`

- [ ] **Step 1: Read the existing log to match formatting**

```bash
head -30 /Users/daniladergachev/Developer/kalico/docs/superpowers/plan-changes-log.md
```

Note the existing entry format (date / what / why / evidence).

- [ ] **Step 2: Prepend a new entry at the top of the log**

Add at the top of `docs/superpowers/plan-changes-log.md` (immediately under the file header, above existing entries — match the existing format observed in Step 1):

```markdown
## 2026-05-05 — MVP global scalar jerk at the bridge config layer

**What:** `rust/motion-bridge/src/config.rs::PlannerLimits` gains a `max_jerk: f64` field; `to_temporal_limits` emits uniform `j_max = [max_jerk; 3]` instead of the prior per-axis derivation `[max_accel*2, max_accel*2, max_z_accel*2]`. Klippy `[printer]` accepts an optional `max_jerk` key; `SET_VELOCITY_LIMIT` accepts an optional `JERK=` parameter. PyO3 boundary uses `Option<f64>` (preserve-on-`None` for runtime updates). Per-axis Cartesian-jerk *configuration* is deferred alongside the per-axis Cartesian-jerk *SOCP relaxation* work the maintainer warning at `rust/temporal/src/topp/constraints.rs:236-247` defers.

**Why:** Phase 4 G28 X homing was stalling with `StalledOnInfeasibleSegment` because the bridge default `j_max[Z] = 2*max_z_accel = 200` was dominating `J_path = min(j_max)` on pure-X moves, forcing the 300 mm rest-to-rest profile into a single full-length S-curve that Stage 1 SLP could not converge on. Codex cross-check refuted a per-grid-projected `J_path[i] = min over axes of j_max[axis] / |c'_axis(s_i)|` proposal on curved-path correctness grounds; doing it properly requires the deferred SOCP-side per-axis Cartesian-jerk work. MVP collapses to a single scalar — the per-axis distinction in the prior defaults was a `2 × per-axis-accel` heuristic, not measured per-axis physics.

**Evidence:** `docs/superpowers/specs/2026-05-05-mvp-global-scalar-jerk-design.md`, `docs/research/stall-homing-move.md`, `rust/trajectory/tests/homing_300mm_pure_x.rs`.
```

- [ ] **Step 3: Commit**

```bash
git add docs/superpowers/plan-changes-log.md
git commit -m "doc(plan-changes-log): record MVP global scalar jerk at bridge

2026-05-05 entry for the bridge-config-layer collapse of per-axis jerk
to a single max_jerk scalar. Phase 4 homing unblock; per-axis jerk
configuration deferred alongside per-axis Cartesian-jerk SOCP work."
```

---

## Task 10: End-to-end verification — run sim test, confirm homing succeeds

**Files:** none modified.

This is the integration gate. After all preceding tasks land, the sim test must transition from FAIL to PASS.

- [ ] **Step 1: Run the full Rust test suite**

```bash
cargo test -p temporal -p trajectory -p motion-bridge
```

Expected: green across all three crates.

- [ ] **Step 2: Run the sim integration test**

```bash
./tools/sim_klippy/run_local.sh
```

Expected output should include:
- `[home] G28 X result: {... 'result': ...}` (no error key)
- `[home] M114: {...}` with `X:0.0` parseable from the result
- `[home] PASS`
- Process exit code 0
- No `StalledOnInfeasibleSegment` substring in `tools/sim_klippy/.local-logs/klippy.log`

- [ ] **Step 3: Verify klippy.log explicitly**

```bash
grep -c "StalledOnInfeasibleSegment" tools/sim_klippy/.local-logs/klippy.log
```

Expected: `0`.

- [ ] **Step 4: Spot-check the sim with a follow-up move (manual)**

Optional sanity (not a release gate):

```bash
./tools/sim_klippy/run_local.sh "G28 X
G1 X10 F1000
M114"
```

Expected: G28 X succeeds; G1 X10 F1000 produces step pulses (visible in elf log); M114 reports X≈10.

- [ ] **Step 5: Final commit (no code changes; just a sentinel marker if desired)**

If the previous commits already represent all changes, this step is just the verification record. The branch is ready for merge to main once all preceding tasks' commits are landed.

If the user has additional manual print verification to do before merging, hand off here. Otherwise the sequence is complete.

---

## Self-review

Spec coverage matrix:

| Spec section | Implementing task |
|---|---|
| §1 summary (max_jerk field, PyO3 Option, SET_VELOCITY_LIMIT JERK=) | Tasks 2-6 |
| §2 motivation | Task 1 (regression proof), Task 10 (end-to-end proof) |
| §3 what stays per-axis (v_max, a_max) | Task 2 (to_temporal_limits preserves per-axis V/A) |
| §4 what collapses (j_max uniform) | Task 2 |
| §5.1 Rust storage | Task 2 |
| §5.2 klippy printer.cfg | Task 6 step 1 |
| §5.3 runtime updates | Tasks 4 + 6 steps 3-4 |
| §5.4 effect on Z | Task 1 (proves Z=6000 path doesn't break Z behavior) |
| §5.5 β-medium output drift | Task 10 (full cargo test catches β regressions) |
| §6.1 Rust regression | Task 1 |
| §6.2 sim test fix | Task 7 |
| §6.3 curved-fixture invariance | Task 10 step 1 |
| §6.4 config layer unit tests | Task 2, Task 3, Task 4 |
| §7.1 constraints.rs note | Task 8 |
| §7.2 plan-changes-log | Task 9 |
| §9 acceptance criteria 1-9 | Tasks 1-9 collectively, validated by Task 10 |

Placeholder scan: every task has explicit code or commands; no "TODO," "TBD," or "implement appropriately" patterns.

Type consistency: `max_jerk` is `f64` everywhere (storage, helper signatures), `Option<f64>` only at the PyO3 boundary; `apply_optional_max_jerk` and `resolve_max_jerk` signatures consistent across Tasks 3, 4.

Plan ready.
