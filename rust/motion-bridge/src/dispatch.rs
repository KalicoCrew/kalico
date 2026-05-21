//! Per-MCU dispatch: maps a `ShapedSegment`'s per-axis NURBS curves onto the
//! configured MCU axis assignment, producing one [`McuPushPlan`] per MCU
//! that has at least one non-trivial curve to load.
//!
//! For CoreXY MCUs (`kinematics == KINEMATICS_COREXY`) the logical X and Y
//! curves are combined into motor-frame A = X+Y and B = X-Y **here on the
//! host** before being serialised over the wire. The MCU therefore receives
//! motor-frame curves in its X/Y handle slots and has no CoreXY transform in
//! its hot path.

use kalico_host_rt::producer::{CurveLoadParams, SegmentPushParams};
use runtime::segment::KinematicTag;
use trajectory::ShapedSegment;

/// `McuAxisConfig::kinematics` tag: Octopus CoreXY, motors A (slot 0) + B (slot 1).
///
/// Derived from [`KinematicTag::CoreXyAndE`] so the wire-ABI discriminant has a
/// single source of truth. The `const _: ()` assertion below pins the mapping so
/// a renumbering of `KinematicTag` produces a compile-time error rather than a
/// silent wire mismatch.
pub const KINEMATICS_COREXY: u8 = KinematicTag::CoreXyAndE as u8;

const _: () = assert!(
    KinematicTag::CoreXyAndE as u8 == 0,
    "wire-ABI invariant broken: KinematicTag::CoreXyAndE discriminant must be 0 \
     (matches KINEMATICS_COREXY on the host and the MCU firmware's kinematics byte)",
);

/// Sentinel "no curve loaded" handle value. The firmware checks
/// `handle == 0xFFFE_FFFE` to skip evaluating that axis for the segment.
pub const UNUSED_HANDLE: u32 = 0xFFFE_FFFE;

pub const AXIS_X: usize = 0;
pub const AXIS_Y: usize = 1;
pub const AXIS_Z: usize = 2;

/// Epsilon for the "all control points equal" trivial-constant test.
const EPS_CONST: f64 = 1e-12;

/// Per-MCU configuration: which `ShapedSegment` axes this MCU is responsible
/// for, plus the firmware kinematics tag.
#[derive(Debug, Clone)]
pub struct McuAxisConfig {
    pub mcu_id: u32,
    /// Indices into `ShapedSegment::axes` (0=X, 1=Y, 2=Z) that this MCU drives.
    pub axes: Vec<usize>,
    /// Kinematics tag forwarded to the MCU in `SegmentPushParams::kinematics`.
    pub kinematics: u8,
    /// Per-MCU runtime sizing limits as reported by `QueryRuntimeCaps`
    /// (or `McuCaps::default()` for firmware that predates the message).
    pub caps: McuCaps,
}

/// Subset of `RuntimeCapsResponse` that the dispatcher needs to enforce
/// per-MCU sizing limits when planning a curve.
///
/// Cubic-only revision (2026-05-20 stepping redesign): NURBS sizing fields
/// (max_control_points / max_knot_vector_len / max_degree) were removed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct McuCaps {
    pub curve_pool_n: u16,
    pub max_pieces_per_curve: u16,
}

impl Default for McuCaps {
    /// Large-profile defaults used when the per-MCU `QueryRuntimeCaps`
    /// round-trip fails (e.g. transport timeout during attach). Mirrors
    /// the H7 `RUNTIME_TARGET_LARGE` Kconfig defaults.
    fn default() -> Self {
        Self {
            curve_pool_n: 16,
            max_pieces_per_curve: 16,
        }
    }
}

impl From<kalico_protocol::messages::RuntimeCapsResponse> for McuCaps {
    fn from(r: kalico_protocol::messages::RuntimeCapsResponse) -> Self {
        Self {
            curve_pool_n: r.curve_pool_n,
            max_pieces_per_curve: r.max_pieces_per_curve,
        }
    }
}

/// One MCU's slice of work for a single shaped segment: the curves it must
/// load (with the axis index they bind to) plus a partially-built
/// `SegmentPushParams` whose handle fields will be filled in once the
/// curve loads return packed handles.
#[derive(Debug, Clone)]
pub struct McuPushPlan {
    pub mcu_id: u32,
    /// `(axis_idx, curve)` pairs in the order the dispatcher discovered them.
    pub curves_to_load: Vec<(usize, CurveLoadParams)>,
    pub params: SegmentPushParams,
}

impl McuPushPlan {
    /// Fill the appropriate `*_handle_packed` field of `params` for the given
    /// shaped-segment axis index.
    pub fn set_handle(&mut self, axis_idx: usize, packed: u32) {
        match axis_idx {
            AXIS_X => self.params.x_handle_packed = packed,
            AXIS_Y => self.params.y_handle_packed = packed,
            AXIS_Z => self.params.z_handle_packed = packed,
            _ => {} // E lives in e_handle_packed and is dispatched separately
        }
    }
}

pub fn is_trivially_constant(curve: &nurbs::ScalarNurbs<f64>) -> bool {
    let cps = curve.control_points();
    if cps.is_empty() {
        return true;
    }
    let first = cps[0];
    cps.iter().all(|&v| (v - first).abs() <= EPS_CONST)
}

/// Build per-MCU push plans for a single shaped segment.
///
/// `t_start_clock` / `t_end_clock` are 64-bit MCU-clock values produced by
/// the temporal-to-clock conversion step (`planner::config::trajectory_to_clock`
/// or equivalent) — same value goes to every MCU for a given segment.
///
/// **CoreXY transform:** when `cfg.kinematics == KINEMATICS_COREXY` and both
/// `AXIS_X` and `AXIS_Y` are in `cfg.axes`, the logical X and Y curves are
/// combined into motor-frame curves before serialisation:
///   - Motor-A curve (stored in `AXIS_X` slot) = X + Y
///   - Motor-B curve (stored in `AXIS_Y` slot) = X − Y
///
/// The slot indices (0 = AXIS_X, 1 = AXIS_Y) are unchanged — only the
/// *contents* differ. Knot vectors are aligned via exact Bézier-piece
/// union before the pointwise add — no approximation, no fit error.
/// After the union, `nurbs::algebra::add` is guaranteed to succeed; if it
/// returns `Err`, the function panics with "post-union add failed — bridge
/// invariant violated".
pub fn build_push_params(
    shaped: &ShapedSegment,
    mcu_configs: &[McuAxisConfig],
    t_start_clock: u64,
    t_end_clock: u64,
) -> Vec<McuPushPlan> {
    let mut plans = Vec::with_capacity(mcu_configs.len());

    for cfg in mcu_configs {
        // For CoreXY MCUs that drive both AXIS_X and AXIS_Y, pre-compute the
        // motor-frame curves once per MCU. These replace the logical X/Y
        // curves in the curves_to_load list below.
        let corexy_motor_curves: Option<(nurbs::ScalarNurbs<f64>, nurbs::ScalarNurbs<f64>)> =
            if cfg.kinematics == KINEMATICS_COREXY
                && cfg.axes.contains(&AXIS_X)
                && cfg.axes.contains(&AXIS_Y)
                && AXIS_X < shaped.axes.len()
                && AXIS_Y < shaped.axes.len()
            {
                let x = &shaped.axes[AXIS_X];
                let y = &shaped.axes[AXIS_Y];
                // Align knot vectors via exact Bézier-piece union (no fit
                // error). After the union, add is guaranteed to succeed;
                // the expect below is the unreachable sentinel.
                let motor_a = nurbs::algebra::add_with_knot_union(x, y).unwrap_or_else(|e| {
                    panic!(
                        "post-union add failed — bridge invariant violated (motor-A): {e:?}"
                    )
                });
                let motor_b_neg_y = nurbs::algebra::scalar_multiply(y, -1.0_f64);
                let motor_b = nurbs::algebra::add_with_knot_union(x, &motor_b_neg_y)
                    .unwrap_or_else(|e| {
                        panic!(
                            "post-union add failed — bridge invariant violated (motor-B): {e:?}"
                        )
                    });
                Some((motor_a, motor_b))
            } else {
                None
            };

        let mut curves_to_load: Vec<(usize, CurveLoadParams)> = Vec::new();
        for &axis_idx in &cfg.axes {
            if axis_idx >= shaped.axes.len() {
                continue;
            }

            // Select the curve: for CoreXY MCUs, substitute motor-frame
            // curves in the X and Y slots; all other axes pass through.
            let curve_params = if let Some((ref motor_a, ref motor_b)) = corexy_motor_curves {
                match axis_idx {
                    AXIS_X => CurveLoadParams::from_scalar_nurbs_normalized(
                        motor_a,
                        shaped.t_start,
                        shaped.t_end,
                    ),
                    AXIS_Y => CurveLoadParams::from_scalar_nurbs_normalized(
                        motor_b,
                        shaped.t_start,
                        shaped.t_end,
                    ),
                    _ => CurveLoadParams::from_scalar_nurbs_normalized(
                        &shaped.axes[axis_idx],
                        shaped.t_start,
                        shaped.t_end,
                    ),
                }
            } else {
                CurveLoadParams::from_scalar_nurbs_normalized(
                    &shaped.axes[axis_idx],
                    shaped.t_start,
                    shaped.t_end,
                )
            };

            // 2026-05-11 fix — DO NOT skip "trivially constant" curves.
            // The previous optimization left the corresponding MCU handle
            // at UNUSED_SENTINEL, and the engine's UNUSED-handle semantic
            // (engine.rs::tick_with_current X/Y/Z branches) returned
            // (0.0, 0.0) for "axis at zero". That's wrong for absolute-
            // coordinate trajectory segments: an axis whose curve was
            // skipped one segment but sent the next (because refit-noise
            // pushed the constant-check just past 1e-12) produced phantom
            // position jumps equal to the actual axis position (e.g.,
            // 100 mm in Y when jogging on X), reliably tripping
            // STEP_BURST_EXCEEDED on the next segment activation.
            //
            // The architectural fix is twofold: (a) the engine now treats
            // UNUSED as "hold prev value" (engine.rs same commit), and
            // (b) the bridge sends every kinematic axis's curve every
            // segment — including constants — so the engine's hold value
            // is always anchored to klippy's current commanded position.
            // Slot-economy cost: every segment uses one slot per
            // kinematic axis (3 for X/Y/Z on a CoreXY+Z setup) instead of
            // 1 for pure-X jogs. With CURVE_POOL_N=16 and credit-flow
            // backpressure (producer.rs::push_segment_with_timeout), this
            // throttles in-flight depth to ~5 segments — fine for the
            // MVP; right-sized per-slot capacity is a future
            // optimization if production prints need more depth.
            curves_to_load.push((axis_idx, curve_params));
        }

        if curves_to_load.is_empty() {
            continue;
        }

        let params = SegmentPushParams {
            id: 0,
            x_handle_packed: UNUSED_HANDLE,
            y_handle_packed: UNUSED_HANDLE,
            z_handle_packed: UNUSED_HANDLE,
            e_handle_packed: UNUSED_HANDLE,
            t_start: t_start_clock,
            t_end: t_end_clock,
            kinematics: cfg.kinematics,
            e_mode: 2, // Travel
            extrusion_ratio: 0.0,
        };

        plans.push(McuPushPlan {
            mcu_id: cfg.mcu_id,
            curves_to_load,
            params,
        });
    }

    plans
}


#[cfg(test)]
mod tests {
    use super::*;
    use geometry::segment::EMode;
    use nurbs::ScalarNurbs;

    fn linear_curve(a: f64, b: f64) -> ScalarNurbs<f64> {
        // degree-3 Bézier with collinear cps a, lerp(1/3), lerp(2/3), b
        let cps = vec![a, a + (b - a) / 3.0, a + 2.0 * (b - a) / 3.0, b];
        ScalarNurbs::try_new(3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0], cps, None)
            .unwrap()
    }

    fn constant_curve(v: f64) -> ScalarNurbs<f64> {
        ScalarNurbs::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![v, v, v, v],
            None,
        )
        .unwrap()
    }

    fn shaped(axes: [ScalarNurbs<f64>; 3]) -> ShapedSegment {
        ShapedSegment {
            axes,
            e_mode: EMode::Travel,
            extrusion_per_xy_mm: 0.0,
            e_independent: None,
            t_start: 0.0,
            t_end: 1.0,
        }
    }

    fn cfgs() -> Vec<McuAxisConfig> {
        vec![
            McuAxisConfig {
                mcu_id: 0,
                axes: vec![AXIS_X, AXIS_Y],
                kinematics: KINEMATICS_COREXY, // 0
                caps: McuCaps::default(),
            },
            McuAxisConfig {
                mcu_id: 1,
                axes: vec![AXIS_Z],
                kinematics: 2,
                caps: McuCaps::default(),
            },
        ]
    }

    /// **Post-2026-05-11.** Every kinematic axis listed in the MCU's
    /// `cfg.axes` gets a curve, even when the curve is trivially
    /// constant. This matches the new "axis is absolute coordinate"
    /// semantic on the MCU side — `UNUSED` handles mean "hold at
    /// `prev_value`", and the bridge must therefore anchor each axis to
    /// klippy's commanded position on every segment.
    #[test]
    fn x_move_sends_curves_for_every_kinematic_axis_on_each_mcu() {
        // X varies, Y and Z constant. cfgs[0] (Octopus) drives X+Y,
        // cfgs[1] (F446) drives Z. Both MCUs must receive curves.
        let seg = shaped([linear_curve(0.0, 10.0), constant_curve(100.0), constant_curve(5.0)]);
        let plans = build_push_params(&seg, &cfgs(), 1_000, 2_000);

        assert_eq!(plans.len(), 2, "both MCUs should get a plan");

        // Octopus: X + Y (both kinematic axes for this MCU).
        let octopus = plans.iter().find(|p| p.mcu_id == 0).expect("octopus plan");
        assert_eq!(octopus.curves_to_load.len(), 2);
        assert_eq!(octopus.curves_to_load[0].0, AXIS_X);
        assert_eq!(octopus.curves_to_load[1].0, AXIS_Y);
        assert_eq!(octopus.params.kinematics, KINEMATICS_COREXY);

        // F446: Z (only kinematic axis for this MCU).
        let f446 = plans.iter().find(|p| p.mcu_id == 1).expect("f446 plan");
        assert_eq!(f446.curves_to_load.len(), 1);
        assert_eq!(f446.curves_to_load[0].0, AXIS_Z);
        assert_eq!(f446.params.kinematics, 2);

        // All handle packed fields still default to UNUSED — the caller
        // fills them after `load_curve` returns.
        assert_eq!(octopus.params.x_handle_packed, UNUSED_HANDLE);
        assert_eq!(octopus.params.y_handle_packed, UNUSED_HANDLE);
        assert_eq!(octopus.params.t_start, 1_000);
        assert_eq!(octopus.params.t_end, 2_000);
        assert_eq!(octopus.params.e_mode, 2);
    }

    #[test]
    fn z_move_sends_curves_for_every_kinematic_axis_on_each_mcu() {
        // Same pattern: Z varies, X+Y constant. Both MCUs still get plans
        // because both have kinematic axes that need to be anchored.
        let seg = shaped([constant_curve(50.0), constant_curve(100.0), linear_curve(0.0, 5.0)]);
        let plans = build_push_params(&seg, &cfgs(), 1_000, 2_000);

        assert_eq!(plans.len(), 2);

        let octopus = plans.iter().find(|p| p.mcu_id == 0).expect("octopus");
        assert_eq!(octopus.curves_to_load.len(), 2);
        let f446 = plans.iter().find(|p| p.mcu_id == 1).expect("f446");
        assert_eq!(f446.curves_to_load.len(), 1);
    }

    #[test]
    fn set_handle_fills_correct_field() {
        let seg = shaped([linear_curve(0.0, 10.0), constant_curve(100.0), constant_curve(5.0)]);
        let mut plans = build_push_params(&seg, &cfgs(), 0, 100);
        // Find the Octopus plan; it's no longer guaranteed to be plans[0]
        // since the iteration order over a Vec preserves cfg order, but
        // be explicit anyway.
        let octopus_idx = plans.iter().position(|p| p.mcu_id == 0).expect("octopus");
        plans[octopus_idx].set_handle(AXIS_X, 0xCAFE);
        assert_eq!(plans[octopus_idx].params.x_handle_packed, 0xCAFE);
        assert_eq!(plans[octopus_idx].params.y_handle_packed, UNUSED_HANDLE);
    }

    /// **Regression for bench session 2026-05-11.** Verify the bridge
    /// sends Y curves for pure-X jogs. The old `is_trivially_constant`
    /// skip would have omitted Y here, leaving the MCU's Y handle at
    /// UNUSED_SENTINEL — which the engine then evaluated as `y = 0.0`
    /// instead of holding `prev_y`. The first segment whose Y curve
    /// non-trivially-noise crossed the 1e-12 threshold would suddenly
    /// produce `y = 100` instead of `y = 0`, generating a 100 mm delta
    /// on motor A (CoreXY: `motor_a = x + y`) and tripping
    /// STEP_BURST_EXCEEDED.
    #[test]
    fn constant_y_axis_for_pure_x_move_still_sends_a_curve() {
        let seg = shaped([linear_curve(0.0, 25.0), constant_curve(100.0), constant_curve(10.0)]);
        let plans = build_push_params(&seg, &cfgs(), 0, 1_000);

        let octopus = plans.iter().find(|p| p.mcu_id == 0).expect("octopus");
        let has_x = octopus.curves_to_load.iter().any(|(axis, _)| *axis == AXIS_X);
        let has_y = octopus.curves_to_load.iter().any(|(axis, _)| *axis == AXIS_Y);
        assert!(has_x, "X curve must be sent on a pure-X move");
        assert!(
            has_y,
            "Y curve must be sent even when constant — the engine's UNUSED-handle hold semantic needs prev_y anchored to klippy's commanded Y for cross-segment continuity"
        );
    }

    /// **Regression for bench CoreXY diagonal bug (2026-05-21).** A pure-X
    /// jog with `KINEMATICS_COREXY` must produce motor-frame curves in the
    /// X/Y handle slots rather than logical X/Y curves.
    ///
    /// Input: X linear 0 → 10, Y constant 100, Z constant 5.
    /// Expected motor-frame output:
    ///   - AXIS_X slot carries motor-A curve (X+Y): CPs [100, 103.33, 106.67, 110]
    ///   - AXIS_Y slot carries motor-B curve (X-Y): CPs [-100, -96.67, -93.33, -90]
    ///
    /// The test uses a Cartesian MCU for Z (mcu_id=1) to verify Z passes
    /// through unchanged, and checks that the slot *indices* are still
    /// AXIS_X / AXIS_Y (the MCU reads motor-A from x_handle, motor-B from
    /// y_handle by convention).
    #[test]
    fn corexy_pure_x_jog_combines_into_motor_frame_curves() {
        // Build a two-MCU config: Octopus (CoreXY, mcu_id=0) + F446 (Z, mcu_id=1).
        let corexy_cfgs = vec![
            McuAxisConfig {
                mcu_id: 0,
                axes: vec![AXIS_X, AXIS_Y],
                kinematics: KINEMATICS_COREXY, // 0
                caps: McuCaps::default(),
            },
            McuAxisConfig {
                mcu_id: 1,
                axes: vec![AXIS_Z],
                kinematics: 2, // CartesianXyzAndE — Z passthrough
                caps: McuCaps::default(),
            },
        ];

        // X: linear 0 → 10 (CPs: 0, 3.333, 6.667, 10)
        // Y: constant 100 (CPs: 100, 100, 100, 100)
        // Z: constant 5
        let seg = shaped([linear_curve(0.0, 10.0), constant_curve(100.0), constant_curve(5.0)]);
        let plans = build_push_params(&seg, &corexy_cfgs, 0, 1_000);

        let octopus = plans.iter().find(|p| p.mcu_id == 0).expect("octopus plan");
        assert_eq!(octopus.curves_to_load.len(), 2);

        // Slot ordering: AXIS_X then AXIS_Y (matches cfg.axes iteration order).
        let (motor_a_slot, motor_a_params) = &octopus.curves_to_load[0];
        let (motor_b_slot, motor_b_params) = &octopus.curves_to_load[1];
        assert_eq!(*motor_a_slot, AXIS_X, "motor-A must be in AXIS_X slot");
        assert_eq!(*motor_b_slot, AXIS_Y, "motor-B must be in AXIS_Y slot");

        // Motor-A = X + Y. Single-piece Bézier with CPs [0+100, 3.33+100, 6.67+100, 10+100].
        assert_eq!(
            motor_a_params.bp_per_piece.len(), 1,
            "single-piece input → single-piece output"
        );
        let a_cps = motor_a_params.bp_per_piece[0];
        let a_expected = [100.0_f32, 103.333_333, 106.666_666, 110.0];
        for (k, (&got, &exp)) in a_cps.iter().zip(a_expected.iter()).enumerate() {
            assert!(
                (got - exp).abs() < 1e-3,
                "motor-A CP[{k}]: got {got}, expected ~{exp}"
            );
        }

        // Motor-B = X - Y. CPs [0-100, 3.33-100, 6.67-100, 10-100].
        assert_eq!(
            motor_b_params.bp_per_piece.len(), 1,
            "single-piece input → single-piece output"
        );
        let b_cps = motor_b_params.bp_per_piece[0];
        let b_expected = [-100.0_f32, -96.666_666, -93.333_333, -90.0];
        for (k, (&got, &exp)) in b_cps.iter().zip(b_expected.iter()).enumerate() {
            assert!(
                (got - exp).abs() < 1e-3,
                "motor-B CP[{k}]: got {got}, expected ~{exp}"
            );
        }

        // Z passes through unchanged (F446 mcu_id=1 is Cartesian, not CoreXY).
        let f446 = plans.iter().find(|p| p.mcu_id == 1).expect("f446 plan");
        assert_eq!(f446.curves_to_load.len(), 1);
        assert_eq!(f446.curves_to_load[0].0, AXIS_Z);
    }

    /// **Regression: multi-piece knot-vector mismatch.** Exercises the knot-union
    /// path in `add_with_knot_union`. X has two Bézier pieces (a two-segment cubic
    /// spline on [0,1]) while Y is a single-piece constant. Without the knot-union
    /// pass the old `nurbs::algebra::add` would return `KnotMismatch` and the
    /// `.expect(...)` would have panicked in release.
    ///
    /// X: two-piece cubic: piece 1 = linear 0→5 on [0, 0.5], piece 2 = linear 5→10 on [0.5, 1].
    /// Y: constant 20.
    /// Expected motor-A: X+Y = two-piece, values 20→25→30.
    /// Expected motor-B: X-Y = two-piece, values -20→-15→-10.
    #[test]
    fn corexy_multi_piece_x_knot_union_combines_correctly() {
        use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};

        // Build a two-piece X curve by constructing two Bézier pieces and recomposing.
        let piece1 = BezierPiece::<f64> {
            u_start: 0.0,
            u_end: 0.5,
            // linear 0→5: Pascal-shifted at 0.0; coeffs = [0, 10] (c0 + c1*(u-0))
            // At u=0.5: 0 + 10*0.5 = 5. Correct.
            coeffs: vec![0.0, 10.0, 0.0, 0.0],
        };
        let piece2 = BezierPiece::<f64> {
            u_start: 0.5,
            u_end: 1.0,
            // linear 5→10: Pascal-shifted at 0.5; c0=5, c1=10.
            coeffs: vec![5.0, 10.0, 0.0, 0.0],
        };
        let x_two_piece = bezier_pieces_to_nurbs(&[piece1, piece2]);

        // Y: single-piece constant 20.
        let y_const = constant_curve(20.0);

        let corexy_cfg = vec![McuAxisConfig {
            mcu_id: 0,
            axes: vec![AXIS_X, AXIS_Y],
            kinematics: KINEMATICS_COREXY,
            caps: McuCaps::default(),
        }];
        let seg = ShapedSegment {
            axes: [x_two_piece, y_const, constant_curve(0.0)],
            e_mode: geometry::segment::EMode::Travel,
            extrusion_per_xy_mm: 0.0,
            e_independent: None,
            t_start: 0.0,
            t_end: 1.0,
        };
        let plans = build_push_params(&seg, &corexy_cfg, 0, 1_000);

        let plan = plans.iter().find(|p| p.mcu_id == 0).expect("plan");
        // Both motor curves must be present and be two-piece (X had 2 pieces, Y had 1;
        // union produces 2 pieces).
        assert_eq!(plan.curves_to_load.len(), 2);
        let (_, motor_a) = &plan.curves_to_load[0];
        let (_, motor_b) = &plan.curves_to_load[1];

        // Motor-A = X+Y: two pieces — values at u=0, 0.5, 1.0 should be 20, 25, 30.
        assert_eq!(motor_a.bp_per_piece.len(), 2, "motor-A must be 2 pieces");
        // First CP of piece 0 ≈ X(0)+Y(0) = 0+20 = 20.
        let a0 = motor_a.bp_per_piece[0];
        assert!((a0[0] as f64 - 20.0).abs() < 0.01, "motor-A piece0 CP0: got {}", a0[0]);
        // Last CP of piece 1 ≈ X(1)+Y(1) = 10+20 = 30.
        let a1 = motor_a.bp_per_piece[1];
        assert!((a1[3] as f64 - 30.0).abs() < 0.01, "motor-A piece1 CP3: got {}", a1[3]);

        // Motor-B = X-Y: two pieces — values at u=0, 0.5, 1.0 should be -20, -15, -10.
        assert_eq!(motor_b.bp_per_piece.len(), 2, "motor-B must be 2 pieces");
        let b0 = motor_b.bp_per_piece[0];
        assert!((b0[0] as f64 - (-20.0)).abs() < 0.01, "motor-B piece0 CP0: got {}", b0[0]);
        let b1 = motor_b.bp_per_piece[1];
        assert!((b1[3] as f64 - (-10.0)).abs() < 0.01, "motor-B piece1 CP3: got {}", b1[3]);
    }

    /// **Pure-Y jog — sign of motor-B path.** X is constant, Y is linear.
    /// Motor-A = X+Y (positive), Motor-B = X-Y (negative Y contribution).
    ///
    /// Input: X constant 50, Y linear 0→10.
    /// Motor-A = 50+Y: CPs [50, 53.33, 56.67, 60].
    /// Motor-B = 50-Y: CPs [50, 46.67, 43.33, 40].
    #[test]
    fn corexy_pure_y_jog_motor_b_has_negative_y_contribution() {
        let corexy_cfg = vec![McuAxisConfig {
            mcu_id: 0,
            axes: vec![AXIS_X, AXIS_Y],
            kinematics: KINEMATICS_COREXY,
            caps: McuCaps::default(),
        }];
        // X constant 50, Y linear 0→10.
        let seg = shaped([constant_curve(50.0), linear_curve(0.0, 10.0), constant_curve(0.0)]);
        let plans = build_push_params(&seg, &corexy_cfg, 0, 1_000);

        let plan = plans.iter().find(|p| p.mcu_id == 0).expect("plan");
        assert_eq!(plan.curves_to_load.len(), 2);
        let (a_slot, motor_a) = &plan.curves_to_load[0];
        let (b_slot, motor_b) = &plan.curves_to_load[1];
        assert_eq!(*a_slot, AXIS_X);
        assert_eq!(*b_slot, AXIS_Y);

        assert_eq!(motor_a.bp_per_piece.len(), 1);
        let a_cps = motor_a.bp_per_piece[0];
        let a_expected = [50.0_f32, 53.333_333, 56.666_666, 60.0];
        for (k, (&got, &exp)) in a_cps.iter().zip(a_expected.iter()).enumerate() {
            assert!(
                (got - exp).abs() < 1e-3,
                "motor-A CP[{k}]: got {got}, expected ~{exp}"
            );
        }

        assert_eq!(motor_b.bp_per_piece.len(), 1);
        let b_cps = motor_b.bp_per_piece[0];
        // B = X - Y = 50 - [0, 3.33, 6.67, 10] = [50, 46.67, 43.33, 40]
        let b_expected = [50.0_f32, 46.666_666, 43.333_333, 40.0];
        for (k, (&got, &exp)) in b_cps.iter().zip(b_expected.iter()).enumerate() {
            assert!(
                (got - exp).abs() < 1e-3,
                "motor-B CP[{k}]: got {got}, expected ~{exp}"
            );
        }
    }

    /// **Diagonal jog — both motors step at distinct rates.** X and Y are both
    /// linear but with different slopes. Motor-A and Motor-B must both be
    /// non-constant and differ from each other.
    ///
    /// Input: X linear 0→6, Y linear 0→4.
    /// Motor-A = X+Y: CPs [0, 3.33, 6.67, 10].
    /// Motor-B = X-Y: CPs [0, 0.67, 1.33, 2].
    #[test]
    fn corexy_diagonal_jog_both_motors_step_at_distinct_rates() {
        let corexy_cfg = vec![McuAxisConfig {
            mcu_id: 0,
            axes: vec![AXIS_X, AXIS_Y],
            kinematics: KINEMATICS_COREXY,
            caps: McuCaps::default(),
        }];
        // X linear 0→6, Y linear 0→4.
        let seg = shaped([linear_curve(0.0, 6.0), linear_curve(0.0, 4.0), constant_curve(0.0)]);
        let plans = build_push_params(&seg, &corexy_cfg, 0, 1_000);

        let plan = plans.iter().find(|p| p.mcu_id == 0).expect("plan");
        assert_eq!(plan.curves_to_load.len(), 2);
        let (_, motor_a) = &plan.curves_to_load[0];
        let (_, motor_b) = &plan.curves_to_load[1];

        assert_eq!(motor_a.bp_per_piece.len(), 1);
        assert_eq!(motor_b.bp_per_piece.len(), 1);

        let a_cps = motor_a.bp_per_piece[0];
        let b_cps = motor_b.bp_per_piece[0];

        // Motor-A = X+Y: linear 0→10. CPs [0, 3.33, 6.67, 10].
        let a_expected = [0.0_f32, 3.333_333, 6.666_666, 10.0];
        for (k, (&got, &exp)) in a_cps.iter().zip(a_expected.iter()).enumerate() {
            assert!(
                (got - exp).abs() < 1e-3,
                "motor-A CP[{k}]: got {got}, expected ~{exp}"
            );
        }

        // Motor-B = X-Y: linear 0→2. CPs [0, 0.667, 1.333, 2].
        let b_expected = [0.0_f32, 0.666_666, 1.333_333, 2.0];
        for (k, (&got, &exp)) in b_cps.iter().zip(b_expected.iter()).enumerate() {
            assert!(
                (got - exp).abs() < 1e-3,
                "motor-B CP[{k}]: got {got}, expected ~{exp}"
            );
        }

        // Sanity: motor-A and motor-B differ (they cannot be equal on a diagonal).
        let a_last = a_cps[3];
        let b_last = b_cps[3];
        assert!(
            (a_last - b_last).abs() > 0.1,
            "motor-A and motor-B must differ on a diagonal jog"
        );
    }
}
