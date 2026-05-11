//! Per-MCU dispatch: maps a `ShapedSegment`'s per-axis NURBS curves onto the
//! configured MCU axis assignment, producing one [`McuPushPlan`] per MCU
//! that has at least one non-trivial curve to load.
//!
//! Trivially-constant curves (all control points equal within `EPS_CONST`)
//! are skipped — the MCU keeps the corresponding handle slot at
//! [`UNUSED_HANDLE`] and the per-sample evaluator treats it as "axis idle".

use kalico_host_rt::producer::{CurveLoadParams, SegmentPushParams};
use trajectory::ShapedSegment;

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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct McuCaps {
    pub max_control_points: u32,
    pub max_knot_vector_len: u32,
    pub max_degree: u8,
    pub curve_pool_n: u16,
}

impl Default for McuCaps {
    /// Large-profile defaults for backward compatibility with firmware
    /// that doesn't yet implement QueryRuntimeCaps.
    fn default() -> Self {
        Self {
            max_control_points: 1830,
            max_knot_vector_len: 1850,
            max_degree: 10,
            curve_pool_n: 16,
        }
    }
}

impl From<kalico_protocol::messages::RuntimeCapsResponse> for McuCaps {
    fn from(r: kalico_protocol::messages::RuntimeCapsResponse) -> Self {
        Self {
            max_control_points: r.max_control_points,
            max_knot_vector_len: r.max_knot_vector_len,
            max_degree: r.max_degree,
            curve_pool_n: r.curve_pool_n,
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
pub fn build_push_params(
    shaped: &ShapedSegment,
    mcu_configs: &[McuAxisConfig],
    t_start_clock: u64,
    t_end_clock: u64,
) -> Vec<McuPushPlan> {
    let mut plans = Vec::with_capacity(mcu_configs.len());

    for cfg in mcu_configs {
        let mut curves_to_load: Vec<(usize, CurveLoadParams)> = Vec::new();
        for &axis_idx in &cfg.axes {
            if axis_idx >= shaped.axes.len() {
                continue;
            }
            let curve = &shaped.axes[axis_idx];
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
            curves_to_load.push((
                axis_idx,
                CurveLoadParams::from_scalar_nurbs_normalized(
                    curve,
                    shaped.t_start,
                    shaped.t_end,
                ),
            ));
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
                kinematics: 1,
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
        assert_eq!(octopus.params.kinematics, 1);

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
}
