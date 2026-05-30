//! Per-MCU axis configuration — MCU identity, axis assignment, kinematics tag,
//! and runtime sizing limits. Used by the enqueue adapter (`enqueue.rs`) to
//! map `ShapedSegment` axes onto per-MCU piece streams.
//!
//! The old segment-era dispatch path (`build_push_params`, `McuPushPlan`,
//! `split_plan_if_needed`, `de_casteljau_split`, `extract_time_window`,
//! `CurveLoadParams`, `SegmentPushParams`, `fits_curve_load`, `UNUSED_HANDLE`,
//! `is_trivially_constant`) has been removed (Task 10).

use runtime::segment::KinematicTag;

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

pub const AXIS_X: usize = 0;
pub const AXIS_Y: usize = 1;
pub const AXIS_Z: usize = 2;
/// Extruder motor slot. A follower of shaped XY motion: it has a piece ring
/// on the MCU (so it must be counted for ring sizing) but no `ShapedSegment`
/// curve — the enqueue adapter skips it (index ≥ segment arity) and the MCU
/// derives E from the shaped XY trajectory.
pub const AXIS_E: usize = 3;

/// Per-MCU configuration: which `ShapedSegment` axes this MCU is responsible
/// for, plus the firmware kinematics tag.
#[derive(Debug, Clone)]
pub struct McuAxisConfig {
    pub mcu_id: u32,
    /// Motor slots this MCU drives. Indices 0=X, 1=Y, 2=Z map into
    /// `ShapedSegment::axes`; follower slots like `AXIS_E` (3) have a ring on the
    /// MCU (counted for ring sizing) but no segment curve — the enqueue adapter
    /// skips any index ≥ the segment's axis count. Used for ring-depth division
    /// and per-axis flow-control keys.
    pub axes: Vec<usize>,
    /// Kinematics tag forwarded to the MCU via the configure-axes command.
    pub kinematics: u8,
    /// Per-MCU runtime sizing limits as reported by `QueryRuntimeCaps`.
    /// Caps are mandatory for motion MCUs — a missing response is a hard
    /// failure at attach time (old/unflashed/mismatched firmware).
    pub caps: McuCaps,
}

/// Subset of `RuntimeCapsResponse` that the dispatcher needs to enforce
/// per-MCU sizing limits when planning a curve.
///
/// Simple-MCU-contract revision (2026-05-28): the MCU now reports a single
/// `total_piece_memory` (bytes). The host derives per-axis budgets from this
/// at `init_planner` time. Each piece is 32 bytes on the wire (`PushPieces`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct McuCaps {
    /// Total bytes available for piece storage across all per-axis rings,
    /// as reported by `RuntimeCapsResponse.total_piece_memory`.
    pub total_piece_memory: u32,
}

impl From<kalico_protocol::messages::RuntimeCapsResponse> for McuCaps {
    fn from(r: kalico_protocol::messages::RuntimeCapsResponse) -> Self {
        Self {
            total_piece_memory: r.total_piece_memory,
        }
    }
}

impl McuCaps {
    /// Maximum number of 32-byte pieces the MCU can hold in total.
    pub fn total_pieces(&self) -> usize {
        self.total_piece_memory as usize / 32
    }
}

/// True when this MCU drives both CoreXY motors and must receive motor-frame
/// `(A, B)` values rather than Cartesian `(X, Y)`. Single source of truth for
/// the CoreXY decision, shared by the piece path (`enqueue.rs`) and the seed
/// path (`build_seed_sends`) so they cannot drift.
pub fn cfg_is_corexy(cfg: &McuAxisConfig) -> bool {
    cfg.kinematics == KINEMATICS_COREXY
        && cfg.axes.contains(&AXIS_X)
        && cfg.axes.contains(&AXIS_Y)
}

/// Map a Cartesian `(x, y)` into this MCU's motor frame:
/// CoreXY → `(x + y, x − y)`; otherwise passthrough `(x, y)`. Z is always
/// passthrough and handled by the caller.
pub fn motor_frame_xy(cfg: &McuAxisConfig, x: f64, y: f64) -> (f64, f64) {
    if cfg_is_corexy(cfg) {
        (x + y, x - y)
    } else {
        (x, y)
    }
}

#[cfg(test)]
mod seed_tests {
    use super::*;

    fn corexy_cfg() -> McuAxisConfig {
        McuAxisConfig {
            mcu_id: 1,
            axes: vec![AXIS_X, AXIS_Y, AXIS_E],
            kinematics: KINEMATICS_COREXY,
            caps: McuCaps { total_piece_memory: 62 * 1024 },
        }
    }
    fn cartesian_z_cfg() -> McuAxisConfig {
        McuAxisConfig {
            mcu_id: 2,
            axes: vec![AXIS_Z],
            kinematics: 1, // CartesianXyzAndE
            caps: McuCaps { total_piece_memory: 62 * 1024 },
        }
    }

    #[test]
    fn cfg_is_corexy_true_only_for_corexy_xy_mcu() {
        assert!(cfg_is_corexy(&corexy_cfg()));
        assert!(!cfg_is_corexy(&cartesian_z_cfg()));
    }

    #[test]
    fn motor_frame_xy_transforms_corexy_passes_through_cartesian() {
        assert_eq!(motor_frame_xy(&corexy_cfg(), 150.0, 150.0), (300.0, 0.0));
        assert_eq!(motor_frame_xy(&corexy_cfg(), 10.0, 4.0), (14.0, 6.0));
        assert_eq!(motor_frame_xy(&cartesian_z_cfg(), 150.0, 150.0), (150.0, 150.0));
    }
}
