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

/// Per-MCU configuration: which `ShapedSegment` axes this MCU is responsible
/// for, plus the firmware kinematics tag.
#[derive(Debug, Clone)]
pub struct McuAxisConfig {
    pub mcu_id: u32,
    /// Indices into `ShapedSegment::axes` (0=X, 1=Y, 2=Z) that this MCU drives.
    pub axes: Vec<usize>,
    /// Kinematics tag forwarded to the MCU via the configure-axes command.
    pub kinematics: u8,
    /// Per-MCU runtime sizing limits as reported by `QueryRuntimeCaps`
    /// (or `McuCaps::default()` for firmware that predates the message).
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

impl Default for McuCaps {
    /// Large-profile defaults used when the per-MCU `QueryRuntimeCaps`
    /// round-trip fails (e.g. transport timeout during attach). 62 KB
    /// matches the H7 `RUNTIME_TARGET_LARGE` total piece buffer.
    fn default() -> Self {
        Self {
            total_piece_memory: 62 * 1024,
        }
    }
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
