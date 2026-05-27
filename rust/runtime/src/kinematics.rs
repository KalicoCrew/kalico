//! Kinematic transforms. Spec §3.1 / §4.2 step 6.
//!
//! **Architecture note (2026-05-21):** The CoreXY transform is now applied by
//! the host bridge (`motion-bridge/src/dispatch.rs::build_push_params`) before
//! curves are serialised to the MCU. The MCU engine is motor-frame end-to-end
//! and no longer calls these functions in its hot path. They are retained here
//! for unit-test coverage of the transform math and as reference documentation
//! of the CoreXY geometry invariant.

/// `CoreXY`: (x, y, z, e) → (a=x+y, b=x−y, z, e).
/// Z and E are passed through unchanged.
///
/// `CoreXY` belt geometry: A = X + Y, B = X − Y. Inverse: X = (A+B)/2, Y = (A−B)/2.
///
/// **Callers:** host-side bridge only (via `nurbs::algebra` on NURBS curves).
/// Not called from any MCU hot path.
#[allow(clippy::inline_always)]
#[inline(always)]
#[cfg(any(test, feature = "host"))]
pub fn corexy_with_e(pos: [f32; 4]) -> [f32; 4] {
    [pos[0] + pos[1], pos[0] - pos[1], pos[2], pos[3]]
}

/// Cartesian identity: (x, y, z, e) → (x, y, z, e). Reserved for Step 6+ (F4x Z-only path).
///
/// **Callers:** host-side bridge only. Not called from any MCU hot path.
#[allow(clippy::inline_always)]
#[inline(always)]
#[cfg(any(test, feature = "host"))]
pub fn cartesian_xyz_with_e(pos: [f32; 4]) -> [f32; 4] {
    pos
}

#[cfg(test)]
mod tests;
