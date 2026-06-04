//! Per-MCU axis configuration — MCU identity, axis assignment, kinematics tag,
//! and runtime sizing limits. Used by the enqueue adapter (`enqueue.rs`) to
//! map `ShapedSegment` axes onto per-MCU piece streams.
//!
//! The old segment-era dispatch path (`build_push_params`, `McuPushPlan`,
//! `split_plan_if_needed`, `de_casteljau_split`, `extract_time_window`,
//! `CurveLoadParams`, `SegmentPushParams`, `fits_curve_load`, `UNUSED_HANDLE`,
//! `is_trivially_constant`) has been removed (Task 10).

use runtime::segment::KinematicTag;
use std::collections::{HashMap, HashSet};

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

impl Default for McuCaps {
    fn default() -> Self {
        // Large-profile fallback for firmware predating `QueryRuntimeCaps`.
        // 62 KB is the H7 SRAM budget for piece storage on the Octopus Pro.
        Self {
            total_piece_memory: 62 * 1024,
        }
    }
}

/// Build the per-MCU planner topology from a host-supplied descriptor list.
///
/// Each `mcus` entry is `(bridge_handle, axes, kinematics_tag)` where `axes`
/// holds `AXIS_*` indices as `u8` and `kinematics_tag` is a `KinematicTag`
/// discriminant. `caps_by_handle` supplies the per-MCU runtime capabilities;
/// a handle absent from the map gets `McuCaps::default()` (large-profile
/// fallback for firmware predating `QueryRuntimeCaps`).
///
/// Order is preserved from `mcus`. No hardcoded MCU identity, axis set, or
/// kinematics — every field comes from the caller.
pub fn build_mcu_configs<S: ::std::hash::BuildHasher>(
    mcus: &[(u32, Vec<u8>, u8)],
    caps_by_handle: &HashMap<u32, McuCaps, S>,
) -> Vec<McuAxisConfig> {
    mcus.iter()
        .map(|(handle, axes, tag)| McuAxisConfig {
            mcu_id: *handle,
            axes: axes.iter().map(|&a| a as usize).collect(),
            kinematics: *tag,
            caps: caps_by_handle.get(handle).copied().unwrap_or_default(),
        })
        .collect()
}

/// True when this MCU drives both CoreXY motors and must receive motor-frame
/// `(A, B)` values rather than Cartesian `(X, Y)`. Single source of truth for
/// the CoreXY decision, shared by the piece path (`enqueue.rs`) and the seed
/// path (`build_seed_sends`) so they cannot drift.
pub fn cfg_is_corexy(cfg: &McuAxisConfig) -> bool {
    cfg.kinematics == KINEMATICS_COREXY && cfg.axes.contains(&AXIS_X) && cfg.axes.contains(&AXIS_Y)
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

/// One MCU's motor-frame seed, already Q16.16-encoded for the
/// `runtime_seed_position` wire command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SeedSend {
    pub mcu_id: u32,
    pub x_q16: i32,
    pub y_q16: i32,
    pub z_q16: i32,
}

/// Encode millimetres as Q16.16 fixed point (the `runtime_seed_position` wire
/// format), rounding to nearest and clamping into `i32` range.
pub fn encode_q16(mm: f64) -> i32 {
    let raw = mm * 65536.0;
    raw.round().clamp(i32::MIN as f64, i32::MAX as f64) as i32
}

/// Build one [`SeedSend`] per configured MCU: apply the per-MCU motor-frame
/// transform to `(x, y)` (Z always passthrough) and Q16.16-encode. Pure — the
/// caller performs the actual `runtime_seed_position` send.
pub fn build_seed_sends(configs: &[McuAxisConfig], x: f64, y: f64, z: f64) -> Vec<SeedSend> {
    configs
        .iter()
        .map(|cfg| {
            let (mx, my) = motor_frame_xy(cfg, x, y);
            SeedSend {
                mcu_id: cfg.mcu_id,
                x_q16: encode_q16(mx),
                y_q16: encode_q16(my),
                z_q16: encode_q16(z),
            }
        })
        .collect()
}

/// Build serial-only [`SeedSend`]s: like [`build_seed_sends`] but skips any
/// MCU whose `mcu_id` is present in `ethercat_mcu_ids`.
///
/// # Why EtherCAT nodes are excluded
///
/// `runtime_seed_position` is a stepper-only serial command: EtherCAT servo
/// endpoints are position-commanded (absolute) and have no serial transport.
/// `kalico_stream_open` already re-seeds all nodes (including EtherCAT) before
/// this loop runs, so skipping EtherCAT here is correct, not an error.
/// Serial MCUs that genuinely lack `host_io` are a broken invariant and must
/// be caught by the caller.
pub fn build_serial_seed_sends<S: ::std::hash::BuildHasher>(
    configs: &[McuAxisConfig],
    ethercat_mcu_ids: &HashSet<u32, S>,
    x: f64,
    y: f64,
    z: f64,
) -> Vec<SeedSend> {
    configs
        .iter()
        .filter(|cfg| !ethercat_mcu_ids.contains(&cfg.mcu_id))
        .map(|cfg| {
            let (mx, my) = motor_frame_xy(cfg, x, y);
            SeedSend {
                mcu_id: cfg.mcu_id,
                x_q16: encode_q16(mx),
                y_q16: encode_q16(my),
                z_q16: encode_q16(z),
            }
        })
        .collect()
}

#[cfg(test)]
mod topology_tests {
    use super::*;
    use std::collections::HashMap;

    #[test]
    fn axis_e_is_three() {
        assert_eq!(AXIS_E, 3);
    }

    #[test]
    fn build_mcu_configs_two_mcu_corexy_with_e() {
        let mut caps = HashMap::new();
        caps.insert(
            7u32,
            McuCaps {
                total_piece_memory: 62 * 1024,
            },
        );
        caps.insert(
            9u32,
            McuCaps {
                total_piece_memory: 32 * 1024,
            },
        );
        // octopus(7) carries X,Y,E corexy; f446(9) carries Z cartesian.
        let mcus = vec![
            (7u32, vec![AXIS_X as u8, AXIS_Y as u8, AXIS_E as u8], 0u8),
            (9u32, vec![AXIS_Z as u8], 1u8),
        ];
        let cfgs = build_mcu_configs(&mcus, &caps);
        assert_eq!(cfgs.len(), 2);
        assert_eq!(cfgs[0].mcu_id, 7);
        assert_eq!(cfgs[0].axes, vec![AXIS_X, AXIS_Y, AXIS_E]);
        assert_eq!(cfgs[0].kinematics, 0);
        assert_eq!(
            cfgs[0].caps,
            McuCaps {
                total_piece_memory: 62 * 1024
            }
        );
        assert_eq!(cfgs[1].mcu_id, 9);
        assert_eq!(cfgs[1].axes, vec![AXIS_Z]);
        assert_eq!(cfgs[1].kinematics, 1);
    }

    #[test]
    fn build_mcu_configs_missing_caps_falls_back_to_default() {
        let caps: HashMap<u32, McuCaps> = HashMap::new();
        let mcus = vec![(7u32, vec![AXIS_X as u8, AXIS_Y as u8], 0u8)];
        let cfgs = build_mcu_configs(&mcus, &caps);
        assert_eq!(cfgs.len(), 1);
        assert_eq!(cfgs[0].caps, McuCaps::default());
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
            caps: McuCaps {
                total_piece_memory: 62 * 1024,
            },
        }
    }
    fn cartesian_z_cfg() -> McuAxisConfig {
        McuAxisConfig {
            mcu_id: 2,
            axes: vec![AXIS_Z],
            kinematics: 1, // CartesianXyzAndE
            caps: McuCaps {
                total_piece_memory: 62 * 1024,
            },
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
        assert_eq!(
            motor_frame_xy(&cartesian_z_cfg(), 150.0, 150.0),
            (150.0, 150.0)
        );
    }

    #[test]
    fn encode_q16_is_mm_times_65536_rounded() {
        assert_eq!(encode_q16(0.0), 0);
        assert_eq!(encode_q16(50.0), 3_276_800); // 50 * 65536
        assert_eq!(encode_q16(150.0), 9_830_400); // 150 * 65536
        assert_eq!(encode_q16(300.0), 19_660_800); // 300 * 65536
    }

    #[test]
    fn build_seed_sends_applies_per_mcu_transform() {
        let configs = vec![corexy_cfg(), cartesian_z_cfg()];
        let sends = build_seed_sends(&configs, 150.0, 150.0, 50.0);
        assert_eq!(sends.len(), 2);

        let octo = sends.iter().find(|s| s.mcu_id == 1).expect("octopus seed");
        assert_eq!(octo.x_q16, encode_q16(300.0)); // motor-A = X+Y
        assert_eq!(octo.y_q16, encode_q16(0.0)); // motor-B = X-Y
        assert_eq!(octo.z_q16, encode_q16(50.0)); // Z passthrough

        let z = sends.iter().find(|s| s.mcu_id == 2).expect("f446 seed");
        assert_eq!(z.x_q16, encode_q16(150.0)); // cartesian passthrough
        assert_eq!(z.y_q16, encode_q16(150.0));
        assert_eq!(z.z_q16, encode_q16(50.0));
    }

    // ── Regression: mixed topology (serial + EtherCAT) seed routing ───────
    //
    // Regression for the bench bug: a config with one serial stepper MCU
    // (mcu_id=2, stepper_y + stepper_z) and one EtherCAT node (mcu_id=1,
    // servo on axis X) caused `set_position` / `SET_KINEMATIC_POSITION` to
    // SIGABRT with "mcu_id 1 has no host_io (broken invariant)" because the
    // seed loop blindly sent `runtime_seed_position` to every McuAxisConfig,
    // including the EtherCAT one which has no serial transport.
    //
    // `build_serial_seed_sends` is the pure helper that makes the routing
    // decision testable without any PyO3 / KalicoHostIo wiring.

    /// Mixed topology: one serial stepper MCU (mcu_id=2) + one EtherCAT servo
    /// node (mcu_id=1). `build_serial_seed_sends` must emit exactly one send
    /// (for the serial MCU) and must not include the EtherCAT mcu_id.
    #[test]
    fn build_serial_seed_sends_skips_ethercat_node() {
        // mcu_id=1 is the EtherCAT servo (X axis, CoreXY kinematics).
        // The kinematics field is intentionally left as KINEMATICS_COREXY even
        // though axes=[AXIS_X] alone is structurally inconsistent for a real
        // CoreXY node — the EtherCAT node is filtered out on mcu_id membership
        // before any kinematics transform (motor_frame_xy) is ever invoked, so
        // the value here is irrelevant for this test.
        let ec_cfg = McuAxisConfig {
            mcu_id: 1,
            axes: vec![AXIS_X],
            kinematics: KINEMATICS_COREXY,
            caps: McuCaps {
                total_piece_memory: 32 * 1024,
            },
        };
        // mcu_id=2 is the serial stepper MCU (Y+Z, cartesian).
        let serial_cfg = McuAxisConfig {
            mcu_id: 2,
            axes: vec![AXIS_Y, AXIS_Z],
            kinematics: 1, // CartesianXyzAndE
            caps: McuCaps {
                total_piece_memory: 62 * 1024,
            },
        };
        let configs = vec![ec_cfg, serial_cfg];
        let ethercat_mcu_ids: HashSet<u32> = [1u32].into_iter().collect();

        let sends = build_serial_seed_sends(&configs, &ethercat_mcu_ids, 100.0, 50.0, 10.0);

        // EtherCAT node must be skipped entirely.
        assert!(
            sends.iter().all(|s| s.mcu_id != 1),
            "EtherCAT mcu_id=1 must not appear in serial seed sends; got: {sends:?}"
        );
        // Serial MCU must receive its seed.
        assert_eq!(
            sends.len(),
            1,
            "exactly one send for the serial MCU; got {sends:?}"
        );
        let serial = &sends[0];
        assert_eq!(serial.mcu_id, 2);
        // mcu_id=2 is cartesian: X and Y are passthroughs.
        assert_eq!(serial.x_q16, encode_q16(100.0));
        assert_eq!(serial.y_q16, encode_q16(50.0));
        assert_eq!(serial.z_q16, encode_q16(10.0));
    }

    /// All-serial topology: no EtherCAT nodes → `build_serial_seed_sends`
    /// must be identical to `build_seed_sends`.
    #[test]
    fn build_serial_seed_sends_all_serial_matches_build_seed_sends() {
        let configs = vec![corexy_cfg(), cartesian_z_cfg()];
        let ethercat_mcu_ids: HashSet<u32> = HashSet::new();
        let serial_sends = build_serial_seed_sends(&configs, &ethercat_mcu_ids, 150.0, 150.0, 50.0);
        let full_sends = build_seed_sends(&configs, 150.0, 150.0, 50.0);
        assert_eq!(
            serial_sends, full_sends,
            "with no EtherCAT nodes, build_serial_seed_sends must match build_seed_sends"
        );
    }

    /// All-EtherCAT topology: every MCU is an EtherCAT node → no sends.
    #[test]
    fn build_serial_seed_sends_all_ethercat_returns_empty() {
        let ec_cfg_1 = McuAxisConfig {
            mcu_id: 1,
            axes: vec![AXIS_X],
            kinematics: KINEMATICS_COREXY,
            caps: McuCaps {
                total_piece_memory: 32 * 1024,
            },
        };
        let ec_cfg_2 = McuAxisConfig {
            mcu_id: 3,
            axes: vec![AXIS_Y],
            kinematics: 1,
            caps: McuCaps {
                total_piece_memory: 32 * 1024,
            },
        };
        let configs = vec![ec_cfg_1, ec_cfg_2];
        let ethercat_mcu_ids: HashSet<u32> = [1u32, 3u32].into_iter().collect();
        let sends = build_serial_seed_sends(&configs, &ethercat_mcu_ids, 100.0, 50.0, 10.0);
        assert!(
            sends.is_empty(),
            "all-EtherCAT topology must produce zero serial seed sends; got {sends:?}"
        );
    }
}
