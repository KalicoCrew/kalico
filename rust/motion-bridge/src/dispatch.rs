use runtime::segment::KinematicTag;
use std::collections::{HashMap, HashSet};

// KINEMATICS_COREXY must equal KinematicTag::CoreXyAndE's wire discriminant; assertion below pins this.
pub const KINEMATICS_COREXY: u8 = KinematicTag::CoreXyAndE as u8;

const _: () = assert!(
    KinematicTag::CoreXyAndE as u8 == 0,
    "wire-ABI invariant broken: KinematicTag::CoreXyAndE discriminant must be 0 \
     (matches KINEMATICS_COREXY on the host and the MCU firmware's kinematics byte)",
);

pub const AXIS_X: usize = 0;
pub const AXIS_Y: usize = 1;
pub const AXIS_Z: usize = 2;
pub const AXIS_E: usize = 3;

#[derive(Debug, Clone)]
pub struct McuAxisConfig {
    pub mcu_id: u32,
    pub axes: Vec<usize>,
    pub kinematics: u8,
    pub caps: McuCaps,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct McuCaps {
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
    pub fn total_pieces(&self) -> usize {
        self.total_piece_memory as usize / 32
    }
}

impl Default for McuCaps {
    fn default() -> Self {
        // Fallback for firmware predating QueryRuntimeCaps: 62 KB = Octopus Pro H7 SRAM budget.
        Self {
            total_piece_memory: 62 * 1024,
        }
    }
}

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

pub fn cfg_is_corexy(cfg: &McuAxisConfig) -> bool {
    cfg.kinematics == KINEMATICS_COREXY && cfg.axes.contains(&AXIS_X) && cfg.axes.contains(&AXIS_Y)
}

pub fn motor_frame_xy(cfg: &McuAxisConfig, x: f64, y: f64) -> (f64, f64) {
    if cfg_is_corexy(cfg) {
        crate::kinematics::forward_corexy(x, y)
    } else {
        (x, y)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SeedSend {
    pub mcu_id: u32,
    pub x_q16: i32,
    pub y_q16: i32,
    pub z_q16: i32,
}

pub fn encode_q16(mm: f64) -> i32 {
    let raw = mm * 65536.0;
    raw.round().clamp(i32::MIN as f64, i32::MAX as f64) as i32
}

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

// `runtime_seed_position` is serial-only; EtherCAT nodes are position-commanded with no serial transport.
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
            kinematics: 1,
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
        assert_eq!(encode_q16(50.0), 3_276_800);
        assert_eq!(encode_q16(150.0), 9_830_400);
        assert_eq!(encode_q16(300.0), 19_660_800);
    }

    #[test]
    fn build_seed_sends_applies_per_mcu_transform() {
        let configs = vec![corexy_cfg(), cartesian_z_cfg()];
        let sends = build_seed_sends(&configs, 150.0, 150.0, 50.0);
        assert_eq!(sends.len(), 2);

        let octo = sends.iter().find(|s| s.mcu_id == 1).expect("octopus seed");
        assert_eq!(octo.x_q16, encode_q16(300.0));
        assert_eq!(octo.y_q16, encode_q16(0.0));
        assert_eq!(octo.z_q16, encode_q16(50.0));

        let z = sends.iter().find(|s| s.mcu_id == 2).expect("f446 seed");
        assert_eq!(z.x_q16, encode_q16(150.0));
        assert_eq!(z.y_q16, encode_q16(150.0));
        assert_eq!(z.z_q16, encode_q16(50.0));
    }

    #[test]
    fn build_serial_seed_sends_skips_ethercat_node() {
        let ec_cfg = McuAxisConfig {
            mcu_id: 1,
            axes: vec![AXIS_X],
            kinematics: KINEMATICS_COREXY,
            caps: McuCaps {
                total_piece_memory: 32 * 1024,
            },
        };
        let serial_cfg = McuAxisConfig {
            mcu_id: 2,
            axes: vec![AXIS_Y, AXIS_Z],
            kinematics: 1,
            caps: McuCaps {
                total_piece_memory: 62 * 1024,
            },
        };
        let configs = vec![ec_cfg, serial_cfg];
        let ethercat_mcu_ids: HashSet<u32> = [1u32].into_iter().collect();

        let sends = build_serial_seed_sends(&configs, &ethercat_mcu_ids, 100.0, 50.0, 10.0);

        assert!(
            sends.iter().all(|s| s.mcu_id != 1),
            "EtherCAT mcu_id=1 must not appear in serial seed sends; got: {sends:?}"
        );
        assert_eq!(
            sends.len(),
            1,
            "exactly one send for the serial MCU; got {sends:?}"
        );
        let serial = &sends[0];
        assert_eq!(serial.mcu_id, 2);
        assert_eq!(serial.x_q16, encode_q16(100.0));
        assert_eq!(serial.y_q16, encode_q16(50.0));
        assert_eq!(serial.z_q16, encode_q16(10.0));
    }

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
