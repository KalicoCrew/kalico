# Motion kinematics config parser + host-side kinematic transforms.
#
# This file is part of the Kalico motion-bridge integration (Stage D).
# Parses [printer] kinematics config and emits a KinematicsSpec for the
# Rust planner. Also hosts the cartesian→motor-slot transform that ties
# motor energization to actual motor motion (mirrors
# rust/runtime/src/kinematics.rs — that file is the authoritative
# stepping-path transform; this one drives enable-pin decisions on the
# host).
import logging


def motor_deltas(kin_name, dx, dy, dz, de):
    """Apply the host-side kinematic transform to cartesian deltas.

    Returns a 4-tuple of per-motor-slot deltas:
      corexy:    (A = dx+dy, B = dx-dy, Z = dz, E = de)
      cartesian: (X = dx,    Y = dy,    Z = dz, E = de)

    Slot indices match `MotionToolhead._configure_axes_per_mcu`'s
    `slot_names` ordering: 0=stepper_x, 1=stepper_y, 2=stepper_z,
    3=extruder. Any kinematic that ships a Rust transform in
    `rust/runtime/src/kinematics.rs` must also ship a matching branch
    here so energization stays tied to motion by construction.
    """
    if kin_name == "corexy":
        return (dx + dy, dx - dy, dz, de)
    # cartesian (and hybrid_corexy, which the bridge currently treats as
    # cartesian on the runtime side — see _configure_axes_per_mcu).
    return (dx, dy, dz, de)


class KinematicsSpec:
    """Parsed kinematics configuration for the Rust planner."""

    def __init__(self, kin_type, axes_config):
        self.kin_type = kin_type
        self.axes_config = axes_config

    def __repr__(self):
        return "KinematicsSpec(%s, %s)" % (self.kin_type, self.axes_config)


def parse_kinematics(config):
    """Parse [printer] kinematics and return a KinematicsSpec.

    Currently supports 'cartesian' and 'corexy'.
    """
    kin_name = config.get("kinematics")
    axes_config = {}

    if kin_name in ("cartesian", "corexy"):
        axes_config["max_velocity"] = config.getfloat("max_velocity", above=0.0)
        axes_config["max_accel"] = config.getfloat("max_accel", above=0.0)
        axes_config["max_z_velocity"] = config.getfloat(
            "max_z_velocity", None, above=0.0
        )
        axes_config["max_z_accel"] = config.getfloat(
            "max_z_accel", None, above=0.0
        )
    else:
        logging.warning(
            "motion_kinematics: unrecognized kinematics '%s', "
            "deferring to legacy path",
            kin_name,
        )

    spec = KinematicsSpec(kin_name, axes_config)
    logging.info("motion_kinematics: parsed %s", spec)
    return spec
