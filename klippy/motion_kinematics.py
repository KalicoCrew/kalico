# Motion kinematics config parser
#
# This file is part of the Kalico motion-bridge integration (Stage D).
# Parses [printer] kinematics config and emits a KinematicsSpec for the
# Rust planner. Phase 1: just parsed config, no runtime kinematics.
import logging


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
        axes_config["max_velocity"] = config.getfloat(
            "max_velocity", above=0.0
        )
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
