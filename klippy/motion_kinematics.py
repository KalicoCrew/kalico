import logging


def motor_deltas(kin_name, dx, dy, dz, de):
    if kin_name == "corexy":
        return (dx + dy, dx - dy, dz, de)
    # cartesian (and hybrid_corexy, treated as cartesian on the runtime side).
    return (dx, dy, dz, de)


class KinematicsSpec:
    def __init__(self, kin_type, axes_config):
        self.kin_type = kin_type
        self.axes_config = axes_config

    def __repr__(self):
        return "KinematicsSpec(%s, %s)" % (self.kin_type, self.axes_config)


def parse_kinematics(config):
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
