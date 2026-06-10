import collections

_homing_info = collections.namedtuple(
    "homing_info",
    [
        "speed",
        "position_endstop",
        "retract_speed",
        "retract_dist",
        "positive_dir",
        "second_homing_speed",
        "use_sensorless_homing",
        "min_home_dist",
        "accel",
    ],
)


class ServoRail:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.axis = self.name.split("_", 1)[1]
        if self.axis not in ("x", "y", "z"):
            raise config.error(
                "servo_%s: axis must be one of x/y/z (got %r)"
                % (self.axis, self.axis)
            )
        protocol = config.get("protocol")
        if protocol != "ethercat":
            raise config.error(
                "servo_%s: only 'protocol: ethercat' is supported "
                "(got %r)" % (self.axis, protocol)
            )
        self.node_name = config.get("node")
        self.rotation_distance = config.getfloat("rotation_distance", above=0.0)
        self.encoder_counts_per_rev = config.getint(
            "encoder_counts_per_rev", minval=1
        )
        self.velocity_ff = config.getboolean("velocity_ff", False)
        self.dynamics_profile = config.get("dynamics_profile", None)
        self.ff_torque_clamp = config.getfloat(
            "ff_torque_clamp", 30.0, above=0.0, maxval=400.0
        )
        self.position_min = config.getfloat("position_min", 0.0)
        self.position_max = config.getfloat(
            "position_max", above=self.position_min
        )
        self.position_endstop = 0.0
        self._active_callbacks = []

    def get_name(self, short=False):
        if short:
            return self.axis
        return self.name

    def get_steppers(self):
        return []

    def add_active_callback(self, cb):
        self._active_callbacks.append(cb)

    def get_endstops(self):
        return []

    def setup_itersolve(self, alloc_func, *params):
        return

    def add_extra_stepper(self, config):
        raise config.error(
            "servo_%s does not support extra steppers" % self.axis
        )

    def get_range(self):
        return self.position_min, self.position_max

    def get_homing_info(self):
        return _homing_info(
            speed=0.0,
            position_endstop=self.position_endstop,
            retract_speed=0.0,
            retract_dist=0.0,
            positive_dir=False,
            second_homing_speed=0.0,
            use_sensorless_homing=False,
            min_home_dist=0.0,
            accel=None,
        )

    def set_position(self, coord):
        return

    def get_commanded_position(self):
        return 0.0

    def get_node_name(self):
        return self.node_name

    def get_counts_per_mm(self):
        return self.encoder_counts_per_rev / self.rotation_distance

    def get_ff_config(self):
        return (self.velocity_ff, self.dynamics_profile, self.ff_torque_clamp)


class BridgeTorqueLine:
    def __init__(self, printer, node_name):
        self._printer = printer
        self._node_name = node_name

    def set_digital(self, print_time, value):
        node = self._printer.lookup_object("ethercat_node " + self._node_name)
        handle = node.get_bridge_handle()
        if handle is None:
            raise self._printer.command_error(
                "servo torque: ethercat_node %s has no bridge handle"
                % (self._node_name,)
            )
        bridge = self._printer.lookup_object("motion_bridge")
        bridge.set_torque(handle, bool(value), print_time)


def register_torque_enable(printer, config, rail):
    from . import stepper_enable

    line = BridgeTorqueLine(printer, rail.get_node_name())
    enable = stepper_enable.StepperEnablePin(line, 0)
    printer.load_object(config, "stepper_enable").register_motor(
        rail.get_name(), rail, enable
    )
