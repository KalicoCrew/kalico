import collections

from ..rail import BaseRail
from . import servo_param

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


def infer_positive_dir(
    config, axis, position_endstop, position_min, position_max
):
    if position_endstop == position_min:
        return False
    if position_endstop == position_max:
        return True
    raise config.error(
        "servo_%s: position_endstop %.3f must equal position_min (%.3f) "
        "or position_max (%.3f)"
        % (axis, position_endstop, position_min, position_max)
    )


class ServoRail(BaseRail):
    def __init__(self, config):
        super().__init__()
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
        self._parse_position_range(config)
        self.endstop_pin = config.get("endstop_pin", None)
        if self.endstop_pin is None:
            self.position_endstop = 0.0
            self.homing_speed = 0.0
            self.homing_retract_dist = 0.0
            self.homing_retract_speed = 0.0
            self.homing_positive_dir = False
            self.homing_following_error = 0.0
            self.homing_max_torque = 0.0
        else:
            self.position_endstop = config.getfloat("position_endstop")
            self._parse_homing_speeds(config)
            self.homing_positive_dir = infer_positive_dir(
                config,
                self.axis,
                self.position_endstop,
                self.position_min,
                self.position_max,
            )
            self.homing_following_error = config.getfloat(
                "homing_following_error", 2.5, above=0.0
            )
            self.homing_max_torque = config.getfloat(
                "homing_max_torque", 50.0, above=0.0, maxval=400.0
            )
        self.following_error = config.getfloat(
            "following_error", None, above=0.0
        )
        self.max_torque = config.getfloat(
            "max_torque", None, above=0.0, maxval=400.0
        )
        self._active_callbacks = []
        try:
            self.sdo_params = servo_param.parse_params_block(
                config.get("params", "")
            )
        except ValueError as e:
            raise config.error("servo_%s params: %s" % (self.axis, e))

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

    def set_position(self, coord):
        return

    def get_commanded_position(self):
        return 0.0

    def get_node_name(self):
        return self.node_name

    def get_counts_per_mm(self):
        return self.encoder_counts_per_rev / self.rotation_distance

    def get_sdo_params(self):
        return self.sdo_params

    def get_homing_drive_limits(self):
        counts_per_mm = self.encoder_counts_per_rev / self.rotation_distance
        return (
            int(round(self.homing_following_error * counts_per_mm)),
            int(round(self.homing_max_torque * 10.0)),
        )

    def get_session_drive_limits(self):
        counts_per_mm = self.encoder_counts_per_rev / self.rotation_distance
        counts = None
        if self.following_error is not None:
            counts = int(round(self.following_error * counts_per_mm))
        tenth_pct = None
        if self.max_torque is not None:
            tenth_pct = int(round(self.max_torque * 10.0))
        return counts, tenth_pct


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
