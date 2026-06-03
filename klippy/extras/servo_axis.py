# Servo axis — a position-commanded EtherCAT drive presented as an axis device.
#
# Distinct from stepper.PrinterRail: no step/dir pins, no microsteps, no
# itersolve, no MCU steppers. Part A scope is the kinematic spec + node
# binding only — no homing, no endstops, no commanded motion.
#
# Config section is a single token: `[servo_x]` (also `[servo_y]`,
# `[servo_z]`). This class is NOT auto-loaded as an extras object: klippy's
# main config loop calls load_object(config, "servo_x", None), which returns
# None silently (there is no module named "servo_x"). Instead the toolhead
# instantiates the rail directly — MotionToolhead._register_axis builds it
# via servo_axis.ServoRail(config.getsection("servo_<axis>")), exactly
# mirroring how stepper.PrinterRail is built from [stepper_<axis>]. The
# section is marked "used" by getsection + the options ServoRail reads, so
# check_unused_options is satisfied. No load_config_prefix / load_config
# here — this module just defines the class.
#
# The contract below mirrors exactly the rail methods/attributes that
# klippy/motion_toolhead.py's BridgeKinematics + MotionToolhead reach on
# the objects in `kin.rails` (so Task 9 can append a ServoRail alongside
# the stepper PrinterRails without provoking AttributeError).
import collections

# Field set copied verbatim from stepper.PrinterRail.get_homing_info()
# (klippy/stepper.py). Kept identical so any caller inspecting the
# namedtuple sees the same shape regardless of rail type.
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
        self.name = config.get_name()  # e.g. "servo_x"
        # Generic over axis: derive "x"/"y"/"z" from the section name by
        # stripping the "servo_" prefix. `[servo_x]` -> "x".
        self.axis = self.name.split("_", 1)[1]
        if self.axis not in ("x", "y", "z"):
            raise config.error(
                "servo_%s: axis must be one of x/y/z (got %r)"
                % (self.axis, self.axis))
        protocol = config.get("protocol")
        if protocol != "ethercat":
            raise config.error(
                "servo_%s: only 'protocol: ethercat' is supported "
                "(got %r)" % (self.axis, protocol)
            )
        # EtherCAT node handle name — Task 9 maps this to the matching
        # [ethercat_node] object.
        self.node_name = config.get("node")
        self.rotation_distance = config.getfloat("rotation_distance", above=0.0)
        # Axis range. Matches PrinterRail's position_min/position_max
        # attributes, which BridgeKinematics reads directly (get_status)
        # and via get_range() (home / set_position / check_move).
        self.position_min = config.getfloat("position_min", 0.0)
        self.position_max = config.getfloat(
            "position_max", above=self.position_min
        )
        self.position_endstop = 0.0

    # -- Identity -------------------------------------------------------
    # BridgeKinematics calls get_name(short=True) (motion_toolhead.py:162,
    # 286) to bucket rails by axis letter, and PrinterRail's get_name is the
    # bound MCU_stepper.get_name which accepts a `short` kwarg. The short
    # form must start with the axis letter so the "xyz".index(name[0])
    # lookup resolves; return the bare axis ("x") for short and the full
    # section name ("servo_x") for the non-short form.
    def get_name(self, short=False):
        if short:
            return self.axis
        return self.name

    # -- Stepper / endstop contract -------------------------------------
    # No MCU steppers: a servo's position feedback comes from the EtherCAT
    # endpoint, not host-side step generation. BridgeKinematics iterates
    # rail.get_steppers() in get_steppers/calc_position/set_position and
    # the corexy endstop cross-wiring; returning [] makes the servo
    # contribute no steppers (correct — calc_position handles the empty
    # case by returning 0.0).
    def get_steppers(self):
        return []

    def get_endstops(self):
        return []

    # itersolve is the host-side step-generation kinematic solver; a
    # position-commanded servo has none. BridgeKinematics._register_axis
    # calls setup_itersolve per-stepper (never on the rail in the bridge
    # path), but PrinterRail also exposes a rail-level setup_itersolve, so
    # keep an inert one for contract parity.
    def setup_itersolve(self, alloc_func, *params):
        return

    def add_extra_stepper(self, config):
        raise config.error(
            "servo_%s does not support extra steppers" % self.axis
        )

    # -- Range / homing spec --------------------------------------------
    def get_range(self):
        return self.position_min, self.position_max

    # Homing is NOT supported in Part A. This namedtuple exists only to
    # satisfy the rail-contract SHAPE the toolhead may inspect (same fields
    # as PrinterRail.get_homing_info()); it does not describe any real homing
    # behaviour. The zero speeds are inert not-implemented placeholders — a
    # future homing part will populate real values derived from config /
    # endstop geometry.
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

    # -- Position ------------------------------------------------------
    # BridgeKinematics.set_position drives per-stepper _set_mcu_position;
    # with no steppers the loop body is skipped, so this rail-level
    # set_position is inert (no commanded motion in Part A).
    def set_position(self, coord):
        return

    def get_commanded_position(self):
        return 0.0

    # -- EtherCAT binding ----------------------------------------------
    # Task 9 reads this to map the servo axis to its [ethercat_node] handle.
    def get_node_name(self):
        return self.node_name
