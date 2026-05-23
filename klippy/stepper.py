# Printer stepper support
#
# Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import logging
import math


class error(Exception):
    pass


######################################################################
# Steppers
######################################################################


# Interface to low-level mcu code. Step pulse generation, position
# tracking, and kinematics solving all live in the Rust motion engine
# (klippy/motion_bridge.py + rust/motion-bridge); this class is the
# host-side bookkeeping shim that records pin assignments, axis
# membership, and current direction so the planner and helper modules
# (homing, z_tilt, motion_report, ...) can keep using the existing
# stepper-object API surface.
class MCU_stepper:
    def __init__(
        self,
        name,
        step_pin_params,
        dir_pin_params,
        rotation_dist,
        steps_per_rotation,
        step_pulse_duration=None,
        units_in_radians=False,
    ):
        self._name = name
        self._rotation_dist = rotation_dist
        self._steps_per_rotation = steps_per_rotation
        self._step_pulse_duration = step_pulse_duration
        self._units_in_radians = units_in_radians
        self._step_dist = rotation_dist / steps_per_rotation
        self._mcu = step_pin_params["chip"]
        self._oid = self._mcu.create_oid()
        self._mcu.register_config_callback(self._build_config)
        self._step_pin = step_pin_params["pin"]
        self._invert_step = step_pin_params["invert"]
        if dir_pin_params["chip"] is not self._mcu:
            raise self._mcu.get_printer().config_error(
                "Stepper dir pin must be on same mcu as step pin"
            )
        self._dir_pin = dir_pin_params["pin"]
        self._invert_dir = self._orig_invert_dir = dir_pin_params["invert"]
        # Step-on-both-edges is the only mode the Rust runtime emits.
        self._step_both_edge = True
        self._req_step_both_edge = False
        self._mcu_position_offset = 0.0
        self._active_callbacks = []
        # Axes this stepper drives. Populated by setup_itersolve from the
        # bridge alloc_func's first bytes-typed argument (e.g. b"z" for
        # cartesian, b"+x" for corexy); queried by is_active_axis.
        self._bridge_active_axes = b""
        # Bridge-mode placeholders kept for API compatibility with helper
        # modules that still pass / inspect these handles.
        self._stepper_kinematics = None
        self._trapq = None
        self._tmc_current_helper = None

    def get_tmc_current_helper(self):
        return self._tmc_current_helper

    def set_tmc_current_helper(self, tmc_current_helper):
        self._tmc_current_helper = tmc_current_helper

    def get_mcu(self):
        return self._mcu

    def get_name(self, short=False):
        if short and self._name.startswith("stepper_"):
            return self._name[8:]
        return self._name

    def units_in_radians(self):
        # Returns true if distances are in radians instead of millimeters
        return self._units_in_radians

    def get_pulse_duration(self):
        return self._step_pulse_duration, self._step_both_edge

    def setup_default_pulse_duration(self, pulse_duration, step_both_edge):
        if self._step_pulse_duration is None:
            self._step_pulse_duration = pulse_duration
        self._req_step_both_edge = step_both_edge

    def setup_itersolve(self, alloc_func, *params):
        # Real stepper kinematics live in Rust, but Python-side callers
        # (z_tilt_ng, quad_gantry_level, homing axis routing) still query
        # is_active_axis to pick the steppers that move on a given axis.
        # All bridge alloc_funcs encode the axes the stepper drives in
        # their first bytes-typed param (e.g. b"z" for cartesian_stepper_alloc,
        # b"+x" / b"-y" for corexy_stepper_alloc). Stash that as the
        # axis-membership lookup table.
        for p in params:
            if isinstance(p, (bytes, bytearray)):
                self._bridge_active_axes = bytes(p)
                break

    def _build_config(self):
        # The kalico runtime emits step pulses by toggling step_pin
        # exactly once per requested step (runtime_emit_step_pulses in
        # src/stepper.c). Every edge — rising AND falling — counts as
        # a step, so the TMC driver is configured with DEDGE=1. The
        # invert_step / step_pulse_ticks args are accepted on the wire
        # for ABI compatibility but ignored on the MCU side (Stage B).
        self._step_both_edge = True
        self._step_pulse_duration = 0.0
        invert_step = -1
        step_pulse_ticks = 0
        self._mcu.add_config_cmd(
            "config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d"
            " step_pulse_ticks=%u"
            % (
                self._oid,
                self._step_pin,
                self._dir_pin,
                invert_step,
                step_pulse_ticks,
            )
        )

    def get_oid(self):
        return self._oid

    def get_step_dist(self):
        return self._step_dist

    def get_rotation_distance(self):
        return self._rotation_dist, self._steps_per_rotation

    def set_rotation_distance(self, rotation_dist):
        mcu_pos = self.get_mcu_position()
        self._rotation_dist = rotation_dist
        self._step_dist = rotation_dist / self._steps_per_rotation
        self.set_stepper_kinematics(self._stepper_kinematics)
        self._set_mcu_position(mcu_pos)

    def get_dir_inverted(self):
        return self._invert_dir, self._orig_invert_dir

    def set_dir_inverted(self, invert_dir):
        invert_dir = not not invert_dir
        if invert_dir == self._invert_dir:
            return
        self._invert_dir = invert_dir
        self._mcu.get_printer().send_event("stepper:set_dir_inverted", self)

    def calc_position_from_coord(self, coord):
        # Bridge: position tracking lives in Rust.
        return 0.0

    def set_position(self, coord):
        # Bridge: position tracking lives in Rust.
        return

    def get_commanded_position(self):
        # Bridge: position tracking lives in Rust.
        return 0.0

    def get_mcu_position(self, cmd_pos=None):
        if cmd_pos is None:
            cmd_pos = self.get_commanded_position()
        mcu_pos_dist = cmd_pos + self._mcu_position_offset
        mcu_pos = mcu_pos_dist / self._step_dist
        if mcu_pos >= 0.0:
            return int(mcu_pos + 0.5)
        return int(mcu_pos - 0.5)

    def _set_mcu_position(self, mcu_pos):
        mcu_pos_dist = mcu_pos * self._step_dist
        self._mcu_position_offset = mcu_pos_dist - self.get_commanded_position()

    def get_past_mcu_position(self, print_time):
        bridge = getattr(self._mcu, '_motion_bridge', None)
        if bridge is not None and getattr(bridge, '_software_trip_active', False):
            try:
                pos_xyz = bridge.get_homing_position_at_time(print_time)
            except Exception:
                return getattr(self, "_bridge_last_trip_step_count",
                               self.get_mcu_position())
            motor_pos = self._calc_motor_position_from_xyz(pos_xyz)
            mcu_pos_dist = motor_pos + self._mcu_position_offset
            mcu_pos = mcu_pos_dist / self._step_dist
            if mcu_pos >= 0.0:
                return int(mcu_pos + 0.5)
            return int(mcu_pos - 0.5)
        return getattr(self, "_bridge_last_trip_step_count",
                       self.get_mcu_position())

    def _calc_motor_position_from_xyz(self, pos_xyz):
        bridge = getattr(self._mcu, '_motion_bridge', None)
        kin = getattr(bridge, '_kinematics_name', 'cartesian') \
            if bridge else 'cartesian'
        axis = self._bridge_active_axes
        if kin == 'corexy':
            if axis in (b'x', b'+x'):
                return pos_xyz[0] + pos_xyz[1]
            elif axis in (b'y', b'-y'):
                return pos_xyz[0] - pos_xyz[1]
            else:
                idx = {b'z': 2}.get(axis, 2)
                return pos_xyz[idx]
        else:
            idx = {b'x': 0, b'+x': 0, b'y': 1, b'+y': 1,
                   b'z': 2}.get(axis, 0)
            return pos_xyz[idx]

    def bridge_set_position_from_step_count(self, step_count):
        # Step 7-D §5.3: bridge-mode trip-position reconciliation. Apply an
        # authoritative MCU step counter snapshot (from a kalico_endstop
        # trip event) directly via _set_mcu_position and retain it for
        # get_past_mcu_position().
        step_count = int(step_count)
        self._bridge_last_trip_step_count = step_count
        self._set_mcu_position(step_count)
        logging.info(
            "[bridge-trace] stepper trip snapshot: stepper=%s count=%d",
            self.get_name(), step_count,
        )

    def mcu_to_commanded_position(self, mcu_pos):
        return mcu_pos * self._step_dist - self._mcu_position_offset

    def dump_steps(self, count, start_clock, end_clock):
        # Bridge: step emission lives in Rust, no C stepcompress to drain.
        return ([], 0)

    def get_stepper_kinematics(self):
        return self._stepper_kinematics

    def set_stepper_kinematics(self, sk):
        old_sk = self._stepper_kinematics
        self._stepper_kinematics = sk
        return old_sk

    def note_homing_end(self):
        # Bridge: homing handled in Rust.
        return

    def get_trapq(self):
        return self._trapq

    def set_trapq(self, tq):
        old_tq = self._trapq
        self._trapq = tq
        return old_tq

    def add_active_callback(self, cb):
        self._active_callbacks.append(cb)

    def generate_steps(self, flush_time):
        # Bridge: step generation lives in Rust. Drain the active-callback
        # list so callers that registered one don't leak references; the
        # Rust side already signals these via its own activity surface, so
        # we don't need to fire them here.
        if self._active_callbacks:
            self._active_callbacks = []

    def is_active_axis(self, axis):
        # Match against the axes recorded at setup_itersolve time. Strings
        # like b"+x" / b"-y" (corexy) are scanned membership-style so a
        # query for "x" returns True on either sign.
        return axis.encode() in self._bridge_active_axes


# Helper code to build a stepper object from a config section
def PrinterStepper(config, units_in_radians=False):
    printer = config.get_printer()
    name = config.get_name()
    # Stepper definition
    ppins = printer.lookup_object("pins")
    step_pin = config.get("step_pin")
    step_pin_params = ppins.lookup_pin(step_pin, can_invert=True)
    dir_pin = config.get("dir_pin")
    dir_pin_params = ppins.lookup_pin(dir_pin, can_invert=True)
    rotation_dist, steps_per_rotation = parse_step_distance(
        config, units_in_radians, True
    )
    step_pulse_duration = config.getfloat(
        "step_pulse_duration", None, minval=0.0, maxval=0.001
    )
    mcu_stepper = MCU_stepper(
        name,
        step_pin_params,
        dir_pin_params,
        rotation_dist,
        steps_per_rotation,
        step_pulse_duration,
        units_in_radians,
    )
    # Phase-stepping mode: read from config; default off (StepTime).
    # The capability check against the MCU's identify bitmap is deferred to
    # connect time (MotionToolhead._configure_axes_per_mcu), when the MCU caps
    # are known.  Config-parse runs before MCU identify.
    mcu_stepper.phase_stepping = config.getboolean("phase_stepping", False)
    # Register with helper modules
    for mname in ["stepper_enable", "force_move", "motion_report"]:
        m = printer.load_object(config, mname)
        m.register_stepper(config, mcu_stepper)
    return mcu_stepper


# Parse stepper gear_ratio config parameter
def parse_gear_ratio(config, note_valid):
    gear_ratio = config.getlists(
        "gear_ratio",
        (),
        seps=(":", ","),
        count=2,
        parser=float,
        note_valid=note_valid,
    )
    result = 1.0
    for g1, g2 in gear_ratio:
        result *= g1 / g2
    return result


# Obtain "step distance" information from a config section
def parse_step_distance(config, units_in_radians=None, note_valid=False):
    # Check rotation_distance and gear_ratio
    if units_in_radians is None:
        # Caller doesn't know if units are in radians - infer it
        rd = config.get("rotation_distance", None, note_valid=False)
        gr = config.get("gear_ratio", None, note_valid=False)
        units_in_radians = rd is None and gr is not None
    if units_in_radians:
        rotation_dist = 2.0 * math.pi
        config.get("gear_ratio", note_valid=note_valid)
    else:
        rotation_dist = config.getfloat(
            "rotation_distance", above=0.0, note_valid=note_valid
        )
    # Check microsteps and full_steps_per_rotation
    microsteps = config.getint("microsteps", minval=1, note_valid=note_valid)
    full_steps = config.getint(
        "full_steps_per_rotation", 200, minval=1, note_valid=note_valid
    )
    if full_steps % 4:
        raise config.error(
            "full_steps_per_rotation invalid in section '%s'"
            % (config.get_name(),)
        )
    gearing = parse_gear_ratio(config, note_valid)
    return rotation_dist, full_steps * microsteps * gearing


######################################################################
# Stepper controlled rails
######################################################################


# A motor control "rail" with one (or more) steppers and one (or more)
# endstops.
class PrinterRail:
    def __init__(
        self,
        config,
        need_position_minmax=True,
        default_position_endstop=None,
        units_in_radians=False,
    ):
        # Primary stepper and endstop
        self.stepper_units_in_radians = units_in_radians
        self.steppers = []
        self.endstops = []
        self.endstop_map = {}
        self.add_extra_stepper(config)
        mcu_stepper = self.steppers[0]
        self._tmc_current_helpers = None
        self.get_name = mcu_stepper.get_name
        self.get_commanded_position = mcu_stepper.get_commanded_position
        self.calc_position_from_coord = mcu_stepper.calc_position_from_coord
        # Primary endstop position
        mcu_endstop = self.endstops[0][0]
        if hasattr(mcu_endstop, "get_position_endstop"):
            self.position_endstop = mcu_endstop.get_position_endstop()
        elif default_position_endstop is None:
            self.position_endstop = config.getfloat("position_endstop")
        else:
            self.position_endstop = config.getfloat(
                "position_endstop", default_position_endstop
            )
        endstop_pin = config.get("endstop_pin", None)
        # check for ":virtual_endstop" to make sure we don't detect ":z_virtual_endstop"
        endstop_is_virtual = (
            endstop_pin is not None and ":virtual_endstop" in endstop_pin
        )

        # Axis range
        if need_position_minmax:
            self.position_min = config.getfloat("position_min", 0.0)
            self.position_max = config.getfloat(
                "position_max", above=self.position_min
            )
        else:
            self.position_min = 0.0
            self.position_max = self.position_endstop
        if (
            self.position_endstop < self.position_min
            or self.position_endstop > self.position_max
        ):
            raise config.error(
                "position_endstop in section '%s' must be between"
                " position_min and position_max" % config.get_name()
            )
        # Homing mechanics
        self.use_sensorless_homing = config.getboolean(
            "use_sensorless_homing", endstop_is_virtual
        )

        self.homing_speed = config.getfloat("homing_speed", 5.0, above=0.0)

        default_second_homing_speed = self.homing_speed / 2.0
        if self.use_sensorless_homing:
            default_second_homing_speed = self.homing_speed

        self.second_homing_speed = config.getfloat(
            "second_homing_speed", default_second_homing_speed, above=0.0
        )
        self.homing_retract_speed = config.getfloat(
            "homing_retract_speed", self.homing_speed, above=0.0
        )
        self.homing_retract_dist = config.getfloat(
            "homing_retract_dist", 5.0, minval=0.0
        )
        self.homing_positive_dir = config.getboolean(
            "homing_positive_dir", None
        )

        self.min_home_dist = config.getfloat(
            "min_home_dist", self.homing_retract_dist, minval=0.0
        )

        self.homing_accel = config.getfloat("homing_accel", None, above=0.0)

        if self.homing_positive_dir is None:
            axis_len = self.position_max - self.position_min
            if self.position_endstop <= self.position_min + axis_len / 4.0:
                self.homing_positive_dir = False
            elif self.position_endstop >= self.position_max - axis_len / 4.0:
                self.homing_positive_dir = True
            else:
                raise config.error(
                    "Unable to infer homing_positive_dir in section '%s'"
                    % (config.get_name(),)
                )
            config.getboolean("homing_positive_dir", self.homing_positive_dir)
        elif (
            self.homing_positive_dir
            and self.position_endstop == self.position_min
        ) or (
            not self.homing_positive_dir
            and self.position_endstop == self.position_max
        ):
            raise config.error(
                "Invalid homing_positive_dir / position_endstop in '%s'"
                % (config.get_name(),)
            )

    def get_tmc_current_helpers(self):
        if self._tmc_current_helpers is None:
            self._tmc_current_helpers = [
                s.get_tmc_current_helper() for s in self.steppers
            ]
        return self._tmc_current_helpers

    def get_range(self):
        return self.position_min, self.position_max

    def get_homing_info(self):
        homing_info = collections.namedtuple(
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
        )(
            self.homing_speed,
            self.position_endstop,
            self.homing_retract_speed,
            self.homing_retract_dist,
            self.homing_positive_dir,
            self.second_homing_speed,
            self.use_sensorless_homing,
            self.min_home_dist,
            self.homing_accel,
        )
        return homing_info

    def get_steppers(self):
        return list(self.steppers)

    def get_endstops(self):
        return list(self.endstops)

    def add_extra_stepper(self, config):
        stepper = PrinterStepper(config, self.stepper_units_in_radians)
        self.steppers.append(stepper)
        if self.endstops and config.get("endstop_pin", None) is None:
            # No endstop defined - use primary endstop
            self.endstops[0][0].add_stepper(stepper)
            return
        endstop_pin = config.get("endstop_pin")
        printer = config.get_printer()
        ppins = printer.lookup_object("pins")
        pin_params = ppins.parse_pin(endstop_pin, True, True)
        # Normalize pin name
        pin_name = "%s:%s" % (pin_params["chip_name"], pin_params["pin"])
        # Look for already-registered endstop
        endstop = self.endstop_map.get(pin_name, None)
        if endstop is None:
            # New endstop, register it
            mcu_endstop = ppins.setup_pin("endstop", endstop_pin)
            self.endstop_map[pin_name] = {
                "endstop": mcu_endstop,
                "invert": pin_params["invert"],
                "pullup": pin_params["pullup"],
            }
            name = stepper.get_name(short=True)
            self.endstops.append((mcu_endstop, name))
            query_endstops = printer.load_object(config, "query_endstops")
            query_endstops.register_endstop(mcu_endstop, name)
        else:
            mcu_endstop = endstop["endstop"]
            changed_invert = pin_params["invert"] != endstop["invert"]
            changed_pullup = pin_params["pullup"] != endstop["pullup"]
            if changed_invert or changed_pullup:
                raise error(
                    "Printer rail %s shared endstop pin %s "
                    "must specify the same pullup/invert settings"
                    % (self.get_name(), pin_name)
                )
        mcu_endstop.add_stepper(stepper)

    def setup_itersolve(self, alloc_func, *params):
        for stepper in self.steppers:
            stepper.setup_itersolve(alloc_func, *params)

    def generate_steps(self, flush_time):
        for stepper in self.steppers:
            stepper.generate_steps(flush_time)

    def set_trapq(self, trapq):
        for stepper in self.steppers:
            stepper.set_trapq(trapq)

    def set_position(self, coord):
        for stepper in self.steppers:
            stepper.set_position(coord)


# Wrapper for dual stepper motor support
def LookupMultiRail(
    config,
    need_position_minmax=True,
    default_position_endstop=None,
    units_in_radians=False,
):
    rail = PrinterRail(
        config, need_position_minmax, default_position_endstop, units_in_radians
    )
    for i in range(1, 99):
        if not config.has_section(config.get_name() + str(i)):
            break
        rail.add_extra_stepper(config.getsection(config.get_name() + str(i)))
    return rail
