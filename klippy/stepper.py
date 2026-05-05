# Printer stepper support
#
# Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import math

from . import chelper


class error(Exception):
    pass


######################################################################
# Steppers
######################################################################

MIN_BOTH_EDGE_DURATION = 0.000000500
MIN_OPTIMIZED_BOTH_EDGE_DURATION = 0.000000150


# Interface to low-level mcu and chelper code
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
        self._oid = oid = self._mcu.create_oid()
        self._mcu.register_config_callback(self._build_config)
        self._step_pin = step_pin_params["pin"]
        self._invert_step = step_pin_params["invert"]
        if dir_pin_params["chip"] is not self._mcu:
            raise self._mcu.get_printer().config_error(
                "Stepper dir pin must be on same mcu as step pin"
            )
        self._dir_pin = dir_pin_params["pin"]
        self._invert_dir = self._orig_invert_dir = dir_pin_params["invert"]
        self._step_both_edge = self._req_step_both_edge = False
        self._mcu_position_offset = 0.0
        self._reset_cmd_tag = self._get_position_cmd = None
        self._active_callbacks = []
        ffi_main, ffi_lib = chelper.get_ffi()
        self._stepqueue = ffi_main.gc(
            ffi_lib.stepcompress_alloc(oid), ffi_lib.stepcompress_free
        )
        ffi_lib.stepcompress_set_invert_sdir(self._stepqueue, self._invert_dir)
        self._mcu.register_stepqueue(self._stepqueue)
        self._stepper_kinematics = None
        self._itersolve_generate_steps = ffi_lib.itersolve_generate_steps
        self._itersolve_check_active = ffi_lib.itersolve_check_active
        self._trapq = ffi_main.NULL
        self._mcu.get_printer().register_event_handler(
            "klippy:connect", self._query_mcu_position
        )
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
        ffi_main, ffi_lib = chelper.get_ffi()
        sk = ffi_main.gc(getattr(ffi_lib, alloc_func)(*params), ffi_lib.free)
        self.set_stepper_kinematics(sk)

    def _build_config(self):
        if self._step_pulse_duration is None:
            self._step_pulse_duration = 0.000002
        invert_step = self._invert_step
        # Check if can enable "step on both edges"
        constants = self._mcu.get_constants()
        ssbe = int(constants.get("STEPPER_STEP_BOTH_EDGE", "0"))
        sbe = int(constants.get("STEPPER_BOTH_EDGE", "0"))
        sou = int(constants.get("STEPPER_OPTIMIZED_UNSTEP", "0"))
        want_both_edges = self._req_step_both_edge
        if self._step_pulse_duration > MIN_BOTH_EDGE_DURATION:
            # If user has requested a very large step pulse duration
            # then disable step on both edges (rise and fall times may
            # not be symetric)
            want_both_edges = False
        elif (
            sbe and self._step_pulse_duration > MIN_OPTIMIZED_BOTH_EDGE_DURATION
        ):
            # Older MCU and user has requested large pulse duration
            want_both_edges = False
        elif not sbe and not ssbe:
            # Older MCU that doesn't support step on both edges
            want_both_edges = False
        elif sou:
            # MCU has optimized step/unstep - better to use that
            want_both_edges = False
        if want_both_edges:
            self._step_both_edge = True
            invert_step = -1
            if sbe:
                # Older MCU requires setting step_pulse_ticks=0 to enable
                self._step_pulse_duration = 0.0
        # Configure stepper object
        step_pulse_ticks = self._mcu.seconds_to_clock(self._step_pulse_duration)
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
        self._mcu.add_config_cmd(
            "reset_step_clock oid=%d clock=0" % (self._oid,), on_restart=True
        )
        step_cmd_tag = self._mcu.lookup_command(
            "queue_step oid=%c interval=%u count=%hu add=%hi"
        ).get_command_tag()
        dir_cmd_tag = self._mcu.lookup_command(
            "set_next_step_dir oid=%c dir=%c"
        ).get_command_tag()
        self._reset_cmd_tag = self._mcu.lookup_command(
            "reset_step_clock oid=%c clock=%u"
        ).get_command_tag()
        self._get_position_cmd = self._mcu.lookup_query_command(
            "stepper_get_position oid=%c",
            "stepper_position oid=%c pos=%i",
            oid=self._oid,
        )
        max_error = self._mcu.get_max_stepper_error()
        max_error_ticks = self._mcu.seconds_to_clock(max_error)
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.stepcompress_fill(
            self._stepqueue, max_error_ticks, step_cmd_tag, dir_cmd_tag
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
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.stepcompress_set_invert_sdir(self._stepqueue, invert_dir)
        self._mcu.get_printer().send_event("stepper:set_dir_inverted", self)

    def calc_position_from_coord(self, coord):
        ffi_main, ffi_lib = chelper.get_ffi()
        return ffi_lib.itersolve_calc_position_from_coord(
            self._stepper_kinematics, coord[0], coord[1], coord[2]
        )

    def set_position(self, coord):
        mcu_pos = self.get_mcu_position()
        sk = self._stepper_kinematics
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.itersolve_set_position(sk, coord[0], coord[1], coord[2])
        self._set_mcu_position(mcu_pos)

    def get_commanded_position(self):
        ffi_main, ffi_lib = chelper.get_ffi()
        return ffi_lib.itersolve_get_commanded_pos(self._stepper_kinematics)

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
        clock = self._mcu.print_time_to_clock(print_time)
        ffi_main, ffi_lib = chelper.get_ffi()
        pos = ffi_lib.stepcompress_find_past_position(self._stepqueue, clock)
        return int(pos)

    def mcu_to_commanded_position(self, mcu_pos):
        return mcu_pos * self._step_dist - self._mcu_position_offset

    def dump_steps(self, count, start_clock, end_clock):
        ffi_main, ffi_lib = chelper.get_ffi()
        data = ffi_main.new("struct pull_history_steps[]", count)
        count = ffi_lib.stepcompress_extract_old(
            self._stepqueue, data, count, start_clock, end_clock
        )
        return (data, count)

    def get_stepper_kinematics(self):
        return self._stepper_kinematics

    def set_stepper_kinematics(self, sk):
        old_sk = self._stepper_kinematics
        mcu_pos = 0
        if old_sk is not None:
            mcu_pos = self.get_mcu_position()
        self._stepper_kinematics = sk
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.itersolve_set_stepcompress(sk, self._stepqueue, self._step_dist)
        self.set_trapq(self._trapq)
        self._set_mcu_position(mcu_pos)
        return old_sk

    def note_homing_end(self):
        ffi_main, ffi_lib = chelper.get_ffi()
        ret = ffi_lib.stepcompress_reset(self._stepqueue, 0)
        if ret:
            raise error("Internal error in stepcompress")
        data = (self._reset_cmd_tag, self._oid, 0)
        ret = ffi_lib.stepcompress_queue_msg(self._stepqueue, data, len(data))
        if ret:
            raise error("Internal error in stepcompress")
        self._query_mcu_position()

    def _query_mcu_position(self):
        if self._mcu.is_fileoutput() or self._mcu.non_critical_disconnected:
            return
        params = self._get_position_cmd.send([self._oid])
        last_pos = params["pos"]
        if self._invert_dir:
            last_pos = -last_pos
        print_time = self._mcu.estimated_print_time(params["#receive_time"])
        clock = self._mcu.print_time_to_clock(print_time)
        ffi_main, ffi_lib = chelper.get_ffi()
        ret = ffi_lib.stepcompress_set_last_position(
            self._stepqueue, clock, last_pos
        )
        if ret:
            raise error("Internal error in stepcompress")
        self._set_mcu_position(last_pos)
        self._mcu.get_printer().send_event("stepper:sync_mcu_position", self)

    def get_trapq(self):
        return self._trapq

    def set_trapq(self, tq):
        ffi_main, ffi_lib = chelper.get_ffi()
        if tq is None:
            tq = ffi_main.NULL
        ffi_lib.itersolve_set_trapq(self._stepper_kinematics, tq)
        old_tq = self._trapq
        self._trapq = tq
        return old_tq

    def add_active_callback(self, cb):
        self._active_callbacks.append(cb)

    def generate_steps(self, flush_time):
        # Check for activity if necessary
        if self._active_callbacks:
            sk = self._stepper_kinematics
            ret = self._itersolve_check_active(sk, flush_time)
            if ret:
                cbs = self._active_callbacks
                self._active_callbacks = []
                for cb in cbs:
                    cb(ret)
        # Generate steps
        sk = self._stepper_kinematics
        ret = self._itersolve_generate_steps(sk, flush_time)
        if ret:
            raise error("Internal error in stepcompress")

    def is_active_axis(self, axis):
        ffi_main, ffi_lib = chelper.get_ffi()
        a = axis.encode()
        return ffi_lib.itersolve_is_active_axis(self._stepper_kinematics, a)


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


endstop_pin_config = collections.namedtuple(
    "endstop_pin_config", ["style", "min_pin", "max_pin", "legacy_pin"]
)


def _read_endstop_pins(config) -> endstop_pin_config:
    min_pin = config.get("endstop_min_pin", None)
    max_pin = config.get("endstop_max_pin", None)
    legacy_pin = config.get("endstop_pin", None)
    if legacy_pin is not None and (min_pin is not None or max_pin is not None):
        raise config.error(
            "endstop_pin cannot be used with endstop_min_pin or endstop_max_pin "
            "in section '%s'" % (config.get_name(),)
        )
    if legacy_pin is not None:
        style = "legacy"
    elif min_pin is not None or max_pin is not None:
        style = "minmax"
    else:
        style = None
    return endstop_pin_config(style, min_pin, max_pin, legacy_pin)


def choose_endstop_name(short_name, *, is_homing_side, side):
    """Return stable endstop name for QUERY_ENDSTOPS output.

    Exactly one endstop per rail keeps the unsuffixed short_name ("x", "y",
    "z") on the homing side (for backwards compatibility). The opposite side is suffixed ("x_min" / "x_max")
    to avoid name collisions.
    """
    if is_homing_side:
        return short_name
    return "%s_%s" % (short_name, side)


SIDE_MIN = 0
SIDE_MAX = 1
SIDE_NAME = ("min", "max")


def _side_from_direction(is_positive_dir):
    return SIDE_MAX if is_positive_dir else SIDE_MIN


class EndstopCollection:
    """Owns one rail side's endstops and query_endstops registration."""

    def __init__(self, printer, config, *, rail_name):
        self.printer = printer
        self._config = config
        self.rail_name = rail_name
        # Registered (mcu_endstop, name) pairs (names are stable and unique).
        self.endstops = []
        # Normalized pin name -> endstop mapping for pin sharing.
        self.endstop_map = {}
        # Reverse lookup to make endstop naming/registration idempotent.
        self._endstop_to_mapping = {}
        self._query_endstops = None

    def get_endstops(self):
        return list(self.endstops)

    def has_endstops(self):
        return bool(self.endstop_map)

    def share_primary_with_stepper(self, stepper):
        """Shares the primary stepper's endstop with another stepper."""
        if not self.endstops:
            return
        self.endstops[0][0].add_stepper(stepper)

    def get_or_create_endstop(self, *, pin):
        """Create or reuse an endstop for pin.

        This method is intentionally side-effect free with respect to steppers;
        callers are responsible for attaching steppers via endstop.add_stepper().
        """
        ppins = self.printer.lookup_object("pins")
        pin_params = ppins.parse_pin(pin, True, True)
        # Normalize to a stable key so different aliases share one endstop.
        pin_name = "%s:%s" % (pin_params["chip_name"], pin_params["pin"])
        mapping = self.endstop_map.get(pin_name)
        if mapping is None:
            mcu_endstop = ppins.setup_pin("endstop", pin)
            mapping = {
                "endstop": mcu_endstop,
                "invert": pin_params["invert"],
                "pullup": pin_params["pullup"],
                "name": None,
            }
            self.endstop_map[pin_name] = mapping
            self._endstop_to_mapping[mcu_endstop] = mapping
        else:
            changed_invert = pin_params["invert"] != mapping["invert"]
            changed_pullup = pin_params["pullup"] != mapping["pullup"]
            if changed_invert or changed_pullup:
                raise error(
                    "Printer rail %s shared endstop pin %s "
                    "must specify the same pullup/invert settings"
                    % (self.rail_name, pin_name)
                )
            mcu_endstop = mapping["endstop"]
        return mcu_endstop

    def register_named_endstop(self, mcu_endstop, *, name):
        mapping = self._endstop_to_mapping.get(mcu_endstop)
        if mapping is None:
            raise error(
                "Internal endstop registration error on rail %s"
                % (self.rail_name,)
            )
        if mapping["name"] is not None:
            return
        mapping["name"] = name
        if self._query_endstops is None:
            self._query_endstops = self.printer.load_object(
                self._config, "query_endstops"
            )
        self._query_endstops.register_endstop(mcu_endstop, name)
        self.endstops.append((mcu_endstop, name))


class RailEndstopConfig:
    """Normalizes rail endstop config into min/max collections."""

    def __init__(self, printer, config):
        self.printer = printer
        self._rail_name = config.get_name()
        self._style = None
        self._primary_pins = None
        self._primary_by_side = [None, None]
        self._primary_legacy_endstop = None
        self._legacy_collection = None
        self._collections = [
            EndstopCollection(
                printer,
                config,
                rail_name=self._rail_name,
            ),
            EndstopCollection(
                printer,
                config,
                rail_name=self._rail_name,
            ),
        ]

    def get_collections(self):
        """Return rail min/max endstop collections."""
        return self._collections[SIDE_MIN], self._collections[SIDE_MAX]

    def get_position_endstop_hint(self, config, configured_homing_positive_dir):
        """Return endstop-derived position_endstop, or None if unavailable."""
        if self._primary_pins.style == "legacy":
            return self._position_endstop_from_endstop(
                self._primary_legacy_endstop
            )

        min_endstop = self._primary_by_side[SIDE_MIN]
        max_endstop = self._primary_by_side[SIDE_MAX]

        # Explicit direction: only the homing-side primary endstop may
        # override position_endstop.
        if configured_homing_positive_dir is not None:
            side_endstop = self._primary_by_side[
                _side_from_direction(configured_homing_positive_dir)
            ]
            return self._position_endstop_from_endstop(side_endstop)

        # Without explicit direction, accept a single unambiguous position.
        position_candidates = [
            self._position_endstop_from_endstop(min_endstop),
            self._position_endstop_from_endstop(max_endstop),
        ]
        if position_candidates[0] is None:
            return position_candidates[1]
        elif position_candidates[1] is None:
            return position_candidates[0]
        else:
            raise config.error(
                "Ambiguous position_endstop hint on rail '%s' in section '%s': "
                "both endstop_min_pin and endstop_max_pin provide a position "
                "hint; set homing_positive_dir or position_endstop explicitly"
                % (self._rail_name, config.get_name())
            )

    def add_primary_stepper(self, config, stepper):
        """Parse and attach primary stepper endstop pins."""
        pins = _read_endstop_pins(config)
        self._enforce_endstop_style(config, pins.style)
        if pins.style is None:
            raise config.error(
                "No endstop pin configured for section '%s'"
                % (config.get_name(),)
            )

        self._primary_pins = pins
        self._primary_by_side[SIDE_MIN] = None
        self._primary_by_side[SIDE_MAX] = None
        self._primary_legacy_endstop = None
        self._legacy_collection = None

        if pins.style == "minmax":
            for side, pin in enumerate((pins.min_pin, pins.max_pin)):
                if pin is None:
                    continue
                mcu_endstop = self._collection_for_side(
                    side
                ).get_or_create_endstop(pin=pin)
                mcu_endstop.add_stepper(stepper)
                self._primary_by_side[side] = mcu_endstop
        else:  # legacy
            self._legacy_collection = EndstopCollection(
                self.printer,
                config,
                rail_name=self._rail_name,
            )
            mcu_endstop = self._legacy_collection.get_or_create_endstop(
                pin=pins.legacy_pin
            )
            mcu_endstop.add_stepper(stepper)
            self._primary_legacy_endstop = mcu_endstop

    def finalize_primary_endstops(self, short_name, homing_positive_dir):
        """Assign names and complete primary registration into min/max."""
        homing_side = _side_from_direction(homing_positive_dir)
        if self._primary_pins.style == "minmax":
            # Keep exactly one unsuffixed rail name (`x` / `y` / `z`) on the
            # homing side (for backwards compatibility).
            # The non-homing side is suffixed.
            for side, mcu_endstop in enumerate(self._primary_by_side):
                if mcu_endstop is None:
                    continue
                self._collection_for_side(side).register_named_endstop(
                    mcu_endstop,
                    name=choose_endstop_name(
                        short_name,
                        is_homing_side=side == homing_side,
                        side=SIDE_NAME[side],
                    ),
                )
        else:  # legacy
            self._collections[homing_side] = self._legacy_collection
            self._legacy_collection.register_named_endstop(
                self._primary_legacy_endstop,
                name=choose_endstop_name(
                    short_name,
                    is_homing_side=True,
                    side=SIDE_NAME[homing_side],
                ),
            )

    def get_homing_pin_string(self, homing_positive_dir):
        """Return original configured pin string for homing side."""
        if self._primary_pins.style == "minmax":
            return (
                self._primary_pins.max_pin
                if homing_positive_dir
                else self._primary_pins.min_pin
            )
        else:
            return self._primary_pins.legacy_pin

    def add_secondary_stepper(self, config, stepper, homing_positive_dir):
        """Parse and attach secondary stepper endstop pins."""
        pins = _read_endstop_pins(config)
        self._enforce_endstop_style(config, pins.style)
        short_name = stepper.get_name(short=True)
        homing_side = _side_from_direction(homing_positive_dir)

        # Determine which rail sides this section applies to and which pins
        # (if any) to use. Passing pin=None to _attach_stepper_to_side() means
        # "share the primary endstop on that side".
        if self._style == "minmax":
            side_pins = ((SIDE_MIN, pins.min_pin), (SIDE_MAX, pins.max_pin))
        else:  # legacy
            side_pins = ((homing_side, pins.legacy_pin),)

        # Secondary steppers must not introduce a new rail side endstop.
        # Require the primary stepper section to define any side that a
        # secondary section explicitly configures so that later sharing is
        # well-defined.
        if self._style == "minmax":
            for side, pin, pin_key in (
                (SIDE_MIN, pins.min_pin, "endstop_min_pin"),
                (SIDE_MAX, pins.max_pin, "endstop_max_pin"),
            ):
                if pin is not None and self._primary_by_side[side] is None:
                    raise config.error(
                        "Section '%s' configures %s, but the primary section '%s' does not."
                        % (config.get_name(), pin_key, self._rail_name)
                    )
        # else: primary stepper must provide homing endpoint, so this is not
        # an issue with legacy endpoint configuration.

        for side, pin in side_pins:
            self._attach_stepper_to_side(
                stepper, short_name, side, pin, homing_positive_dir
            )

    def _attach_stepper_to_side(
        self, stepper, short_name, side, pin, homing_positive_dir
    ):
        collection = self._collection_for_side(side)
        if pin is None:
            collection.share_primary_with_stepper(stepper)
            return
        mcu_endstop = collection.get_or_create_endstop(pin=pin)

        mcu_endstop.add_stepper(stepper)

        homing_side = _side_from_direction(homing_positive_dir)
        collection.register_named_endstop(
            mcu_endstop,
            name=choose_endstop_name(
                short_name,
                is_homing_side=side == homing_side,
                side=SIDE_NAME[side],
            ),
        )

    def _enforce_endstop_style(self, config, section_style):
        if section_style is None:
            return
        elif self._style is None:
            self._style = section_style
            return
        elif section_style == self._style:
            return
        else:
            raise config.error(
                "Mixed endstop pin styles on rail '%s' in section '%s': use either "
                "endstop_pin OR endstop_{min,max}_pin consistently"
                % (self._rail_name, config.get_name())
            )

    def _collection_for_side(self, side):
        if side not in (SIDE_MIN, SIDE_MAX):
            raise error(
                "Internal endstop side '%s' on rail %s"
                % (side, self._rail_name)
            )
        return self._collections[side]

    def _position_endstop_from_endstop(self, mcu_endstop):
        if mcu_endstop is None or not hasattr(
            mcu_endstop, "get_position_endstop"
        ):
            return None
        return mcu_endstop.get_position_endstop()


######################################################################
# Stepper controlled rails
######################################################################


# A motor control "rail" with one (or more) steppers and one (or more)
# endstops.
class PrinterRail:
    """Coordinates rail motion state and exposes normalized endstops."""

    def __init__(
        self,
        config,
        need_position_minmax=True,
        default_position_endstop=None,
        units_in_radians=False,
    ):
        self.stepper_units_in_radians = units_in_radians
        self.steppers = []
        self.printer = config.get_printer()

        # Collect endstops per rail side (min/max) while building steppers.
        # Naming and QUERY_ENDSTOPS registration for the primary endstop(s) is
        # finalized after homing direction is known.
        self._endstop_config = RailEndstopConfig(self.printer, config)
        self.endstops_min, self.endstops_max = (
            self._endstop_config.get_collections()
        )
        configured_homing_positive_dir = config.getboolean(
            "homing_positive_dir", None
        )

        stepper = self._add_primary_stepper(config)
        self.position_endstop = self._determine_position_endstop(
            config,
            default_position_endstop,
            configured_homing_positive_dir,
        )

        self._tmc_current_helpers = None
        self.get_name = stepper.get_name
        self.get_commanded_position = stepper.get_commanded_position
        self.calc_position_from_coord = stepper.calc_position_from_coord

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

        self.homing_positive_dir = configured_homing_positive_dir
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

        self._endstop_config.finalize_primary_endstops(
            stepper.get_name(short=True),
            self.homing_positive_dir,
        )
        self.endstops_min, self.endstops_max = (
            self._endstop_config.get_collections()
        )
        self._enforce_homing_side_endstop(config)

        endstop_pin_string = self._endstop_config.get_homing_pin_string(
            self.homing_positive_dir
        )

        endstop_is_virtual = (
            endstop_pin_string is not None
            and ":virtual_endstop" in endstop_pin_string
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
        # homing_positive_dir already determined above

        self.min_home_dist = config.getfloat(
            "min_home_dist", self.homing_retract_dist, minval=0.0
        )

        self.homing_accel = config.getfloat("homing_accel", None, above=0.0)

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
        return self.get_endstops_for_direction(self.homing_positive_dir)

    def get_endstops_for_direction(self, is_positive_dir):
        """Return endstops that should guard motion in the given direction."""
        side = _side_from_direction(is_positive_dir)
        return (self.endstops_min, self.endstops_max)[side].get_endstops()

    def _determine_position_endstop(
        self,
        config,
        default_position_endstop,
        configured_homing_positive_dir,
    ):
        """Determine numeric position_endstop.

        Prefer a hint from the configured endstop object (e.g. TMC virtual
        endstops), and fall back to the config/default value when no hint is
        available.
        """
        position_endstop_hint = self._endstop_config.get_position_endstop_hint(
            config,
            configured_homing_positive_dir,
        )
        if position_endstop_hint is not None:
            return position_endstop_hint
        elif default_position_endstop is None:
            return config.getfloat("position_endstop")
        else:
            return config.getfloat("position_endstop", default_position_endstop)

    def _enforce_homing_side_endstop(self, config):
        """Require at least one endstop on the homing side."""
        homing_side = _side_from_direction(self.homing_positive_dir)
        homing_collection = (self.endstops_min, self.endstops_max)[homing_side]
        if not homing_collection.has_endstops():
            raise config.error(
                "No endstop pin configured on homing (%s) side for rail '%s' in section '%s'"
                % (SIDE_NAME[homing_side], self.get_name(), config.get_name())
            )

    def _add_primary_stepper(self, config):
        stepper = PrinterStepper(config, self.stepper_units_in_radians)
        self.steppers.append(stepper)
        self._endstop_config.add_primary_stepper(config, stepper)
        return stepper

    def _add_secondary_stepper(self, config):
        stepper = PrinterStepper(config, self.stepper_units_in_radians)
        self.steppers.append(stepper)
        self._endstop_config.add_secondary_stepper(
            config,
            stepper,
            self.homing_positive_dir,
        )

    def add_foreign_stepper(self, stepper):
        """
        Connect a stepper of another rail to the endstops of this rail.
        Used e.g. to setup CoreXY kinematics.
        """
        for dir in (True, False):
            endstops = self.get_endstops_for_direction(dir)
            if not endstops:
                continue
            endstops[0][0].add_stepper(stepper)

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
        rail._add_secondary_stepper(
            config.getsection(config.get_name() + str(i))
        )
    return rail
