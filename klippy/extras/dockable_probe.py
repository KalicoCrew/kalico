# Dockable Probe
#   This provides support for probes that are magnetically coupled
#   to the toolhead and stowed in a dock when not in use and
#
# Copyright (C) 2018-2023  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2021       Paul McGowan <mental405@gmail.com>
# Copyright (C) 2023       Alan Smith <alan@airpost.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from math import acos, atan2, cos, fabs, floor, hypot, pi, sin, sqrt

from . import probe

PROBE_VERIFY_DELAY = 0.1

PROBE_UNKNOWN = 0
PROBE_ATTACHED = 1
PROBE_DOCKED = 2

MULTI_OFF = 0
MULTI_FIRST = 1
MULTI_ON = 2

HINT_VERIFICATION_ERROR = """
{0}: A probe attachment verification method
was not provided. A method to verify the probes attachment
state must be specified to prevent unintended behavior.

At least one of the following must be specified:
'check_open_attach', 'probe_sense_pin', 'dock_sense_pin'

Please see {0}.md and config_Reference.md.
"""

VIRTUAL_Z_ENDSTOP_ERROR = """
DockableProbe cannot be used as Z endstop if a z position
is defined in approach/dock/extract."
"""


# Helper class to handle polling pins for probe attachment states
class PinPollingHelper:
    def __init__(self, config, endstop):
        self.printer = config.get_printer()
        self.query_endstop = endstop
        self.last_verify_time = 0
        self.last_verify_state = None

    def query_pin(self, curtime):
        if (
            curtime > (self.last_verify_time + PROBE_VERIFY_DELAY)
            or self.last_verify_state is None
        ):
            self.last_verify_time = curtime
            toolhead = self.printer.lookup_object("toolhead")
            query_time = toolhead.get_last_move_time()
            self.last_verify_state = not not self.query_endstop(query_time)
        return self.last_verify_state

    def query_pin_inv(self, curtime):
        return not self.query_pin(curtime)


# Helper class to verify probe attachment status
class ProbeState:
    def __init__(self, config, aProbe):
        self.printer = config.get_printer()

        if (
            not config.fileconfig.has_option(
                config.section, "check_open_attach"
            )
            and not config.fileconfig.has_option(
                config.section, "probe_sense_pin"
            )
            and not config.fileconfig.has_option(
                config.section, "dock_sense_pin"
            )
        ):
            raise self.printer.config_error(
                HINT_VERIFICATION_ERROR.format(aProbe.name)
            )

        self.printer.register_event_handler("klippy:ready", self._handle_ready)

        # Configure sense pins as endstops so they
        # can be polled at specific times
        ppins = self.printer.lookup_object("pins")

        def configEndstop(pin):
            pin_params = ppins.lookup_pin(pin, can_invert=True, can_pullup=True)
            mcu = pin_params["chip"]
            mcu_endstop = mcu.setup_pin("endstop", pin_params)
            helper = PinPollingHelper(config, mcu_endstop.query_endstop)
            return helper

        probe_sense_helper = None
        dock_sense_helper = None

        # Setup sensor pins, if configured, otherwise use probe endstop
        # as a dummy sensor.
        ehelper = PinPollingHelper(config, aProbe.query_endstop)

        # Probe sense pin is optional
        probe_sense_pin = config.get("probe_sense_pin", None)
        if probe_sense_pin is not None:
            probe_sense_helper = configEndstop(probe_sense_pin)
            self.probe_sense_pin = probe_sense_helper.query_pin
        else:
            self.probe_sense_pin = ehelper.query_pin_inv

        # If check_open_attach is specified, it takes precedence
        # over probe_sense_pin
        check_open_attach = None
        if config.fileconfig.has_option(config.section, "check_open_attach"):
            check_open_attach = config.getboolean("check_open_attach")

            if check_open_attach:
                self.probe_sense_pin = ehelper.query_pin_inv
            else:
                self.probe_sense_pin = ehelper.query_pin

        # Dock sense pin is optional
        self.dock_sense_pin = None
        dock_sense_pin = config.get("dock_sense_pin", None)
        if dock_sense_pin is not None:
            dock_sense_helper = configEndstop(dock_sense_pin)
            self.dock_sense_pin = dock_sense_helper.query_pin

    def _handle_ready(self):
        self.last_verify_time = 0
        self.last_verify_state = PROBE_UNKNOWN

    def get_probe_state(self):
        curtime = self.printer.get_reactor().monotonic()
        return self.get_probe_state_with_time(curtime)

    def get_probe_state_with_time(self, curtime):
        if (
            self.last_verify_state == PROBE_UNKNOWN
            or curtime > self.last_verify_time + PROBE_VERIFY_DELAY
        ):
            self.last_verify_time = curtime
            self.last_verify_state = PROBE_UNKNOWN

            a = self.probe_sense_pin(curtime)

            if self.dock_sense_pin is not None:
                d = self.dock_sense_pin(curtime)

                if a and not d:
                    self.last_verify_state = PROBE_ATTACHED
                elif d and not a:
                    self.last_verify_state = PROBE_DOCKED
            else:
                if a:
                    self.last_verify_state = PROBE_ATTACHED
                elif not a:
                    self.last_verify_state = PROBE_DOCKED
        return self.last_verify_state


class DockableProbe:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.name = config.get_name()

        # Configuration Options
        self.position_endstop = config.getfloat("z_offset")
        self.x_offset = config.getfloat("x_offset", 0.0)
        self.y_offset = config.getfloat("y_offset", 0.0)
        self.speed = config.getfloat("speed", 5.0, above=0.0)
        self.lift_speed = config.getfloat("lift_speed", self.speed, above=0.0)
        self.dock_retries = config.getint("dock_retries", 0)
        self.auto_attach_detach = config.getboolean("auto_attach_detach", True)
        self.restore_toolhead = config.getboolean("restore_toolhead", True)
        self.travel_speed = config.getfloat(
            "travel_speed", self.speed, above=0.0
        )
        self.attach_speed = config.getfloat(
            "attach_speed", self.travel_speed, above=0.0
        )
        self.detach_speed = config.getfloat(
            "detach_speed", self.travel_speed, above=0.0
        )
        self.sample_retract_dist = config.getfloat(
            "sample_retract_dist", 2.0, above=0.0
        )
        self.detach_dockable_before_z_home = config.getboolean(
            "detach_dockable_before_z_home", False
        )

        # Positions (approach, detach, etc)
        self.approach_position = self._parse_coord(config, "approach_position")
        self.detach_position = self._parse_coord(config, "detach_position")
        self.extract_position = self._parse_coord(
            config, "extract_position", self.approach_position
        )
        self.insert_position = self._parse_coord(
            config, "insert_position", self.extract_position
        )
        self.dock_position = self._parse_coord(config, "dock_position")
        self.z_hop = config.getfloat("z_hop", 0.0, above=0.0)

        self.dock_requires_z = (
            self.approach_position[2] is not None
            or self.dock_position[2] is not None
        )
        # Entry distance
        self.approach_distance = self._get_distance(
            self.dock_position, self.approach_position
        )
        self.detach_distance = self._get_distance(
            self.dock_position, self.detach_position
        )
        self.insert_distance = self._get_distance(
            self.dock_position, self.insert_position
        )

        # Helpers to avoid dock and moves out of range ()
        max_safe_distance = min(
            self.approach_distance,
            self.detach_distance,
            self.insert_distance,
        )
        self.safe_dock_distance = config.getfloat(
            "safe_dock_distance", max_safe_distance, maxval=max_safe_distance
        )
        self.safe_position = self._parse_coord(
            config, "safe_position", self.approach_position
        )
        self.safe_points = self._get_safe_points(self.safe_position)

        # Macros to run before attach and after detach
        gcode_macro = self.printer.load_object(config, "gcode_macro")
        self.activate_gcode = gcode_macro.load_template(
            config, "activate_gcode", ""
        )
        self.deactivate_gcode = gcode_macro.load_template(
            config, "deactivate_gcode", ""
        )

        self.pre_attach_gcode = gcode_macro.load_template(
            config, "pre_attach_gcode", ""
        )
        self.post_attach_gcode = gcode_macro.load_template(
            config, "post_attach_gcode", ""
        )

        self.pre_detach_gcode = gcode_macro.load_template(
            config, "pre_detach_gcode", ""
        )
        self.post_detach_gcode = gcode_macro.load_template(
            config, "post_detach_gcode", ""
        )

        # Pins
        ppins = self.printer.lookup_object("pins")
        pin = config.get("pin")
        pin_params = ppins.lookup_pin(pin, can_invert=True, can_pullup=True)
        mcu = pin_params["chip"]
        self.mcu_endstop = mcu.setup_pin("endstop", pin_params)

        # Wrappers
        self.get_mcu = self.mcu_endstop.get_mcu
        self.add_stepper = self.mcu_endstop.add_stepper
        self.get_steppers = self.mcu_endstop.get_steppers
        self.home_wait = self.mcu_endstop.home_wait
        self.query_endstop = self.mcu_endstop.query_endstop
        self.finish_home_complete = self.wait_trigger_complete = None

        # State
        self.multi = MULTI_OFF
        self._last_homed = None

        pstate = ProbeState(config, self)
        self.get_probe_state = pstate.get_probe_state
        self.last_probe_state = PROBE_UNKNOWN

        self.probe_states = {
            PROBE_ATTACHED: "ATTACHED",
            PROBE_DOCKED: "DOCKED",
            PROBE_UNKNOWN: "UNKNOWN",
        }

        # Gcode Commands
        self.gcode.register_command(
            "QUERY_DOCKABLE_PROBE",
            self.cmd_QUERY_DOCKABLE_PROBE,
            desc=self.cmd_QUERY_DOCKABLE_PROBE_help,
        )

        self.gcode.register_command(
            "MOVE_TO_APPROACH_PROBE",
            self.cmd_MOVE_TO_APPROACH_PROBE,
            desc=self.cmd_MOVE_TO_APPROACH_PROBE_help,
        )
        self.gcode.register_command(
            "MOVE_TO_DOCK_PROBE",
            self.cmd_MOVE_TO_DOCK_PROBE,
            desc=self.cmd_MOVE_TO_DOCK_PROBE_help,
        )
        self.gcode.register_command(
            "MOVE_TO_EXTRACT_PROBE",
            self.cmd_MOVE_TO_EXTRACT_PROBE,
            desc=self.cmd_MOVE_TO_EXTRACT_PROBE_help,
        )
        self.gcode.register_command(
            "MOVE_TO_INSERT_PROBE",
            self.cmd_MOVE_TO_INSERT_PROBE,
            desc=self.cmd_MOVE_TO_INSERT_PROBE_help,
        )
        self.gcode.register_command(
            "MOVE_TO_DETACH_PROBE",
            self.cmd_MOVE_TO_DETACH_PROBE,
            desc=self.cmd_MOVE_TO_DETACH_PROBE_help,
        )

        self.gcode.register_command(
            "SET_DOCKABLE_PROBE",
            self.cmd_SET_DOCKABLE_PROBE,
            desc=self.cmd_SET_DOCKABLE_PROBE_help,
        )
        self.gcode.register_command(
            "ATTACH_PROBE",
            self.cmd_ATTACH_PROBE,
            desc=self.cmd_ATTACH_PROBE_help,
        )
        self.gcode.register_command(
            "DETACH_PROBE",
            self.cmd_DETACH_PROBE,
            desc=self.cmd_DETACH_PROBE_help,
        )
        self.gcode.register_command(
            "MOVE_AVOIDING_DOCK",
            self.cmd_MOVE_AVOIDING_DOCK,
            desc=self.cmd_MOVE_AVOIDING_DOCK_help,
        )

        # Event Handlers
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )

        self.printer.register_event_handler(
            "klippy:mcu_identify", self._handle_config
        )

    # Parse a string coordinate representation from the config
    # and return a list of numbers.
    #
    # e.g. "233, 10, 0" -> [233, 10, 0]
    def _parse_coord(self, config, name, default=None, expected_dims=3):
        if default:
            val = config.get(name, None)
        else:
            val = config.get(name)
        error_msg = "Unable to parse {0} in {1}: {2}"
        if not val:
            return default
        try:
            vals = [float(x.strip()) for x in val.split(",")]
        except Exception as e:
            raise config.error(error_msg.format(name, self.name, str(e)))
        supplied_dims = len(vals)
        if not 2 <= supplied_dims <= expected_dims:
            raise config.error(
                error_msg.format(
                    name, self.name, "Invalid number of coordinates"
                )
            )
        p = [None] * 3
        p[:supplied_dims] = vals
        return p

    def _handle_config(self):
        kin = self.printer.lookup_object("toolhead").get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis("z"):
                self.add_stepper(stepper)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")
        self.safe_move = self.printer.lookup_object("safe_move")
        rails = self.toolhead.get_kinematics().rails
        endstops = [es for rail in rails for es, name in rail.get_endstops()]
        positions = [
            self.approach_position,
            self.dock_position,
            self.extract_position,
        ]
        if self in endstops and any(pos[2] is not None for pos in positions):
            raise self.printer.config_error(VIRTUAL_Z_ENDSTOP_ERROR)

    #######################################################################
    # GCode Commands
    #######################################################################

    cmd_QUERY_DOCKABLE_PROBE_help = (
        "Prints the current probe state,"
        + " valid probe states are UNKNOWN, ATTACHED, and DOCKED"
    )

    def cmd_QUERY_DOCKABLE_PROBE(self, gcmd):
        self.last_probe_state = self.get_probe_state()
        state = self.probe_states[self.last_probe_state]

        gcmd.respond_info("Probe Status: %s" % (state))

    def get_status(self, curtime):
        # Use last_'status' here to be consistent with QUERY_PROBE_'STATUS'.
        return {
            "last_status": self.last_probe_state,
            "auto_attach_detach": self.auto_attach_detach,
        }

    cmd_MOVE_TO_APPROACH_PROBE_help = (
        "Move close to the probe dock before attaching"
    )

    def cmd_MOVE_TO_APPROACH_PROBE(self, gcmd):
        self._align_z()

        self._move_avoiding_dock(self.approach_position, self.travel_speed)

        if len(self.approach_position) > 2:
            self.toolhead.manual_move(
                [None, None, self.approach_position[2]], self.speed
            )

    cmd_MOVE_TO_DOCK_PROBE_help = (
        "Move to connect the toolhead/dock to the probe"
    )

    def cmd_MOVE_TO_DOCK_PROBE(self, gcmd):
        if len(self.dock_position) > 2:
            self.toolhead.manual_move(
                [None, None, self.dock_position[2]], self.speed
            )

        self.toolhead.manual_move(
            [self.dock_position[0], self.dock_position[1], None],
            self.attach_speed,
        )

    cmd_MOVE_TO_EXTRACT_PROBE_help = (
        "Move away from the dock with the probe attached"
    )

    def cmd_MOVE_TO_EXTRACT_PROBE(self, gcmd):
        if len(self.extract_position) > 2:
            self.toolhead.manual_move(
                [None, None, self.extract_position[2]], self.speed
            )

        self.toolhead.manual_move(
            [self.extract_position[0], self.extract_position[1], None],
            self.attach_speed,
        )

    cmd_MOVE_TO_INSERT_PROBE_help = (
        "Move near the dock with the probe attached before detaching"
    )

    def cmd_MOVE_TO_INSERT_PROBE(self, gcmd):
        self._move_avoiding_dock(self.insert_position, self.travel_speed)

        if len(self.insert_position) > 2:
            self.toolhead.manual_move(
                [None, None, self.insert_position[2]], self.speed
            )

    cmd_MOVE_TO_DETACH_PROBE_help = (
        "Move away from the dock to detach the probe"
    )

    def cmd_MOVE_TO_DETACH_PROBE(self, gcmd):
        if len(self.detach_position) > 2:
            self.toolhead.manual_move(
                [None, None, self.detach_position[2]], self.lift_speed
            )

        self.toolhead.manual_move(
            [self.detach_position[0], self.detach_position[1], None],
            self.detach_speed,
        )

    cmd_SET_DOCKABLE_PROBE_help = "Set probe parameters"

    def cmd_SET_DOCKABLE_PROBE(self, gcmd):
        auto = gcmd.get("AUTO_ATTACH_DETACH", None)
        if auto is None:
            return

        if int(auto) == 1:
            self.auto_attach_detach = True
        else:
            self.auto_attach_detach = False

    cmd_ATTACH_PROBE_help = (
        "Check probe status and attach probe using the movement gcodes"
    )

    def cmd_ATTACH_PROBE(self, gcmd):
        return_pos = self.toolhead.get_position()
        self.attach_probe(return_pos)

    cmd_DETACH_PROBE_help = (
        "Check probe status and detach probe using the movement gcodes"
    )

    def cmd_DETACH_PROBE(self, gcmd):
        return_pos = self.toolhead.get_position()
        self.detach_probe(return_pos)

    cmd_MOVE_AVOIDING_DOCK_help = "Move to X Y avoiding dock safe area"

    def cmd_MOVE_AVOIDING_DOCK(self, gcmd):
        pos = self.toolhead.get_position()
        x = gcmd.get("X", pos[0], parser=float)
        y = gcmd.get("Y", pos[1], parser=float)
        speed = gcmd.get("SPEED", self.travel_speed, parser=float, above=0.0)
        self._move_avoiding_dock([x, y], speed)

    def attach_probe(self, return_pos=None, always_restore_toolhead=False):
        self._lower_probe()

        retry = 0
        while (
            self.get_probe_state() != PROBE_ATTACHED
            and retry < self.dock_retries + 1
        ):
            if self.get_probe_state() != PROBE_DOCKED:
                raise self.printer.command_error(
                    "Attach Probe: Probe not detected in dock, aborting"
                )
            self.pre_attach_gcode.run_gcode_from_command()
            # Call these gcodes as a script because we don't have enough
            # structs/data to call the cmd_...() funcs and supply 'gcmd'.
            # This method also has the advantage of calling user-written gcodes
            # if they've been defined.
            self.gcode.run_script_from_command(
                """
                MOVE_TO_APPROACH_PROBE
                MOVE_TO_DOCK_PROBE
                MOVE_TO_EXTRACT_PROBE
            """
            )
            self.post_attach_gcode.run_gcode_from_command()

            retry += 1

        if self.get_probe_state() != PROBE_ATTACHED:
            raise self.printer.command_error("Probe attach failed!")

        if return_pos and (self.restore_toolhead or always_restore_toolhead):
            # return to the original XY position, if inside safe_dock area
            # move to the closest point
            self._move_avoiding_dock(return_pos[:2], self.travel_speed)
            # Do NOT return to the original Z position after attach
            # as the probe might crash into the bed.

    def detach_probe(self, return_pos=None):
        retry = 0
        while (
            self.get_probe_state() != PROBE_DOCKED
            and retry < self.dock_retries + 1
        ):
            self.pre_detach_gcode.run_gcode_from_command()
            # Call these gcodes as a script because we don't have enough
            # structs/data to call the cmd_...() funcs and supply 'gcmd'.
            # This method also has the advantage of calling user-written gcodes
            # if they've been defined.
            self.gcode.run_script_from_command(
                """
                MOVE_TO_INSERT_PROBE
                MOVE_TO_DOCK_PROBE
                MOVE_TO_DETACH_PROBE
            """
            )
            self.post_detach_gcode.run_gcode_from_command()

            retry += 1

        if self.get_probe_state() != PROBE_DOCKED:
            raise self.printer.command_error("Probe detach failed!")

        if return_pos and self.restore_toolhead:
            # return to the original XY position, if inside safe_dock area
            # move to the closest point
            self._move_avoiding_dock(return_pos[:2], self.travel_speed)
            # Return to original Z position after detach as
            # there's no chance of the probe crashing into the bed.
            self.toolhead.manual_move(
                [None, None, return_pos[2]], self.lift_speed
            )
        self._raise_probe()

    def auto_detach_probe(self, return_pos=None):
        if self.get_probe_state() == PROBE_DOCKED:
            return
        if self.auto_attach_detach:
            self.detach_probe(return_pos)

    def auto_attach_probe(self, return_pos=None, always_restore_toolhead=False):
        if self.get_probe_state() == PROBE_ATTACHED:
            return
        if not self.auto_attach_detach:
            raise self.printer.command_error(
                "Cannot probe, probe is not "
                "attached and auto-attach is disabled"
            )
        self.attach_probe(return_pos, always_restore_toolhead)

    #######################################################################
    # Functions for calculating points and moving the toolhead
    #######################################################################

    # Move to position avoiding the dock
    def _move_avoiding_dock(self, end_point, speed):
        start_point = self.toolhead.get_position()[:2]
        end_point = end_point[:2]
        dock = self.dock_position[:2]
        if not start_point or start_point == end_point:
            return
        radius = self.safe_dock_distance
        if radius == 0:
            self.toolhead.manual_move([end_point[0], end_point[1], None], speed)
            return

        # redefine start_point outside safe dock area
        coords = []
        if radius > self._get_distance(dock, start_point):
            start_point = self._get_closest_exitpoint(start_point, end_point)
            coords.append(start_point)

        # Check if trajectory intersect safe dock area
        intersect_points = self._get_intersect_points(start_point, end_point)

        # Define endpoint
        if len(intersect_points) == 1:
            end_point = intersect_points[0]
        # Calculate trajectory around the dock
        elif intersect_points:
            # input_tangent point
            safe_point = self._get_closest_point(end_point, self.safe_points)
            tangent_points = self._get_tangent_points(
                radius - 0.1, dock, start_point
            )
            arc_input = self._get_closest_point(safe_point, tangent_points)
            clockwise = self._get_rotation_direction(start_point, arc_input)
            coords.append(arc_input)

            # output tangent point
            safe_point = self._get_closest_point(start_point, self.safe_points)
            tangent_points = self._get_tangent_points(radius, dock, end_point)
            arc_output = self._get_closest_point(safe_point, tangent_points)
            # determine arc travel
            arc_points = self._arc_points(
                arc_input, arc_output, dock, clockwise
            )
            coords.extend(arc_points)
        coords.append(end_point)

        for x, y in coords:
            self.toolhead.manual_move([x, y, None], speed)

    # Find a point on a vector line at a specific distance
    def _get_point_on_vector(self, point, angle, magnitude=1):
        x = point[0] - magnitude * cos(angle)
        y = point[1] - magnitude * sin(angle)
        return (x, y)

    # Determine the vector of two points
    def _get_distance(self, point1, point2):
        x1, y1 = point1[:2]
        x2, y2 = point2[:2]
        return hypot(x2 - x1, y2 - y1)

    def _get_angle(self, point1, point2):
        x1, y1 = point1[:2]
        x2, y2 = point2[:2]
        return atan2(y2 - y1, x2 - x1) + pi

    # Determine tangent points to a circle from external point
    def _get_tangent_points(self, radius, center, point2):
        cx, cy = center[:2]
        d = self._get_distance(center, point2)
        angle = self._get_angle(center, point2)
        angle = angle - pi
        if d < radius:
            return None
        dev = acos(radius / d)
        x_tangent1 = cx + radius * cos(angle + dev)
        y_tangent1 = cy + radius * sin(angle + dev)
        x_tangent2 = cx + radius * cos(angle - dev)
        y_tangent2 = cy + radius * sin(angle - dev)
        return [(x_tangent1, y_tangent1), (x_tangent2, y_tangent2)]

    # determine closest exit point X or Y while toolhead inside safe_dock_zone
    def _get_closest_exitpoint(self, point1, point2):
        cx, cy = self.dock_position[:2]
        # Choose point2 if point1 is the dock position
        if point1[:2] != [cx, cy]:
            dx, dy = point1[0] - cx, point1[1] - cy
            reference_point = point1[:2]
        elif point2[:2] != [cx, cy]:
            dx, dy = point2[0] - cx, point2[1] - cy
            reference_point = point2[:2]
        else:
            raise self.printer.command_error(
                "_move_avoiding_dock : Unable to determine exit point"
            )
        d = hypot(dx, dy)
        # Ensure exit point is outside dock area.
        magnitude = self.safe_dock_distance + 10e-8
        x1 = cx + magnitude * dx / d
        y1 = cy + magnitude * dy / d
        x2 = cx - magnitude * dx / d
        y2 = cy - magnitude * dy / d

        return self._get_closest_point(reference_point, [(x1, y1), (x2, y2)])

    # determine intersect points between a line and a circle
    def _get_intersect_points(self, point1, point2):
        x1, y1 = point1[:2]
        x2, y2 = point2[:2]
        cx, cy = self.dock_position[:2]
        r = self.safe_dock_distance - 10e-5  # fix floating point issue

        dx, dy = x2 - x1, y2 - y1
        a = dx**2 + dy**2
        b = 2 * (dx * (x1 - cx) + dy * (y1 - cy))
        c = (x1 - cx) ** 2 + (y1 - cy) ** 2 - r**2
        disc = b**2 - 4 * a * c

        if disc < 0 or a == 0:
            return []  # Nothing to return

        t1 = (-b + sqrt(disc)) / (2 * a)
        t2 = (-b - sqrt(disc)) / (2 * a)

        intersec = []
        if 0 <= t1 <= 1:
            intersec.append((x1 + t1 * dx, y1 + t1 * dy))
        if 0 <= t2 <= 1 and t1 != t2:
            intersec.append((x1 + t2 * dx, y1 + t2 * dy))
        return intersec

    # Determine closest point
    def _get_closest_point(self, target_point, points):
        if not points:
            return None
        x_target, y_target = target_point[:2]
        distances = [hypot(x - x_target, y - y_target) for x, y in points]
        # find index of the closest point
        index = distances.index(min(distances))
        return points[index]

    # Determine arc points , a simplified version of gcode_arcs
    def _arc_points(self, point1, point2, center, clockwise):
        x1, y1 = point1[:2]
        x2, y2 = point2[:2]
        cx, cy = center[:2]
        # Radius vector from center to current location
        r_P = x1 - cx
        r_Q = y1 - cy
        # Determine angular travel
        rt_Alpha = x2 - cx
        rt_Beta = y2 - cy
        angular_travel = atan2(
            r_P * rt_Beta - r_Q * rt_Alpha, r_P * rt_Alpha + r_Q * rt_Beta
        )

        if angular_travel < 0.0:
            angular_travel += 2.0 * pi
        if clockwise:
            angular_travel -= 2.0 * pi
        # Determine number of segments
        radius = hypot(r_P, r_Q)
        flat_mm = radius * angular_travel

        mm_of_travel = fabs(flat_mm)
        segments = max(1.0, floor(mm_of_travel))

        # Generate coordinates
        theta_per_segment = angular_travel / segments
        coords = []

        for i in range(1, int(segments)):
            cos_Ti = cos(i * theta_per_segment)
            sin_Ti = sin(i * theta_per_segment)
            r_P = (x1 - cx) * cos_Ti - (y1 - cy) * sin_Ti
            r_Q = (x1 - cx) * sin_Ti + (y1 - cy) * cos_Ti
            coords.append((cx + r_P, cy + r_Q))

        coords.append(point2[:2])
        return coords

    def _get_rotation_direction(self, point1, point2):
        angle1 = self._get_angle(point1, self.dock_position)
        angle2 = self._get_angle(point2, self.dock_position)
        # Direction of rotation
        angle_diff = (angle2 - angle1) % (2 * pi)
        return angle_diff > pi

    def _get_safe_points(self, p1):
        angle = self._get_angle(self.dock_position, p1)
        safe_point1 = self._get_point_on_vector(
            self.dock_position, angle + pi / 4, self.safe_dock_distance
        )
        safe_point2 = self._get_point_on_vector(
            self.dock_position, angle - pi / 4, self.safe_dock_distance
        )
        return [safe_point1, safe_point2]

    # Align z axis to prevent crashes
    def _align_z(self):
        curtime = self.printer.get_reactor().monotonic()
        homed_axes = self.toolhead.get_status(curtime)["homed_axes"]
        self._last_homed = homed_axes

        if self.dock_requires_z:
            self._align_z_required()

        if self.z_hop > 0.0:
            if "z" in self._last_homed:
                tpos = self.toolhead.get_position()
                if tpos[2] < self.z_hop:
                    self.toolhead.manual_move(
                        [None, None, self.z_hop], self.lift_speed
                    )
            else:
                self._force_z_hop()

    def _align_z_required(self):
        if "z" not in self._last_homed:
            raise self.printer.command_error(
                "Cannot attach/detach probe, must home Z axis first"
            )

        self.toolhead.manual_move(
            [None, None, self.approach_position[2]], self.lift_speed
        )

    # Hop z and return to un-homed state
    def _force_z_hop(self):
        self.safe_move.move(
            self.toolhead,
            "z",
            self.z_hop,
            self.lift_speed,
            allow_unsafe=True,
        )

    #######################################################################
    # Probe Wrappers
    #######################################################################
    def _raise_probe(self):
        toolhead = self.printer.lookup_object("toolhead")
        start_pos = toolhead.get_position()
        self.deactivate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self.printer.command_error(
                "Toolhead moved during probe deactivate_gcode script"
            )

    def _lower_probe(self):
        toolhead = self.printer.lookup_object("toolhead")
        start_pos = toolhead.get_position()
        self.activate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self.printer.command_error(
                "Toolhead moved during probe activate_gcode script"
            )

    def multi_probe_begin(self, always_restore_toolhead=False):
        self.multi = MULTI_FIRST

        # Attach probe before moving to the first probe point and
        # return to current position. Move because this can be called
        # before a multi _point_ probe and a multi probe at the same
        # point but for the latter the toolhead is already in position.
        # If the toolhead is not returned to the current position it
        # will complete the probing next to the dock. This behavior
        # is driven by probe.py which defines always_restore_toolhead
        return_pos = self.toolhead.get_position()
        self.auto_attach_probe(return_pos, always_restore_toolhead)

    def multi_probe_end(self):
        self.multi = MULTI_OFF

        return_pos = self.toolhead.get_position()
        # Move away from the bed to ensure the probe isn't triggered,
        # preventing detaching in the event there's no probe/dock sensor.
        self.toolhead.manual_move(
            [None, None, return_pos[2] + 2], self.lift_speed
        )
        self.auto_detach_probe(return_pos)

    def probe_prepare(self, hmove):
        if self.multi == MULTI_OFF or self.multi == MULTI_FIRST:
            return_pos = self.toolhead.get_position()
            self.auto_attach_probe(return_pos)
        if self.multi == MULTI_FIRST:
            self.multi = MULTI_ON

    def probe_finish(self, hmove):
        self.wait_trigger_complete.wait()
        if self.multi == MULTI_OFF:
            return_pos = self.toolhead.get_position()
            # Move away from the bed to ensure the probe isn't triggered,
            # preventing detaching in the event there's no probe/dock sensor.
            self.toolhead.manual_move(
                [None, None, return_pos[2] + 2], self.lift_speed
            )
            self.auto_detach_probe(return_pos)

    def home_start(
        self, print_time, sample_time, sample_count, rest_time, triggered=True
    ):
        self.finish_home_complete = self.mcu_endstop.home_start(
            print_time, sample_time, sample_count, rest_time, triggered
        )
        r = self.printer.get_reactor()
        self.wait_trigger_complete = r.register_callback(self.wait_for_trigger)
        return self.finish_home_complete

    def wait_for_trigger(self, eventtime):
        self.finish_home_complete.wait()

    def get_position_endstop(self):
        return self.position_endstop

    def probing_move(self, pos, speed, gcmd):
        phoming = self.printer.lookup_object("homing")
        return phoming.probing_move(self, pos, speed)


def load_config(config):
    msp = DockableProbe(config)
    config.get_printer().add_object("probe", probe.PrinterProbe(config, msp))
    return msp
