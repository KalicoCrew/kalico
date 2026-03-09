# Support for Marlin/Smoothie/Reprap style firmware retraction via G10/G11
# Zhop funtionality includes:
#   - Standard zhop (vertical move up, travel, vertical move down)
#
# Copyright (C) 2023  Florian-Patrice Nagel <flopana77@gmail.com>
# Copyright (C) 2019  Len Trigg <lenbok@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# Constants
ZHOP_MOVE_SPEED_FRACTION = 0.8
MAX_Z_MARGIN = 0.1


class FirmwareRetraction:
    # Class init
    def __init__(self, config):
        self.config_ref = config
        self.printer = config.get_printer()

        # Get retraction params from config, used also in RESET_RETRACTION
        # and initiate params
        self._get_config_params()

        # Initialize variables
        self.is_retracted = False  # Retract state flag
        self.do_zhop = False  # zhop state flag
        self.current_z_hop_height = 0.0
        self.last_position = [0.0, 0.0, 0.0, 0.0]
        self.next_transform = None

        # Get maximum printer z move velocity for zhop moves
        printer_config = config.getsection("printer")
        max_velocity = printer_config.getfloat("max_velocity")
        self.max_z_vel = printer_config.getfloat("max_z_velocity", max_velocity)

        self.printer.register_event_handler("klippy:ready", self._handle_ready)

        # Register new G-code commands for setting/retrieving retraction
        # parameters as well as clearing retraction state
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "SET_RETRACTION",
            self.cmd_SET_RETRACTION,
            desc=self.cmd_SET_RETRACTION_help,
        )
        self.gcode.register_command(
            "GET_RETRACTION",
            self.cmd_GET_RETRACTION,
            desc=self.cmd_GET_RETRACTION_help,
        )
        self.gcode.register_command(
            "CLEAR_RETRACTION",
            self.cmd_CLEAR_RETRACTION,
            desc=self.cmd_CLEAR_RETRACTION_help,
        )
        self.gcode.register_command(
            "RESET_RETRACTION",
            self.cmd_RESET_RETRACTION,
            desc=self.cmd_RESET_RETRACTION_help,
        )

        # Register new G-code commands for firmware retraction/unretraction
        self.gcode.register_command("G10", self.cmd_G10)
        self.gcode.register_command("G11", self.cmd_G11)

    # Helper method to get retraction parameters from config
    def _get_config_params(self):
        self.retract_length = self.config_ref.getfloat(
            "retract_length", 0.0, minval=0.0
        )
        self.retract_speed = self.config_ref.getfloat(
            "retract_speed", 20.0, minval=1
        )
        self.unretract_extra_length = self.config_ref.getfloat(
            "unretract_extra_length", 0.0, minval=0.0
        )
        self.unretract_speed = self.config_ref.getfloat(
            "unretract_speed", 10.0, minval=1
        )
        self.z_hop_height = self.config_ref.getfloat(
            "z_hop_height", 0.0, minval=0.0
        )
        self.clear_zhop = self.config_ref.getboolean(
            "clear_zhop_on_z_moves", False
        )
        self.unretract_length = (
            self.retract_length + self.unretract_extra_length
        )

    # Helper method to register commands and instantiate required objects
    def _handle_ready(self):
        self.gcode_move = self.printer.lookup_object("gcode_move")
        self.exclude_objects = self.printer.lookup_object(
            "exclude_object", None
        )

        # Register move transformation while printer ready
        # important to track Z change request
        self.next_transform = self.gcode_move.set_move_transform(
            self, force=True
        )

        # Get maximum_z
        toolhead = self.printer.lookup_object("toolhead")
        kin = toolhead.get_kinematics()
        self.maximum_z = kin.get_status(None)["axis_maximum"][2]

        # Register Events to clear retraction when a new print is started, an
        # ongoing print is canceled or a print is finished
        # GCode streaming mode (most commonly done via OctoPrint)
        self.printer.register_event_handler(
            "homing:homing_move_begin", self._execute_clear_z_hop
        )
        self.printer.register_event_handler(
            "stepper_enable:motor_off", self._execute_clear_z_hop
        )

    # Helper method to return the current retraction parameters
    def get_status(self, eventtime):
        return {
            "retract_length": self.retract_length,
            "retract_speed": self.retract_speed,
            "unretract_extra_length": self.unretract_extra_length,
            "unretract_speed": self.unretract_speed,
            "z_hop_height": self.z_hop_height,
            "unretract_length": self.unretract_length,
            "retract_state": self.is_retracted,
            "zhop_state": self.do_zhop,
        }

    # Command to set the firmware retraction parameters
    cmd_SET_RETRACTION_help = "Set firmware retraction parameters"

    def cmd_SET_RETRACTION(self, gcmd):
        self.retract_length = gcmd.get_float(
            "RETRACT_LENGTH", self.retract_length, minval=0.0
        )
        self.retract_speed = gcmd.get_float(
            "RETRACT_SPEED", self.retract_speed, minval=1.0
        )
        self.unretract_extra_length = gcmd.get_float(
            "UNRETRACT_EXTRA_LENGTH",
            self.unretract_extra_length,
            minval=-1.0,
        )
        self.unretract_speed = gcmd.get_float(
            "UNRETRACT_SPEED", self.unretract_speed, minval=1.0
        )
        self.z_hop_height = gcmd.get_float(
            "Z_HOP_HEIGHT", self.z_hop_height, minval=0.0
        )  # z_hop_height with 0mm min. to prevent nozzle crash
        self.unretract_length = (
            self.retract_length + self.unretract_extra_length
        )

    # Command to report the current firmware retraction parameters
    cmd_GET_RETRACTION_help = "Report firmware retraction parameters and states"

    def cmd_GET_RETRACTION(self, gcmd):
        gcmd.respond_info(
            "RETRACT_LENGTH=%.5f RETRACT_SPEED=%.5f "
            "UNRETRACT_EXTRA_LENGTH=%.5f UNRETRACT_SPEED=%.5f "
            " Z_HOP_HEIGHT=%.5f "
            " RETRACT_STATE=%s "
            " ZHOP_STATE=%s"
            % (
                self.retract_length,
                self.retract_speed,
                self.unretract_extra_length,
                self.unretract_speed,
                self.z_hop_height,
                self.is_retracted,
                self.do_zhop,
            )
        )

    # Command to clear FW retraction (add to CANCEL macros at the beginning)
    cmd_CLEAR_RETRACTION_help = (
        "Clear retraction state without retract move if enabled"
    )

    def cmd_CLEAR_RETRACTION(self, gcmd):
        self._execute_clear_z_hop()
        self.is_retracted = False  # Reset retract flag to enable G10 command
        gcmd.respond_info(
            "Retraction was cleared. zhop is undone on next move."
        )

    # Command to reset retraction parameters
    cmd_RESET_RETRACTION_help = "Reset retraction parameters to default values"

    def cmd_RESET_RETRACTION(self, gcmd):
        self._get_config_params()  # Reset retraction parameters to config values

    # Helper to clear z_hop
    def _execute_clear_z_hop(self, *args):
        self.do_zhop = False
        self.current_z_hop_height = 0.0  # prevent nozzle crash if G11 occurs

    # Helper to skip command while in exclude object
    def _test_in_excluded_region(self):
        if self.exclude_objects is not None:
            return self.exclude_objects._test_in_excluded_region()
        else:
            return False

    # Gcode Command G10 to perform firmware retraction
    def cmd_G10(self, gcmd):
        extrude_factor, speed_factor = self._get_factors()

        if (
            not self.is_retracted and not self._test_in_excluded_region()
        ):  # If filament isn't retracted
            # Check if FW retraction enabled
            if self.retract_length == 0.0 and self.z_hop_height == 0.0:
                gcmd.respond_info(
                    "Retraction length and z_hop zero. Firmware retraction "
                    "disabled. G10 Command ignored!"
                )
            else:
                # Store retract parameters for moves when retracted until next G11
                self.current_unretract_length = self.unretract_length
                self.current_unretract_speed = self.unretract_speed
                self._execute_clear_z_hop()
                # get position (track of bed_mesh, z_thermal_adjust changes)
                position = self.get_position()
                # do retraction
                if self.retract_length > 0.0:
                    position[3] -= self.retract_length * extrude_factor
                    self.move(position, self.retract_speed * speed_factor)
                # do zhop
                if self._limit_zhop(gcmd, position):
                    self.do_zhop = True
                    self.move(
                        position,
                        ZHOP_MOVE_SPEED_FRACTION
                        * self.max_z_vel
                        * speed_factor,
                    )
                # update toolhead position
                self.gcode_move.reset_last_position()
                self.is_retracted = True

    # GCode Command G11 to perform filament unretraction
    def cmd_G11(self, gcmd):
        extrude_factor, speed_factor = self._get_factors()
        if (
            self.is_retracted and not self._test_in_excluded_region()
        ):  # Check if the filament is retracted
            if (
                self.current_unretract_length == 0.0
                and self.current_z_hop_height == 0.0
            ):  # Check if FW retraction enabled
                gcmd.respond_info(
                    "Retraction length and z_hop zero. Firmware retraction "
                    "disabled. G11 Command ignored!"
                )
            else:
                # get position (track of bed_mesh, z_thermal_adjust changes)
                position = self.get_position()
                # undo zhop
                self._execute_clear_z_hop()
                self.move(
                    position,
                    ZHOP_MOVE_SPEED_FRACTION * self.max_z_vel * speed_factor,
                )
                # un-retract
                if self.current_unretract_length > 0.0:
                    position[3] += (
                        self.current_unretract_length * extrude_factor
                    )
                    self.move(
                        position, self.current_unretract_speed * speed_factor
                    )
                # update toolhead position
                self.gcode_move.reset_last_position()
            self.is_retracted = False

    # gcode_move transform position helper
    def get_position(self):
        position = self.next_transform.get_position()
        position[2] -= self.current_z_hop_height
        self.last_position[:] = position
        return position

    # gcode_move transform move helper
    def move(self, newpos, speed):
        # cancel zhop on z moves
        if (
            self.do_zhop
            and self.clear_zhop
            and newpos[2] != self.last_position[2]
        ):
            self._execute_clear_z_hop()

        adjusted_pos = [
            newpos[0],
            newpos[1],
            newpos[2] + self.current_z_hop_height,
            newpos[3],
        ]
        self.next_transform.move(adjusted_pos, speed)
        self.last_position[:] = newpos

    # Helper to limit z_hop while z_max is reached
    def _limit_zhop(self, gcmd, pos):
        z_margin = self.maximum_z - pos[2] - MAX_Z_MARGIN
        if self.z_hop_height > z_margin:
            self.current_z_hop_height = max(0, z_margin)
            gcmd.respond_info(
                "firmware_retraction : z_hop is limited to %.5f"
                % self.current_z_hop_height
            )
        else:
            self.current_z_hop_height = self.z_hop_height
        return self.current_z_hop_height > 0.0

    # Helper to get extrude_factor & speed_factor
    def _get_factors(self):
        gcodestatus = self.gcode_move.get_status()
        return [gcodestatus["extrude_factor"], gcodestatus["speed_factor"]]


def load_config(config):
    return FirmwareRetraction(config)
