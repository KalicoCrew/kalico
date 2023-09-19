from contextlib import contextmanager


class SensorLessHomingHelper(object):
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.toolhead = None
        self.gcode = self.printer.lookup_object("gcode")

        # Run current
        self.tmc_stepper_y_name = config.get("tmc_stepper_y_name")
        self.tmc_stepper_x_name = config.get("tmc_stepper_x_name")

        self.pconfig = self.printer.lookup_object("configfile")

        # Read config
        self.home_current = config.getfloat("home_current")
        self.minimum_homing_distance = config.getfloat(
            "minimum_homing_distance", 10
        )
        self.retract_distance = config.getfloat("retract_distance", 10)
        self.retract_speed = config.getfloat("retract_speed", 20)
        self.stallguard_reset_time = config.getfloat("stallguard_reset_time", 1)
        self.use_homing_status = config.getboolean("use_homing_status", True)

        self.gcode.register_command(
            "_HOME_X", self.cmd_HOME_X, "Sensorless homing X axis"
        )
        self.gcode.register_command(
            "_HOME_Y", self.cmd_HOME_Y, "Sensorless homing X axis"
        )

        self.printer.register_event_handler(
            "klippy:connect", self.handle_connect
        )

    def handle_connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")

    @contextmanager
    def set_xy_motor_current(self, homing_current):
        x_stepper_name = self.tmc_stepper_x_name.split()[1]
        self.gcode.run_script_from_command(
            "SET_TMC_CURRENT STEPPER={} CURRENT={}".format(
                x_stepper_name, homing_current
            )
        )

        y_stepper_name = self.tmc_stepper_y_name.split()[1]
        self.gcode.run_script_from_command(
            "SET_TMC_CURRENT STEPPER={} CURRENT={}".format(
                y_stepper_name, homing_current
            )
        )

        try:
            yield
        finally:
            curtime = self.printer.get_reactor().monotonic()
            settings = self.pconfig.get_status(curtime)["settings"]
            self.gcode.run_script_from_command(
                "SET_TMC_CURRENT STEPPER={} CURRENT={}".format(
                    x_stepper_name,
                    settings[self.tmc_stepper_x_name]["run_current"],
                )
            )
            self.gcode.run_script_from_command(
                "SET_TMC_CURRENT STEPPER={} CURRENT={}".format(
                    y_stepper_name,
                    settings[self.tmc_stepper_y_name]["run_current"],
                )
            )

    def cmd_HOME_X(self, gcmd):
        # Check if X axis is homed and its last known position
        curtime = self.printer.get_reactor().monotonic()
        kin_status = self.toolhead.get_status(curtime)

        pos = self.toolhead.get_position()

        if self.use_homing_status and "x" not in kin_status["homed_axes"]:
            gcmd.respond_info(
                "X is not homed {}. Will perform the retract before home.".format(
                    kin_status["homed_axes"]
                )
            )
            # Run the sensorless homing to the opposite direction
            with self.set_xy_motor_current(self.home_current):
                move_pos = pos[:]
                move_pos[0] = 0
                current_pos = pos[:]
                current_pos[0] = self.minimum_homing_distance
                endstops = (
                    self.toolhead.get_kinematics().rails[0].get_endstops()
                )

                # Do a manual homing
                phoming = self.printer.lookup_object("homing")
                self.toolhead.set_position(current_pos, homing_axes=[0])
                phoming.manual_home(
                    toolhead=self.toolhead,
                    endstops=endstops,
                    pos=move_pos,
                    speed=self.retract_speed,
                    triggered=True,
                    check_triggered=False,
                )

        elif (
            kin_status["axis_maximum"][0] - pos[0]
            < self.minimum_homing_distance
        ):
            gcmd.respond_info(
                "X is homed but too closed to the maximum range {}. Will perform the retract before home.".format(
                    pos[0]
                )
            )
            pos[0] -= self.minimum_homing_distance
            self.toolhead.manual_move(pos, self.retract_speed)
        else:
            gcmd.respond_info(
                "X is homed {} and away from maximum range {}.".format(
                    kin_status["homed_axes"], pos[0]
                )
            )

        with self.set_xy_motor_current(self.home_current):
            self.gcode.run_script_from_command("G28 X")
            self.toolhead.wait_moves()

            # Retract
            pos = self.toolhead.get_position()
            pos[0] -= self.retract_distance
            self.toolhead.move(pos, self.retract_speed)
            self.toolhead.dwell(self.stallguard_reset_time)

    def cmd_HOME_Y(self, gcmd):
        # Check if Y axis is homed and its last known position
        curtime = self.printer.get_reactor().monotonic()
        kin_status = self.toolhead.get_status(curtime)

        pos = self.toolhead.get_position()

        if self.use_homing_status and "y" not in kin_status["homed_axes"]:
            gcmd.respond_info(
                "Y is not homed {}. Will perform the retract before home.".format(
                    kin_status["homed_axes"]
                )
            )

            # Run the sensorless homing to the opposite direction
            with self.set_xy_motor_current(self.home_current):
                move_pos = pos[:]
                move_pos[1] = 0
                current_pos = pos[:]
                current_pos[1] = self.minimum_homing_distance
                endstops = (
                    self.toolhead.get_kinematics().rails[1].get_endstops()
                )

                # Do a manual homing
                phoming = self.printer.lookup_object("homing")
                self.toolhead.set_position(current_pos, homing_axes=[1])
                phoming.manual_home(
                    toolhead=self.toolhead,
                    endstops=endstops,
                    pos=move_pos,
                    speed=self.retract_speed,
                    triggered=True,
                    check_triggered=False,
                )
        elif (
            kin_status["axis_maximum"][1] - pos[1]
            < self.minimum_homing_distance
        ):
            gcmd.respond_info(
                "Y is homed but too closed to the maximum range {}. Will perform the retract before home.".format(
                    pos[1]
                )
            )
            pos[1] -= self.minimum_homing_distance
            self.toolhead.manual_move(pos, self.retract_speed)
        else:
            gcmd.respond_info(
                "Y is homed {} and away from maximum range {}.".format(
                    kin_status["homed_axes"], pos[1]
                )
            )

        with self.set_xy_motor_current(self.home_current):
            self.gcode.run_script_from_command("G28 Y")
            self.toolhead.wait_moves()

            # Retract
            pos = self.toolhead.get_position()
            pos[1] -= self.retract_distance
            self.toolhead.move(pos, self.retract_speed)
            self.toolhead.dwell(self.stallguard_reset_time)


def load_config(config):
    return SensorLessHomingHelper(config)
