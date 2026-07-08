from ..homing import Homing
from .compat import ConfigWrapper
from .heater import IndxToolboardHeater


class IndxToolboard:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.config = config
        self.name = config.get_name()

        mcu_name = config.get("mcu")
        mcu = printer.lookup_object(f"mcu {mcu_name}", None)
        if mcu is None:
            raise config.error(f"[{self.name}] could not find MCU {mcu_name}")
        self.mcu = mcu

        ppins = printer.lookup_object("pins")
        ppins.register_chip(self.name, IndxToolboardPins(self, ppins))
        resolver = ppins.get_pin_resolver(mcu.get_name())
        resolver.alias_pin("vin_mon", "PA2")
        resolver.alias_pin("led", "PA8")
        resolver.alias_pin("endstop", "PA9")
        resolver.alias_pin("part_cooling", "PA1")
        resolver.alias_pin("part_cooling_tacho", "PA0")
        resolver.alias_pin("motor_step", "PB12")
        resolver.alias_pin("motor_dir", "PB23")
        resolver.alias_pin("motor_enable", "PB4")
        resolver.alias_pin("motor_cs", "PA10")
        resolver.alias_pin("motor_sclk", "PA5")
        resolver.alias_pin("motor_mosi", "PA4")
        resolver.alias_pin("motor_miso", "PA7")
        resolver.alias_pin("ldc_int", "PB10")
        resolver.alias_pin("ldc_clk", "PB11")
        resolver.alias_pin("fan0_out", "PA21")
        resolver.alias_pin("fan0_tacho", "PA20")
        resolver.alias_pin("temp_bracket", "PA11")
        resolver.alias_pin("temp_board", "PB8")
        resolver.alias_pin("temp_ldc", "PB9")
        resolver.alias_pin("i2c0_sda", "PA22")
        resolver.alias_pin("i2c0_scl", "PA23")
        resolver.alias_pin("encoder_cs", "PB0")
        resolver.alias_pin("encoder_sclk", "PB3")
        resolver.alias_pin("encoder_mosi", "PB2")
        resolver.alias_pin("encoder_miso", "PB1")
        resolver.alias_pin("loadcell_cs", "PA18")
        resolver.alias_pin("loadcell_sclk", "PA17")
        resolver.alias_pin("loadcell_mosi", "PA16")
        resolver.alias_pin("loadcell_miso", "PA19")
        resolver.alias_pin("loadcell_drdy", "PB22")

        sercom_config_i2c0 = ConfigWrapper(
            printer,
            f"samd_sercom {self.name}_indx_periph",
            {
                "sercom": "sercom3",
                "tx_pin": f"{mcu.get_name()}:i2c0_sda",
                "clk_pin": f"{mcu.get_name()}:i2c0_scl",
            },
        )
        printer.load_object(sercom_config_i2c0, sercom_config_i2c0.get_name())

        sercom_config_encoder = ConfigWrapper(
            printer,
            f"samd_sercom {self.name}_indx_encoder",
            {
                "sercom": "sercom5",
                "tx_pin": f"{mcu.get_name()}:encoder_mosi",
                "rx_pin": f"{mcu.get_name()}:encoder_miso",
                "clk_pin": f"{mcu.get_name()}:encoder_sclk",
            },
        )
        printer.load_object(
            sercom_config_encoder, sercom_config_encoder.get_name()
        )

        sercom_config_loadcell = ConfigWrapper(
            printer,
            f"samd_sercom {self.name}_indx_loadcell",
            {
                "sercom": "sercom1",
                "tx_pin": f"{mcu.get_name()}:loadcell_mosi",
                "rx_pin": f"{mcu.get_name()}:loadcell_miso",
                "clk_pin": f"{mcu.get_name()}:loadcell_sclk",
            },
        )
        printer.load_object(
            sercom_config_loadcell, sercom_config_loadcell.get_name()
        )

        self.heater = IndxToolboardHeater(self, config)
        self.dock_measurement = IndxDockMeasurement(self)

        gcode = printer.lookup_object("gcode")
        gcode.register_command("INDX_LED_FORCE_COLOR", self.cmd_LED_FORCE_COLOR)
        gcode.register_command("INDX_LED_SET_CURRENT", self.cmd_LED_SET_CURRENT)

    def get_status(self, eventtime):
        return {
            "last_dock_measurement": self.dock_measurement.last_dock_measurement
        }

    def cmd_LED_FORCE_COLOR(self, gcmd):
        cmd = self.mcu.lookup_command(
            "indx_led_force_color r=%c g=%c b=%c force=%c",
        )
        if gcmd.get("CLEAR", None) is None:
            r = gcmd.get_int("RED", minval=0, maxval=255)
            g = gcmd.get_int("GREEN", minval=0, maxval=255)
            b = gcmd.get_int("BLUE", minval=0, maxval=255)
            cmd.send([r, g, b, 1])
        else:
            cmd.send([0, 0, 0, 0])

    def cmd_LED_SET_CURRENT(self, gcmd):
        cmd = self.mcu.lookup_command(
            "indx_led_set_current r=%c g=%c b=%c",
        )
        r = gcmd.get_int("RED", minval=0, maxval=255)
        g = gcmd.get_int("GREEN", minval=0, maxval=255)
        b = gcmd.get_int("BLUE", minval=0, maxval=255)
        cmd.send([r, g, b])


class IndxToolboardPins:
    def __init__(self, toolboard, ppins):
        self.toolboard = toolboard
        self.ppins = ppins

    def setup_pin(self, pin_type, pin_params):
        if pin_type == "pwm" and pin_params["pin"] == "heater":
            return self.toolboard.heater
        else:
            raise self.ppins.error(
                f"unknown INDX virtual pin {pin_params['pin']}"
            )


class IndxDockMeasurement:
    def __init__(self, toolboard):
        self.toolboard = toolboard
        self.printer = toolboard.printer
        self.last_dock_measurement = None

        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "INDX_DOCK_MEASURE",
            self.cmd_INDX_DOCK_MEASURE,
            desc=self.cmd_INDX_DOCK_MEASURE_help,
        )

    cmd_INDX_DOCK_MEASURE_help = (
        "Measure INDX dock X/Y position by energizing XY motors and homing."
    )

    def cmd_INDX_DOCK_MEASURE(self, gcmd):
        x_first = gcmd.get_int("X_FIRST", 0, minval=0, maxval=1)
        axes = [0, 1] if x_first else [1, 0]
        try:
            measurement = self.measure(axes)
        except self.printer.command_error:
            self.printer.lookup_object("stepper_enable").motor_off()
            raise
        gcmd.respond_info(
            "INDX_DOCK_MEASURE: measured dock position X=%.3f Y=%.3f"
            % (measurement["x"], measurement["y"])
        )

    def _enable_motors(self, toolhead, kin):
        stepper_enable = self.printer.lookup_object("stepper_enable", None)
        if stepper_enable is None:
            return
        print_time = toolhead.get_last_move_time()
        for stepper in kin.get_steppers():
            if not (stepper.is_active_axis("x") or stepper.is_active_axis("y")):
                continue
            enable = stepper_enable.lookup_enable(stepper.get_name())
            enable.motor_enable(print_time)
        toolhead.dwell(0.100)

    def _get_kinematic_position(self, kin):
        stepper_positions = {
            stepper.get_name(): (
                stepper.get_mcu_position() * stepper.get_step_dist()
            )
            for stepper in kin.get_steppers()
        }
        return kin.calc_position(stepper_positions)

    def measure(self, axes):
        toolhead = self.printer.lookup_object("toolhead")
        kin = toolhead.get_kinematics()
        self._enable_motors(toolhead, kin)
        kinematic_position_before = self._get_kinematic_position(kin)

        homing_state = Homing(self.printer)
        homing_state.set_axes(axes)
        kin.home(homing_state)

        final_position = toolhead.get_position()
        kinematic_position_after = self._get_kinematic_position(kin)
        start_position = [
            final + before - after
            for final, after, before in zip(
                final_position[:2],
                kinematic_position_after[:2],
                kinematic_position_before[:2],
            )
        ]
        measurement = {
            "position": start_position,
            "x": start_position[0],
            "y": start_position[1],
            "axis_order": "".join(["XY"[axis] for axis in axes]),
        }
        self.last_dock_measurement = measurement
        return measurement
