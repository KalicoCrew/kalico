# Printer cooling fan
#
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import output_pin, pulse_counter


class Fan:
    def __init__(self, config, default_shutdown_speed=0.0):
        self.printer = config.get_printer()
        self.last_fan_value = self.last_req_value = 0.0
        self.last_pwm_value = 0.0
        # Read config
        self.kick_start_time = config.getfloat(
            "kick_start_time", 0.1, minval=0.0
        )
        self.min_power = config.getfloat(
            "min_power", default=None, minval=0.0, maxval=1.0
        )
        self.off_below = config.getfloat(
            "off_below", default=None, minval=0.0, maxval=1.0
        )
        if self.off_below is not None:
            config.deprecate("off_below")
        self.initial_speed = config.getfloat(
            "initial_speed", default=None, minval=0.0, maxval=1.0
        )

        # handles switchover of variable
        # if new var is not set, and old var is, set new var to old var
        # if new var is not set and neither is old var, set new var to default of 0.0
        # if new var is set, use new var
        if self.min_power is not None and self.off_below is not None:
            raise config.error(
                "min_power and off_below are both set. Remove one!"
            )
        if self.min_power is None:
            if self.off_below is None:
                # both unset, set to 0.0
                self.min_power = 0.0
            else:
                self.min_power = self.off_below

        self.max_power = config.getfloat(
            "max_power", 1.0, above=0.0, maxval=1.0
        )
        if self.min_power > self.max_power:
            raise config.error(
                "min_power=%f can't be larger than max_power=%f"
                % (self.min_power, self.max_power)
            )

        cycle_time = config.getfloat("cycle_time", 0.010, above=0.0)
        hardware_pwm = config.getboolean("hardware_pwm", False)
        shutdown_speed = config.getfloat(
            "shutdown_speed", default_shutdown_speed, minval=0.0, maxval=1.0
        )
        # Setup pwm object
        ppins = self.printer.lookup_object("pins")
        self.mcu_fan = ppins.setup_pin("pwm", config.get("pin"))
        self.mcu_fan.setup_max_duration(0.0)
        self.mcu_fan.setup_cycle_time(cycle_time, hardware_pwm)

        if hardware_pwm:
            shutdown_power = max(0.0, min(self.max_power, shutdown_speed))
        else:
            # the config allows shutdown_power to be > 0 and < 1, but it is validated
            # in MCU_pwm._build_config().
            shutdown_power = max(0.0, shutdown_speed)

        self.mcu_fan.setup_start_value(0.0, shutdown_power)
        self.enable_pin = None
        enable_pin = config.get("enable_pin", None)
        if enable_pin is not None:
            self.enable_pin = ppins.setup_pin("digital_out", enable_pin)
            self.enable_pin.setup_max_duration(0.0)

        # Create gcode request queue
        self.gcrq = output_pin.GCodeRequestQueue(
            config, self.mcu_fan.get_mcu(), self._apply_speed
        )

        # Setup tachometer
        self.tachometer = FanTachometer(config)

        # Register callbacks
        self.printer.register_event_handler(
            "gcode:request_restart", self._handle_request_restart
        )
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def get_mcu(self):
        return self.mcu_fan.get_mcu()

    def _apply_speed(self, print_time, value):
        if value == self.last_fan_value:
            return "discard", 0.0
        if value > 0:
            # Scale value between min_power and max_power
            value = min(value, 1.0)
            pwm_value = (
                value * (self.max_power - self.min_power) + self.min_power
            )
        else:
            pwm_value = 0.0
        if self.enable_pin:
            if value > 0 and self.last_fan_value == 0:
                self.enable_pin.set_digital(print_time, 1)
            elif value == 0 and self.last_fan_value > 0:
                self.enable_pin.set_digital(print_time, 0)
        if (
            value
            and value < 1.0
            and self.kick_start_time
            and (not self.last_fan_value or value - self.last_fan_value > 0.5)
        ):
            # Run fan at full speed for specified kick_start_time
            self.last_req_value = value

            self.last_fan_value = 1.0
            self.last_pwm_value = self.max_power

            self.mcu_fan.set_pwm(print_time, self.max_power)

            return "delay", self.kick_start_time
        self.last_fan_value = self.last_req_value = value
        self.last_pwm_value = pwm_value
        self.mcu_fan.set_pwm(print_time, pwm_value)

    def set_speed(self, value, print_time=None):
        self.gcrq.send_async_request(value, print_time)

    def set_speed_from_command(self, value):
        self.gcrq.queue_gcode_request(value)

    def _handle_request_restart(self, print_time):
        self.set_speed(0.0, print_time)

    def _handle_ready(self):
        if self.initial_speed:
            self.set_speed_from_command(self.initial_speed)

    def get_status(self, eventtime):
        tachometer_status = self.tachometer.get_status(eventtime)
        return {
            "power": self.last_pwm_value,
            "value": self.last_req_value,
            "speed": self.last_req_value * self.max_power,
            "rpm": tachometer_status["rpm"],
        }


class FanTachometer:
    def __init__(self, config):
        printer = config.get_printer()
        self._freq_counter = None

        pin = config.get("tachometer_pin", None)
        if pin is not None:
            self.ppr = config.getint("tachometer_ppr", 2, minval=1)
            poll_time = config.getfloat(
                "tachometer_poll_interval", 0.0015, above=0.0
            )
            sample_time = 1.0
            self._freq_counter = pulse_counter.FrequencyCounter(
                printer, pin, sample_time, poll_time
            )

    def get_status(self, eventtime):
        if self._freq_counter is not None:
            rpm = self._freq_counter.get_frequency() * 30.0 / self.ppr
        else:
            rpm = None
        return {"rpm": rpm}


class PrinterFan:
    """The "fan" object, for a [fan] section or for [fan <name>] sections.

    M106/M107 without P control the active fans. printer.fan and heater
    controls use the active fans through this object.
    """

    cmd_ACTIVATE_FAN_help = "Select the fans controlled by M106 and M107"

    def __init__(self, config):
        # Heater controls reach the fan they use through a .fan attribute
        self.fan = self
        self.fans = {}
        self.fans_by_index = {}
        self.active_fans = []
        self.requested_speed = 0.0
        self.gcode = config.get_printer().lookup_object("gcode")
        self.gcode.register_command("M106", self.cmd_M106)
        self.gcode.register_command("M107", self.cmd_M107)
        if config.get_prefix_sections("fan "):
            self.gcode.register_command(
                "ACTIVATE_FAN",
                self.cmd_ACTIVATE_FAN,
                desc=self.cmd_ACTIVATE_FAN_help,
            )
        else:
            self.add_fan("fan", Fan(config))

    def add_fan(self, name, fan, index=None):
        self.fans[name] = fan
        if index is not None:
            self.fans_by_index[index] = fan
        # Other indexes are separate fans for slicers, such as an aux fan
        if not index:
            self.active_fans.append(fan)

    def get_status(self, eventtime):
        fan = (self.active_fans or list(self.fans.values()))[0]
        return fan.get_status(eventtime)

    def set_speed(self, value, print_time=None):
        for fan in self.active_fans:
            fan.set_speed(value, print_time)

    def _set_speed_from_gcode(self, gcmd, value):
        # A config without indexes ignores P, as it always did
        index = None
        if self.fans_by_index:
            index = gcmd.get_int("P", None, minval=0)
        if index is None:
            self.requested_speed = value
            for fan in self.active_fans:
                fan.set_speed_from_command(value)
        elif index in self.fans_by_index:
            self.fans_by_index[index].set_speed_from_command(value)
        else:
            # Slicers may command fans the printer doesn't have
            gcmd.respond_info("Ignoring unknown fan index %d" % (index,))

    def cmd_M106(self, gcmd):
        value = gcmd.get_float("S", 255.0, minval=0.0) / 255.0
        self._set_speed_from_gcode(gcmd, value)

    def cmd_M107(self, gcmd):
        self._set_speed_from_gcode(gcmd, 0.0)

    def cmd_ACTIVATE_FAN(self, gcmd):
        names = [name.strip() for name in gcmd.get("FAN").split(",")]
        for name in names:
            if name not in self.fans:
                raise gcmd.error("Unknown fan '%s'" % (name,))
        new_fans = [self.fans[name] for name in names]
        for fan in self.active_fans:
            if fan not in new_fans:
                fan.set_speed_from_command(0.0)
        for fan in new_fans:
            if fan not in self.active_fans:
                fan.set_speed_from_command(self.requested_speed)
        self.active_fans = new_fans


class NamedFan:
    cmd_SET_FAN_SPEED_help = "Sets the speed of a fan"

    def __init__(self, config, printer_fan):
        self.fan = Fan(config)
        self.printer_fan = printer_fan
        name = config.get_name().split()[-1]
        index = config.getint("index", None, minval=0)
        if index in printer_fan.fans_by_index:
            raise config.error("Fan index %d used more than once" % (index,))
        printer_fan.add_fan(name, self.fan, index)
        printer_fan.gcode.register_mux_command(
            "SET_FAN_SPEED",
            "FAN",
            name,
            self.cmd_SET_FAN_SPEED,
            desc=self.cmd_SET_FAN_SPEED_help,
        )

    def get_status(self, eventtime):
        status = self.fan.get_status(eventtime)
        status["active"] = self.fan in self.printer_fan.active_fans
        return status

    def cmd_SET_FAN_SPEED(self, gcmd):
        self.fan.set_speed_from_command(gcmd.get_float("SPEED", minval=0.0))


def load_config(config):
    return PrinterFan(config)


def load_config_prefix(config):
    name = config.get_name()
    if config.has_section("fan"):
        raise config.error(
            "[fan] and [%s] can't be used together. Give every fan a name,"
            " for example by renaming [fan] to [fan part]." % (name,)
        )
    printer_fan = config.get_printer().load_object(config, "fan")
    if not isinstance(printer_fan, PrinterFan):
        raise config.error("Can't setup [%s] and heated_fan together" % (name,))
    return NamedFan(config, printer_fan)
