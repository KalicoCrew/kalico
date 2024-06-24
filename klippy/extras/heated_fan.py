# Support heater fans
#
# Copyright (C) 2024  Rogerio Goncalves <rogerlz@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import fan

PIN_MIN_TIME = 0.100


class HeatedFan:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = config.get_printer().lookup_object("gcode")

        try:
            self.printer.lookup_object("fan")
        except:
            self.printer.add_object("fan", self)
        else:
            raise self.printer.config_error(
                "Can't setup fan and heated_fan together"
            )

        # setup heater
        pheaters = self.printer.load_object(config, "heaters")
        self.heater = pheaters.setup_heater(config, "F")
        self.heater_temp = config.getfloat("heater_temp", 50.0)
        self.stats = self.heater.stats

        # setup fan
        self.fan = fan.Fan(config, default_shutdown_speed=1.0)
        self.min_speed = config.getfloat(
            "min_speed", 1.0, minval=0.0, maxval=1.0
        )
        self.idle_timeout = config.getfloat("idle_timeout", 60.0, minval=0.0)

        self.set_speed = 0.0
        self.timeout_timer = None

        self.gcode.register_command(
            "SET_HEATED_FAN_TARGET", self.cmd_SET_HEATED_FAN_TARGET
        )
        self.gcode.register_command("M106", self.cmd_M106)
        self.gcode.register_command("M107", self.cmd_M107)

        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    def cmd_SET_HEATED_FAN_TARGET(self, gcmd):
        self.heater_temp = gcmd.get_float("TARGET", 0)

    def cmd_M106(self, gcmd):
        self.set_speed = gcmd.get_float("S", 255.0, minval=0.0) / 255.0
        if not self.set_speed:
            return self.cmd_M107(gcmd)

        if self.set_speed < self.min_speed:
            self.set_speed = self.min_speed

        self.fan.set_speed_from_command(self.set_speed)
        _, target_temp = self.heater.get_temp(self.reactor.monotonic())
        # if the heater is off or has a different target temp than configured, set it
        if not target_temp or target_temp != self.heater_temp:
            self.heater.set_temp(self.heater_temp)

    def cmd_M107(self, gcmd):
        self.set_speed = 0.0
        self.heater.set_temp(0)
        self.start_timeout_timer()

    def start_timeout_timer(self):
        self.fan.set_speed_from_command(self.min_speed)
        self.timeout_timer = self.reactor.register_timer(
            self.handle_timeout_timer,
            self.reactor.monotonic() + self.idle_timeout,
        )

    def handle_ready(self):
        self.reactor.register_timer(
            self.callback, self.reactor.monotonic() + PIN_MIN_TIME
        )

    def handle_timeout_timer(self, eventtime):
        self.fan.set_speed_from_command(self.set_speed)
        return self.reactor.NEVER

    def callback(self, eventtime):
        _, target_temp = self.heater.get_temp(eventtime)
        # if the heater has a target and the fan is off, turn it on to min fan speed
        if target_temp and not self.set_speed:
            self.fan.set_speed_from_command(self.min_speed)
        return eventtime + 1.0

    def get_status(self, eventtime):
        status = self.fan.get_status(eventtime)
        status.update(self.heater.get_status(eventtime))
        return status


def load_config(config):
    return HeatedFan(config)
