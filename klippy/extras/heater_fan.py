# Support fans that are enabled when a heater is on
#
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import fan

PIN_MIN_TIME = 0.100


class PrinterHeaterFan:
    cmd_SET_FAN_SPEED_help = "Sets the speed of a fan"

    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.load_object(config, "heaters")
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.heater_names = config.getlist("heater", ("extruder",))
        self.heater_temp = config.getfloat("heater_temp", 50.0)
        self.heaters = []
        self.fan = fan.Fan(config, default_shutdown_speed=1.0)
        self.enabled_fan_speed = config.getfloat(
            "fan_speed", 1.0, minval=0.0, maxval=1.0
        )
        self.last_speed = 0.0
        self.last_gcode_speed = 0.0

        # register SET_FAN_SPEED mux command so users can override this fan
        self.fan_name = config.get_name().split()[-1]
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "SET_FAN_SPEED",
            "FAN",
            self.fan_name,
            self.cmd_SET_FAN_SPEED,
            desc=self.cmd_SET_FAN_SPEED_help,
        )


    def handle_ready(self):
        pheaters = self.printer.lookup_object("heaters")
        self.heaters = [pheaters.lookup_heater(n) for n in self.heater_names]
        reactor = self.printer.get_reactor()
        reactor.register_timer(
            self.callback, reactor.monotonic() + PIN_MIN_TIME
        )

    def get_status(self, eventtime):
        return self.fan.get_status(eventtime)

    def callback(self, eventtime):
        speed = self.last_gcode_speed
        for heater in self.heaters:
            current_temp, target_temp = heater.get_temp(eventtime)
            if (target_temp or current_temp > self.heater_temp) and self.enabled_fan_speed > speed:
                speed = self.enabled_fan_speed
        if speed != self.last_speed:
            self.last_speed = speed
            curtime = self.printer.get_reactor().monotonic()
            print_time = self.fan.get_mcu().estimated_print_time(curtime)
            self.fan.set_speed(print_time + PIN_MIN_TIME, speed)
        return eventtime + 1.0

    # SET_FAN_SPEED can only override this fan speed to a speed higher than the minimum one set by the heater
    def cmd_SET_FAN_SPEED(self, gcmd):
        self.last_gcode_speed = gcmd.get_float("SPEED", 0.0)
        speed = max(self.last_speed, self.last_gcode_speed)
        self.fan.set_speed_from_command(speed)


def load_config_prefix(config):
    return PrinterHeaterFan(config)
