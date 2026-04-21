# Support fans that are enabled when a heater is on
#
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import fan

PIN_MIN_TIME = 0.100


class PrinterHeaterFan:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.load_object(config, "heaters")
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.heater_names = config.getlist("heater", ("extruder",))
        self.heater_temp = config.getfloat("heater_temp", 50.0)
        self.heaters = []
        self.fan_speed = config.getfloat(
            "fan_speed", 1.0, minval=0.0, maxval=1.0
        )
        self.last_speed = 0.0

        # Delegate mode: `fan:` references an existing fan config section
        # instead of giving this heater_fan its own pin.
        self._delegate_ref = config.get("fan", None)
        pin_set = config.get("pin", None) is not None
        if self._delegate_ref is not None and pin_set:
            raise config.error(
                "[%s]: specify either `pin:` (classic heater_fan) or"
                " `fan:` (delegate to another fan), not both"
                % (config.get_name(),)
            )
        self._section_name = config.get_name()
        self._delegate_target = None
        if self._delegate_ref is None:
            # Classic mode — own a pin.
            self.fan = fan.Fan(config, default_shutdown_speed=1.0)
        else:
            # Delegate mode — no self.fan; resolution happens at ready.
            self.fan = None

    def handle_ready(self):
        pheaters = self.printer.lookup_object("heaters")
        self.heaters = [pheaters.lookup_heater(n) for n in self.heater_names]
        if self._delegate_ref is not None:
            target = self.printer.lookup_object(self._delegate_ref, None)
            if target is None:
                raise self.printer.config_error(
                    "[%s]: fan reference %r not found"
                    % (self._section_name, self._delegate_ref)
                )
            target_fan = getattr(target, "fan", None)
            if not isinstance(target_fan, fan.Fan):
                raise self.printer.config_error(
                    "[%s]: fan reference %r does not expose a fan.Fan"
                    % (self._section_name, self._delegate_ref)
                )
            self._delegate_target = target_fan
            self._delegate_target.register_floor(self._section_name)
            # Synchronize: the diff-check in callback pairs last_speed with
            # the registered floor; force them both to 0.0 explicitly so the
            # invariant doesn't depend on FanFloorRegistry's default.
            self._delegate_target.update_floor(self._section_name, 0.0)
        reactor = self.printer.get_reactor()
        reactor.register_timer(
            self.callback, reactor.monotonic() + PIN_MIN_TIME
        )

    def get_status(self, eventtime):
        if self._delegate_target is not None:
            # Merge target's status; floor takes precedence over any
            # same-named key the target might add in the future.
            return {
                **self._delegate_target.get_status(eventtime),
                "floor": self.last_speed,
            }
        return self.fan.get_status(eventtime)

    def callback(self, eventtime):
        speed = 0.0
        for heater in self.heaters:
            current_temp, target_temp = heater.get_temp(eventtime)
            if target_temp or current_temp > self.heater_temp:
                speed = self.fan_speed
        if speed != self.last_speed:
            self.last_speed = speed
            if self._delegate_target is not None:
                self._delegate_target.update_floor(self._section_name, speed)
            else:
                self.fan.set_speed(speed)
        return eventtime + 1.0


def load_config_prefix(config):
    return PrinterHeaterFan(config)
