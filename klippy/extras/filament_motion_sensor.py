# Filament Motion Sensor Module
#
# Copyright (C) 2021 Joshua Wherrett <thejoshw.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import filament_switch_sensor

CHECK_RUNOUT_TIMEOUT = 0.250


class EncoderSensor:
    def __init__(self, config):
        # Read config
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        switch_pin = config.get("switch_pin")
        self.extruder_name = config.get("extruder")
        self.detection_length = config.getfloat(
            "detection_length", 7.0, above=0.0
        )
        # Configure pins
        buttons = self.printer.load_object(config, "buttons")
        buttons.register_buttons([switch_pin], self.encoder_event)
        # Get printer objects
        self.reactor = self.printer.get_reactor()
        self.runout_helper = filament_switch_sensor.RunoutHelper(config, self)
        self.get_status = self.runout_helper.get_status
        self.extruder = None
        self.estimated_print_time = None
        # Initialise internal state
        self.filament_runout_pos = None
        # Register commands and event handlers
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler(
            "idle_timeout:printing", self._handle_printing
        )
        self.printer.register_event_handler(
            "idle_timeout:ready", self._handle_not_printing
        )
        self.printer.register_event_handler(
            "idle_timeout:idle", self._handle_not_printing
        )

    def _update_filament_runout_pos(self, eventtime=None):
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        self.filament_runout_pos = (
            self.get_extruder_pos(eventtime) + self.detection_length
        )

    def _handle_ready(self):
        self.extruder = self.printer.lookup_object(self.extruder_name)
        self.estimated_print_time = self.printer.lookup_object(
            "mcu"
        ).estimated_print_time
        self._update_filament_runout_pos()
        self._extruder_pos_update_timer = self.reactor.register_timer(
            self._extruder_pos_update_event
        )

    def _handle_printing(self, print_time):
        self.reactor.update_timer(
            self._extruder_pos_update_timer, self.reactor.NOW
        )

    def _handle_not_printing(self, print_time):
        self.reactor.update_timer(
            self._extruder_pos_update_timer, self.reactor.NEVER
        )

    def get_extruder_pos(self, eventtime=None):
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        print_time = self.estimated_print_time(eventtime)
        return self.extruder.find_past_position(print_time)

    def _extruder_pos_update_event(self, eventtime):
        extruder_pos = self.get_extruder_pos(eventtime)
        # Check for filament runout
        self.runout_helper.note_filament_present(
            eventtime, extruder_pos < self.filament_runout_pos
        )
        return eventtime + CHECK_RUNOUT_TIMEOUT

    def encoder_event(self, eventtime, state):
        if self.extruder is not None:
            self._update_filament_runout_pos(eventtime)
            # Check for filament insertion
            # Filament is always assumed to be present on an encoder event
            self.runout_helper.note_filament_present(eventtime, True)

    def get_sensor_status(self):
        return (
            "Filament Sensor %s: %s\n"
            "Filament Detected: %s\n"
            "Detection Length: %.2f\n"
            "Smart: %s\n"
            "Always Fire Events: %s"
            % (
                self.runout_helper.name,
                (
                    "enabled"
                    if self.runout_helper.sensor_enabled
                    else "disabled"
                ),
                "true" if self.runout_helper.filament_present else "false",
                self.detection_length,
                "true" if self.runout_helper.smart else "false",
                "true" if self.runout_helper.always_fire_events else "false",
            )
        )

    def sensor_get_status(self, eventtime):
        return {"detection_length": float(self.detection_length)}

    def get_info(self, gcmd):
        detection_length = gcmd.get_float("DETECTION_LENGTH", None, minval=0.0)
        if detection_length is None:
            gcmd.respond_info(self.get_sensor_status())
            return True
        return False

    def reset_needed(self, enable=None, always_fire_events=None):
        if enable and not self.runout_helper.sensor_enabled:
            return True
        return False

    def set_filament_sensor(self, gcmd):
        reset_needed = False
        detection_length = gcmd.get_float("DETECTION_LENGTH", None, minval=0.0)
        if detection_length is not None:
            if detection_length != self.detection_length:
                reset_needed = True
            self.detection_length = detection_length
        return reset_needed

    def reset(self):
        self._update_filament_runout_pos()
        self.runout_helper.note_filament_present(self.reactor.monotonic(), True)


def load_config_prefix(config):
    return EncoderSensor(config)
