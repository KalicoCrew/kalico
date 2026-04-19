# Generic Filament Sensor Module
#
# Copyright (C) 2019  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

CHECK_RUNOUT_TIMEOUT = 0.250


class RunoutHelper:
    def __init__(self, config, defined_sensor, runout_distance=0):
        self.name = config.get_name().split()[-1]
        self.defined_sensor = defined_sensor
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        # Read config
        self.runout_pause = config.getboolean("pause_on_runout", True)
        if self.runout_pause:
            self.printer.load_object(config, "pause_resume")
        self.runout_gcode = None
        self.immediate_runout_gcode = None
        self.insert_gcode = None
        gcode_macro = self.printer.load_object(config, "gcode_macro")
        if self.runout_pause or config.get("runout_gcode", None) is not None:
            self.runout_gcode = gcode_macro.load_template(
                config, "runout_gcode", ""
            )
        if config.get("insert_gcode", None) is not None:
            self.insert_gcode = gcode_macro.load_template(
                config, "insert_gcode"
            )
        self.pause_delay = config.getfloat("pause_delay", 0.5, minval=0.0)
        self.event_delay = config.getfloat("event_delay", 3.0, minval=0.0)
        self.runout_distance = runout_distance
        # Internal state
        self.min_event_systime = self.reactor.NEVER
        self.filament_present = False
        self.sensor_enabled = True
        self.smart = config.getboolean("smart", False)
        self.always_fire_events = config.getboolean("always_fire_events", False)
        self.runout_position = 0.0
        self.runout_elapsed = 0.0
        self.runout_distance_timer = None
        # Register commands and event handlers
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.gcode.register_mux_command(
            "QUERY_FILAMENT_SENSOR",
            "SENSOR",
            self.name,
            self.cmd_QUERY_FILAMENT_SENSOR,
            desc=self.cmd_QUERY_FILAMENT_SENSOR_help,
        )
        self.gcode.register_mux_command(
            "SET_FILAMENT_SENSOR",
            "SENSOR",
            self.name,
            self.cmd_SET_FILAMENT_SENSOR,
            desc=self.cmd_SET_FILAMENT_SENSOR_help,
        )

    def _handle_ready(self):
        self.min_event_systime = self.reactor.monotonic() + 2.0

    def _runout_event_handler(self, eventtime):
        if self.immediate_runout_gcode is not None:
            self._exec_gcode("", self.immediate_runout_gcode)
        # Pausing from inside an event requires that the pause portion
        # of pause_resume execute immediately.
        if self.runout_distance > 0:
            if self.runout_distance_timer is None:
                self.runout_position = self.defined_sensor.get_extruder_pos(
                    eventtime
                )
                self.runout_distance_timer = self.reactor.register_timer(
                    self._pause_after_distance, self.reactor.NOW
                )
        else:
            self._execute_runout(eventtime)

    def _execute_runout(self, eventtime):
        pause_prefix = ""
        if self.runout_pause:
            pause_resume = self.printer.lookup_object("pause_resume")
            pause_resume.send_pause_command()
            pause_prefix = "PAUSE\n"
            self.printer.get_reactor().pause(eventtime + self.pause_delay)
        self._exec_gcode(pause_prefix, self.runout_gcode)
        self.reset_runout_distance_info()

    def reset_runout_distance_info(self):
        self.runout_elapsed = 0.0
        if self.runout_distance_timer is not None:
            self.reactor.unregister_timer(self.runout_distance_timer)
            self.runout_distance_timer = None

    def _pause_after_distance(self, eventtime):
        runout_elapsed = max(
            0.0,
            self.defined_sensor.get_extruder_pos(eventtime)
            - self.runout_position,
        )
        if runout_elapsed < self.runout_distance:
            self.runout_elapsed = runout_elapsed
            return eventtime + CHECK_RUNOUT_TIMEOUT
        else:
            self._execute_runout(eventtime)
            return self.reactor.NEVER

    def _insert_event_handler(self, eventtime):
        self._exec_gcode("", self.insert_gcode)

    def _exec_gcode(self, prefix, template):
        try:
            self.gcode.run_script(prefix + template.render() + "\nM400")
        except Exception:
            logging.exception("Script running error")
        self.min_event_systime = self.reactor.monotonic() + self.event_delay

    def note_filament_present(
        self, eventtime, is_filament_present=None, force=False, immediate=False
    ):
        if is_filament_present is None:
            is_filament_present = self.filament_present
        if is_filament_present == self.filament_present and not force:
            return
        self.filament_present = is_filament_present
        # with debouncing the event time is passed into us as we are
        # called with a delay so the current time is not the event time
        if eventtime < self.min_event_systime or (
            not self.always_fire_events and not self.sensor_enabled
        ):
            # do not process during the initialization time, duplicates,
            # during the event delay time, while an event is running, or
            # when the sensor is disabled
            return
        if is_filament_present:
            self.printer.send_event("filament:insert", eventtime, self.name)
        else:
            self.printer.send_event("filament:runout", eventtime, self.name)
        if not self.sensor_enabled:
            return
        # Determine "printing" status
        now = self.reactor.monotonic()
        idle_timeout = self.printer.lookup_object("idle_timeout")
        print_stats = self.printer.lookup_object("print_stats")
        is_printing = (
            print_stats.get_status(now)["state"] == "printing"
            if self.smart
            else idle_timeout.get_status(now)["state"] == "Printing"
        )
        # Perform filament action associated with status change (if any)
        if is_filament_present:
            if not is_printing and self.insert_gcode is not None:
                # insert detected
                self.min_event_systime = self.reactor.NEVER
                logging.info(
                    "Filament Sensor %s: insert event detected, Time %.2f"
                    % (self.name, now)
                )
                self.reactor.register_callback(self._insert_event_handler)
        elif is_printing and self.runout_gcode is not None:
            # runout detected
            self.min_event_systime = self.reactor.NEVER
            logging.info(
                "Filament Sensor %s: runout event detected, Time %.2f"
                % (self.name, now)
            )
            self.reactor.register_callback(
                self._execute_runout
                if immediate
                else self._runout_event_handler
            )

    def get_status(self, eventtime):
        status = {
            "filament_detected": bool(self.filament_present),
            "enabled": bool(self.sensor_enabled),
            "smart": bool(self.smart),
            "always_fire_events": bool(self.always_fire_events),
        }
        status.update(self.defined_sensor.sensor_get_status(eventtime))
        return status

    cmd_QUERY_FILAMENT_SENSOR_help = "Query the status of the Filament Sensor"

    def cmd_QUERY_FILAMENT_SENSOR(self, gcmd):
        msg = "Filament Sensor %s: filament %s" % (
            self.name,
            "detected" if self.filament_present else "not detected",
        )
        gcmd.respond_info(msg)

    cmd_SET_FILAMENT_SENSOR_help = "Sets the filament sensor on/off"

    def cmd_SET_FILAMENT_SENSOR(self, gcmd):
        enable = gcmd.get_int("ENABLE", None, minval=0, maxval=1)
        reset = gcmd.get_int("RESET", None, minval=0, maxval=1)
        smart = gcmd.get_int("SMART", None, minval=0, maxval=1)
        always_fire_events = gcmd.get_int(
            "ALWAYS_FIRE_EVENTS", None, minval=0, maxval=1
        )
        reset_needed = False
        if (
            enable is None
            and reset is None
            and smart is None
            and always_fire_events is None
            and self.defined_sensor.get_info(gcmd)
        ):
            return
        if enable is not None:
            self.sensor_enabled = enable
        if reset is not None and reset:
            reset_needed = True
        if smart is not None:
            self.smart = smart
        if always_fire_events is not None:
            self.always_fire_events = always_fire_events
        if self.defined_sensor.set_filament_sensor(gcmd):
            reset_needed = True
        if self.defined_sensor.reset_needed(
            enable=enable, always_fire_events=always_fire_events
        ):
            reset_needed = True
        if reset_needed:
            self.defined_sensor.reset()


class SwitchSensor:
    def __init__(self, config):
        self.printer = config.get_printer()
        gcode_macro = self.printer.load_object(config, "gcode_macro")
        buttons = self.printer.load_object(config, "buttons")
        switch_pin = config.get("switch_pin")
        runout_distance = config.getfloat("runout_distance", 0.0, minval=0.0)
        buttons.register_debounce_button(
            switch_pin, self._button_handler, config
        )

        self.check_on_print_start = config.getboolean(
            "check_on_print_start", False
        )
        self.reactor = self.printer.get_reactor()
        self.estimated_print_time = None
        self.runout_helper = RunoutHelper(config, self, runout_distance)
        if config.get("immediate_runout_gcode", None) is not None:
            self.runout_helper.immediate_runout_gcode = (
                gcode_macro.load_template(config, "immediate_runout_gcode", "")
            )
        self.get_status = self.runout_helper.get_status
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler(
            "idle_timeout:printing", self._handle_printing
        )

    def _handle_ready(self):
        self.estimated_print_time = self.printer.lookup_object(
            "mcu"
        ).estimated_print_time

    def _handle_printing(self, print_time):
        if self.check_on_print_start:
            self.runout_helper.note_filament_present(
                self.reactor.monotonic(), None, True, True
            )

    def _button_handler(self, eventtime, state):
        self.runout_helper.note_filament_present(eventtime, state)

    def get_extruder_pos(self, eventtime=None):
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        print_time = self.estimated_print_time(eventtime)
        extruder = self.printer.lookup_object("toolhead").get_extruder()
        return extruder.find_past_position(print_time)

    def get_sensor_status(self):
        return (
            "Filament Sensor %s: %s\n"
            "Filament Detected: %s\n"
            "Runout Distance: %.2f\n"
            "Runout Elapsed: %.2f\n"
            "Smart: %s\n"
            "Always Fire Events: %s\n"
            "Check on Print Start: %s"
            % (
                self.runout_helper.name,
                "enabled" if self.runout_helper.sensor_enabled else "disabled",
                "true" if self.runout_helper.filament_present else "false",
                self.runout_helper.runout_distance,
                self.runout_helper.runout_elapsed,
                "true" if self.runout_helper.smart else "false",
                "true" if self.runout_helper.always_fire_events else "false",
                "true" if self.check_on_print_start else "false",
            )
        )

    def sensor_get_status(self, eventtime):
        return {
            "runout_distance": float(self.runout_helper.runout_distance),
            "runout_elapsed": float(self.runout_helper.runout_elapsed),
            "check_on_print_start": bool(self.check_on_print_start),
        }

    def get_info(self, gcmd):
        runout_distance = gcmd.get_float("RUNOUT_DISTANCE", None, minval=0.0)
        check_on_print_start = gcmd.get_int(
            "CHECK_ON_PRINT_START", None, minval=0, maxval=1
        )
        if runout_distance is None and check_on_print_start is None:
            gcmd.respond_info(self.get_sensor_status())
            return True
        return False

    def reset_needed(self, enable=None, always_fire_events=None):
        if enable is not None and enable != self.runout_helper.sensor_enabled:
            return True
        if always_fire_events is not None and always_fire_events:
            return True
        return False

    def set_filament_sensor(self, gcmd):
        runout_distance = gcmd.get_float("RUNOUT_DISTANCE", None, minval=0.0)
        check_on_print_start = gcmd.get_int(
            "CHECK_ON_PRINT_START", None, minval=0, maxval=1
        )
        if runout_distance is not None:
            self.runout_helper.runout_distance = runout_distance
        if check_on_print_start is not None:
            self.check_on_print_start = check_on_print_start
        # No reset is needed when changing the runout_distance, so we always
        # return False
        return False

    def reset(self):
        self.runout_helper.reset_runout_distance_info()
        self.runout_helper.note_filament_present(
            self.reactor.monotonic(), self.runout_helper.filament_present, True
        )


def load_config_prefix(config):
    return SwitchSensor(config)
