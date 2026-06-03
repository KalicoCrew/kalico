# Clog Detection
#
# Copyright (C) 2026 Ella Fox <ella@fox.gal>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

LOST_STEPS_MAX = 1 << 20

# This aims to detect clogged nozzles by reading the state of two signals:
# 1. When the extruder motor stalls (TMC stall/lost steps)
# 2. The load cell reaches a sustained downward (negative) force
#
# Both of these signals must occur, in order to avoid false positives.
#
# This is only possible where the load cell is part of the filament path.
# A good use case for this is where the load cell is also a probe,
# such as when there is a load cell applied to a hotend's heatsink.
#
# The implementation is left open so that any load cell can be used.
# It may also be possible to implement clog detection by using a load cell
# in-line with the reverse-bowden tube.


class ClogDetect:
    def __init__(self, config):
        self._printer = config.get_printer()
        self._load_cell_name = config.get(
            "load_cell", default="load_cell_probe"
        )
        self._extruder_name = config.get("extruder", default="extruder")

        # Number of skipped steps required to trigger a clog detection event
        # Default may not be sensible yet. Need to find a good place to start.
        self._skipped_steps = config.getfloat("skipped_steps", default=20.0)

        # Undetermined whether this is a sensible force to look for.
        # The threshold for a sensible default probably sits somewhere in the >2kg range
        # The extruder under normal operating conditions is unlikely to exert more than that
        self._force_threshold = config.getfloat("force", default=4000.0)

        # Frequency in which this runs the detection routine.
        # 4Hz is the same as filament_motion_sensor's defaults
        # 1Hz is the update rate to TMC stall guard
        # Somewhere in this middle ground is the sweet spot no doubt.
        self._poll_rate = config.getfloat("poll_rate", default=4.0)
        self._clog_detected_gcode = config.get(
            "clog_detected_gcode", default=None
        )
        self._load_cell = None
        self._extruder = None
        self._toolhead = None
        self._tmc = None
        self._stall_mode = None
        self._steps_per_mm = None
        self._mcu = None
        self._stall_count = 0.0
        self._prev_pos = None
        self._prev_lost_steps = None
        self._clog_detected = False
        self._printer.register_event_handler("klippy:connect", self._on_connect)
        self._printer.register_event_handler("klippy:ready", self._on_ready)

    def _on_connect(self):
        self._load_cell = self._printer.lookup_object(self._load_cell_name)
        extruder = self._printer.lookup_object(self._extruder_name)
        if extruder.extruder_stepper is None:
            raise self._printer.config_error(
                "clog_detect: extruder '%s' has no stepper"
                % (self._extruder_name,)
            )
        self._extruder = extruder
        self._toolhead = self._printer.lookup_object("toolhead")
        stepper = extruder.extruder_stepper.stepper
        stepper_name = stepper.get_name()
        for name, obj in self._printer.lookup_objects():
            parts = name.split()
            if (
                len(parts) >= 2
                and parts[0].startswith("tmc")
                and " ".join(parts[1:]) == stepper_name
            ):
                self._tmc = obj
                break
        if self._tmc is None:
            raise self._printer.config_error(
                "clog_detect: no TMC driver found for extruder '%s'"
                % (stepper_name,)
            )
        if self._tmc.fields.lookup_register("lost_steps") is not None:
            self._stall_mode = "lost_steps"
        elif self._tmc.fields.lookup_register("sg_result") is not None:
            self._stall_mode = "sg_result"
            self._steps_per_mm = 1.0 / stepper.get_step_dist()
        else:
            raise self._printer.config_error(
                "clog_detect: the driver for extruder '%s' does not support"
                " stall detection" % (stepper_name,)
            )
        self._mcu = stepper.get_mcu()
        logging.info(
            "clog_detect: using %s mode for '%s'",
            self._stall_mode,
            stepper_name,
        )

    def _on_ready(self):
        reactor = self._printer.get_reactor()
        reactor.register_timer(
            self._poll, reactor.monotonic() + (1.0 / self._poll_rate)
        )

    def _poll(self, eventtime):
        interval = 1.0 / self._poll_rate
        if self._tmc is None:
            return eventtime + interval
        if self._toolhead.get_extruder() is not self._extruder:
            return eventtime + interval
        status = self._load_cell.get_status(eventtime)
        force_g = status.get("force_g")
        if force_g is None or force_g > -self._force_threshold:
            self._stall_count = 0.0
            return eventtime + interval
        if self._stall_mode == "lost_steps":
            val = self._tmc.mcu_tmc.get_register("LOST_STEPS")
            lost = self._tmc.fields.get_field("lost_steps", val)
            if self._prev_lost_steps is not None:
                delta = lost - self._prev_lost_steps
                if delta < 0:
                    delta += LOST_STEPS_MAX
                self._stall_count += delta
            self._prev_lost_steps = lost
        else:
            val = self._tmc.mcu_tmc.get_register("SG_RESULT")
            sg = self._tmc.fields.get_field("sg_result", val)
            print_time = self._mcu.estimated_print_time(eventtime)
            pos_now = self._extruder.find_past_position(print_time)
            if sg == 0 and self._prev_pos is not None:
                self._stall_count = max(
                    0.0,
                    self._stall_count
                    + (pos_now - self._prev_pos) * self._steps_per_mm,
                )
            self._prev_pos = pos_now
        if self._stall_count >= self._skipped_steps:
            self._trigger_clog(eventtime)
        return eventtime + interval

    def _trigger_clog(self, eventtime):
        logging.info("clog_detect: clog detected at %.3f", eventtime)
        self._printer.send_event("clog_detect:detected", eventtime)
        self._clog_detected = True
        self._stall_count = 0.0
        if self._clog_detected_gcode is not None:
            gcode = self._printer.lookup_object("gcode")
            reactor = self._printer.get_reactor()
            script = self._clog_detected_gcode

            def _run(et):
                gcode.run_script(script)
                self._clog_detected = False

            reactor.register_callback(_run)
        else:
            self._clog_detected = False

    def get_status(self, eventtime):
        return {
            "stall_count": self._stall_count,
            "clog_detected": self._clog_detected,
        }


def load_config_prefix(config):
    return ClogDetect(config)
