# Clog Detection
#
# Copyright (C) 2026 Ella Fox <ella@fox.gal>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

# Detects clogged nozzles using two concurrent signals:
# 1. The extruder TMC driver's DIAG pin asserts when the motor stalls
#    (sg_result ≤ 2 × SGTHRS, driven by hardware in real time)
# 2. The load cell reads downward force above the configured threshold
#
# When both conditions are met simultaneously, a clog is detected.
# The DIAG pin is inferred from the diag_pin setting in the extruder's TMC section.

_TMC_PREFIXES = (
    "tmc2209",
    "tmc2208",
    "tmc2240",
    "tmc5160",
    "tmc2130",
    "tmc2660",
    "tmc2262",
)


class ClogDetect:
    def __init__(self, config):
        self._printer = config.get_printer()
        self._load_cell_name = config.get(
            "load_cell", default="load_cell_probe"
        )
        self._extruder_name = config.get("extruder", default="extruder")
        self._force_threshold = config.getfloat("force", default=4000.0)
        self._clog_detected_gcode = config.get(
            "clog_detected_gcode", default=None
        )
        self._load_cell = None
        self._extruder = None
        self._toolhead = None
        self._clog_detected = False
        self._enabled = True
        self._tared = False
        # Find diag_pin from the extruder's TMC section at config time
        diag_pin = None
        tmc_name = None
        for prefix in _TMC_PREFIXES:
            section = "%s %s" % (prefix, self._extruder_name)
            if config.has_section(section):
                tmc_cfg = config.getsection(section)
                diag_pin = tmc_cfg.get("diag_pin", None)
                if diag_pin is not None:
                    tmc_name = section
                    break
        if diag_pin is None:
            raise config.error(
                "clog_detect: no diag_pin found in any TMC section for"
                " extruder '%s'. Configure diag_pin in the extruder's"
                " TMC section." % (self._extruder_name,)
            )
        buttons = self._printer.load_object(config, "buttons")
        buttons.register_button_push(diag_pin, self._on_diag_edge)
        logging.info(
            "clog_detect: armed on diag_pin '%s' from '%s'",
            diag_pin,
            tmc_name,
        )
        if self._printer.lookup_object("clog_detect_commands", None) is None:
            self._printer.add_object(
                "clog_detect_commands", ClogDetectCommands(self._printer)
            )
        self._printer.register_event_handler("klippy:connect", self._on_connect)
        self._printer.register_event_handler("load_cell:tare", self._on_tare)

    def _on_connect(self):
        self._load_cell = self._printer.lookup_object(self._load_cell_name)
        extruder = self._printer.lookup_object(self._extruder_name)
        if extruder.extruder_stepper is None:
            # We should never hit this.
            raise self._printer.config_error(
                "clog_detect: extruder '%s' has no stepper"
                % (self._extruder_name,)
            )
        self._extruder = extruder
        self._toolhead = self._printer.lookup_object("toolhead")

    def _on_tare(self, load_cell):
        if not self._tared:
            logging.info("clog_detect: tare received, detection armed")
            self._tared = True

    def _on_diag_edge(self, eventtime):
        if not self._tared or not self._enabled or self._clog_detected:
            return
        if self._toolhead.get_extruder() is not self._extruder:
            return
        force_g = self._load_cell.get_status(eventtime).get("force_g")
        if force_g is not None and force_g <= -self._force_threshold:
            self._trigger_clog(eventtime)

    def _trigger_clog(self, eventtime):
        logging.info("clog_detect: clog detected at %.3f", eventtime)
        self._printer.send_event("clog_detect:detected", eventtime)
        self._clog_detected = True
        if self._clog_detected_gcode is not None:
            gcode = self._printer.lookup_object("gcode")
            reactor = self._printer.get_reactor()
            script = self._clog_detected_gcode

            def _run(et):
                gcode.run_script(script)

            reactor.register_callback(_run)

    def reset(self):
        self._clog_detected = False

    def set_enabled(self, enabled):
        self._enabled = enabled

    def get_status(self, eventtime):
        return {
            "enabled": self._enabled,
            "clog_detected": self._clog_detected,
        }


def load_config(config):
    return ClogDetect(config)


def load_config_prefix(config):
    return ClogDetect(config)


class ClogDetectCommands:
    def __init__(self, printer):
        self._printer = printer
        gcode = printer.lookup_object("gcode")
        gcode.register_command(
            "CLOG_DETECTION",
            self._cmd_clog_detection,
            desc="Control clog detection: NAME={name} (optional),"
            " ENABLED=true/false, RESET=1",
        )

    def _resolve(self, gcmd):
        name = gcmd.get("NAME", None)
        instances = dict(self._printer.lookup_objects("clog_detect"))
        if name is None:
            if len(instances) == 1:
                return list(instances.values())
            raise gcmd.error(
                "NAME required: %s"
                % ", ".join(n.split()[-1] for n in instances)
            )
        instance = instances.get("clog_detect " + name)
        if instance is None:
            raise gcmd.error("Unknown clog_detect '%s'" % (name,))
        return [instance]

    def _cmd_clog_detection(self, gcmd):
        targets = self._resolve(gcmd)
        if gcmd.get_int("RESET", 0):
            for t in targets:
                t.reset()
            gcmd.respond_info("Clog detection reset")
        enabled_str = gcmd.get("ENABLED", None)
        if enabled_str is not None:
            enabled = enabled_str.lower() in ("1", "true", "yes")
            for t in targets:
                t.set_enabled(enabled)
            gcmd.respond_info(
                "Clog detection %s" % ("enabled" if enabled else "disabled")
            )
