# Synthetic endstop states for endstops on disconnected non-critical MCUs
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import re

from .danger_options import get_danger_options

OPEN_VALUES = ("open", "false", "0")
TRIGGERED_VALUES = ("triggered", "closed", "true", "1")

# List options: endstop names are given as values, because names reported
# by QUERY_ENDSTOPS may contain ':' (e.g. "mmu:filament_tension") or upper
# case letters, neither of which survives as a config option name. Every
# other option in the section is a single endstop ("<name>: <state>").
LIST_OPTIONS = (("open", 0), ("triggered", 1))


class NonCriticalEndstopDisconnectValues:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.section = config.get_name()
        self.values = {}
        list_options = [option for option, state in LIST_OPTIONS]
        for name in config.get_prefix_options(""):
            if name in list_options:
                continue
            raw = config.get(name).strip()
            if raw.lower() in OPEN_VALUES:
                self._add(config, name, 0)
            elif raw.lower() in TRIGGERED_VALUES:
                self._add(config, name, 1)
            elif ":" in raw:
                # "mmu:filament_tension: triggered" is read by the config
                # parser as option "mmu" with value "filament_tension: ..."
                rest, state = [p.strip() for p in raw.rsplit(":", 1)]
                if state.lower() in TRIGGERED_VALUES:
                    state = "triggered"
                else:
                    state = "open"
                raise config.error(
                    "Invalid value '%s' for endstop '%s' in section [%s]."
                    " Endstop names containing ':' must be listed in the"
                    " 'open' or 'triggered' option instead, for example"
                    " '%s: %s:%s'"
                    % (raw, name, self.section, state, name, rest)
                )
            else:
                raise config.error(
                    "Invalid value '%s' for endstop '%s' in section [%s];"
                    " must be one of 'open' or 'triggered'"
                    % (raw, name, self.section)
                )
        for option, state in LIST_OPTIONS:
            raw = config.get(option, "")
            for name in re.split(r"[,\n]", raw):
                name = name.strip()
                if name:
                    self._add(config, name, state)
        if not get_danger_options().ignore_non_critical_endstop_disconnect:
            logging.warning(
                "[%s] is configured but [danger_options]"
                " ignore_non_critical_endstop_disconnect is disabled;"
                " fallback values will not be used",
                self.section,
            )
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )

    def _handle_connect(self):
        query_endstops = self.printer.lookup_object("query_endstops", None)
        known = set()
        if query_endstops is not None:
            known = {name for _, name in query_endstops.endstops}
        unknown = sorted(set(self.values) - known)
        if unknown:
            raise self.printer.config_error(
                "Unknown endstop(s) in [non_critical_endstop_disconnect_values]:"
                " %s (known endstops, as reported by QUERY_ENDSTOPS: %s)"
                % (", ".join(unknown), ", ".join(sorted(known)) or "none")
            )

    def _add(self, config, name, state):
        if name in self.values and self.values[name] != state:
            raise config.error(
                "Endstop '%s' is configured as both 'open' and 'triggered'"
                " in section [%s]" % (name, self.section)
            )
        self.values[name] = state

    def has_value(self, name):
        return name in self.values

    def get_value(self, name):
        return self.values[name]


def load_config(config):
    return NonCriticalEndstopDisconnectValues(config)
