# Synthetic endstop states for endstops on disconnected non-critical MCUs
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from .danger_options import get_danger_options

OPEN_VALUES = ("open", "false", "0")
TRIGGERED_VALUES = ("triggered", "closed", "true", "1")


class NonCriticalEndstopDisconnectValues:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.values = {}
        for name in config.get_prefix_options(""):
            raw = config.get(name).strip().lower()
            if raw in OPEN_VALUES:
                self.values[name] = 0
            elif raw in TRIGGERED_VALUES:
                self.values[name] = 1
            else:
                raise config.error(
                    "Invalid value '%s' for endstop '%s' in section [%s];"
                    " must be one of 'open' or 'triggered'"
                    % (raw, name, config.get_name())
                )
        if not get_danger_options().ignore_non_critical_endstop_disconnect:
            logging.warning(
                "[%s] is configured but [danger_options]"
                " ignore_non_critical_endstop_disconnect is disabled;"
                " fallback values will not be used",
                config.get_name(),
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
                " %s" % (", ".join(unknown),)
            )

    def has_value(self, name):
        return name in self.values

    def get_value(self, name):
        return self.values[name]


def load_config(config):
    return NonCriticalEndstopDisconnectValues(config)
