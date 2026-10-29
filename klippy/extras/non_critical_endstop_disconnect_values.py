# Synthetic endstop states for endstops on disconnected non-critical MCUs
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import re

from .danger_options import get_danger_options

# Config options of the section and the endstop state they stand for.
# Endstop names are given as values (not as option names), because names
# reported by QUERY_ENDSTOPS may contain ':' (e.g. "mmu:mmu_shared_exit")
# or upper case letters, neither of which survives as a config option name.
STATES = (("open", 0), ("triggered", 1))


class NonCriticalEndstopDisconnectValues:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.values = {}
        for option, state in STATES:
            raw = config.get(option, "")
            for name in re.split(r"[,\n]", raw):
                name = name.strip()
                if not name:
                    continue
                if name in self.values and self.values[name] != state:
                    raise config.error(
                        "Endstop '%s' is listed as both 'open' and"
                        " 'triggered' in section [%s]"
                        % (name, config.get_name())
                    )
                self.values[name] = state
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
                " %s (known endstops, as reported by QUERY_ENDSTOPS: %s)"
                % (", ".join(unknown), ", ".join(sorted(known)) or "none")
            )

    def has_value(self, name):
        return name in self.values

    def get_value(self, name):
        return self.values[name]


def load_config(config):
    return NonCriticalEndstopDisconnectValues(config)
