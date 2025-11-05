"""
Cocoa Press chocolate core preheater
"""

from __future__ import annotations

import enum
import typing

from .profiles import PreheatProfile, PreheatProfileManager

if typing.TYPE_CHECKING:
    from ...configfile import ConfigWrapper
    from ...gcode import GCodeCommand, GCodeDispatch
    from ...printer import Printer
    from .toolhead import CocoaToolheadControl


class PreheatState(str, enum.Enum):
    preheating = "preheating"
    paused = "paused"
    complete = "complete"
    stopped = "stopped"
    cancelled = "cancelled"


class CocoaPreheater:
    name: str
    cocoa_toolhead: CocoaToolheadControl
    printer: Printer
    gcode: GCodeDispatch
    profile_manager: PreheatProfileManager

    time_remaining: float
    state: None | PreheatState
    profile: None | PreheatProfile

    def __init__(
        self,
        cocoa_toolhead: CocoaToolheadControl,
        config: ConfigWrapper,
    ):
        self.cocoa_toolhead = cocoa_toolhead
        self.name = self.cocoa_toolhead.name
        self.mux_name = self.cocoa_toolhead.mux_name
        self.logger = cocoa_toolhead.logger.getChild("preheater")

        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.profile_manager = self.printer.lookup_object(
            "cocoa_preheater_profiles", None
        )
        if self.profile_manager is None:
            self.profile_manager = PreheatProfileManager(config)
            self.printer.add_object(
                "cocoa_preheater_profiles", self.profile_manager
            )

        # self.printer.register_event_handler("klippy:ready", self._on_ready)
        self.printer.register_event_handler(
            "cocoa_toolhead:attached", self._on_attached
        )
        self.printer.register_event_handler(
            "cocoa_toolhead:detached", self._on_detached
        )

        self.profile = None
        self.time_remaining = 0.0
        self.state = None

        self._is_attached = None
        self._timer = None
        self._last_wake = None

        self.logger.info(f"[cocoa_preheater] {self.name=} {self.mux_name=}")
        self.gcode.register_mux_command(
            "PREHEATER_START", "TOOL", self.mux_name, self.cmd_PREHEATER_START
        )

        # Shim for old Cocoa Screen
        if self.name == "cocoa_toolhead":
            self.printer.add_object("cocoa_preheater", self)

    def get_status(self, eventtime):
        return {
            "status": self.state,
            "profile": self.profile,
            "time_remaining": round(self.time_remaining, 1),
            "profiles": self.profile_manager.get_status(eventtime),
        }

    def _on_attached(self, _name):
        self._is_attached = True
        if self._timer is not None:
            self.state = PreheatState.preheating

    def _on_detached(self, _name):
        self._is_attached = False
        if self._timer is not None:
            self.state = PreheatState.paused

    def _preheat_timer_callback(self, eventtime):
        reactor = self.printer.get_reactor()

        if self._is_attached and self._last_wake is not None:
            self.time_remaining = max(
                self.time_remaining - (eventtime - self._last_wake),
                0.0,
            )

        self._last_wake = eventtime

        if self.time_remaining > 0.0:
            return eventtime + 1.0  # Wake again in 1 second

        else:
            self.state = PreheatState.complete
            self._stop_preheating("complete")
            if self.profile:
                self.gcode.respond_info(
                    f"Cocoa Press: Preheating {self.profile['name']} complete"
                )
            return reactor.NEVER

    def _start_preheating(self, profile: PreheatProfile):
        reactor = self.printer.get_reactor()

        if self._timer:
            # Currently preheating, cancel that and start a new preheat
            self._stop_preheating()

        self.profile = profile.copy()
        self.time_remaining = float(profile["duration"])
        self.state = PreheatState.preheating

        self.gcode.run_script_from_command(
            f'SET_HEATER_TEMPERATURE HEATER="{self.cocoa_toolhead.body_heater_name.split()[-1]}" TARGET={self.profile["body"]:0.2f}'
        )
        self.gcode.run_script_from_command(
            f'SET_HEATER_TEMPERATURE HEATER="{self.cocoa_toolhead.extruder_name.split()[-1]}" TARGET={self.profile["nozzle"]:0.2f}'
        )

        self._last_wake = None
        self._timer = reactor.register_timer(
            self._preheat_timer_callback,
            reactor.NOW,
        )

        self.gcode.register_mux_command(
            "PREHEATER_STOP", "TOOL", self.mux_name, self.cmd_PREHEATER_STOP
        )
        self.gcode.register_mux_command(
            "PREHEATER_WAIT", "TOOL", self.mux_name, self.cmd_PREHEATER_WAIT
        )
        if "PREHEATER_CANCEL" not in self.gcode.ready_gcode_handlers:
            self.gcode.register_mux_command(
                "PREHEATER_CANCEL",
                "TOOL",
                self.mux_name,
                self.cmd_PREHEATER_CANCEL,
            )

        self.printer.send_event("cocoa_preheater:start", self.profile)

    def _stop_preheating(self, reason="cancel"):
        reactor = self.printer.get_reactor()

        self.duration = None
        self._last_wake = None

        if self._timer:
            reactor.unregister_timer(self._timer)
            self._timer = None

        self.gcode.register_mux_command(
            "PREHEATER_STOP", "TOOL", self.mux_name, None
        )
        self.gcode.register_mux_command(
            "PREHEATER_WAIT", "TOOL", self.mux_name, None
        )

        if reason == "cancel":
            self.gcode.run_script_from_command(
                f'SET_HEATER_TEMPERATURE HEATER="{self.cocoa_toolhead.body_heater_name.split()[-1]}" TARGET=0'
            )
            self.gcode.run_script_from_command(
                f'SET_HEATER_TEMPERATURE HEATER="{self.cocoa_toolhead.extruder_name.split()[-1]}" TARGET=0'
            )
            self.gcode.register_mux_command(
                "PREHEATER_CANCEL", "TOOL", self.mux_name, None
            )

        self.printer.send_event(
            "cocoa_preheater:stop",
            reason,
            self.profile,
        )

    def _is_preheating(self, eventtime) -> bool:
        return self._timer is not None and self.time_remaining > 0.0

    def cmd_PREHEATER_START(self, gcmd):
        """Preheat a chocolate core. `PREHEATER_START NAME=`"""

        name = gcmd.get("NAME")

        try:
            profile = self.profile_manager.get_profile(name)
        except KeyError:
            raise gcmd.error(f"Preheat profile {name} not found")

        self._start_preheating(profile)

        if gcmd.get("WAIT", None):
            self.printer.wait_while(self._is_preheating)

    def cmd_PREHEATER_STOP(self, gcmd: GCodeCommand):
        """Stop a preheat action, leaving the heaters at temperature."""

        if not self._timer:
            raise gcmd.error("No preheating in progress")

        self.state = PreheatState.stopped
        self._stop_preheating("stop")

    def cmd_PREHEATER_CANCEL(self, gcmd: GCodeCommand):
        """Cancel a current preheat action"""

        self.state = PreheatState.cancelled
        self._stop_preheating("cancel")

    def cmd_PREHEATER_WAIT(self, gcmd: GCodeCommand):
        """Wait for a pending preheat to complete"""

        if not self._timer:
            raise gcmd.error("Not currently preheating, unable to wait")

        self.printer.wait_while(self._is_preheating)
