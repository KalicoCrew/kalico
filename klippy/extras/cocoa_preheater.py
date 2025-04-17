"""
Cocoa Press chocolate core preheater
"""

from __future__ import annotations

import enum
import copy
import typing
from typing import TypedDict, NotRequired

if typing.TYPE_CHECKING:
    from klippy.gcode import GCodeDispatch


module_name = "cocoa_preheater"


class PreheatState(str, enum.Enum):
    preheating = "preheating"
    paused = "paused"
    complete = "complete"
    cancelled = "cancelled"


class PreheatProfile(TypedDict):
    name: str
    body: float
    nozzle: float
    duration: int
    default: NotRequired[bool]


DEFAULT_PROFILES = [
    PreheatProfile(
        name="Milk Cocoa Core",
        body=30,
        nozzle=30,
        duration=1200,
        default=True,
    ),
    PreheatProfile(
        name="White Cocoa Core",
        body=30,
        nozzle=30,
        duration=1200,
        default=True,
    ),
    PreheatProfile(
        name="Dark Cocoa Core",
        body=33.0,
        nozzle=33.1,
        duration=1200,
        default=True,
    ),
]


class PreheatProfileManager:
    def __init__(self, config):
        self._config = config
        self._pconfig = config.get_printer().lookup_object("configfile")

        self.default_profiles = {
            default_profile["name"]: default_profile
            for default_profile in DEFAULT_PROFILES
        }
        self.profiles = copy.deepcopy(self.default_profiles)

        for section in config.get_prefix_sections(f"{module_name} "):
            name = section.get_name().split(" ", maxsplit=1)[-1]
            body = section.getfloat("body", above=0.0)
            nozzle = section.getfloat("nozzle", above=0.0)
            duration = section.getint("duration", minval=1)

            self.profiles[name] = PreheatProfile(
                name=name,
                body=body,
                nozzle=nozzle,
                duration=duration,
            )
            if name in self.default_profiles:
                self.profiles[name]["default"] = False

    def get_status(self):
        return self.profiles

    def get_profile(self, name: str) -> PreheatProfile:
        return self.profiles[name]

    def save_profile(
        self, name: str, body: float, nozzle: float, duration: int
    ):
        self.profiles[name] = PreheatProfile(
            name=name,
            body=body,
            nozzle=nozzle,
            duration=duration,
        )

        section = f"{module_name} {name}"
        self._pconfig.set(section, "body", f"{body:0.2f}")
        self._pconfig.set(section, "nozzle", f"{nozzle:0.2f}")
        self._pconfig.set(section, "duration", f"{duration}")

    def delete_profile(self, name):
        if name not in self.profiles:
            raise KeyError(f"Profile {name} does not exist")

        self._pconfig.remove_section(f"{module_name} {name}")

        if name in self.default_profiles:
            self.profiles[name] = self.default_profiles[name].copy()
        else:
            self.profiles.pop(name)


class CocoaPreheater:
    profile_manager: PreheatProfileManager
    time_remaining: float
    state: None | PreheatState
    profile: None | PreheatProfile
    gcode: GCodeDispatch

    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")

        self.printer.register_event_handler("klippy:ready", self._on_ready)
        self.printer.register_event_handler(
            "cocoa_toolhead:attached", self._on_attached
        )
        self.printer.register_event_handler(
            "cocoa_toolhead:detached", self._on_detached
        )

        self.profile_manager = PreheatProfileManager(config)
        self.profile = None
        self.time_remaining = 0.0
        self.state = None

        self._is_attached = None
        self._timer = None
        self._last_wake = None

    def get_status(self, eventtime):
        return {
            "status": self.state,
            "profile": self.profile,
            "time_remaining": round(self.time_remaining, 1),
            "profiles": self.profile_manager.get_status(),
        }

    def _on_ready(self):
        self.cocoa_toolhead = self.printer.lookup_object("cocoa_toolhead")

        self.gcode.register_command("PREHEATER_START", self.cmd_PREHEATER_START)
        self.gcode.register_command(
            "PREHEATER_SAVE_PROFILE", self.cmd_PREHEATER_SAVE_PROFILE
        )
        self.gcode.register_command(
            "PREHEATER_DELETE_PROFILE", self.cmd_PREHEATER_DELETE_PROFILE
        )

    def _on_attached(self):
        self._is_attached = True
        if self._timer is not None:
            self.state = PreheatState.preheating

    def _on_detached(self):
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

        self.profile = profile
        self.time_remaining = float(profile["duration"])
        self.state = PreheatState.preheating

        self.gcode.run_script_from_command(
            f'SET_HEATER_TEMPERATURE HEATER="{self.cocoa_toolhead.body_heater_name.split()[-1]}" TARGET={self.profile["body"]:0.2f}'
        )
        self.gcode.run_script_from_command(
            f'SET_HEATER_TEMPERATURE HEATER="{self.cocoa_toolhead.extruder_name.split()[-1]}" TARGET={self.profile["nozzle"]:0.2f}'
        )

        self._timer = reactor.register_timer(
            self._preheat_timer_callback,
            reactor.NOW,
        )

        self.gcode.register_command(
            "PREHEATER_CANCEL", self.cmd_PREHEATER_CANCEL
        )
        self.gcode.register_command("PREHEATER_WAIT", self.cmd_PREHEATER_WAIT)
        self.printer.send_event(
            "cocoa_preheater:start",
            self.profile,
        )

    def _stop_preheating(self, reason="cancel"):
        reactor = self.printer.get_reactor()

        self.duration = None

        if self._timer:
            reactor.unregister_timer(self._timer)
            self._timer = None

        if reason == "cancel":
            self.gcode.run_script_from_command(
                f'SET_HEATER_TEMPERATURE HEATER="{self.cocoa_toolhead.body_heater_name.split()[-1]}" TARGET=0'
            )
            self.gcode.run_script_from_command(
                f'SET_HEATER_TEMPERATURE HEATER="{self.cocoa_toolhead.extruder_name.split()[-1]}" TARGET=0'
            )

        if self.profile:
            self.printer.send_event(
                f"cocoa_preheater:{reason}",
                self.profile,
            )

        self.gcode.register_command("PREHEATER_CANCEL", None)
        self.gcode.register_command("PREHEATER_WAIT", None)

    def _is_preheating(self, eventtime) -> bool:
        return self._timer is not None and self.time_remaining > 0.0

    def cmd_PREHEATER_START(self, gcmd):
        """Preheat a chocolate core. `PREHEATER_START NAME=`"""

        name = gcmd.get("NAME")

        try:
            profile = self.profile_manager.get_profile(name)
        except KeyError:
            gcmd.respond_error(f"Preheat profile {name} not found")
        else:
            self._start_preheating(profile)

        if gcmd.get("WAIT", None):
            self.printer.wait_while(self._is_preheating)

    def cmd_PREHEATER_CANCEL(self, gcmd):
        """Cancel a current preheat action"""

        if not self.profile:
            gcmd.respond_error("No preheating in progress")

        else:
            self.state = PreheatState.cancelled
            self._stop_preheating("cancel")

    def cmd_PREHEATER_WAIT(self, gcmd):
        """Wait for a pending preheat to complete"""

        if self._timer is None:
            gcmd.respond_error("Not current preheating, unable to wait")
            return

        self.printer.wait_while(self._is_preheating)

    def cmd_PREHEATER_SAVE_PROFILE(self, gcmd):
        "Save a preheating profile. `PREHEATER_SAVE_PROFILE NAME= BODY= NOZZLE= DURATION=`"

        name = gcmd.get("NAME")
        body = gcmd.get_float("BODY", above=0.0)
        nozzle = gcmd.get_float("NOZZLE", above=0.0)
        duration = gcmd.get_int("DURATION", minval=1)

        self.profile_manager.save_profile(name, body, nozzle, duration)
        self.gcode.run_script_from_command("SAVE_CONFIG RESTART=0")
        gcmd.respond_info("Saved new profile")

    def cmd_PREHEATER_DELETE_PROFILE(self, gcmd):
        "Delete a saved profile. `PREHEATER_DELETE_PROFILE NAME=`"

        name = gcmd.get("NAME")

        try:
            self.profile_manager.delete_profile(name)

        except KeyError:
            gcmd.respond_error(f"Preheat profile {name} not found")

        else:
            self.gcode.run_script_from_command("SAVE_CONFIG RESTART=0")


def load_config(config):
    return CocoaPreheater(config)
