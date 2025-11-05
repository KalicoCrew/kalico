"""
Global preheating profile manager
"""

from __future__ import annotations

import copy
import typing
from typing import NotRequired, TypedDict

if typing.TYPE_CHECKING:
    from klippy.configfile import ConfigWrapper, PrinterConfig
    from klippy.gcode import GCodeCommand, GCodeDispatch
    from klippy.printer import Printer


class PreheatProfile(TypedDict):
    name: str
    body: float
    nozzle: float
    duration: int
    default: NotRequired[bool]


class PreheatProfileManager:
    config: ConfigWrapper
    pconfig: PrinterConfig
    gcode: GCodeDispatch
    printer: Printer

    def __init__(self, config: ConfigWrapper):
        self.config = config
        self.printer = self.config.get_printer()
        self.pconfig = self.printer.lookup_object("configfile")
        self.gcode = self.printer.lookup_object("gcode")

        self._profile_status_cache = None

        self.default_profiles = {}
        for section in config.get_prefix_sections("cocoa_preheater_default "):
            name = section.get_name().split(" ", maxsplit=1)[-1]
            body = section.getfloat("body", above=0.0)
            nozzle = section.getfloat("nozzle", above=0.0)
            duration = section.getint("duration", minval=1)

            self.default_profiles[name] = PreheatProfile(
                name=name,
                body=body,
                nozzle=nozzle,
                duration=duration,
                default=True,
            )

        self.profiles = copy.deepcopy(self.default_profiles)
        for section in config.get_prefix_sections("cocoa_preheater "):
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

        self.gcode.register_command(
            "PREHEATER_SAVE_PROFILE", self.cmd_PREHEATER_SAVE_PROFILE
        )
        self.gcode.register_command(
            "PREHEATER_DELETE_PROFILE", self.cmd_PREHEATER_DELETE_PROFILE
        )

    def get_status(self, eventtime):
        if self._profile_status_cache is None:
            self._profile_status_cache = copy.deepcopy(self.profiles)
        return self._profile_status_cache

    def get_profile(self, name: str) -> PreheatProfile:
        return self.profiles[name]

    def save_profile(
        self,
        name: str,
        body: float,
        nozzle: float,
        duration: int,
    ):
        self.profiles[name] = PreheatProfile(
            name=name,
            body=body,
            nozzle=nozzle,
            duration=duration,
        )
        if name in self.default_profiles:
            self.profiles[name]["default"] = False

        section = f"cocoa_preheater {name}"
        self.pconfig.set(section, "body", f"{body:0.2f}")
        self.pconfig.set(section, "nozzle", f"{nozzle:0.2f}")
        self.pconfig.set(section, "duration", f"{duration}")

        # Invalidate cache after updating self.profiles
        self._profile_status_cache = None

    def delete_profile(self, name):
        if name not in self.profiles:
            raise KeyError(f"Profile {name} does not exist")

        self.pconfig.remove_section(f"cocoa_preheater {name}")

        if name in self.default_profiles:
            self.profiles[name] = self.default_profiles[name].copy()
        else:
            self.profiles.pop(name)

        # Invalidate cache after updating self.profiles
        self._profile_status_cache = None

    def cmd_PREHEATER_SAVE_PROFILE(self, gcmd: GCodeCommand):
        "Save a preheating profile. `PREHEATER_SAVE_PROFILE NAME= BODY= NOZZLE= DURATION=`"

        name = gcmd.get("NAME")
        body = gcmd.get_float("BODY", above=0.0)
        nozzle = gcmd.get_float("NOZZLE", above=0.0)
        duration = gcmd.get_int("DURATION", minval=1)

        self.save_profile(name, body, nozzle, duration)
        self.gcode.run_script_from_command("SAVE_CONFIG RESTART=0")

        gcmd.respond_info("Saved new profile")

    def cmd_PREHEATER_DELETE_PROFILE(self, gcmd: GCodeCommand):
        "Delete a saved profile. `PREHEATER_DELETE_PROFILE NAME=`"

        name = gcmd.get("NAME")

        try:
            self.delete_profile(name)
        except KeyError:
            raise gcmd.error(f"Preheat profile {name} not found")

        self.gcode.run_script_from_command("SAVE_CONFIG RESTART=0")
