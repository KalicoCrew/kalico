# Save arbitrary variables so that values can be kept across restarts.
#
# Copyright (C) 2020 Dushyant Ahuja <dusht.ahuja@gmail.com>
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import ast
import configparser
import logging
import pathlib
import typing

from klippy.gcode import CommandError

if typing.TYPE_CHECKING:
    from klippy.printer import Printer, SubsystemComponentCollection


class SaveVariables:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.filename = pathlib.Path(
            config.get(
                "filename", self.printer.get_user_path() / "user_variables.cfg"
            )
        ).expanduser()
        self.allVariables = {}
        try:
            if self.filename.exists():
                self.load_variables()
            else:
                self.allVariables = {}
        except self.printer.command_error as e:
            raise config.error(str(e))
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SAVE_VARIABLE",
            self.cmd_SAVE_VARIABLE,
            desc=self.cmd_SAVE_VARIABLE_help,
        )

    def load_variables(self):
        allvars = {}
        varfile = configparser.ConfigParser()
        try:
            varfile.read(self.filename)
            if varfile.has_section("Variables"):
                for name, val in varfile.items("Variables"):
                    allvars[name] = ast.literal_eval(val)
        except:
            msg = "Unable to parse existing variable file"
            logging.exception(msg)
            raise self.printer.command_error(msg)
        self.allVariables = allvars

    cmd_SAVE_VARIABLE_help = "Save arbitrary variables to disk"

    def cmd_SAVE_VARIABLE(self, gcmd):
        varname = gcmd.get("VARIABLE")
        value = gcmd.get("VALUE")
        try:
            value = ast.literal_eval(value)
        except ValueError as e:
            raise gcmd.error("Unable to parse '%s' as a literal" % (value,))

        self.set_variable(varname, value)

    def set_variable(self, varname, value):
        newvars = dict(self.allVariables)
        newvars[varname] = value
        # Write file
        varfile = configparser.ConfigParser()
        varfile.add_section("Variables")
        for name, val in sorted(newvars.items()):
            varfile.set("Variables", name, repr(val))
        try:
            f = open(self.filename, "w")
            varfile.write(f)
            f.close()
        except:
            msg = "Unable to save variable"
            logging.exception(msg)
            raise CommandError(msg)

        self.load_variables()

    def get_status(self, eventtime):
        return {"variables": self.allVariables}


def load_config(config):
    return SaveVariables(config)


## Kalico API


class SaveVariablesAPI:
    def __init__(self, printer: Printer):
        self._save_variables: SaveVariables = printer.lookup_object(
            "save_variables"
        )

    def __getitem__(self, name):
        if self._save_variables is None:
            raise CommandError("save_variables is not enabled")
        return self._save_variables.allVariables[name]

    def __setitem__(self, name, value):
        if self._save_variables is None:
            raise CommandError("save_variables is not enabled")
        self._save_variables.set_variable(name, value)

    def __contains__(self, name):
        return (
            self._save_variables and name in self._save_variables.allVariables
        )

    def __iter__(self):
        yield from iter(self._save_variables.allVariables)

    def items(self):
        return self._save_variables.allVariables.items()

    def get(self, name, default=None):
        if not self.__contains__(name):
            return default
        return self.__getitem__(name)


def register_components(subsystem: SubsystemComponentCollection):
    subsystem.register_component("kalico_api", "saved_vars", SaveVariablesAPI)
