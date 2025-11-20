from __future__ import annotations

import typing

from klippy.gcode import CommandError

if typing.TYPE_CHECKING:
    from klippy.configfile import ConfigWrapper
    from klippy.extras.save_variables import SaveVariables
    from klippy.printer import Printer


class SaveVariablesWrapper:
    def __init__(self, printer: Printer, config: ConfigWrapper):
        self._save_variables: SaveVariables = printer.load_object(
            config, "save_variables"
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


__all__ = ("SaveVariablesWrapper",)
