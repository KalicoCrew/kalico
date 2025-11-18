from __future__ import annotations

import importlib.util
import pathlib
import sys
import typing

from klippy import configfile
from klippy.extras.gcode_macro import (
    GCodeMacro,
)

from . import kalico
from .kalico.gcode_macro import Macro
from .kalico.loader import load_context

if typing.TYPE_CHECKING:
    from klippy.printer import Printer

sys.modules["kalico"] = kalico


# gcode_macro template shim
class MacroApiTemplate:
    def __init__(self, config, macro: Macro):
        self.printer = config.get_printer()

        self.macro = macro
        self.name = macro.name
        self.params = macro._help

    def create_template_context(self):
        return {}  # Shim

    def run_gcode_from_command(self, context=None):
        return self.macro._call_from_context(context)

    def __call__(self, context=None):
        return self.run_gcode_from_command(context)


class MacroLoader:
    def __init__(self, config: configfile.ConfigWrapper):
        self.printer: Printer = config.get_printer()
        self._config = config
        self._root_path = pathlib.Path(
            self._config.printer.get_start_args()["config_file"]
        ).parent

        self.kalico = kalico.Kalico(self.printer)

        self.printer.register_event_handler("klippy:configured", self.load)

    # def _build_kalico_module(self):
    #     if "kalico" not in sys.modules:
    #         kalico_api = types.ModuleType(
    #             "kalico", "virtual module for the Kalico Python API"
    #         )
    #         for exposed in real_kalico.__all__:
    #             setattr(
    #                 kalico_api,
    #                 exposed,
    #                 getattr(real_kalico, exposed),
    #             )
    #         sys.modules["kalico"] = kalico_api

    #     return sys.modules["kalico"]

    def load(self):
        files = self._config.getlist("python", [], sep="\n")
        for file in files:
            if not file.strip():
                continue
            self._load_file(file)

    def _load_file(self, filename):
        file = self._root_path / filename

        if not file.exists():
            raise configfile.error(
                f"Error loading python macros: {file} does not exist"
            )

        spec = importlib.util.spec_from_file_location(file.name, file)
        module = importlib.util.module_from_spec(spec)

        load_context.set_loader(self)
        spec.loader.exec_module(module)

    def _register_macro(self, macro: Macro, rename_existing=None):
        section = f"gcode_macro {macro.name}"

        config = self._config.getsection(section)
        if not config.has_section(section):
            config.fileconfig.add_section(section)
            config.fileconfig.set(section, "description", macro.__doc__)
            config.fileconfig.set(
                section,
                "gcode",
                f"# {macro._source_file.relative_to(self._root_path)}:{macro.name}",
            )
            config.access_tracking[(section.lower(), "gcode")] = ""
            if rename_existing:
                config.fileconfig.set(
                    section, "rename_existing", rename_existing
                )

        template = MacroApiTemplate(config, macro)

        gcode_macro = self.printer.lookup_object(section, None)
        if gcode_macro:
            gcode_macro.template = template
        else:
            gcode_macro = GCodeMacro(config, template)
            self.printer.add_object(section, gcode_macro)


def load_config(config):
    return MacroLoader(config)
