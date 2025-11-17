from __future__ import annotations

import importlib.util
import pathlib
import sys
import types
import typing

from klippy import configfile
from klippy.extras.gcode_macro import (
    GCodeMacro,
)

from .kalico import Kalico
from .macro import Macro, MacroFunction


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
        self.printer = config.get_printer()
        self._config = config
        self._root_path = pathlib.Path(
            self._config.printer.get_start_args()["config_file"]
        ).parent

        self.kalico = Kalico(self.printer)
        self._build_kalico_module()
        self.load()

    def _macro_decorator(
        self,
        func: typing.Optional[MacroFunction],
        /,
        rename_existing: typing.Optional[str] = None,
    ) -> typing.Callable[[MacroFunction], Macro]:
        def macro_decorator(func: MacroFunction) -> Macro:
            wrapped_macro = Macro(self, func)
            self._register_macro(wrapped_macro, rename_existing)
            return wrapped_macro

        if func is not None:
            return macro_decorator(func)

        return macro_decorator

    def get_context(self):
        return {
            "config": self._config,
            "gcode_macro": self._macro_decorator,
        }

    def _build_kalico_module(self):
        if "kalico" not in sys.modules:
            kalico = types.ModuleType(
                "kalico", "virtual module for the Kalico Python API"
            )
            kalico.Kalico = Kalico
            sys.modules["kalico"] = kalico

        return sys.modules["kalico"]

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
        kalico = self._build_kalico_module()

        # TODO: change these to be scoped, using something like "kalico.gcode_macro.loader(self)"
        for k, v in self.get_context().items():
            setattr(kalico, k, v)
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
