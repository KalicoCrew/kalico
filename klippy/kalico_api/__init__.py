from __future__ import annotations

import contextlib
import importlib
import importlib.util
import pathlib
import sys
import types
import typing

from klippy import configfile
from klippy.configfile import ConfigWrapper
from klippy.extras.gcode_macro import (
    GCodeMacro,
)

from . import kalico
from .kalico.gcode_macro import Macro
from .kalico.loader import load_context

if typing.TYPE_CHECKING:
    from klippy.printer import Printer


# gcode_macro template shim
class MacroApiTemplate:
    def __init__(self, config: ConfigWrapper, macro: Macro):
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


sys.modules["kalico"] = kalico


@contextlib.contextmanager
def temporary_module(module: types.ModuleType):
    sys.modules[module.__name__] = module
    try:
        yield
    finally:
        for key in list(sys.modules):
            if key.startswith(f"{module.__name__}."):
                sys.modules.pop(key)
        del sys.modules[module.__name__]


class KalicoAPI:
    def __init__(self, config: ConfigWrapper):
        self.printer: Printer = config.get_printer()
        self._config = config
        self._root_path: pathlib.Path = self.printer.get_user_path()

        self.kalico = kalico.Kalico(self.printer)
        kalico.config = kalico.Configuration(config)
        load_context.set_loader(self)

        self.load()

    def load(self):
        module = types.ModuleType(
            "printer_config",
            "Virtual parent module for imported user configuration",
        )

        with temporary_module(module):
            files: list[str] = self._config.getlist("python", [], sep="\n")
            for file in files:
                if not file.strip():
                    continue
                self._load_file(module, file)

    def _load_file(self, module: types.ModuleType, filename: str):
        file: pathlib.Path = self._root_path / filename
        module_name = f"{module.__name__}.{file.name}"

        if file.is_dir() and (file / "__init__.py").is_file():
            file = file / "__init__.py"

        if not file.exists():
            raise configfile.error(
                f"Error loading python macros: {file} does not exist"
            )

        spec = importlib.util.spec_from_file_location(module_name, file)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module

        spec.loader.exec_module(module)

    def _register_macro(self, macro: Macro, rename_existing=None):
        section = f"gcode_macro {macro.name}"

        config = self._config.getsection(section)
        if not config.has_section(section):
            config.fileconfig.add_section(section)
            config.fileconfig.set(section, "gcode", macro._source)
            config.access_tracking[(section.lower(), "gcode")] = macro._source
            if macro.__doc__:
                config.fileconfig.set(section, "description", macro.__doc__)
                config.get(section, "description")
            if rename_existing:
                config.fileconfig.set(
                    section, "rename_existing", rename_existing
                )
                config.get(section, "rename_existing")

        template = MacroApiTemplate(config, macro)

        gcode_macro = self.printer.lookup_object(section, None)
        if gcode_macro:
            gcode_macro.template = template
        else:
            gcode_macro = GCodeMacro(config, template)
            self.printer.add_object(section, gcode_macro)


def add_printer_objects(config: ConfigWrapper):
    printer = config.get_printer()
    section = config.getsection("kalico_api")

    printer.add_object("kalico_api", KalicoAPI(section))
