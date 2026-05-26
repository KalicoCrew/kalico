from __future__ import annotations

import inspect
import typing

from klippy.configfile import error as ConfigError
from klippy.configfile import sentinel

from ..loader import load_context

if typing.TYPE_CHECKING:
    from configparser import RawConfigParser

    from klippy.configfile import ConfigWrapper, PrinterConfig
    from klippy.printer import Printer

from ..types import Decorator, GCodeFunction, validate_gcode_function

Scalar = typing.Union[str, int, float, bool]
Value = typing.Union[Scalar, typing.List[Scalar], typing.Tuple[Scalar, ...]]


class Configuration:
    _finalized: bool = False

    def __init__(self, config: ConfigWrapper):
        self._printer: Printer = config.get_printer()
        self._pconfig: PrinterConfig = self._printer.lookup_object("configfile")
        self._config = config.getsection("printer")
        self._fileconfig: RawConfigParser = config.fileconfig

        self._printer.register_event_handler(
            "klippy:configured", self.__finalize
        )

    def __finalize(self, _):
        self._finalized = True

    def get(self, section: str, name: str = None) -> ConfigurationSection:
        if name:
            section = f"{section} {name}"
        return ConfigurationSection(self, self._config.getsection(section))

    def define(
        self,
        section: str,
        name: str = None,
        /,
        **options: dict[str, Value],
    ) -> ConfigurationSection:
        "Define a configuration section"
        config = self.get(section, name)
        config.update(options)
        return config

    def has(self, section: str):
        return self._config.has_section(section)

    def include(self, include):
        # escape hatch to include legacy .cfg files
        caller_filename = inspect.stack()[1].filename
        self._pconfig._resolve_include(
            caller_filename,
            include,
            self._fileconfig,
            visited=set(),
        )

    __getitem__ = get
    __contains__ = has
    __call__ = define


class ConfigurationSection:
    def __init__(self, configuration: Configuration, config: ConfigWrapper):
        self._configuration = configuration
        self._printer: Printer = config.get_printer()
        self._pconfig: PrinterConfig = self._printer.lookup_object("configfile")
        self._config: ConfigWrapper = config.getsection("printer")
        self._fileconfig: RawConfigParser = config.fileconfig

        self.section = config.get_name()
        self.name = self.section.split()[-1]

    def _is_used(self, option: str):
        tracking_id = (self.section.lower(), option.lower())
        return tracking_id in self._config.access_tracking

    def _create_section(self):
        if not self._fileconfig.has_section(self.section):
            self._fileconfig.add_section(self.section)

    def has(self, option: str):
        return self._config.has(option)

    def get(self, option: str, default: Scalar = sentinel):
        return self._config.get(option, default=default)

    def set(self, option: str, value: Scalar):
        if self._configuration._finalized:
            return

        self._create_section()

        if self._is_used(option):
            raise ConfigError(f"[{self.section}] {option!r} is already in use")

        if callable(value):
            self.gcode(option, value)

        else:
            self._fileconfig.set(self.section, option, str(value))
            if isinstance(value, (list, tuple)):
                acc_id = (self.section.lower(), option.lower())
                self._config.raw_values[acc_id] = value

    def update(self, options: dict[str, Scalar]):
        if self._configuration._finalized:
            return

        self._create_section()

        for option, value in options.items():
            self.set(option, value)

    @typing.overload
    def gcode(self, option: str) -> Decorator[GCodeFunction]: ...
    @typing.overload
    def gcode(self, option: str, function: GCodeFunction) -> GCodeFunction: ...

    def gcode(self, option: str, function: GCodeFunction = None):
        def decorator(gcode_function: GCodeFunction) -> GCodeFunction:
            if self._configuration._finalized:
                return gcode_function

            try:
                validate_gcode_function(gcode_function)
            except ConfigError as e:
                raise ConfigError(
                    f"{self.section} {option} is not a valid gcode function, {e}"
                )

            gcode = self._printer.lookup_object("gcode")
            gcode_macro = self._printer.load_object(self._config, "gcode_macro")

            function_key = f"{self.section.lower()}.{option}"
            template = GCodeFunctionTemplate(
                self._config,
                function_key,
                gcode_function,
            )

            gcode_macro._gcode_functions[function_key] = template
            gcode.register_mux_command(
                "CALL_GCODE_FUNCTION",
                "KEY",
                function_key,
                template,
            )
            self._fileconfig.set(
                self.section, option, f"!!gcode_function:{function_key}"
            )
            return gcode_function

        if function:
            return decorator(function)

        return decorator


class GCodeFunctionTemplate:
    def __init__(self, config, key: str, function: GCodeFunction):
        self.printer = config.get_printer()

        self.key = key
        self.function = function
        self.name = self.function.__name__
        self.params = None

    def create_template_context(self):
        return {}  # Shim

    def run_gcode_from_command(self, context=None):
        return self.function(load_context.loader.kalico)

    def __call__(self, context=None):
        return self.run_gcode_from_command(context)

    def render(self, context=None):
        return f"CALL_GCODE_FUNCTION KEY='{self.key}'"
