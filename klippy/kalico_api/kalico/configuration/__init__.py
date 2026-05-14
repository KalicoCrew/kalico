from __future__ import annotations

import typing

from klippy.configfile import error as ConfigError
from klippy.configfile import sentinel

from ..loader import load_context

if typing.TYPE_CHECKING:
    from configparser import RawConfigParser

    from klippy.configfile import ConfigWrapper
    from klippy.printer import Printer

from ..types import Decorator, GCodeFunction, validate_gcode_function

Scalar = typing.Union[str, int, float, bool]
Value = typing.Union[Scalar, typing.List[Scalar], typing.Tuple[Scalar, ...]]


class Configuration:
    _finalized: bool = False

    def __init__(self, config: ConfigWrapper):
        self._printer: Printer = config.get_printer()
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
        self, section: str, name: str = None, /, **options: dict[str, Value]
    ) -> ConfigurationSection:
        config = self.get(section, name)
        config.update(options)
        return config

    def has(self, section: str):
        return self._config.has_section(section)

    __getitem__ = get
    __contains__ = has
    __call__ = define


class ConfigurationSection:
    def __init__(self, configuration: Configuration, config: ConfigWrapper):
        self._configuration = configuration
        self._printer: Printer = config.get_printer()
        self._config = config.getsection("printer")
        self._fileconfig: RawConfigParser = config.fileconfig
        self._section = config.get_name()

    def _is_used(self, option: str):
        tracking_id = (self._section.lower(), option.lower())
        return tracking_id in self._config.access_tracking

    def _create_section(self):
        if not self._fileconfig.has_section(self._section):
            self._fileconfig.add_section(self._section)

    def has(self, option: str):
        return self._config.has(option)

    def get(self, option: str, default: Scalar = sentinel):
        return self._config.get(option, default=default)

    def set(self, option: str, value: Scalar):
        if self._configuration._finalized:
            return

        self._create_section()

        if self._is_used(option):
            raise ConfigError(f"[{self._section}] {option!r} is already in use")

        if callable(value):
            self.gcode(option, value)

        else:
            self._fileconfig.set(self._section, option, str(value))

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
            if Configuration.__FINALIZE:
                return gcode_function

            try:
                validate_gcode_function(gcode_function)
            except ConfigError as e:
                raise ConfigError(
                    f"{self._section} {option} is not a valid gcode function, {e}"
                )

            function_key = f"{self._section.lower()}.{option}"
            gcode_macro = self._printer.load_object(self._config, "gcode_macro")

            gcode_macro._gcode_functions[function_key] = GCodeFunctionTemplate(
                self._config, gcode_function
            )
            self._fileconfig.set(
                self._section, option, f"!!gcode_function:{function_key}"
            )
            return gcode_function

        if function:
            return decorator(function)

        return decorator


class GCodeFunctionTemplate:
    def __init__(self, config, function: GCodeFunction):
        self.printer = config.get_printer()

        self.function = function
        self.name = self.function.__name__
        self.params = None

    def create_template_context(self):
        return {}  # Shim

    def run_gcode_from_command(self, context=None):
        return self.function(load_context.loader.kalico)

    def __call__(self, context=None):
        return self.run_gcode_from_command(context)
