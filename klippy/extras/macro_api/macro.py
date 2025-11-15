from __future__ import annotations

import enum
import inspect
import json
import logging
import pathlib
import traceback
import typing

from klippy import configfile, util
from klippy.extras.gcode_macro import TemplateVariableWrapperPython
from klippy.gcode import CommandError

if typing.TYPE_CHECKING:
    from klippy.printer import Printer
    from klippy.reactor import SelectReactor

    from . import MacroLoader

from .kalico import Kalico


def _is_param_converter(func):
    "Is this function callable with a single string argument"
    if not callable(func):
        return False
    sig = inspect.signature(func)
    return len(sig.parameters) == 1


def _validate_parameters(func: typing.Callable):
    sig = inspect.signature(func)
    source_file = pathlib.Path(inspect.getfile(func))

    if not sig.parameters:
        raise configfile.error(
            f"Error when loading macro {source_file}:{func.__name__}."
            " Macro functions must accept a parameter for the Printer object"
        )

    param_kalico, *parameters = sig.parameters.values()
    if param_kalico.annotation != Kalico:
        raise configfile.error(
            f"Error when loading macro {source_file}:{func.__name__}."
            " First parameter must be of type Kalico"
        )

    for parameter in parameters:
        if parameter.annotation not in (
            parameter.empty,
            str,
            int,
            bool,
            float,
        ) and not _is_param_converter(parameter.annotation):
            raise configfile.error(
                f"Error when loading macro {source_file}:{func.__name__}."
                f" Parameter '{parameter.name}: {parameter.annotation}' may only be str, int, float, or bool"
            )

    return parameters


def _document_parameters(parameters: list[inspect.Parameter]):
    help = {}

    for parameter in parameters:
        doc = help.setdefault(parameter.name.upper(), {})
        if parameter.default is parameter.empty:
            doc["required"] = True

        if issubclass(parameter.annotation, enum.Enum):
            doc["type"] = "enum"
            doc["choices"] = [e.value for e in parameter.annotation]
            if parameter.default is not None:
                doc["default"] = parameter.default.value
        else:
            doc["type"] = parameter.annotation.__name__
            if parameter.default is not None:
                doc["default"] = parameter.default

    return help


MacroParams = typing.ParamSpec("MacroParams")
MacroReturn = typing.TypeVar("MacroReturn")
MacroFunction = typing.Callable[
    typing.Concatenate["Kalico", MacroParams], MacroReturn
]


class Macro(typing.Generic[MacroParams, MacroReturn]):
    __context: list[dict]
    __printer: Printer

    def __init__(
        self,
        loader: MacroLoader,
        func: typing.Callable[
            typing.Concatenate[Kalico, MacroParams], MacroReturn
        ],
    ):
        self.__printer = loader.printer
        self.__kalico = loader.kalico
        self.__func = func

        self.name = func.__name__.upper()
        self._signature = inspect.signature(func)
        self._parameters = _validate_parameters(func)
        self._help = _document_parameters(self._parameters)
        self._source_file = pathlib.Path(inspect.getsourcefile(func))

        self.__doc__ = func.__doc__
        self.__context = []
        self.__vars = None

    @property
    def vars(self) -> TemplateVariableWrapperPython:
        if not self.__vars:
            gcode_macro = self.__printer.lookup_object(
                f"gcode_macro {self.name}"
            )
            self.__vars = TemplateVariableWrapperPython(gcode_macro)
        return self.__vars

    @property
    def raw_params(self) -> str:
        if not self.__context:
            return ""
        return self.__context[-1].get("rawparams", "")

    @property
    def params(self) -> dict[str, str]:
        if not self.__context:
            return {}
        return self.__context[-1].get("params", {})

    def __bind(self, *args, **kwargs):
        return self._signature.bind(self.__kalico, *args, **kwargs)

    def _call_from_context(self, context: dict):
        kwargs = {}
        params = context.get("params", {})

        for paramspec in self._parameters:
            param_name = paramspec.name.upper()

            if param_name not in params:
                if paramspec.default is paramspec.empty:
                    raise self.__printer.command_error(
                        f"{self.name} requires {param_name}\n"
                        + json.dumps(
                            self._parameters.get(
                                param_name, "Unknown parameter"
                            )
                        )
                    )
                else:
                    continue

            value = params[param_name]

            # Special case for boolean values
            if paramspec.annotation is bool:
                value = bool(int(value))

            elif callable(paramspec.annotation):
                value = paramspec.annotation(value)

            kwargs[paramspec.name] = value

        bound = self.__bind(**kwargs)
        return self.__call__(*bound.args, **bound.kwargs)

    def __call__(
        self,
        kalico: Kalico,
        *args: MacroParams.args,
        **kwargs: MacroParams.kwargs,
    ) -> MacroReturn:
        try:
            return self.__func(kalico, *args, **kwargs)

        except Exception as e:
            tbe = traceback.TracebackException.from_exception(
                e, capture_locals=True
            )
            # Drop the frame for the macro wrapper
            tbe.stack.pop(0)
            # Drop locals for internal frames
            for frame in tbe.stack:
                if frame.filename.startswith(str(util.klippy_dir)):
                    frame.locals = None
            lines = list(tbe.format())
            logging.error(f"Error in {self.name}: {e}\n" + "".join(lines))
            raise CommandError(
                f"Error in {self.name}: {e}\n{lines[1]}", log=False
            )

    def delay(
        self,
        delay: float,
        /,
        *args: MacroParams.args,
        **kwargs: MacroParams.kwargs,
    ) -> Timer:
        "Schedule the function to run after a delay"
        bound = self.__bind(*args, **kwargs)
        return Timer(self.__printer, self, bound, delay)

    def every(
        self,
        period: float,
        /,
        *args: MacroParams.args,
        **kwargs: MacroParams.kwargs,
    ) -> Timer:
        "Schedule the macro to run every `period` seconds"
        if period < 0.1:
            raise ValueError(
                "Recurring timers must have a delay greater than 0.1"
            )
        bound = self.__bind(*args, **kwargs)
        return Timer(self.__printer, self, bound, period, repeat=True)


class Timer(typing.Generic[MacroParams, MacroReturn]):
    __reactor: SelectReactor

    def __init__(
        self,
        printer: Printer,
        macro: Macro[MacroParams, MacroReturn],
        bound: inspect.BoundArguments,
        delay: float,
        repeat=False,
    ):
        self.__macro = macro
        self.__bound = bound
        self.__reactor = printer.get_reactor()
        self.__delay = abs(delay)
        self.__repeat = repeat

        waketime = self.__reactor.monotonic() + abs(delay)
        self.__timer = self.__reactor.register_timer(self.__callback, waketime)

    def __callback(self, eventtime):
        try:
            self.__macro(*self.__bound.args, **self.__bound.kwargs)

        finally:
            if self.__repeat:
                return self.__reactor.monotonic() + self.__delay

            self.cancel()
            return self.__reactor.NEVER

    @property
    def is_pending(self) -> bool:
        return self.__timer is not None

    def cancel(self):
        if self.__timer is not None:
            self.__reactor.unregister_timer(self.__timer)
            self.__timer = None
