from __future__ import annotations

import enum
import inspect
import json
import logging
import pathlib
import traceback
import typing

from typing_extensions import Concatenate, ParamSpec

from klippy import configfile, util
from klippy.extras.gcode_macro import (
    TemplateVariableWrapperPython,
)
from klippy.gcode import CommandError

from .context import Kalico
from .loader import load_context

try:
    from typing import Concatenate, ParamSpec
except:
    from typing_extensions import Concatenate, ParamSpec

if typing.TYPE_CHECKING:
    from klippy.printer import Printer

    from .. import MacroLoader


MacroParams = ParamSpec("MacroParams")
MacroReturn = typing.TypeVar("MacroReturn")


class MacroFunction(typing.Generic[MacroParams, MacroReturn], typing.Protocol):
    def __call__(
        self,
        kalico: Kalico,
        *args: MacroParams.args,
        **kwds: MacroParams.kwargs,
    ) -> MacroReturn: ...


def _is_param_converter(func):
    "Is this function callable with a single string argument"
    if typing.get_origin(func) == typing.Annotated:
        return _is_param_converter(typing.get_args(func)[0])
    if func in (inspect._empty, str, int, bool, float):
        return True
    if not callable(func):
        return False
    sig = inspect.signature(func)
    if not sig.parameters:
        return False
    param, *other_params = list(sig.parameters.values())
    # First parameter must be untyped or str
    if param.annotation not in (sig.empty, str):
        return False
    # Any others must be optional
    if not all(p.default is not sig.empty for p in other_params):
        return False
    return True


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
        if not _is_param_converter(parameter.annotation):
            raise configfile.error(
                f"Error when loading macro {source_file}:{func.__name__}."
                f" Parameter '{parameter.name}: {parameter.annotation}' may only be str, int, float, or bool"
            )

        if parameter.name.startswith("_"):
            if parameter.name.lstrip("_") in sig.parameters:
                raise configfile.error(
                    f"Error when loading macro {source_file}:{func.__name__}."
                    f" Hidden parameter {parameter.name} conflicts with {parameter.name.lstrip('_')}"
                )

    return parameters


def _document_parameters(parameters: list[inspect.Parameter]):
    help = {}

    for parameter in parameters:
        if parameter.name.startswith("_"):
            continue

        doc = help.setdefault(parameter.name.upper(), {})
        if parameter.default is parameter.empty:
            doc["required"] = True

        param_type = parameter.annotation or str
        validators = []

        if typing.get_origin(param_type) == typing.Annotated:
            param_type, *validators = typing.get_args(param_type)

        if inspect.isclass(param_type) and issubclass(param_type, enum.Enum):
            doc["type"] = "enum"
            doc["choices"] = [e.value for e in param_type]

            if parameter.default is not None:
                assert isinstance(parameter.default, param_type)
                doc["default"] = parameter.default.value
            continue

        if param_type in (str, float, int, bool):
            doc["type"] = param_type.__name__

        elif callable(param_type):
            sig = inspect.signature(param_type)
            if sig.return_annotation is not sig.empty:
                doc["type"] = sig.return_annotation.__name__

        else:
            doc["type"] = param_type.__name__

        if parameter.default is not None:
            doc["default"] = parameter.default

        if validators:
            doc["valid"] = {}

            for validator in validators:
                if hasattr(validator, "description"):
                    doc["valid"].update(validator.description)
                elif validator.__doc__:
                    doc["valid"].setdefault("custom", []).append(
                        validator.__doc__
                    )

    return help


class Macro(typing.Generic[MacroParams, MacroReturn]):
    __context: list[dict]
    __printer: Printer

    name: str

    def __init__(
        self,
        loader: MacroLoader,
        func: typing.Callable[Concatenate[Kalico, MacroParams], MacroReturn],
    ):
        self.__printer = loader.printer
        self.__kalico = loader.kalico
        self.__func = func

        self.name = func.__name__.upper()
        self._signature = inspect.signature(func)
        self._parameters = _validate_parameters(func)
        self._help = _document_parameters(self._parameters)
        self._source_file = pathlib.Path(inspect.getsourcefile(func))

        self._converters = {}
        self._validators = {}

        for param in self._parameters:
            if typing.get_origin(param.annotation) == typing.Annotated:
                converter, *validators = typing.get_args(param.annotation)
                self._converters[param.name] = converter
                self._validators[param.name] = validators
            elif param.annotation is bool:
                self._converters[param.name] = lambda s: bool(int(s))

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

    def _bind_and_validate(self, *args, **kwargs):
        bound = self._signature.bind(self.__kalico, *args, **kwargs)
        for param, (validators) in self._validators.items():
            if param not in bound.arguments:
                continue
            if not all(
                validate(bound.arguments[param]) for validate in validators
            ):
                raise CommandError(
                    f"{self.name} parameter {param} failed validation"
                )
        return bound

    def _bind_from_context(self, context: dict):
        kwargs = {}
        params = context.get("params", {})

        for paramspec in self._parameters:
            param_name = paramspec.name.upper()

            if param_name not in params:
                if paramspec.default is paramspec.empty:
                    raise CommandError(
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
            type_ = paramspec.annotation
            validators = []
            if typing.get_origin(type_) == typing.Annotated:
                type_ = typing.get_args(type_)[0]

            # Special case for boolean values
            if type_ is bool:
                value = bool(int(value))

            elif callable(type_):
                value = paramspec.annotation(value)

            for validator in validators:
                if not validator(value):
                    raise CommandError(
                        f"{self.name} parameter {param_name} failed to validate."
                        f" {inspect.getsource(validator)}"
                    )

            kwargs[paramspec.name] = value

        return self._bind_and_validate(**kwargs)

    def _call_from_context(self, context: dict):
        bound = self._bind_from_context(context)
        return self.__call__(*bound.args, **bound.kwargs)

    def __call__(
        self,
        kalico: Kalico,
        *args: MacroParams.args,
        **kwargs: MacroParams.kwargs,
    ) -> MacroReturn:
        bound = self._bind_and_validate(*args, **kwargs)

        try:
            return self.__func(*bound.args, **bound.kwargs)
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


@typing.overload
def gcode_macro(
    function: typing.Callable[Concatenate[Kalico, MacroParams], MacroReturn],
    /,
) -> Macro[MacroParams, MacroReturn]: ...


@typing.overload
def gcode_macro(
    *, rename_existing: str
) -> typing.Callable[
    [typing.Callable[Concatenate[Kalico, MacroParams], MacroReturn]],
    Macro[MacroParams, MacroReturn],
]: ...


def gcode_macro(
    func: typing.Optional[MacroFunction],
    /,
    rename_existing: typing.Optional[str] = None,
) -> typing.Callable[[MacroFunction], Macro]:
    def macro_decorator(func: MacroFunction) -> Macro:
        wrapped_macro = Macro(load_context.loader, func)
        load_context.loader._register_macro(wrapped_macro, rename_existing)
        return wrapped_macro

    if func is not None:
        return macro_decorator(func)

    return macro_decorator
