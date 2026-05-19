from __future__ import annotations

import inspect
import types
import typing

from .context import Kalico

Function = typing.TypeVar("Handler", bound=typing.Callable)


class Decorator(typing.Generic[Function], typing.Protocol):
    def __call__(self, handler: Function) -> Function: ...


@typing.runtime_checkable
class GCodeFunction(typing.Protocol):
    def __call__(self, kalico: Kalico): ...


def validate_gcode_function(func: types.FunctionType):
    if not callable(func):
        raise ValueError("must be a function")

    signature = inspect.signature(func)
    if not signature.parameters:
        return ValueError("must have a parameter for `Kalico`")

    first_param, *others = signature.parameters.values()
    if first_param.annotation not in (signature.empty, Kalico):
        return ValueError("first parameter must be of type `Kalico`")

    for param in others:
        if (
            param.kind not in (param.VAR_KEYWORD, param.VAR_POSITIONAL)
            and param.default is signature.empty
        ):
            raise ValueError(
                f"Parameter {param.name} must have a default value"
            )


def is_gcode_function(func: types.FunctionType):
    try:
        validate_gcode_function(func)
    except ValueError:
        return False
    return True
