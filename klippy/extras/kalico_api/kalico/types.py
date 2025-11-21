from __future__ import annotations

import typing

if typing.TYPE_CHECKING:
    from .context import Kalico


Function = typing.TypeVar("Handler", bound=typing.Callable)


class Decorator(typing.Generic[Function], typing.Protocol):
    def __call__(self, handler: Function) -> Function: ...


@typing.runtime_checkable
class GCodeFunction(typing.Protocol):
    def __call__(self, kalico: Kalico): ...
