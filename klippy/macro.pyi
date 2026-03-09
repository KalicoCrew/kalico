import collections.abc
import math
import typing

rawparams: str
params: dict[str, str]
own_vars: dict[str, typing.Any]

printer: dict[str, dict[str, typing.Any]]

def emit(gcode: str) -> None:
    "Run a G-Code"

def wait_while(condition: collections.abc.Callable[[], bool]) -> None:
    "Wait while a condition is True"

def wait_until(condition: collections.abc.Callable[[], bool]) -> None:
    "Wait until a condition is True"

def wait_moves() -> None:
    "Wait until all moves are completed"

Result = typing.TypeVar("ReturnT")

def blocking(function: collections.abc.Callable[[], Result]) -> Result:
    "Run a blocking task in a thread, waiting for the result"

def sleep(timeout: float) -> None:
    "Wait a given number of seconds"

def set_gcode_variable(macro: str, variable: str, value: typing.Any) -> None:
    "Save a variable to a gcode_macro"

def action_log(msg: str) -> typing.Literal[""]:
    "Log a message to klippy.log"

def action_emergency_stop(
    msg: str = "action_emergency_stop",
) -> typing.Literal[""]:
    "Immediately shutdown Kalico"

def action_respond_info(msg: str) -> typing.Literal[""]:
    "Send a message to the console"

def action_raise_error(msg) -> None:
    "Raise a G-Code command error"

def action_call_remote_method(method: str, **kwargs) -> typing.Literal[""]:
    "Call a Kalico webhooks method"

emergency_stop = action_emergency_stop
respond_info = action_respond_info
raise_error = action_raise_error
call_remote_method = action_call_remote_method

TYPE_CHECKING: False

__all__ = (
    "params",
    "rawparams",
    "own_vars",
    "printer",
    "emit",
    "wait_while",
    "wait_until",
    "wait_moves",
    "blocking",
    "sleep",
    "set_gcode_variable",
    "emergency_stop",
    "respond_info",
    "raise_error",
    "call_remote_method",
    "action_call_remote_method",
    "action_emergency_stop",
    "action_log",
    "action_raise_error",
    "action_respond_info",
    "math",
)
