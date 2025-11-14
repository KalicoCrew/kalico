# Kalico Python Macro typing

import math
import typing

BlockingResult = typing.TypeVar("BlockingResult")

class GCodeCommand(typing.Protocol):
    def format(self, *args, **params) -> str:
        "Return a formatted GCode string"

    def __call__(self, *args, **params):
        "Run GCode with parameters"

class GCode:
    def __getattr__(self, name) -> GCodeCommand: ...
    def __call__(self, command: str):
        "Run GCode"

class Printer:
    'The magic "Printer" object for macros'

    status: dict[str, dict[str, typing.Any]]
    "The printer status object"

    vars: dict[str, typing.Any]
    "The current macro's variables"

    saved_vars: dict[str, typing.Any]
    "Variables from save_variables"

    raw_params: str
    "the raw parameters passed to the macro"

    params: dict[str, str]
    "macro parameters, without type parsing"

    gcode: GCode
    "Helper for calling other GCode"

    def wait_while(self, condition: typing.Callable[[], bool]):
        "Wait while a condition is True"

    def wait_until(self, condition: typing.Callable[[], bool]):
        "Wait until a condition is True"

    def wait_moves(self):
        "Wait until all moves are completed"

    def blocking(
        self, function: typing.Callable[[], BlockingResult]
    ) -> BlockingResult:
        "Run a blocking task in a thread, waiting for the result"

    def sleep(self, timeout: float):
        "Wait a given number of seconds"

    def set_gcode_variable(self, macro: str, variable: str, value: typing.Any):
        "Save a variable to a gcode_macro"

    def emergency_stop(self, msg: str = "action_emergency_stop"):
        "Immediately shutdown Kalico"

    def respond(self, prefix: str, msg: str):
        "Send a message to the console"

    def respond_info(self, msg: str):
        "Send a message to the console prefixed with //"

    def respond_raw(self, msg: str):
        "Send a message directly to the console"

    def raise_error(self, msg):
        "Raise a G-Code command error"

    def call_remote_method(self, method: str, **kwargs):
        "Call a Kalico webhooks method"

    def set_temperature(
        self,
        heater_name: str,
        temp: typing.Optional[float] = None,
    ):
        "Set the target temperature for a heater"

    def temperature_wait(
        self,
        sensor_name,
        min_temp: float = -math.inf,
        max_temp: float = math.inf,
    ):
        """
        Wait for a heater or sensor to reach a temperature

        If no minimum or maximum is given, this will wait for the heater's control loop to settle
        """

    def set_fan_speed(self, fan_name: str, speed: float):
        "Set the speed of a fan"

    def save_gcode_state(
        self,
        name: str = None,
        move_on_restore: bool = False,
        move_speed: float = None,
    ) -> typing.ContextManager:
        "Save and restore the current gcode state"

type Macro = typing.Callable[typing.Concatenate[Printer, ...], None]

def gcode_macro(
    name: str,
    rename_existing: typing.Optional[str],
) -> typing.Callable[[Macro], Macro]: ...
