# Kalico Python Macro typing

import typing

BlockingResult = typing.TypeVar("BlockingResult")

class GCodeCommand(typing.Protocol):
    def __call__(self, **params):
        "Run GCode with parameters"

class GCode:
    def __getattribute__(self, name) -> GCodeCommand: ...

class Printer:
    'The magic "Printer" object for macros'

    status: dict[str, dict[str, typing.Any]]
    vars: dict[str, typing.Any]

    raw_params: str
    params: dict[str, str]

    gcode: GCode

    def emit(self, gcode: str):
        "Run GCode"

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

type Macro = typing.Callable[typing.Concatenate[Printer, ...], None]

def gcode_macro(
    name: str,
    rename_existing: typing.Optional[str],
) -> typing.Callable[[Macro], Macro]: ...
