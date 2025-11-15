from __future__ import annotations

import logging
import threading
import typing

from klippy.extras.gcode_macro import (
    GetStatusWrapperPython,
    TemplateVariableWrapperPython,
)

from .fans import FanAPI
from .gcode import GCodeAPI
from .gcode_move import MoveAPI
from .heaters import HeatersAPI
from .save_variables import SaveVariablesWrapper

if typing.TYPE_CHECKING:
    from klippy.extras.gcode_macro import GCodeMacro
    from klippy.gcode import GCodeDispatch
    from klippy.printer import Printer
    from klippy.reactor import SelectReactor


BlockingResult = typing.TypeVar("BlockingResult")


class Kalico:
    'The magic "Printer" object for macros'

    status: GetStatusWrapperPython
    saved_vars: SaveVariablesWrapper

    fans: FanAPI
    gcode: GCodeAPI
    heaters: HeatersAPI
    move: MoveAPI

    def __init__(self, printer: Printer):
        self._printer = printer

        self._gcode: GCodeDispatch = printer.lookup_object("gcode")

        self.status = GetStatusWrapperPython(printer)
        self.saved_vars = SaveVariablesWrapper(printer)

        self.fans = FanAPI(printer)
        self.gcode = GCodeAPI(self._gcode)
        self.heaters = HeatersAPI(printer)
        self.move = MoveAPI(printer)

    def __repr__(self):
        return "<Kalico>"

    def wait_while(self, condition: typing.Callable[[], bool]):
        "Wait while a condition is True"

        def inner(eventtime):
            return condition()

        self._printer.wait_while(inner)

    def wait_until(self, condition: typing.Callable[[], bool]):
        "Wait until a condition is True"

        def inner(eventtime):
            return not condition()

        self._printer.wait_until(condition)

    def wait_moves(self):
        "Wait until all moves are completed"
        toolhead = self._printer.lookup_object("toolhead")
        toolhead.wait_moves()

    def blocking(
        self, function: typing.Callable[[], BlockingResult]
    ) -> BlockingResult:
        "Run a blocking task in a thread, waiting for the result"
        completion = self._printer.get_reactor().completion()

        def run():
            try:
                ret = function()
                completion.complete((False, ret))
            except Exception as e:
                completion.complete((True, e))

        t = threading.Thread(target=run, daemon=True)
        t.start()
        [is_exception, ret] = completion.wait()
        if is_exception:
            raise ret
        else:
            return ret

    def sleep(self, timeout: float):
        "Wait a given number of seconds"
        reactor: SelectReactor = self._printer.get_reactor()
        deadline = reactor.monotonic() + timeout

        def check(event):
            return deadline > reactor.monotonic()

        self._printer.wait_while(check)

    def set_gcode_variable(self, macro: str, variable: str, value: typing.Any):
        "Save a variable to a gcode_macro"
        macro: GCodeMacro = self._printer.lookup_object(f"gcode_macro {macro}")
        macro.variables = {**macro.variables, variable: value}

    def get_gcode_variables(self, macro: str) -> TemplateVariableWrapperPython:
        macro = self._printer.lookup_object(f"gcode_macro {macro}")
        return TemplateVariableWrapperPython(macro)

    def emergency_stop(self, msg: str = "action_emergency_stop"):
        "Immediately shutdown Kalico"
        self._printer.invoke_shutdown(f"Shutdown due to {msg}")

    def respond(self, prefix: str, msg: str):
        "Send a message to the console"
        self._gcode.respond_raw(f"{prefix} {msg}")

    def respond_info(self, msg: str):
        "Send a message to the console"
        self._gcode.respond_info(msg)

    def respond_raw(self, msg: str):
        self._gcode.respond_raw(msg)

    def raise_error(self, msg):
        "Raise a G-Code command error"
        raise self._printer.command_error(msg)

    def call_remote_method(self, method: str, **kwargs):
        "Call a Kalico webhooks method"
        webhooks = self._printer.lookup_object("webhooks")
        try:
            webhooks.call_remote_method(method, **kwargs)
        except self._printer.command_error:
            logging.exception("Remote call error")
