import threading
import typing

import klippy.configfile
import klippy.extras.danger_options
import klippy.gcode
import klippy.reactor


class Restart(Exception): ...


class PrinterShim:
    class GCode:
        error = Exception

        def __init__(self):
            self.ready_gcode_handlers = {}
            self.mutex = threading.Lock()
            self.reactor = klippy.reactor.Reactor()
            self.interrupt_token = klippy.gcode.InterruptToken(
                self.reactor, lambda: False
            )

        def get_interrupt_token(self):
            return self.interrupt_token

        def get_mutex(self):
            return self.mutex

        def register_command(self, cmd, func, *_, **__):
            self.ready_gcode_handlers[cmd.upper()] = func

        def respond_info(self, msg):
            print("info", msg)

        def respond_raw(self, msg):
            print("raw", msg)

        def request_restart(self, reason):
            raise Restart(reason)

        def call(self, cmdline):
            command, *paramlist = cmdline.split()
            func = self.ready_gcode_handlers[command.upper()]
            params = dict(param.split("=", 1) for param in paramlist)
            gcmd = klippy.gcode.GCodeCommand(
                self,
                command,
                cmdline,
                params,
                False,
                klippy.gcode.GCodeContextScope(
                    klippy.gcode.GCodeContextRoot(self.get_mutex()),
                    self.get_interrupt_token(),
                ),
            )
            print("Calling", func, "with", params)
            func(gcmd)

    def __init__(self, start_args):
        self.start_args = start_args
        self.objects = {}
        self.add_object("gcode", self.GCode())
        self.add_object("configfile", klippy.configfile.PrinterConfig(self))

        self.call = self.lookup_object("gcode").call

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.lookup_object("gcode").reactor.finalize()

    def get_start_args(self):
        return self.start_args

    def add_object(self, name, obj):
        self.objects[name] = obj

    @typing.overload
    def lookup_object(self, name: typing.Literal["gcode"]) -> GCode: ...
    @typing.overload
    def lookup_object(
        self, name: typing.Literal["configfile"]
    ) -> klippy.configfile.PrinterConfig: ...

    def lookup_object(self, name):
        return self.objects[name]

    def lookup_objects(self, pfx=""):
        if pfx:
            pfx = pfx + " "
        return [(k, v) for k, v in self.objects.items() if k.startswith(pfx)]

    def load_config(self):
        config = self.lookup_object("configfile").read_main_config()
        self.add_object(
            "danger_options",
            klippy.extras.danger_options.load_config(
                config.getsection("danger_options")
            ),
        )
        return config

    def set_rollover_info(self, *_): ...
