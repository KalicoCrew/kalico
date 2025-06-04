import typing
import klippy.configfile
import klippy.gcode
import klippy.extras.danger_options


class Restart(Exception): ...


class PrinterShim:
    class GCode:
        error = Exception

        def __init__(self):
            self.ready_gcode_handlers = {}

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
                self, command, cmdline, params, False
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
        pass

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
