import typing

import klippy.configfile
import klippy.extras.danger_options
import klippy.gcode


class Restart(Exception): ...


class PrinterShim:
    class GCode:
        error = Exception

        def __init__(self):
            self.ready_gcode_handlers = {}
            self.mux_commands = {}

        def register_command(self, cmd, func, *_, **__):
            self.ready_gcode_handlers[cmd.upper()] = func

        def register_mux_command(self, cmd, key, value, func, desc=None):
            prev = self.mux_commands.get(cmd)
            if prev is None:

                def handler(gcmd):
                    return self._cmd_mux(cmd, gcmd)

                self.register_command(cmd, handler, desc=desc)
                self.mux_commands[cmd] = prev = (key, {})
            prev_key, prev_values = prev
            prev_values[value] = func

        def _cmd_mux(self, command, gcmd):
            key, values = self.mux_commands[command]
            if None in values:
                key_param = gcmd.get(key, None)
            else:
                key_param = gcmd.get(key)
            if key_param not in values:
                raise gcmd.error(
                    "The value '%s' is not valid for %s" % (key_param, key)
                )
            values[key_param](gcmd)

        def respond_info(self, msg, log=True):
            print("info", msg)

        def respond_raw(self, msg):
            print("raw", msg)

        def request_restart(self, reason):
            raise Restart(reason)

        def call(self, cmdline, **kwargs):
            if kwargs:
                parts = [cmdline] + [
                    "%s=%s" % (k, v) for k, v in kwargs.items()
                ]
                cmdline = " ".join(parts)
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
