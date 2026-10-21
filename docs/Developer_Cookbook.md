# Developer Cookbook

Recipes for common tasks when writing a Kalico host module. Read the
"Adding a host module" section of [Code Overview](Code_Overview.md)
first; it covers how modules are loaded and the module lifecycle.

## Reading config options

Read and validate every option in the module's `__init__()`, so that
config mistakes are reported at startup:

```
class MyModule:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.speed = config.getfloat("speed", 50.0, above=0.0)
        self.count = config.getint("count", 3, minval=1, maxval=10)
        self.mode = config.getchoice("mode", ["fast", "safe"], "safe")
```

`config.get_name()` returns the full section name (e.g.
`my_module left`), so the last word is the instance name of a
`load_config_prefix()` module.

To reject a bad combination of options, raise `config.error`:

```
if self.count > 5 and self.mode == "fast":
    raise config.error("count must be 5 or less in fast mode")
```

## Looking up other modules

Other modules may not exist yet when `__init__()` runs. Look them up
from a `klippy:connect` handler, or at time of use:

```
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")
        # Optional module: returns None instead of raising
        self.probe = self.printer.lookup_object("probe", None)
```

## Registering a G-Code command

```
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "MY_COMMAND", self.cmd_MY_COMMAND, desc=self.cmd_MY_COMMAND_help
        )

    cmd_MY_COMMAND_help = "Do something useful"

    def cmd_MY_COMMAND(self, gcmd):
        distance = gcmd.get_float("DISTANCE", 10.0, above=0.0)
        if distance > 100.0:
            raise gcmd.error("DISTANCE must be 100 or less")
        gcmd.respond_info("Moving %.1fmm" % (distance,))
```

Raising `gcmd.error` (or `self.printer.command_error`) reports the
message to the user and aborts the command without shutting down the
printer.

For a module with several config instances, register one command per
instance with `register_mux_command()`. The user then picks the
instance with a parameter, e.g. `SET_SERVO SERVO=my_servo ANGLE=90`:

```
        gcode.register_mux_command(
            "SET_SERVO", "SERVO", self.name, self.cmd_SET_SERVO,
            desc=self.cmd_SET_SERVO_help
        )
```

## Moving the toolhead

First get the toolhead object:

```
toolhead = self.printer.lookup_object("toolhead")
```

The toolhead object is not always available at module initialization
time so it should be looked up at time of use.

All moves with the toolhead are absolute moves. However you can instruct
the toolhead not to move an axis by passing `None` for a coordinate to
`manual_move()`. E.g this will move the toolhead to z=10mm at its current
x/y position:

```
toolhead.manual_move([None, None, 10.], speed)
```

To make a relative move, use the current toolhead position. This will
lift the Z axis by 10mm:

```
pos = toolhead.get_position()
toolhead.manual_move([None, None, pos[2] + 10.], speed)
```

Moves are queued, so `manual_move()` returns before the motion happens.
Call `toolhead.wait_moves()` to wait until all queued moves are done,
e.g. before reading a sensor at the new position.

## Running G-Code from a module

From inside a G-Code command handler, use `run_script_from_command()`:

```
    def cmd_MY_COMMAND(self, gcmd):
        gcode = self.printer.lookup_object("gcode")
        gcode.run_script_from_command("G28 Z\nG1 Z10 F600")
```

From anywhere else (a timer, an event handler), use `run_script()`,
which waits for the G-Code mutex first. Calling `run_script()` from a
command handler would deadlock.

## Doing work periodically

Use a reactor timer. The callback returns the time it should run next,
or `reactor.NEVER` to stop:

```
        self.printer.register_event_handler(
            "klippy:ready", self._handle_ready
        )

    def _handle_ready(self):
        reactor = self.printer.get_reactor()
        self.timer = reactor.register_timer(self._poll, reactor.NOW)

    def _poll(self, eventtime):
        self.value = self._read_sensor()
        return eventtime + 1.0
```

Timer callbacks run in the main thread and must not block.

## Exposing status to macros and clients

Implement `get_status()` on the object returned by `load_config()`.
The returned dict is available to macros as `printer["my_module"]` and
to API clients through `objects/query`:

```
    def get_status(self, eventtime):
        return {"value": self.value, "mode": self.mode}
```

Document new status fields in [Status Reference](Status_Reference.md).

## Reporting errors

| Situation | Use |
|---|---|
| Bad config value | `raise config.error(...)` in `__init__()` |
| Bad command parameter or a command that can't run now | `raise gcmd.error(...)` |
| A condition that makes it unsafe to continue (e.g. a heater fault) | `self.printer.invoke_shutdown(msg)` |
