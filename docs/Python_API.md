# Python API for Kalico

```
[kalico_api]
python: 
    macros.py
    another_file.py
```

## Writing macros in Python

Kalico provides a strongly-typed interface for writing new macros and controlling your printer.

For the python type stubs, see [kalico.pyi](../kalico.pyi). Copying this next to your python files or updating your editor to point to these stubs will enable completion and in-editor documentation for the Kalico interface.

In your python, types and some global values are imported from the virtual `kalico` module.
The `kalico.Kalico` interface is grouped by subsystem you are interfacing with.

### `@gcode_macro`

This decorator registers your function as a GCode command. This allows it to be called
from the console, from other macros, or even legacy `[gcode_macro]`s. Functions should be fully typed when possible, and the first parameter **must** be typed with the `Kalico` object.

```python
from kalico import gcode_macro, Kalico

@gcode_macro
def scrub_nozzle(k: Kalico, count: int, speed: float = 100, travel_speed: float = None):
    'Scrub the nozzle on a gantry-mounted brush'
    brush_y_position = 10
    start_x_position = 290
    end_x_position = 350
    brush_x_range = (300, 340)

    if not travel_speed:
        travel_speed = speed

    k.move(x=brush_x_range[0], y=brush_y_position, speed=travel_speed)
    for _ in range(count):
        k.move(x=brush_x_range[0], speed=speed)
        k.move(x=brush_x_range[1], speed=speed)
    k.move(x=end_x_position, speed=travel_speed)
```

Your function parameters are automatically mapped to GCode parameters. This example of a nozzle scrub macro gets called from GCode as `SCRUB_NOZZLE COUNT= [SPEED=100] [TRAVEL_SPEED=]`. Notice the `count` parameter has no default value, and is therefore required. `speed` and `travel_speed` have default values.

If no annotation is used on a parameter, it is directly passed as a string. Type annotations for parameters must be a callable object accepting a single `str` and returning the value.

Helpers for validating numbers can be used in the type annotations.

```python
from kalico import Kalico, Range

@gcode_macro
def validated(
    kalico: Kalico,
    count: Annotated[int, Range(0, 10)]
): ...
```

### GCode Variables

`@gcode_macro` returns a `Macro` instance. This provides helpers for accessing gcode variables or accessing raw context parameters.

```python
@gcode_macro
def countdown(k: Kalico, count: int = 10):
    # k.set_gcode_variable('countdown', 'timer', count)
    countdown.vars['timer'] = count

    if count > 0:
        k.respond_info(f"Countdown: {count}s remaining")
        countdown.delay(1.0, count=count-1)

    else:
        k.respond_info("Countdown complete")
```

### Events and Timers

The API provides two helpers for scheduling function calls, as well as a method to register handlers for the many available events.

```python
from kalico import Kalico, event_handler

def init_bedfans(k: Kalico, eventtime: float):
    # This code only runs once
    k.fans.set_speed('bed_fans', 0.)

def update_bedfans(k: Kalico, eventtime: float):
    # This code will run every 10 seconds
    if k.status.heater_bed.target:
        if not k.status['fan_generic bed_fans'].speed:
            k.fans.set_speed('bed_fans', 1.)
    else:
        if k.status['fan_generic bed_fans'].speed:
            k.fans.set_speed('bed_fans', 0.)


@event_handler('klippy:ready')
def on_ready(k: Kalico):
    k.timer(0.0, init_bedfans)
    k.interval(10.0, bedfans_loop)
```

See [events.py](../klippy/extras/kalico_api/kalico/events.py) for available events.

Scheduling a timer returns an `Interval` from where you can `interval.cancel()` events.
As a warning, rapidly recurring intervals may cause print issues or Kalico crashes.

### Automatic Macro Documentation

The `gcode` status object has been enhanced to include descriptions of parameters.

```json
{ "gcode": {
    "SCRUB_NOZZLE": {
        "help": "Scrub the nozzle on a gantry-mounted brush",
        "params": {
            "COUNT": {
                "type": "int",
                "required": true
            },
            "SPEED": {
                "type": "float",
                "default": 100.0
            },
            "TRAVEL_SPEED": {
                "type": "float"
            },
        }
    }
}}
```

Python `enum.Enum` classes are documented with a list of their values

```python
class Location(enum.Enum):
    front = 'FRONT'
    back = 'BACK'

@gcode_macro
def park_toolhead(k: Kalico, location: Location = Location.FRONT): ...
```

```json
"LOCATION": {
    "type": "enum",
    "choices": ["FRONT", "BACK"],
    "default": "FRONT"
}
```

### Error Handling

The Python API provides enhanced error handling for python gcode macros. Local variables are shown in the traceback, and command errors start at the failing call.

```
Error in DO_THE_THING: Must home axis first: -999.000 0.000 0.000 [0.000]
Traceback (most recent call last):
  File "/home/printer/printer_data/config/macros.py", line 46, in do_the_thing
    k.move(dx=x_distance)
    ~~~~~~^^^^^^^^^^^^^^^
    x_distance = -999
    k = <Kalico>
  File "/home/printer/klipper/klippy/extras/kalico_api/kalico/gcode_move.py", line 40, in __call__
    self._gcode_move.move_to(newpos, speed)
    ~~~~~~~~~~~~~~~~~~~~~~~~^^^^^^^^^^^^^^^
```

## The `Kalico` API

```python
class Kalico:
    status: GetStatusWrapperPython
    saved_vars: SaveVariablesWrapper

    fans: FanAPI
    gcode: GCodeAPI
    heaters: HeatersAPI
    move: MoveAPI

    def wait_while(self, condition: typing.Callable[[], bool]):
        """Wait while a condition is True"""
    def wait_until(self, condition: typing.Callable[[], bool]):
        """Wait until a condition is True"""
    def wait_moves(self) -> None:
        """Wait until all moves are completed"""
    def blocking(
        self, function: typing.Callable[[], BlockingResult]
    ) -> BlockingResult:
        """Run a blocking task in a thread, waiting for the result"""
    def sleep(self, timeout: float):
        """Wait a given number of seconds"""
    def set_gcode_variable(self, macro: str, variable: str, value: typing.Any):
        """Save a variable to a gcode_macro"""
    def get_gcode_variables(
        self, macro: str
    ) -> TemplateVariableWrapperPython: ...
    def emergency_stop(self, msg: str = "action_emergency_stop"):
        """Immediately shutdown Kalico"""
    def respond(self, prefix: str, msg: str):
        """Send a message to the console"""
    def respond_info(self, msg: str):
        """Send a message to the console"""
    def respond_raw(self, msg: str): ...
    def raise_error(self, msg) -> None:
        """Raise a G-Code command error"""
    def call_remote_method(self, method: str, **kwargs):
        """Call a Kalico webhooks method"""
    def timer(self, delay: float, callback: TimerCallback) -> Timer:
        """Schedule a callback to run after a delay"""
    def interval(self, period: float, callback: TimerCallback) -> Interval: ...
```

### MoveAPI

Unlike the traditional `G0`/`G1` commands, the Move API does not use global state for most operations. All movements are explicitly relative or absolute, and you can mix relative and absolute in a single call.

```python


class MoveAPI:
    def __call__(
        self,
        x: float | None = None, y: float | None = None, z: float | None = None, e: float | None = None,
        *,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        de: float = 0.0,
        speed: float | None = None,
    ):
        """
        Move to a position

        `speed` is in mm/s and unlike `G1 Fx` only affects this movement.
        """

    def set_gcode_offset(
        self,
        x: float | None = None,
        y: float | None = None,
        z: float | None = None,
        *,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        move: bool = False,
        speed: float = None,
    ):
        """
        Set GCode offsets

        `speed` is in mm/s
        """
    def set_speed(self, speed: float):
        """Set the speed for future moves in mm/s"""
    def set_speed_factor(self, speed_factor: float = 1.0):
        """Set the movement speed multiplier"""
    def set_extrude_factor(self, extrude_factor: float = 1.0):
        """Set the extrusion multiplier"""

@gcode_macro
def my_macro(k: Kalico):
    # Set the global speed to 200 mm/s
    k.set_speed(200)
    # This is equivalent to
    k.gcode.g0(F=12000)

    # Move to X=5, and move Y by -5mm
    k.move(x=5, dy=-5)

    # Move X by 50mm, at 400 mm/s
    k.move(dx=50, speed=400)
```

### GCodeAPI

This provides a pythonic interface for calling GCode, with some aliases for common GCodes.

```python
class GcodeAPI:
    def __getattr__(self, command: str) -> GCodeCommand: ...
    def __call__(self, command: str): ...
    def absolute_movement(self) -> None: ...
    def relative_movement(self) -> None: ...
    def absolute_extrusion(self) -> None: ...
    def relative_extrusion(self) -> None: ...

class GCodeCommand:
    def format(self, *args: str, **params) -> str:
        "Get the formatted GCode"
    def __call__(self, *args: str, **params): ...


def macro(kalico: Kalico):
    # These calls are equivalent
    kalico.gcode('G28 X METHOD=CONTACT')
    kalico.gcode.g28('Z', method='contact')

    # Certain values are mapped to their logical GCode equivalent
    assert (
        kalico.gcode.set_heater_temperature.format(heater="extruder", temp=220, wait=True)
        == "SET_HEATER_TEMPERATURE HEATER=extruder TEMP=220 WAIT=1"
    )

    # You can even save your own aliases
    home = kalico.gcode.g28
    home('Z', method='contact')
```

### HeatersAPI

```python
class HeatersAPI:
    def set_temperature(self, heater_name: str, temp: float | None = None):
        """Set the target temperature for a heater"""

    def temperature_wait(
        self,
        sensor_name: str,
        min_temp: float | None = None,
        max_temp: float | None = None,
    ):
        """
        Wait for a heater or sensor to reach a temperature

        If no minimum or maximum is given, this will wait for the heater's control loop to settle.
        """
```

`heaters.temperature_wait` differs from `TEMPERATURE_WAIT` in that when called on an active heater, the
 heater control is used to determine when to stop waiting. This is the behavior of the `M109` `M190` or `SET_HEATER_TEMPERATURE WAIT=1`.

### FanAPI

This provides a single interface that can set both `[fan]` or `[fan_generic]` speeds.

```python
class FanAPI:
    def set_speed(self, fan_name: str, speed: float):
        """Set the speed of a fan"""
```
