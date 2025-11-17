import enum

import pytest

from kalico import Kalico, gcode_macro


class Direction(enum.Enum):
    up = -1
    down = 1


class Location(enum.Enum):
    front = "FRONT"
    back = "BACK"


@gcode_macro
def hello_world(p: Kalico, name: str = "World"):
    "Say hello"

    p.gcode.respond(msg=f"Hello, {name}!")
    assert p.status.gcode.commands["HELLO_WORLD"]["help"] == "Say hello"
    assert p.status.gcode.commands["HELLO_WORLD"]["params"] == {
        "NAME": {"type": "str", "default": "World"}
    }


@gcode_macro
def do_the_thing(
    k: Kalico,
    direction: Direction = None,
    location: Location = Location.front,
):
    "Run a suite of api tests"

    # Test to make sure the enum parameters are handled correctly
    assert k.status.gcode.commands["DO_THE_THING"]["params"] == {
        "DIRECTION": {"type": "enum", "choices": [-1, 1]},
        "LOCATION": {
            "type": "enum",
            "choices": ["FRONT", "BACK"],
            "default": "FRONT",
        },
    }

    with pytest.raises(k._printer.command_error):
        k.move(x=-999)

    # Attribute proxy for gcode
    k.gcode.g28("x", "y")
    assert k.status.toolhead.homed_axes == "xy"

    # And direct gcode calls
    k.gcode("g28")
    assert k.status.toolhead.homed_axes == "xyz"
    assert k.status.gcode_move.absolute_coordinates

    assert k.status.toolhead.position.x == 5.0
    with k.move.save_state(restore_position=True):
        k.gcode.g0(x=5, y=5, z=5)
        assert k.status.toolhead.position[:3] == (5.0, 5.0, 5.0)

        k.gcode.relative_movement()
        assert not k.status.gcode_move.absolute_coordinates

        k.gcode.g0(x=5)
        assert k.status.toolhead.position.x == 10.0
    assert k.status.toolhead.position.x == 5.0

    # Ensure the gcode state was restored
    assert k.status.gcode_move.absolute_coordinates
    assert k.status.gcode_move.homing_origin.y == 0.0

    # Test the new move API
    k.move(dx=5)
    assert k.status.toolhead.position.x == 10.0

    k.move(dx=-5)
    assert k.status.toolhead.position.x == 5.0

    k.move(x=3)
    assert k.status.toolhead.position.x == 3.0

    # Test gcode offsets
    assert k.status.toolhead.position.y == 5.0

    k.move.set_gcode_offset(y=5)
    assert k.status.gcode_move.homing_origin.x == 0.0
    assert k.status.gcode_move.homing_origin.y == 5.0

    k.move.set_gcode_offset(dy=5, move=True)
    assert k.status.gcode_move.homing_origin.y == 10.0
    assert k.status.gcode_move.position.y == 10.0

    # Check the variable proxy saves values
    do_the_thing.vars["test"] = 1
    assert k.status["gcode_macro DO_THE_THING"].test == 1


@gcode_macro
def _init_countdown(k: Kalico, counter: int = None):
    k.set_gcode_variable("countdown", "counter", counter)

    countdown(k)


@gcode_macro
def countdown(k: Kalico):
    counter = countdown.vars["counter"] - 1
    countdown.vars["counter"] = counter

    if counter > 0:
        k.respond_info(f"Countdown {countdown.vars['counter']}")
        countdown.delay(1)

    else:
        k.raise_error("countdown finished")


_init_countdown.delay(0)
