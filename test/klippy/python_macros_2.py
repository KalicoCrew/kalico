from kalico import Kalico, gcode_macro
import enum
import pytest


class Direction(enum.Enum):
    up = -1
    down = 1


class Location(enum.StrEnum):
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
    p: Kalico,
    direction: Direction = None,
    location: Location = Location.front,
):
    "Run a suite of api tests"

    # Test to make sure the enum parameters are handled correctly
    assert p.status.gcode.commands["DO_THE_THING"]["params"] == {
        "DIRECTION": {"type": "enum", "choices": [-1, 1]},
        "LOCATION": {
            "type": "enum",
            "choices": ["FRONT", "BACK"],
            "default": "FRONT",
        },
    }

    with pytest.raises(p._printer.command_error):
        p.move(x=-999)

    # Attribute proxy for gcode
    p.gcode.g28("x", "y")
    assert p.status.toolhead.homed_axes == "xy"

    # And direct gcode calls
    p.gcode("g28")
    assert p.status.toolhead.homed_axes == "xyz"
    assert p.status.gcode_move.absolute_coordinates

    with p.save_gcode_state():
        p.gcode.g0(x=5, y=5, z=5)
        assert p.status.toolhead.position[:3] == (5.0, 5.0, 5.0)

        p.gcode.relative_movement()
        assert not p.status.gcode_move.absolute_coordinates

        p.gcode.g0(x=5)
        assert p.status.toolhead.position.x == 10.0

    # Ensure the gcode state was restored
    assert p.status.gcode_move.absolute_coordinates
    assert p.status.gcode_move.homing_origin.y == 0.0

    # Test the new move API
    p.move(dx=5)
    assert p.status.toolhead.position.x == 15.0

    p.move(dx=-5)
    assert p.status.toolhead.position.x == 10.0

    p.move(x=3)
    assert p.status.toolhead.position.x == 3.0

    # Test gcode offsets
    assert p.status.toolhead.position.y == 5.0

    p.move.set_gcode_offset(y=5)
    assert p.status.gcode_move.homing_origin.x == 0.0
    assert p.status.gcode_move.homing_origin.y == 5.0

    p.move.set_gcode_offset(dy=5, move=True)
    assert p.status.gcode_move.homing_origin.y == 10.0
    assert p.status.gcode_move.position.y == 10.0

    # Check the variable proxy saves values
    p.vars["test"] = 1
    assert p.status["gcode_macro DO_THE_THING"].test == 1
