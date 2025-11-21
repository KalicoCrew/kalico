from kalico import Kalico, gcode_macro

from .asserts import assert_eq

assert __name__ == "printer_config.kalico_api.hello_world"


@gcode_macro
def hello_world(p: Kalico, name: str = "World"):
    "Say hello"

    p.gcode.respond(msg=f"Hello, {name}!")
    assert_eq(p.status.gcode.commands["HELLO_WORLD"]["help"], "Say hello")
    assert_eq(
        p.status.gcode.commands["HELLO_WORLD"]["params"],
        {"NAME": {"type": "str", "default": "World"}},
    )
