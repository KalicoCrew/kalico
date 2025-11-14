@gcode_macro("HELLO_WORLD")  # noqa
def example_macro(p, name: str = "World"):
    p.gcode.respond(msg=f"Hello, {name}!")


@gcode_macro("DO_THE_THING")  # noqa
def exercise(p):
    # Attribute proxy for gcode
    p.gcode.g28("x", "y")
    assert p.status["toolhead"]["homed_axes"] == "xy"

    # And direct gcode calls
    p.gcode("g28")
    assert p.status["toolhead"]["homed_axes"] == "xyz"

    assert p.status["gcode_move"]["absolute_coordinates"]
    with p.save_gcode_state():
        p.gcode.g91()
        p.gcode.g0(x=123)
        assert not p.status["gcode_move"]["absolute_coordinates"]
    # Ensure the gcode state was restored
    assert p.status["gcode_move"]["absolute_coordinates"]

    # Check the variable proxy saves values
    p.vars["test"] = 1
    assert p.status["gcode_macro DO_THE_THING"]["test"] == 1
