@gcode_macro("HELLO_WORLD")  # noqa
def example_macro(p, name: str = "World"):
    p.gcode.respond(msg=f"Hello, {name}!")
