@gcode_macro("HELLO_WORLD")  # noqa
def example_macro(p, name: str = "World"):
    p.respond_info(f"Hello, {name}!")
