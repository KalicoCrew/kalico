"""Tests for the pin/serial override layer.

Given a vendored printer.cfg-style text, applying the overrides yields
a config where STM32 pin names are replaced with gpiochip0/gpioN
equivalents, SPI bus names are replaced with sim_spiN, and the [mcu]
serial line points at our sim socket."""
from tools.sim_klippy.orchestrator.overrides import apply_overrides


def test_pin_substitution_basic():
    cfg_in = """
[mcu]
serial: /dev/serial/by-id/usb-Klipper_stm32h723xx_490017000851323235363233-if00

[stepper_x]
step_pin: PG4
dir_pin: !PC1
enable_pin: !PA2

[tmc5160 stepper_x]
cs_pin: PC7
spi_bus: spi1
"""
    overrides = {
        "mcu_main.gpio": {
            "PG4": "gpiochip0/gpio9",
            "PC1": "gpiochip0/gpio10",
            "PA2": "gpiochip0/gpio11",
            "PC7": "gpiochip0/gpio12",
        },
        "mcu_main.spi": {"spi1": "sim_spi0"},
        "mcu_main.serial": {
            "usb-Klipper_stm32h723xx_*": "/tmp/klipper_sim_h7",
        },
    }
    out = apply_overrides(cfg_in, overrides)
    assert "PG4" not in out
    assert "gpiochip0/gpio9" in out
    assert "!gpiochip0/gpio10" in out  # ! prefix preserved
    assert "spi1" not in out
    assert "sim_spi0" in out
    assert "/dev/serial/by-id/usb-Klipper" not in out
    assert "/tmp/klipper_sim_h7" in out


def test_word_boundary_safety():
    """PA2 must not match inside PA20."""
    cfg_in = "step_pin: PA20\nother_pin: PA2\n"
    overrides = {"mcu_main.gpio": {"PA2": "gpiochip0/gpio11"}}
    out = apply_overrides(cfg_in, overrides)
    assert "PA20" in out  # untouched
    assert "gpiochip0/gpio11" in out  # PA2 replaced
    # Make sure PA20 stayed PA20 and didn't become "gpiochip0/gpio110"
    assert "gpiochip0/gpio110" not in out


def test_spi_bus_word_boundary():
    """spi1 must not match inside spi10 (if such a string existed)."""
    cfg_in = "spi_bus: spi1\nfoo: spi10\n"
    overrides = {"mcu_main.spi": {"spi1": "sim_spi0"}}
    out = apply_overrides(cfg_in, overrides)
    assert "sim_spi0" in out
    assert "spi10" in out  # untouched


def test_load_overrides(tmp_path):
    """load_overrides reads a TOML file and returns the dict."""
    from tools.sim_klippy.orchestrator.overrides import load_overrides
    p = tmp_path / "test.toml"
    p.write_text('''
[mcu_main.gpio]
PA2 = "gpiochip0/gpio11"
PG4 = "gpiochip0/gpio9"

[mcu_main.spi]
spi1 = "sim_spi0"
''')
    out = load_overrides(p)
    assert out["mcu_main.gpio"]["PA2"] == "gpiochip0/gpio11"
    assert out["mcu_main.spi"]["spi1"] == "sim_spi0"
