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


def test_config_inject_appends_missing_key():
    """``[<section>.config_inject]`` adds key=value into the section."""
    cfg_in = """
[beacon]
serial: /tmp/foo
x_offset: 0
"""
    overrides = {
        "beacon.config_inject": {"skip_firmware_version_check": "True"},
    }
    out = apply_overrides(cfg_in, overrides)
    assert "skip_firmware_version_check: True" in out
    # Existing keys retained.
    assert "x_offset: 0" in out
    assert "serial: /tmp/foo" in out


def test_config_inject_does_not_duplicate_existing_key():
    """If the key is already present, no second copy is added."""
    cfg_in = """
[beacon]
serial: /tmp/foo
skip_firmware_version_check: False
"""
    overrides = {
        "beacon.config_inject": {"skip_firmware_version_check": "True"},
    }
    out = apply_overrides(cfg_in, overrides)
    assert out.count("skip_firmware_version_check") == 1
    # The user's value wins.
    assert "skip_firmware_version_check: False" in out


def test_config_inject_section_boundary_respected():
    """Injection writes into the named section only, not into the next one."""
    cfg_in = """
[beacon]
serial: /tmp/foo

[stepper_x]
step_pin: PG4
"""
    overrides = {
        "beacon.config_inject": {"skip_firmware_version_check": "True"},
    }
    out = apply_overrides(cfg_in, overrides)
    # The injected line must appear before [stepper_x], not after.
    beacon_pos = out.index("skip_firmware_version_check")
    stepper_pos = out.index("[stepper_x]")
    assert beacon_pos < stepper_pos


def test_config_inject_missing_section_is_noop():
    """If the section doesn't exist, apply_overrides leaves cfg unchanged."""
    cfg_in = "[stepper_x]\nstep_pin: PG4\n"
    overrides = {
        "beacon.config_inject": {"skip_firmware_version_check": "True"},
    }
    out = apply_overrides(cfg_in, overrides)
    assert out == cfg_in


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
