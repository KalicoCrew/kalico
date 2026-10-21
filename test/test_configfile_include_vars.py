import configparser
import pathlib
import typing

import pytest
from klippy_testing import PrinterShim


def _load(config_root):
    start_args = {"config_file": str(config_root / "printer.cfg")}
    with PrinterShim(start_args) as printer:
        return printer.load_config()


def test_include_vars(
    config_root: typing.Annotated[pathlib.Path, "test_configs/include_vars"],
):
    config = _load(config_root)
    for name in ("toolhead0", "toolhead1"):
        mcu = config.getsection("mcu %s" % (name,))
        assert mcu.get("serial") == "/dev/serial/by-id/usb-%s" % (name,)
        # Variables are inherited by nested includes
        sensor = config.getsection("temperature_sensor %s" % (name,))
        assert sensor.get("sensor_pin") == "%s:PA3" % (name,)
    assert config.has_section("fan_generic toolhead0_fan")
    assert not config.has_section("fan_generic toolhead1_fan")


def test_include_without_vars_is_not_rendered(
    config_root: typing.Annotated[pathlib.Path, "test_configs/include_vars"],
):
    config = _load(config_root)
    gcode = config.getsection("gcode_macro PLAIN").get("gcode")
    assert "{! literal !}" in gcode


def test_include_undefined_var(
    config_root: typing.Annotated[
        pathlib.Path, "test_configs/include_vars_undefined"
    ],
):
    with pytest.raises(configparser.Error, match="mcu_path"):
        _load(config_root)


def test_include_invalid_var_line(
    config_root: typing.Annotated[
        pathlib.Path, "test_configs/include_vars_invalid"
    ],
):
    with pytest.raises(configparser.Error, match="not a variable"):
        _load(config_root)
