import pathlib
import typing

import pytest
from klippy.extras.danger_options import get_danger_options
from klippy_testing import PrinterShim


class PressureAdvanceShim:
    """Minimal shim that mimics ExtruderStepper's pressure advance handling.

    Registers SET_PRESSURE_ADVANCE and validates using the same danger_options
    bounds as the real ExtruderStepper.
    """

    def __init__(self, printer, config):
        self.printer = printer
        self.name = "extruder"
        danger_options = get_danger_options()
        self.pressure_advance = config.getfloat(
            "pressure_advance", 0.0,
            minval=danger_options.override_pressure_advance_min,
            maxval=danger_options.override_pressure_advance_max,
        )
        self.pressure_advance_smooth_time = config.getfloat(
            "pressure_advance_smooth_time", 0.040,
            above=danger_options.override_pressure_advance_smooth_time_min,
            maxval=danger_options.override_pressure_advance_smooth_time_max,
        )
        gcode = printer.lookup_object("gcode")
        gcode.register_mux_command(
            "SET_PRESSURE_ADVANCE",
            "EXTRUDER",
            None,
            self.cmd_SET_PRESSURE_ADVANCE,
        )
        gcode.register_mux_command(
            "SET_PRESSURE_ADVANCE",
            "EXTRUDER",
            self.name,
            self.cmd_SET_PRESSURE_ADVANCE,
        )

    def cmd_SET_PRESSURE_ADVANCE(self, gcmd):
        danger_options = get_danger_options()
        pressure_advance = gcmd.get_float(
            "ADVANCE", self.pressure_advance,
            minval=danger_options.override_pressure_advance_min,
            maxval=danger_options.override_pressure_advance_max,
        )
        smooth_time = gcmd.get_float(
            "SMOOTH_TIME",
            self.pressure_advance_smooth_time,
            minval=danger_options.override_pressure_advance_smooth_time_min,
            maxval=danger_options.override_pressure_advance_smooth_time_max,
        )
        self.pressure_advance = pressure_advance
        self.pressure_advance_smooth_time = smooth_time


def _load_printer_with_pa(start_args):
    """Load config, danger_options, and register pressure advance handler."""
    printer = PrinterShim(start_args)
    config = printer.load_config()
    extruder_cfg = config.getsection("extruder")
    pa_shim = PressureAdvanceShim(printer, extruder_cfg)
    printer.add_object("extruder_pa", pa_shim)
    return printer, pa_shim, config


def test_extruder_overrides_pressure_advance(
    config_root: typing.Annotated[pathlib.Path, "test_configs/extruder_overrides"],
):
    """Test that pressure advance overrides work correctly"""
    start_args = {"config_file": str(config_root / "printer.cfg")}
    printer, pa, config = _load_printer_with_pa(start_args)

    # Test that config values work with overrides
    assert pa.pressure_advance == 1.5
    assert pa.pressure_advance_smooth_time == 0.8

    # Test SET_PRESSURE_ADVANCE with values within override bounds
    printer.call("SET_PRESSURE_ADVANCE", EXTRUDER="extruder", ADVANCE=1.0, SMOOTH_TIME=0.5)
    assert pa.pressure_advance == 1.0
    assert pa.pressure_advance_smooth_time == 0.5

    # Test SET_PRESSURE_ADVANCE at override limits
    printer.call("SET_PRESSURE_ADVANCE", EXTRUDER="extruder", ADVANCE=2.0, SMOOTH_TIME=1.0)
    assert pa.pressure_advance == 2.0
    assert pa.pressure_advance_smooth_time == 1.0


def test_extruder_overrides_config_limits(
    config_root: typing.Annotated[pathlib.Path, "test_configs/extruder_overrides"],
):
    """Test that extruder config limits work with overrides"""
    start_args = {"config_file": str(config_root / "printer.cfg")}
    printer = PrinterShim(start_args)
    config = printer.load_config()
    danger_options = get_danger_options()
    extruder_cfg = config.getsection("extruder")

    # Test that overridden config values are accepted within override bounds
    max_e_dist = extruder_cfg.getfloat(
        "max_extrude_only_distance", 50.0,
        minval=danger_options.override_max_extrude_only_distance_min,
        maxval=danger_options.override_max_extrude_only_distance_max,
    )
    assert max_e_dist == 150.0

    instant_corner_v = extruder_cfg.getfloat(
        "instantaneous_corner_velocity", 1.0,
        minval=danger_options.override_instantaneous_corner_velocity_min,
        maxval=danger_options.override_instantaneous_corner_velocity_max,
    )
    assert instant_corner_v == 8.0

    max_e_velocity = extruder_cfg.getfloat(
        "max_extrude_only_velocity", 300.0,
        above=danger_options.override_max_extrude_only_velocity_min,
        maxval=danger_options.override_max_extrude_only_velocity_max,
    )
    assert max_e_velocity == 1500.0

    max_e_accel = extruder_cfg.getfloat(
        "max_extrude_only_accel", 3000.0,
        above=danger_options.override_max_extrude_only_accel_min,
        maxval=danger_options.override_max_extrude_only_accel_max,
    )
    assert max_e_accel == 15000.0


def test_extruder_overrides_fail_bounds(
    config_root: typing.Annotated[pathlib.Path, "test_configs/extruder_overrides_fail"],
):
    """Test that values outside override bounds fail"""
    start_args = {"config_file": str(config_root / "printer.cfg")}

    printer, pa, config = _load_printer_with_pa(start_args)

    # Test pressure advance below min override
    with pytest.raises(Exception, match="must have minimum of 0"):
        printer.call("SET_PRESSURE_ADVANCE", EXTRUDER="extruder", ADVANCE=-0.1)

    # Test pressure advance above max override
    with pytest.raises(Exception, match="must have maximum of 2"):
        printer.call("SET_PRESSURE_ADVANCE", EXTRUDER="extruder", ADVANCE=3.0)

    # Test smooth_time below min override
    with pytest.raises(Exception, match="must have minimum of 0"):
        printer.call("SET_PRESSURE_ADVANCE", EXTRUDER="extruder", ADVANCE=0.025, SMOOTH_TIME=-0.1)

    # Test smooth_time above max override
    with pytest.raises(Exception, match="must have maximum of 1"):
        printer.call("SET_PRESSURE_ADVANCE", EXTRUDER="extruder", ADVANCE=0.025, SMOOTH_TIME=2.0)


def test_extruder_defaults_behavior(
    config_root: typing.Annotated[pathlib.Path, "test_configs/extruder_defaults"],
):
    """Test that default behavior works without overrides"""
    start_args = {"config_file": str(config_root / "printer.cfg")}

    printer, pa, config = _load_printer_with_pa(start_args)

    # Test that default config values are applied
    assert pa.pressure_advance == 0.025
    assert pa.pressure_advance_smooth_time == 0.040

    # Test SET_PRESSURE_ADVANCE at original limits
    printer.call("SET_PRESSURE_ADVANCE", EXTRUDER="extruder", ADVANCE=0.025, SMOOTH_TIME=0.200)
    assert pa.pressure_advance == 0.025
    assert pa.pressure_advance_smooth_time == 0.200

    # Test that values beyond original limits fail
    with pytest.raises(Exception, match="must have maximum of 0.2"):
        printer.call("SET_PRESSURE_ADVANCE", EXTRUDER="extruder", ADVANCE=0.025, SMOOTH_TIME=0.300)

    with pytest.raises(Exception, match="must have minimum of 0"):
        printer.call("SET_PRESSURE_ADVANCE", EXTRUDER="extruder", ADVANCE=-0.1)


def test_extruder_overrides_none_values(
    config_root: typing.Annotated[pathlib.Path, "test_configs/extruder_overrides"],
):
    """Test that override limits are respected"""
    start_args = {"config_file": str(config_root / "printer.cfg")}

    printer, pa, config = _load_printer_with_pa(start_args)

    # Test that pressure advance can go up to the override max of 2.0
    printer.call("SET_PRESSURE_ADVANCE", EXTRUDER="extruder", ADVANCE=1.9, SMOOTH_TIME=0.9)
    assert pa.pressure_advance == 1.9
    assert pa.pressure_advance_smooth_time == 0.9
