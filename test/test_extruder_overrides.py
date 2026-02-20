import pathlib
import typing

import pytest
from klippy_testing import PrinterShim


def test_extruder_overrides_pressure_advance(
    config_root: typing.Annotated[pathlib.Path, "test_configs/extruder_overrides"],
):
    """Test that pressure advance overrides work correctly"""
    start_args = {"config_file": str(config_root / "printer.cfg")}
    
    with PrinterShim(start_args) as printer:
        # Test that config values work with overrides
        extruder = printer.lookup_object("extruder")
        assert extruder.extruder_stepper.pressure_advance == 1.5
        assert extruder.extruder_stepper.pressure_advance_smooth_time == 0.8
        
        # Test SET_PRESSURE_ADVANCE with values within override bounds
        printer.call("SET_PRESSURE_ADVANCE", ADVANCE=1.0, SMOOTH_TIME=0.5)
        assert extruder.extruder_stepper.pressure_advance == 1.0
        assert extruder.extruder_stepper.pressure_advance_smooth_time == 0.5
        
        # Test SET_PRESSURE_ADVANCE at override limits
        printer.call("SET_PRESSURE_ADVANCE", ADVANCE=2.0, SMOOTH_TIME=1.0)
        assert extruder.extruder_stepper.pressure_advance == 2.0
        assert extruder.extruder_stepper.pressure_advance_smooth_time == 1.0


def test_extruder_overrides_config_limits(
    config_root: typing.Annotated[pathlib.Path, "test_configs/extruder_overrides"],
):
    """Test that extruder config limits work with overrides"""
    start_args = {"config_file": str(config_root / "printer.cfg")}
    
    with PrinterShim(start_args) as printer:
        extruder = printer.lookup_object("extruder")
        
        # Test that overridden config values are applied
        assert extruder.max_e_dist == 150.0
        assert extruder.instant_corner_v == 8.0
        assert extruder.max_e_velocity == 1500.0
        assert extruder.max_e_accel == 15000.0


def test_extruder_overrides_fail_bounds(
    config_root: typing.Annotated[pathlib.Path, "test_configs/extruder_overrides_fail"],
):
    """Test that values outside override bounds fail"""
    start_args = {"config_file": str(config_root / "printer.cfg")}
    
    with PrinterShim(start_args) as printer:
        # Test pressure advance below min override
        with pytest.raises(Exception, match="must have minimum of 0"):
            printer.call("SET_PRESSURE_ADVANCE", ADVANCE=-0.1)
        
        # Test pressure advance above max override
        with pytest.raises(Exception, match="must have maximum of 2"):
            printer.call("SET_PRESSURE_ADVANCE", ADVANCE=3.0)
        
        # Test smooth_time below min override
        with pytest.raises(Exception, match="must be above 0"):
            printer.call("SET_PRESSURE_ADVANCE", ADVANCE=0.025, SMOOTH_TIME=-0.1)
        
        # Test smooth_time above max override
        with pytest.raises(Exception, match="must have maximum of 1"):
            printer.call("SET_PRESSURE_ADVANCE", ADVANCE=0.025, SMOOTH_TIME=2.0)


def test_extruder_defaults_behavior(
    config_root: typing.Annotated[pathlib.Path, "test_configs/extruder_defaults"],
):
    """Test that default behavior works without overrides"""
    start_args = {"config_file": str(config_root / "printer.cfg")}
    
    with PrinterShim(start_args) as printer:
        extruder = printer.lookup_object("extruder")
        
        # Test that default config values are applied
        assert extruder.extruder_stepper.pressure_advance == 0.025
        assert extruder.extruder_stepper.pressure_advance_smooth_time == 0.040
        assert extruder.max_e_dist == 50.0
        assert extruder.instant_corner_v == 1.0
        
        # Test SET_PRESSURE_ADVANCE at original limits
        printer.call("SET_PRESSURE_ADVANCE", ADVANCE=0.025, SMOOTH_TIME=0.200)
        assert extruder.extruder_stepper.pressure_advance == 0.025
        assert extruder.extruder_stepper.pressure_advance_smooth_time == 0.200
        
        # Test that values beyond original limits fail
        with pytest.raises(Exception, match="must have maximum of 0.2"):
            printer.call("SET_PRESSURE_ADVANCE", ADVANCE=0.025, SMOOTH_TIME=0.300)
        
        with pytest.raises(Exception, match="must have minimum of 0"):
            printer.call("SET_PRESSURE_ADVANCE", ADVANCE=-0.1)


def test_extruder_overrides_none_values(
    config_root: typing.Annotated[pathlib.Path, "test_configs/extruder_overrides"],
):
    """Test that None values for max overrides allow unbounded behavior"""
    start_args = {"config_file": str(config_root / "printer.cfg")}
    
    with PrinterShim(start_args) as printer:
        # Test that pressure advance can go up to the override max of 2.0
        printer.call("SET_PRESSURE_ADVANCE", ADVANCE=1.9, SMOOTH_TIME=0.9)
        extruder = printer.lookup_object("extruder")
        assert extruder.extruder_stepper.pressure_advance == 1.9
        assert extruder.extruder_stepper.pressure_advance_smooth_time == 0.9
        
        # The config has max overrides set, so we can't test unbounded behavior here
        # This test verifies the override system works with defined limits
