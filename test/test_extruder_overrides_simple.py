import pathlib
import typing

from klippy_testing import PrinterShim


def test_extruder_overrides_danger_options_values(
    config_root: typing.Annotated[
        pathlib.Path, "test_configs/extruder_overrides"
    ],
):
    """Test that danger_options override max values are loaded correctly"""
    start_args = {"config_file": str(config_root / "printer.cfg")}

    with PrinterShim(start_args) as printer:
        config = printer.load_config()
        danger_options = printer.lookup_object("danger_options")

        assert danger_options.override_pressure_advance_max == 2.0
        assert danger_options.override_pressure_advance_smooth_time_max == 1.0
        assert danger_options.override_max_extrude_only_distance_max == 200.0
        assert danger_options.override_instantaneous_corner_velocity_max == 10.0
        assert danger_options.override_max_extrude_cross_section_max == 100.0
        assert danger_options.override_max_extrude_only_velocity_max == 2000.0
        assert danger_options.override_max_extrude_only_accel_max == 20000.0


def test_extruder_overrides_defaults(
    config_root: typing.Annotated[
        pathlib.Path, "test_configs/extruder_defaults"
    ],
):
    """Test that default override values are None (unbounded) without overrides"""
    start_args = {"config_file": str(config_root / "printer.cfg")}

    with PrinterShim(start_args) as printer:
        config = printer.load_config()
        danger_options = printer.lookup_object("danger_options")

        assert danger_options.override_pressure_advance_max is None
        assert danger_options.override_pressure_advance_smooth_time_max == 0.200
        assert danger_options.override_max_extrude_only_distance_max is None
        assert danger_options.override_instantaneous_corner_velocity_max is None
        assert danger_options.override_max_extrude_cross_section_max is None
        assert danger_options.override_max_extrude_only_velocity_max is None
        assert danger_options.override_max_extrude_only_accel_max is None


def test_extruder_overrides_restrictive(
    config_root: typing.Annotated[
        pathlib.Path, "test_configs/extruder_overrides_fail"
    ],
):
    """Test that partial override values load correctly"""
    start_args = {"config_file": str(config_root / "printer.cfg")}

    with PrinterShim(start_args) as printer:
        config = printer.load_config()
        danger_options = printer.lookup_object("danger_options")

        assert danger_options.override_pressure_advance_max == 2.0
        assert danger_options.override_pressure_advance_smooth_time_max == 1.0
        # Unset overrides should remain at defaults
        assert danger_options.override_max_extrude_only_distance_max is None
        assert danger_options.override_max_extrude_cross_section_max is None


def test_extruder_config_validation_with_overrides(
    config_root: typing.Annotated[
        pathlib.Path, "test_configs/extruder_overrides"
    ],
):
    """Test that extruder config values beyond default limits are accepted with overrides"""
    start_args = {"config_file": str(config_root / "printer.cfg")}

    with PrinterShim(start_args) as printer:
        config = printer.load_config()
        extruder_section = config.getsection("extruder")

        # These values would normally be outside the default limits
        # but should be accepted because of the overrides
        assert extruder_section.getfloat("pressure_advance") == 1.5
        assert extruder_section.getfloat("pressure_advance_smooth_time") == 0.8
        assert extruder_section.getfloat("max_extrude_only_distance") == 150.0
        assert extruder_section.getfloat("instantaneous_corner_velocity") == 8.0
        assert extruder_section.getfloat("max_extrude_cross_section") == 80.0
        assert extruder_section.getfloat("max_extrude_only_velocity") == 1500.0
        assert extruder_section.getfloat("max_extrude_only_accel") == 15000.0
