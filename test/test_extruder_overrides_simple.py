import pathlib
import typing

from klippy_testing import PrinterShim


def test_smooth_time_override_loaded(
    config_root: typing.Annotated[
        pathlib.Path, "test_configs/extruder_overrides"
    ],
):
    """Test that the smooth_time override max value is loaded correctly"""
    start_args = {"config_file": str(config_root / "printer.cfg")}

    with PrinterShim(start_args) as printer:
        config = printer.load_config()
        danger_options = printer.lookup_object("danger_options")

        assert danger_options.override_pressure_advance_smooth_time_max == 1.0


def test_smooth_time_override_default(
    config_root: typing.Annotated[
        pathlib.Path, "test_configs/extruder_defaults"
    ],
):
    """Test that the default smooth_time override is 0.200"""
    start_args = {"config_file": str(config_root / "printer.cfg")}

    with PrinterShim(start_args) as printer:
        config = printer.load_config()
        danger_options = printer.lookup_object("danger_options")

        assert danger_options.override_pressure_advance_smooth_time_max == 0.200


def test_smooth_time_config_accepted_with_override(
    config_root: typing.Annotated[
        pathlib.Path, "test_configs/extruder_overrides"
    ],
):
    """Test that smooth_time beyond default limit is accepted with override"""
    start_args = {"config_file": str(config_root / "printer.cfg")}

    with PrinterShim(start_args) as printer:
        config = printer.load_config()
        extruder_section = config.getsection("extruder")

        # 0.8 would normally exceed the 0.200 default limit
        # but should be accepted because of the override
        assert extruder_section.getfloat("pressure_advance_smooth_time") == 0.8
