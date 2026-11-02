import pathlib
import typing

import pytest
from klippy_testing import PrinterShim, Restart

from klippy.extras import input_shaper, shaper_calibrate


def _save_calibration(start_args, shaper_name, shaper_freq):
    with PrinterShim(start_args) as printer:
        printer.load_config()
        shaper_calibrate.ShaperCalibrate(None).save_params(
            printer.lookup_object("configfile"), "x", shaper_name, shaper_freq
        )
        with pytest.raises(Restart):
            printer.call("SAVE_CONFIG")


def _load_status(start_args):
    with PrinterShim(start_args) as printer:
        config = printer.load_config()
        section = config.getsection("input_shaper")
        factory = input_shaper.ShaperFactory()
        shaper_x = factory.create_shaper("x", section)
        factory.create_shaper("y", section)
        # Raises on options that the selected shaper types do not read
        printer.lookup_object("configfile").check_unused_options(config, True)
        return shaper_x.params.get_status()


def test_calibrated_smoother_replaces_configured_frequency(
    config_root: typing.Annotated[
        pathlib.Path, "test_configs/input_shaper_save"
    ],
):
    start_args = {"config_file": str(config_root / "printer.cfg")}
    _save_calibration(start_args, "smooth_mzv", 79.6)

    status = _load_status(start_args)
    assert status["shaper_type"] == "smooth_mzv"
    assert status["smoother_freq"] == "79.600"


def test_calibrated_shaper_ignores_configured_smoother_frequency(
    config_root: typing.Annotated[
        pathlib.Path, "test_configs/input_shaper_save"
    ],
):
    start_args = {"config_file": str(config_root / "printer.cfg")}
    _save_calibration(start_args, "mzv", 61.2)

    status = _load_status(start_args)
    assert status["shaper_type"] == "mzv"
    assert status["shaper_freq"] == "61.200"


def test_recalibrating_between_shaper_and_smoother(
    config_root: typing.Annotated[
        pathlib.Path, "test_configs/input_shaper_save"
    ],
):
    start_args = {"config_file": str(config_root / "printer.cfg")}
    _save_calibration(start_args, "smooth_ei", 70.1)
    _save_calibration(start_args, "ei", 55.5)

    status = _load_status(start_args)
    assert status["shaper_type"] == "ei"
    assert status["shaper_freq"] == "55.500"

    _save_calibration(start_args, "smooth_zv", 44.4)

    status = _load_status(start_args)
    assert status["shaper_type"] == "smooth_zv"
    assert status["smoother_freq"] == "44.400"
