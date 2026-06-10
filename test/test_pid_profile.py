import pathlib
import typing

from klippy_testing import PrinterShim

import klippy.extras.heaters as heaters


class _FakeControl:
    def __init__(self, profile):
        self._profile = profile

    def get_profile(self):
        return self._profile


class _FakeHeater:
    """Minimal stand-in for a Heater exposing what save_profile() touches."""

    def __init__(self, configfile, gcode, control):
        self.short_name = "heater_bed"
        self.configfile = configfile
        self.gcode = gcode
        self._control = control

    def get_control(self):
        return self._control


def _make_pmgr(heater):
    pmgr = heaters.Heater.ProfileManager.__new__(heaters.Heater.ProfileManager)
    pmgr.outer_instance = heater
    pmgr.profiles = {}
    return pmgr


def test_save_profile_persists_inner_pid_values(
    config_root: typing.Annotated[pathlib.Path, "test_configs/autosave"],
):
    start_args = {"config_file": str(config_root / "printer.cfg")}
    with PrinterShim(start_args) as printer:
        pconfig = printer.lookup_object("configfile")
        # read_main_config() initializes the autosave fileconfig that
        # configfile.set() writes into.
        printer.load_config()

        profile = {
            "pid_target": 70.0,
            "pid_tolerance": 0.02,
            "control": "dual_loop_pid",
            "smooth_time": None,
            "pid_kp": 42.894,
            "pid_ki": 0.376,
            "pid_kd": 1224.094,
            "inner_pid_kp": 60.499,
            "inner_pid_ki": 2.425,
            "inner_pid_kd": 377.360,
            "name": "default",
        }
        heater = _FakeHeater(
            pconfig, printer.lookup_object("gcode"), _FakeControl(profile)
        )
        pmgr = _make_pmgr(heater)

        pmgr.save_profile(profile_name="default", verbose=False)

        pending = pconfig.status_save_pending["heater_bed"]
        # The recalibrated outer-loop values are saved...
        assert pending["pid_kp"] == "42.894"
        # ...and the inner-loop values must be saved too.
        assert pending["inner_pid_kp"] == "60.499"
        assert pending["inner_pid_ki"] == "2.425"
        assert pending["inner_pid_kd"] == "377.360"
