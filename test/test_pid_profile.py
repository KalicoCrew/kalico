import pathlib
import typing

from klippy_testing import PrinterShim

import klippy.extras.heaters as heaters
import klippy.gcode

PID_PARAM_BASE = heaters.PID_PARAM_BASE


def _dual_loop_profile():
    """A fresh dual_loop_pid profile dict (tests mutate it)."""
    return {
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


def _make_pmgr(heater):
    pmgr = heaters.Heater.ProfileManager.__new__(heaters.Heater.ProfileManager)
    pmgr.outer_instance = heater
    pmgr.profiles = {}
    return pmgr


def _gcmd(heater, params):
    return klippy.gcode.GCodeCommand(
        heater.gcode, "PID_PROFILE", "PID_PROFILE", params, False
    )


######################################################################
# save_profile() autosave persistence
######################################################################


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


def test_save_profile_persists_inner_pid_values(
    config_root: typing.Annotated[pathlib.Path, "test_configs/autosave"],
):
    start_args = {"config_file": str(config_root / "printer.cfg")}
    with PrinterShim(start_args) as printer:
        pconfig = printer.lookup_object("configfile")
        # read_main_config() initializes the autosave fileconfig that
        # configfile.set() writes into.
        printer.load_config()

        heater = _FakeHeater(
            pconfig,
            printer.lookup_object("gcode"),
            _FakeControl(_dual_loop_profile()),
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


######################################################################
# set_values() / get_values() / load_profile() over a real control
######################################################################


class _FakeReactor:
    def monotonic(self):
        return 0.0


class _FakeConfig:
    def getfloat(self, key, default=None, **kw):
        return {"inner_target_temp": 135.0}.get(key, default)

    def error(self, msg):
        return Exception(msg)


class _FakeGCode:
    error = Exception

    def __init__(self):
        self.messages = []

    def respond_info(self, msg):
        self.messages.append(msg)

    def respond_raw(self, msg):
        self.messages.append(msg)


class _FakeConfigFile:
    def __init__(self):
        self.data = {}

    def set(self, section, key, val):
        self.data.setdefault(section, {})[key] = val


class _FakeDualLoopHeater:
    """Heater stand-in that exercises the real dual-loop control path."""

    short_name = "heater_bed"

    def __init__(self):
        self.reactor = _FakeReactor()
        self.config = _FakeConfig()
        self.gcode = _FakeGCode()
        self.configfile = _FakeConfigFile()
        self.smooth_time = 1.0
        self.target_temp = 0.0
        self.control = None

    def get_max_power(self):
        return 1.0

    def get_smooth_time(self):
        return 1.0

    def get_temp(self, eventtime):
        return (25.0, self.target_temp)

    def set_inv_smooth_time(self, value):
        pass

    def get_control(self):
        return self.control

    def set_control(self, control, keep_target=True):
        old = self.control
        self.control = control
        if not keep_target:
            self.target_temp = 0.0
        return old

    # Use the real lookup_control so ControlDualLoopPID is constructed.
    lookup_control = heaters.Heater.lookup_control


def _make_dual_loop_setup():
    profile = _dual_loop_profile()
    heater = _FakeDualLoopHeater()
    heater.control = heaters.ControlDualLoopPID(profile, heater)
    pmgr = _make_pmgr(heater)
    pmgr.profiles = {"default": profile}
    return heater, pmgr


def test_set_values_dual_loop_preserves_inner_loop():
    """SET_VALUES on a dual_loop_pid heater must not crash and must keep
    the inner-loop gains when they are not supplied on the command line."""
    heater, pmgr = _make_dual_loop_setup()

    gcmd = _gcmd(
        heater,
        {
            "SET_VALUES": "default",
            "TARGET": "70",
            "KP": "50",
            "KI": "0.5",
            "KD": "1300",
        },
    )

    pmgr.set_values("default", gcmd, True)

    control = heater.get_control()
    assert isinstance(control, heaters.ControlDualLoopPID)
    # Outer loop gains updated from the command...
    assert control.primary_pid.Kp == 50.0 / PID_PARAM_BASE
    # ...inner loop gains preserved from the previous profile.
    assert control.secondary_pid.Kp == 60.499 / PID_PARAM_BASE
    assert pmgr.profiles["default"]["inner_pid_kp"] == 60.499
    assert pmgr.profiles["default"]["inner_pid_ki"] == 2.425
    assert pmgr.profiles["default"]["inner_pid_kd"] == 377.360


def test_set_values_dual_loop_accepts_inner_params():
    """SET_VALUES accepts INNER_KP/INNER_KI/INNER_KD for the inner loop."""
    heater, pmgr = _make_dual_loop_setup()

    gcmd = _gcmd(
        heater,
        {
            "SET_VALUES": "default",
            "TARGET": "70",
            "KP": "50",
            "KI": "0.5",
            "KD": "1300",
            "INNER_KP": "70",
            "INNER_KI": "3",
            "INNER_KD": "400",
        },
    )

    pmgr.set_values("default", gcmd, True)

    control = heater.get_control()
    assert control.secondary_pid.Kp == 70.0 / PID_PARAM_BASE
    assert control.secondary_pid.Ki == 3.0 / PID_PARAM_BASE
    assert control.secondary_pid.Kd == 400.0 / PID_PARAM_BASE
    assert pmgr.profiles["default"]["inner_pid_kp"] == 70.0


def test_get_values_dual_loop_reports_inner():
    """GET_VALUES reports the inner-loop gains for a dual_loop_pid heater."""
    heater, pmgr = _make_dual_loop_setup()

    gcmd = _gcmd(heater, {"GET_VALUES": "default"})
    pmgr.get_values("default", gcmd, True)

    output = "\n".join(heater.gcode.messages)
    assert "inner_pid_Kp=60.499" in output
    assert "inner_pid_Ki=2.425" in output
    assert "inner_pid_Kd=377.360" in output


def test_load_high_verbose_dual_loop_reports_inner():
    """LOAD ... VERBOSE=high reports the inner-loop gains."""
    heater, pmgr = _make_dual_loop_setup()

    gcmd = _gcmd(
        heater,
        {"LOAD": "default", "VERBOSE": "high", "LOAD_CLEAN": "1"},
    )
    pmgr.load_profile("default", gcmd, True)

    output = "\n".join(heater.gcode.messages)
    assert "inner_pid_Kp=60.499" in output
    assert "inner_pid_Ki=2.425" in output
    assert "inner_pid_Kd=377.360" in output
