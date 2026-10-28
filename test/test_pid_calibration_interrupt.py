"""Cancellation during dual-loop PID cooldown leaves the heater off."""

import pytest
import test_gcode_interrupt as commands

from klippy.extras import heaters, pid_calibrate
from klippy.gcode import WaitInterruption

dispatch = commands.dispatch


def test_primary_pid_cooldown_interrupt_turns_off_heater(dispatch):
    gcode, reactor, printer, _ = dispatch
    manager = commands.heater_manager(dispatch)
    heater, _ = commands.calibration_heater(dispatch)
    original = heaters.ControlDualLoopPID.__new__(heaters.ControlDualLoopPID)
    original.inner_target_temp = 120.0
    original.check_busy = lambda *args: True
    heater.control = original
    pid = pid_calibrate.PIDCalibrate.__new__(pid_calibrate.PIDCalibrate)
    pid.printer = printer

    def calibrate(gcmd):
        with heater.calibration() as calibration:
            pid._calibrate(
                manager, heater, 70.0, 0.02, False, gcmd, calibration, False
            )
        pytest.fail("cancelled calibration continued")

    def interrupt(eventtime):
        assert heater.target_temp > 0.0
        assert heater.control is original
        assert gcode.get_mutex().test()
        gcode.run_script("HEATER_INTERRUPT")

    def action():
        reactor.register_callback(interrupt, reactor.monotonic() + 0.005)
        with pytest.raises(WaitInterruption):
            gcode.run_script("PID_COOLDOWN")

    gcode.register_command("PID_COOLDOWN", calibrate)
    commands.run_in_reactor(reactor, action)
    assert heater.target_temp == 0.0
    assert heater.control is original
    assert heater._calibration is None
