"""An interrupted MPC fan sweep stops the calibration fan."""

from types import SimpleNamespace

import pytest
import test_gcode_interrupt as commands

from klippy.extras import control_mpc
from klippy.gcode import WaitInterruption

dispatch = commands.dispatch


def test_mpc_fan_sweep_interrupt_restores_fan(dispatch):
    gcode, reactor, printer, _ = dispatch
    heater, _ = commands.calibration_heater(dispatch)
    speeds = []
    fan = SimpleNamespace(set_speed=speeds.append)
    original = SimpleNamespace(cooling_fan=fan)
    mpc = control_mpc.MpcCalibrate(printer, heater, original)
    mpc.measure_power = lambda *args: 1.0

    def wait_stable(gcmd, cycles):
        if speeds and speeds[-1] == 1.0:
            reactor.register_callback(
                lambda eventtime: gcode.run_script("HEATER_INTERRUPT")
            )
            gcmd.wait_while(lambda eventtime: True, interval=30.0)

    mpc.wait_stable = wait_stable

    def calibrate(gcmd):
        mpc.transfer_test(gcmd, 20.0, 5.0, 2, {"post_block_temp": 200.0})
        pytest.fail("cancelled calibration continued")

    gcode.register_command("MPC_FAN_TEST", calibrate)

    def action():
        with pytest.raises(WaitInterruption):
            gcode.run_script("MPC_FAN_TEST")

    commands.run_in_reactor(reactor, action)
    assert speeds == [0.0, 1.0, 0.0]
