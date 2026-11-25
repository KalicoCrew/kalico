from types import SimpleNamespace

import pytest

from klippy.extras.z_thermal_adjust import ZThermalAdjuster


class Sink:
    def __init__(self):
        self.moves = []

    def move(self, pos, speed):
        self.moves.append(list(pos))


def make(enabled, homed):
    a = ZThermalAdjuster.__new__(ZThermalAdjuster)
    a.temp_coeff = 0.0064
    a.off_above_z = 1e9
    a.max_z_adjust_mm = 1e9
    a.z_step_dist = 0.001
    a.z_adjust_mm = 0.0
    a.smoothed_temp = 25.0
    a.ref_temperature = 20.0  # drifted: would give -0.032
    a.adjust_enable = enabled
    a.last_position = [0.0, 0.0, 5.0, 0.0]
    a.next_transform = Sink()
    a.toolhead = SimpleNamespace(
        get_status=lambda t: {"homed_axes": "xyz" if homed else "xy"}
    )
    a.printer = SimpleNamespace(
        get_reactor=lambda: SimpleNamespace(monotonic=lambda: 0.0)
    )
    return a


@pytest.mark.parametrize("enabled", [True, False])
def test_xy_move_with_z_unhomed_has_no_dz(enabled):
    a = make(enabled, homed=False)
    a.z_adjust_mm = 0.0004
    a.move([10.0, 0.0, 5.0, 0.0], 100)
    assert a.next_transform.moves[-1][2] == pytest.approx(5.0004)
    assert a.z_adjust_mm == 0.0004


def test_disabled_never_updates_offset():
    a = make(False, homed=True)
    a.calc_adjust([0.0, 0.0, 5.0, 0.0])
    assert a.z_adjust_mm == 0.0


def test_enabled_homed_updates_offset():
    a = make(True, homed=True)
    a.calc_adjust([0.0, 0.0, 5.0, 0.0])
    assert a.z_adjust_mm == pytest.approx(-0.032)
