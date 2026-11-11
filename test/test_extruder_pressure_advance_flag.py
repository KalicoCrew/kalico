import types

import pytest

from klippy.kinematics import extruder

EA_INDEX = 3
TRAPQ_Y_R_ARG = 9


def _make_extruder(extruder_stepper):
    pe = extruder.PrinterExtruder.__new__(extruder.PrinterExtruder)
    pe.trapq = object()
    pe.extruder_stepper = extruder_stepper
    pe.appended = []
    pe.trapq_append = lambda *args: pe.appended.append(args)
    return pe


def _make_move(axes_d):
    move_d = sum(d * d for d in axes_d[:3]) ** 0.5 or abs(axes_d[EA_INDEX])
    return types.SimpleNamespace(
        axes_d=axes_d,
        axes_r=[d / move_d for d in axes_d],
        accel=1000.0,
        start_v=0.0,
        cruise_v=50.0,
        accel_t=0.05,
        cruise_t=0.1,
        decel_t=0.05,
        start_pos=[0.0, 0.0, 0.0, 0.0],
        end_pos=list(axes_d),
    )


def _pa_flag(pe, axes_d):
    pe.process_move(1.0, _make_move(axes_d), EA_INDEX)
    return pe.appended[-1][TRAPQ_Y_R_ARG]


@pytest.mark.parametrize(
    "extruder_stepper",
    [
        None,
        types.SimpleNamespace(pressure_advance=0.0),
        types.SimpleNamespace(pressure_advance=0.04),
    ],
    ids=["no-stepper", "stepper-pa-zero", "stepper-pa-set"],
)
def test_extruding_xy_move_enables_pressure_advance(extruder_stepper):
    pe = _make_extruder(extruder_stepper)
    assert _pa_flag(pe, [10.0, 0.0, 0.0, 0.5]) == 1.0


@pytest.mark.parametrize(
    "axes_d",
    [[10.0, 0.0, 0.0, -0.5], [0.0, 0.0, 0.0, 5.0], [0.0, 0.0, 0.0, -5.0]],
    ids=["xy-retract", "extrude-only", "retract-only"],
)
def test_non_extruding_xy_moves_disable_pressure_advance(axes_d):
    pe = _make_extruder(types.SimpleNamespace(pressure_advance=0.04))
    assert _pa_flag(pe, axes_d) == 0.0
