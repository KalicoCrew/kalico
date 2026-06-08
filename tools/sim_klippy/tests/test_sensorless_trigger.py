import pytest

from tools.sim_klippy.orchestrator.sensorless_trigger import SensorlessTrigger
from tools.sim_klippy.orchestrator.tmc5160_emulator import TMC5160Emulator

pytestmark = pytest.mark.sim_unit


class _StepCounter:
    def __init__(self):
        self.count = 0

    def __call__(self):
        return self.count


def test_diag_low_when_far_from_wall():
    chip = TMC5160Emulator()
    fired = []
    chip.set_diag_callback(lambda h: fired.append(h))
    counter = _StepCounter()
    trigger = SensorlessTrigger(
        chip=chip,
        step_counter=counter,
        rotation_distance_mm=40.0,
        steps_per_rotation=200 * 16,
        endstop_mm=300.0,
        sg_threshold=80,
        homing_direction=1,
    )
    counter.count = 0
    trigger.tick()
    assert chip._diag_high is False
    assert fired == []


def test_diag_asserts_at_endstop():
    chip = TMC5160Emulator()
    fired = []
    chip.set_diag_callback(lambda h: fired.append(h))
    counter = _StepCounter()
    trigger = SensorlessTrigger(
        chip=chip,
        step_counter=counter,
        rotation_distance_mm=40.0,
        steps_per_rotation=200 * 16,
        endstop_mm=300.0,
        sg_threshold=80,
        homing_direction=1,
    )
    counter.count = 0
    trigger.tick()
    # 300 mm * (200 * 16) / 40 = 24000 steps to reach wall
    counter.count = 24000
    trigger.tick()
    assert chip._diag_high is True
    assert fired == [True]


def test_diag_clears_when_retreating():
    chip = TMC5160Emulator()
    fired = []
    chip.set_diag_callback(lambda h: fired.append(h))
    counter = _StepCounter()
    trigger = SensorlessTrigger(
        chip=chip,
        step_counter=counter,
        rotation_distance_mm=40.0,
        steps_per_rotation=3200,
        endstop_mm=300.0,
        sg_threshold=80,
        homing_direction=1,
    )
    counter.count = 24000
    trigger.tick()
    assert chip._diag_high is True
    counter.count = 12000
    trigger.tick()
    assert chip._diag_high is False
    assert fired == [True, False]


def test_negative_homing_direction():
    chip = TMC5160Emulator()
    counter = _StepCounter()
    trigger = SensorlessTrigger(
        chip=chip,
        step_counter=counter,
        rotation_distance_mm=40.0,
        steps_per_rotation=3200,
        endstop_mm=0.0,
        sg_threshold=80,
        homing_direction=-1,
    )
    counter.count = 0
    trigger.tick()
    # distance_to_wall = endstop_mm - position_mm = 0 - 0 = 0 → SG = 0
    # SG < SGTHRS → DIAG high
    assert chip._diag_high is True
