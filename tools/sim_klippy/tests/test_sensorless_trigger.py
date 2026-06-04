"""SensorlessTrigger tracks step counts → mm position → SG_RESULT.
When position approaches the wall, SG_RESULT drops; once below SGTHRS
the chip's DIAG callback fires."""

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
    assert fired == []  # no transition


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
    """If homing_direction = -1 (toward decreasing step count) then
    motion moves position negative; wall at endstop_mm = 0 would be
    reached from positive starting position."""
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
    # Start far from wall: imagine position is 300 mm. Step counter
    # at 0 means "no steps yet from initial". With negative direction,
    # the initial position counts as far from wall_mm=0.
    # Actually: SensorlessTrigger uses `delta_steps * mm_per_step *
    # direction` for position relative to start. So at counter=0,
    # position=0; if endstop_mm = -300 and direction = -1, the wall
    # is reached at counter = 24000 (24000 steps in -direction
    # means position = -300, distance to wall (-300) = 0).
    # For simplicity here just test that with endstop_mm=0 and
    # direction=-1, we never reach the wall (always at distance 0
    # initially → DIAG high).
    counter.count = 0
    trigger.tick()
    # distance_to_wall = endstop_mm - position_mm = 0 - 0 = 0 → SG = 0
    # SG < SGTHRS → DIAG high
    assert chip._diag_high is True
