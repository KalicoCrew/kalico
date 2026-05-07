"""Models sensorless-homing StallGuard triggering for one stepper.

Polls a step-count source (firmware FFI or sim shim), converts to mm
position via rotation_distance / steps_per_rotation, computes a
synthetic SG_RESULT that decreases as the position approaches the
wall, and asks the chip emulator to assert/clear DIAG via its
callback when SG_RESULT crosses a threshold."""
from typing import Callable


class SensorlessTrigger:
    def __init__(
        self,
        chip,
        step_counter: Callable[[], int],
        rotation_distance_mm: float,
        steps_per_rotation: int,
        endstop_mm: float,
        sg_threshold: int,
        homing_direction: int,
    ):
        self._chip = chip
        self._step_counter = step_counter
        self._mm_per_step = rotation_distance_mm / steps_per_rotation
        self._endstop_mm = endstop_mm
        self._sg_threshold = sg_threshold
        self._direction = homing_direction
        self._initial_count = step_counter()

    def tick(self) -> None:
        delta_steps = self._step_counter() - self._initial_count
        position_mm = delta_steps * self._mm_per_step * self._direction
        distance_to_wall = abs(self._endstop_mm - position_mm)
        # Linear model: SG_RESULT = max(0, min(1023, distance * 50))
        # 50/mm gives a reasonable curve: 20mm from wall → 1000 SG;
        # 1mm → 50 SG. Threshold of 80 fires at ~1.6mm from wall.
        sg = max(0, min(1023, int(distance_to_wall * 50)))
        self._chip.set_load(sg)
        self._chip.maybe_trigger_diag(self._sg_threshold)
