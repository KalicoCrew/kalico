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
        # 50/mm: 20mm from wall → 1000 SG; 1mm → 50 SG. Threshold 80 fires at ~1.6mm.
        sg = max(0, min(1023, int(distance_to_wall * 50)))
        self._chip.set_load(sg)
        self._chip.maybe_trigger_diag(self._sg_threshold)
