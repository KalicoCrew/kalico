from __future__ import annotations

import contextlib
import typing

if typing.TYPE_CHECKING:
    from klippy.extras.gcode_move import GCodeMove
    from klippy.printer import Printer


class MoveAPI:
    def __init__(self, printer: Printer):
        self._gcode_move: GCodeMove = printer.lookup_object("gcode_move")

    def __call__(
        self,
        x: typing.Optional[float] = None,
        y: typing.Optional[float] = None,
        z: typing.Optional[float] = None,
        e: typing.Optional[float] = None,
        *,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        de: float = 0.0,
        speed: typing.Optional[float] = None,
    ):
        """
        Move to a position

        `speed` is in mm/s and unlike `G1 Fx` only affects this movement.
        """
        pos = self._gcode_move.last_position
        newpos = [
            (x if x is not None else pos[0]) + dx,
            (y if y is not None else pos[1]) + dy,
            (z if z is not None else pos[2]) + dz,
            (e if e is not None else pos[3]) + de,
        ]
        self._gcode_move.move_to(newpos, speed)

    def set_gcode_offset(
        self,
        x: typing.Optional[float] = None,
        y: typing.Optional[float] = None,
        z: typing.Optional[float] = None,
        *,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        move: bool = False,
        speed: float = None,
    ):
        """
        Set GCode offsets

        `speed` is in mm/s
        """
        offsets = self._gcode_move.homing_position
        x = (x if x is not None else offsets[0]) + dx
        y = (y if y is not None else offsets[1]) + dy
        z = (z if z is not None else offsets[2]) + dz
        self._gcode_move.set_gcode_offset(x, y, z, move=move, speed=speed)

    def set_speed(self, speed: float):
        "Set the speed for future moves in mm/s"
        self._gcode_move.set_speed(speed)

    def set_speed_factor(self, speed_factor: float = 1.0):
        "Set the movement speed factor"
        self._gcode_move.set_speed_factor(speed_factor)

    def set_extrude_factor(self, extrude_factor: float = 1.0):
        self._gcode_move.set_extrude_factor(extrude_factor)

    @contextlib.contextmanager
    def save_state(
        self,
        restore_position: bool = False,
        speed: float = None,
    ):
        """
        Save the current gcode state
        """
        state = self._gcode_move.get_state()
        try:
            yield state
        finally:
            self._gcode_move.restore_state(
                state,
                restore_position=restore_position,
                speed=speed,
            )
