# Safe move
#
# Allows safe moves of an axis even when unhomed, as long as an endstop is available in the respective direction.
# This is used by modules like safe_z_home and dockable_probe during their Z hops, and be used via a Gcode
# command as well.
from klippy.extras.homing import MoveResult, any_complete


class SafeMove:
    """Execute safe single-axis moves using directional endstops."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.homing = self.printer.load_object(config, "homing")
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SAFE_MOVE", self.cmd_SAFE_MOVE, desc=self.cmd_SAFE_MOVE_help
        )
        self.last_axis = None
        self.last_dist = None
        self.last_result = None

    def move(self, toolhead, axis, dist, speed, allow_unsafe=False):
        """Move on one axis, stopping early if protected endstops trigger."""
        axis_lower = axis.lower()
        if axis_lower not in "xyz":
            raise self.printer.command_error(
                "SAFE_MOVE: AXIS must be X, Y, or Z"
            )
        if dist == 0.0:
            return

        axis_idx = "xyz".index(axis_lower)
        positive = dist > 0.0

        kin = toolhead.get_kinematics()
        endstops = self._get_endstops(kin, axis_idx, positive, allow_unsafe)

        reactor = self.printer.get_reactor()
        curtime = reactor.monotonic()
        kin_status = kin.get_status(curtime)

        axis_min = kin_status["axis_minimum"][axis_idx]
        axis_max = kin_status["axis_maximum"][axis_idx]

        was_unhomed = False
        position = toolhead.get_position()
        if axis_lower not in kin_status["homed_axes"]:
            was_unhomed = True
            # Assume a safe coordinate so the requested move stays in range.
            position[axis_idx] = axis_max if dist < 0.0 else axis_min
            toolhead.set_position(position, homing_axes=[axis_idx])
            position = toolhead.get_position()

        target_pos = list(position)
        target_pos[axis_idx] = position[axis_idx] + dist
        # Clamp moves for homed axes to avoid out-of-range errors.
        target_pos[axis_idx] = max(
            axis_min, min(axis_max, target_pos[axis_idx])
        )
        if target_pos[axis_idx] == position[axis_idx]:
            return

        try:
            if endstops:
                epos, res = self.homing.endstop_move(
                    endstops,
                    target_pos,
                    speed,
                    complete=any_complete,
                )
                self.last_dist = epos[axis_idx] - position[axis_idx]
                self.last_result = res

                if res == MoveResult.ALREADY_AT_ENDSTOP:
                    raise self.printer.command_error(
                        "Toolhead is already at endstop - unsafe to continue."
                    )
            elif allow_unsafe:
                move_cmd = [None, None, None, None]
                move_cmd[axis_idx] = target_pos[axis_idx]
                toolhead.manual_move(move_cmd, speed)

                self.last_dist = dist
                self.last_result = MoveResult.FULL_MOVE
            else:
                raise self.printer.command_error(
                    "SAFE_MOVE: No endstop protects axis %s in the %s direction"
                    % (
                        axis.upper(),
                        "positive" if positive else "negative",
                    )
                )

            self.last_axis = axis_lower
        finally:
            if was_unhomed:
                kin.clear_homing_state([axis_idx])

    def _get_endstops(self, kin, axis_idx, positive, allow_unsafe):
        endstops = kin.get_endstops_for_safe_move(axis_idx, positive)
        if endstops is None:
            if allow_unsafe:
                return []
            raise self.printer.command_error(
                f"SAFE_MOVE: kinematics do not support axis {'XYZ'[axis_idx]} in the {'positive' if positive else 'negative'} direction"
            )
        if len(endstops) == 0 and not allow_unsafe:
            raise self.printer.command_error(
                f"SAFE_MOVE: No endstops configured for axis {'XYZ'[axis_idx]} in the {'positive' if positive else 'negative'} direction"
            )
        return endstops

    cmd_SAFE_MOVE_help = "Perform a safe axis move"

    def cmd_SAFE_MOVE(self, gcmd):
        axis = gcmd.get("AXIS", None)
        if axis is None:
            raise gcmd.error("AXIS must be specified")

        dist = gcmd.get_float("DIST")
        if dist == 0.0:
            return
        speed = gcmd.get_float("SPEED", above=0.0)
        allow_unsafe = gcmd.get_int("ALLOW_UNSAFE", 0)

        toolhead = self.printer.lookup_object("toolhead")
        try:
            self.move(
                toolhead, axis, dist, speed, allow_unsafe=bool(allow_unsafe)
            )
        except self.printer.command_error as err:
            raise gcmd.error(str(err))

    def get_status(self, eventtime):
        return {
            "last_axis": self.last_axis,
            "last_dist": self.last_dist,
            "last_result": self.last_result,
        }


def load_config(config):
    return SafeMove(config)
