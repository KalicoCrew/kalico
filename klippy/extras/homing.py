import contextlib
import logging

from klippy import structured_log
from klippy.bridge_endstop import AXIS_ENDSTOP_IDS, BridgeEndstop

HOMING_POLL_PERIOD = 0.001
TRIP_DEADLINE_MARGIN = 5.0
NO_MOVEMENT_EPSILON = 0.005


def _endstop_section(config, axis_name):
    for prefix in ("stepper_", "servo_"):
        section = prefix + axis_name
        if config.has_section(section):
            return section
    return None


def _enable_homing_motors(stepper_enable, rail):
    steppers = rail.get_steppers()
    if not steppers:
        stepper_enable.motor_debug_enable(rail.get_name(), True)
        return
    for s in steppers:
        stepper_enable.motor_debug_enable(s.get_name(), True)


@contextlib.contextmanager
def _servo_drive_limits(bridge, handle, limits):
    if handle is None or limits is None:
        yield
        return
    bridge.set_drive_limits(handle, limits[0], limits[1])
    try:
        yield
    except BaseException:
        try:
            bridge.restore_drive_limits(handle)
        except Exception:
            logging.warning(
                "homing: restore_drive_limits failed while handling a"
                " homing error",
                exc_info=True,
            )
        raise
    bridge.restore_drive_limits(handle)


def _run_servo_guarded_trip(
    gcmd, bridge, axis, stepper_enable, rail, servo_handle, servo_limits, trip
):
    try:
        with _servo_drive_limits(bridge, servo_handle, servo_limits):
            result = trip()
        _check_servo_drive_fault(gcmd, bridge, axis, servo_handle)
    except BaseException:
        if servo_handle is not None:
            stepper_enable.motor_debug_enable(rail.get_name(), False)
        raise
    return result


def _check_servo_drive_fault(gcmd, bridge, axis, servo_handle):
    if servo_handle is None:
        return
    fault = bridge.take_drive_fault(servo_handle)
    if fault is not None:
        raise gcmd.error(
            "%s homing: drive fault 0x%04x at endstop contact — "
            "following-error/torque limit exceeded" % ("XYZ"[axis], fault)
        )


def _homed_axis_position(provider, axis, trip_pos, final_pos, trigger_height):
    if provider is not None and hasattr(provider, "measured_trip_position"):
        measured = provider.measured_trip_position(axis, trip_pos, final_pos)
        if measured is not None:
            return measured
    return trigger_height + (final_pos[axis] - trip_pos[axis])


def _verify_latched_trip(gcmd, axis, endstop, doorbell_clock):
    query = getattr(endstop, "query_trip_state", None)
    if query is None:
        return
    latch = query()
    if not latch["tripped"]:
        raise gcmd.error(
            "%s endstop: doorbell event arrived but the MCU latch shows no"
            " trip — duplicate or stale trip event" % ("XYZ"[axis],)
        )
    if latch["trip_clock"] != (doorbell_clock & 0xFFFFFFFF):
        raise gcmd.error(
            "%s endstop: latch/doorbell clock mismatch — latch=%d"
            " doorbell_low32=%d"
            % (
                "XYZ"[axis],
                latch["trip_clock"],
                doorbell_clock & 0xFFFFFFFF,
            )
        )


def _no_trigger_error_message(axis, endstop, max_travel):
    base = "%s endstop did not trigger within %.1fmm of travel" % (
        "XYZ"[axis],
        max_travel,
    )
    query = getattr(endstop, "query_trip_state", None)
    if query is None:
        return base
    latch = query()
    if latch["tripped"]:
        return (
            "%s endstop tripped (latched clock %d) but the trip event was"
            " lost — doorbell never reached the host"
            % ("XYZ"[axis], latch["trip_clock"])
        )
    return base


class Homing:
    def __init__(self, config):
        self.printer = config.get_printer()
        self._config = config
        self._axes = None

        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("G28", self.cmd_G28, desc="Home")
        gcode.register_command(
            "_HOME_TEST",
            self.cmd_HOME_TEST,
            desc="Bench only: home one axis with override SPEED/MAX_TRAVEL",
        )

    def resolve_endstops(self):
        if self._config is None:
            raise self.printer.config_error(
                "homing: resolve_endstops called twice"
            )
        config, self._config = self._config, None
        ppins = self.printer.lookup_object("pins")

        self._axes = {}
        for axis_index, axis_name in enumerate("xyz"):
            section = _endstop_section(config, axis_name)
            if section is None:
                continue
            axis_config = config.getsection(section)
            endstop_pin = axis_config.get("endstop_pin", None)
            if endstop_pin is None:
                continue
            pin_params = ppins.parse_pin(
                endstop_pin, can_invert=True, can_pullup=True
            )
            chip = pin_params["chip"]
            if hasattr(chip, "setup_bridge_endstop"):
                entry = self._provider_entry(
                    axis_config, axis_index, chip, pin_params
                )
            elif hasattr(chip, "create_oid"):
                entry = {
                    "endstop": BridgeEndstop(
                        pin_params, AXIS_ENDSTOP_IDS[axis_index]
                    ),
                    "provider": None,
                    "trigger_height": None,
                }
            else:
                raise config.error(
                    "endstop_pin '%s' in [%s]: chip '%s' is neither an MCU"
                    " nor a virtual endstop provider"
                    % (endstop_pin, section, pin_params["chip_name"])
                )
            self._axes[axis_index] = entry

        query_endstops = self.printer.load_object(config, "query_endstops")
        for axis_index in sorted(self._axes):
            query_endstops.register_endstop(
                self._axes[axis_index]["endstop"], "xyz"[axis_index]
            )

    def _provider_entry(self, axis_config, axis_index, chip, pin_params):
        endstop = chip.setup_bridge_endstop(pin_params, axis_index)
        trigger_height = None
        if hasattr(chip, "get_position_endstop"):
            trigger_height = chip.get_position_endstop()
            if axis_config.get("position_endstop", None) is not None:
                raise axis_config.error(
                    "[%s] must not set position_endstop: its virtual endstop"
                    " '%s' supplies the trigger height"
                    % (axis_config.get_name(), pin_params["chip_name"])
                )
        return {
            "endstop": endstop,
            "provider": chip,
            "trigger_height": trigger_height,
        }

    def cmd_G28(self, gcmd):
        if self._axes is None:
            raise gcmd.error("G28: homing endstops were never resolved")
        requested = [
            i for i, a in enumerate("XYZ") if gcmd.get(a, None) is not None
        ]
        if not requested:
            requested = sorted(self._axes.keys())
        toolhead = self.printer.lookup_object("toolhead")
        bridge = self.printer.lookup_object("motion_bridge")
        kin = toolhead.get_kinematics()
        for axis in requested:
            entry = self._axes.get(axis)
            if entry is None:
                raise gcmd.error("G28: axis %s has no endstop" % ("XYZ"[axis],))
            self._home_axis(gcmd, toolhead, bridge, kin, axis, entry)

    def cmd_HOME_TEST(self, gcmd):
        if self._axes is None:
            raise gcmd.error("_HOME_TEST: homing endstops were never resolved")
        axis_name = gcmd.get("AXIS").upper()
        if axis_name not in ("X", "Y", "Z"):
            raise gcmd.error("_HOME_TEST: AXIS must be X, Y, or Z")
        axis = "XYZ".index(axis_name)
        entry = self._axes.get(axis)
        if entry is None:
            raise gcmd.error("_HOME_TEST: axis %s has no endstop" % axis_name)
        speed = gcmd.get_float("SPEED", None, above=0.0)
        max_travel = gcmd.get_float("MAX_TRAVEL", None, above=0.0)
        toolhead = self.printer.lookup_object("toolhead")
        bridge = self.printer.lookup_object("motion_bridge")
        kin = toolhead.get_kinematics()
        self._home_axis(
            gcmd, toolhead, bridge, kin, axis, entry, speed, max_travel
        )

    def _home_axis(
        self,
        gcmd,
        toolhead,
        bridge,
        kin,
        axis,
        entry,
        speed_override=None,
        max_travel_override=None,
    ):
        rail = kin._axis_rails().get(axis)
        if rail is None:
            raise gcmd.error("G28: no rail for axis %s" % ("XYZ"[axis],))
        hi = rail.get_homing_info()
        pos_min, pos_max = rail.get_range()
        trigger_height = entry["trigger_height"]
        if trigger_height is None:
            trigger_height = hi.position_endstop
        direction = 1.0 if hi.positive_dir else -1.0
        speed = speed_override if speed_override is not None else hi.speed
        max_travel = (
            max_travel_override
            if max_travel_override is not None
            else abs(pos_max - pos_min)
        )

        stepper_enable = self.printer.lookup_object("stepper_enable")
        homing_deltas = [0.0, 0.0, 0.0]
        homing_deltas[axis] = 1.0
        for active_rail in kin.active_rails(*homing_deltas):
            _enable_homing_motors(stepper_enable, active_rail)

        servo_handle = None
        servo_limits = None
        if hasattr(rail, "get_node_name"):
            node = self.printer.lookup_object(
                "ethercat_node " + rail.get_node_name()
            )
            servo_handle = node.get_bridge_handle()
            servo_limits = rail.get_homing_drive_limits()

        self._set_homing_current(toolhead, rail, pre_homing=True)
        try:
            trip_pos, final_pos = _run_servo_guarded_trip(
                gcmd,
                bridge,
                axis,
                stepper_enable,
                rail,
                servo_handle,
                servo_limits,
                lambda: self.trip_move(
                    gcmd,
                    toolhead,
                    bridge,
                    axis,
                    direction,
                    speed,
                    max_travel,
                    entry,
                ),
            )

            newpos = list(toolhead.get_position())
            newpos[axis] = _homed_axis_position(
                entry["provider"], axis, trip_pos, final_pos, trigger_height
            )
            toolhead.set_position(newpos, homing_axes=[axis])
            structured_log.event(
                "homing",
                "axis_homed",
                msg="homing: %s trigger=%.4f overshoot=%+.4f set %s=%.4f"
                % (
                    "XYZ"[axis],
                    trigger_height,
                    final_pos[axis] - trip_pos[axis],
                    "XYZ"[axis],
                    newpos[axis],
                ),
                axis="XYZ"[axis],
                trigger_height=trigger_height,
                overshoot=final_pos[axis] - trip_pos[axis],
                homed_position=newpos[axis],
            )
            if hi.retract_dist:
                retractpos = list(toolhead.get_position())
                retractpos[axis] -= direction * hi.retract_dist
                toolhead.move(retractpos, hi.retract_speed)
                toolhead.wait_moves()
            _check_servo_drive_fault(gcmd, bridge, axis, servo_handle)
        except BaseException:
            try:
                self._set_homing_current(toolhead, rail, pre_homing=False)
            except Exception:
                logging.exception(
                    "homing: current restore failed during error unwind"
                )
            raise
        else:
            self._set_homing_current(toolhead, rail, pre_homing=False)

    def _set_homing_current(self, toolhead, rail, pre_homing):
        print_time = toolhead.get_last_move_time()
        dwell_time = 0.0
        for current_helper in rail.get_tmc_current_helpers():
            if current_helper is None:
                continue
            dwell_time = max(
                dwell_time,
                current_helper.set_current_for_homing(print_time, pre_homing),
            )
        if dwell_time:
            toolhead.dwell(dwell_time)

    def trip_move(
        self, gcmd, toolhead, bridge, axis, direction, speed, max_travel, entry
    ):
        endstop = entry["endstop"]
        endstop_mcu = endstop.bridge_mcu_handle()
        if endstop_mcu is None:
            raise gcmd.error(
                "trip_move: endstop MCU for axis %s is not attached to the"
                " bridge" % ("XYZ"[axis],)
            )
        if endstop.is_triggered():
            raise gcmd.error(
                "%s endstop already triggered — move off the trigger before"
                " homing or probing" % ("XYZ"[axis],)
            )
        toolhead.wait_moves()
        start_axis_pos = toolhead.get_position()[axis]
        provider = entry["provider"]
        if provider is not None and hasattr(provider, "trip_move_begin"):
            provider.trip_move_begin(entry)
        try:
            endstop.arm(HOMING_POLL_PERIOD)
            bridge.home_axis_start(
                axis,
                direction,
                speed,
                max_travel,
                endstop.endstop_id,
                endstop_mcu,
            )
            reactor = self.printer.get_reactor()
            deadline = (
                reactor.monotonic() + max_travel / speed + TRIP_DEADLINE_MARGIN
            )
            while True:
                try:
                    result = bridge.home_axis_poll()
                except Exception as e:
                    bridge.home_abort()
                    raise gcmd.error(
                        "%s trip move failed: %s" % ("XYZ"[axis], e)
                    )
                if result is not None:
                    break
                if reactor.monotonic() > deadline:
                    bridge.home_abort()
                    raise gcmd.error(
                        _no_trigger_error_message(axis, endstop, max_travel)
                    )
                reactor.pause(reactor.monotonic() + 0.010)
        finally:
            disarm = getattr(endstop, "disarm", None)
            if disarm is not None:
                try:
                    disarm()
                except Exception:
                    logging.exception(
                        "trip_move: remote trigger disarm failed during unwind"
                    )
            if provider is not None and hasattr(provider, "trip_move_end"):
                provider.trip_move_end(entry)
        trip_pos, final_pos, trip_clock = result
        _verify_latched_trip(gcmd, axis, endstop, trip_clock)
        if abs(trip_pos[axis] - start_axis_pos) < NO_MOVEMENT_EPSILON:
            raise gcmd.error(
                "%s endstop triggered prior to movement — trigger is stuck"
                " or miswired" % ("XYZ"[axis],)
            )
        return trip_pos, final_pos


def load_config(config):
    return Homing(config)
