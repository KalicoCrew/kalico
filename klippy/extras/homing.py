import logging

from klippy.bridge_endstop import AXIS_ENDSTOP_IDS, BridgeEndstop

HOMING_POLL_PERIOD = 0.001
TRIP_DEADLINE_MARGIN = 5.0
NO_MOVEMENT_EPSILON = 0.005


class Homing:
    def __init__(self, config):
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object("pins")

        self._axes = {}
        for axis_index, axis_name in enumerate("xyz"):
            section = "stepper_" + axis_name
            if not config.has_section(section):
                continue
            stepper_config = config.getsection(section)
            endstop_pin = stepper_config.get("endstop_pin", None)
            if endstop_pin is None:
                continue
            pin_params = ppins.parse_pin(
                endstop_pin, can_invert=True, can_pullup=True
            )
            chip = pin_params["chip"]
            if hasattr(chip, "setup_bridge_endstop"):
                entry = self._provider_entry(
                    stepper_config, axis_index, chip, pin_params
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

        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("G28", self.cmd_G28, desc="Home")
        gcode.register_command(
            "_HOME_TEST",
            self.cmd_HOME_TEST,
            desc="Bench only: home one axis with override SPEED/MAX_TRAVEL",
        )

        query_endstops = self.printer.load_object(config, "query_endstops")
        for axis_index in sorted(self._axes):
            query_endstops.register_endstop(
                self._axes[axis_index]["endstop"], "xyz"[axis_index]
            )

    def _provider_entry(self, stepper_config, axis_index, chip, pin_params):
        endstop = chip.setup_bridge_endstop(pin_params, axis_index)
        trigger_height = None
        if hasattr(chip, "get_position_endstop"):
            trigger_height = chip.get_position_endstop()
            if stepper_config.get("position_endstop", None) is not None:
                raise stepper_config.error(
                    "[%s] must not set position_endstop: its virtual endstop"
                    " '%s' supplies the trigger height"
                    % (stepper_config.get_name(), pin_params["chip_name"])
                )
        return {
            "endstop": endstop,
            "provider": chip,
            "trigger_height": trigger_height,
        }

    def cmd_G28(self, gcmd):
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
        for s in rail.get_steppers():
            stepper_enable.motor_debug_enable(s.get_name(), True)

        self._set_homing_current(toolhead, rail, pre_homing=True)
        try:
            trip_pos, final_pos = self.trip_move(
                gcmd,
                toolhead,
                bridge,
                axis,
                direction,
                speed,
                max_travel,
                entry,
            )

            overshoot = final_pos[axis] - trip_pos[axis]
            newpos = list(toolhead.get_position())
            newpos[axis] = trigger_height + overshoot
            toolhead.set_position(newpos, homing_axes=[axis])
            logging.info(
                "homing: %s trigger=%.4f overshoot=%+.4f set %s=%.4f",
                "XYZ"[axis],
                trigger_height,
                overshoot,
                "XYZ"[axis],
                newpos[axis],
            )
            if hi.retract_dist:
                retractpos = list(toolhead.get_position())
                retractpos[axis] -= direction * hi.retract_dist
                toolhead.move(retractpos, hi.retract_speed)
                toolhead.wait_moves()
        except BaseException:
            # The primary error must reach the operator; a failed current
            # restore during the unwind is logged, not raised over it.
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
                        "%s endstop did not trigger within %.1fmm of travel"
                        % ("XYZ"[axis], max_travel)
                    )
                reactor.pause(reactor.monotonic() + 0.010)
        finally:
            if provider is not None and hasattr(provider, "trip_move_end"):
                provider.trip_move_end(entry)
        trip_pos, final_pos = result
        if abs(trip_pos[axis] - start_axis_pos) < NO_MOVEMENT_EPSILON:
            raise gcmd.error(
                "%s endstop triggered prior to movement — trigger is stuck"
                " or miswired" % ("XYZ"[axis],)
            )
        return trip_pos, final_pos


def load_config(config):
    return Homing(config)
