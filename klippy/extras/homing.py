import logging

HOMING_POLL_PERIOD = 0.001
HOMING_TIMEOUT = 30.0


class _BridgeEndstop:
    def __init__(self, entry):
        self._entry = entry

    def query_endstop(self, print_time):
        return bool(self.query_state(print_time)["triggered"])

    def query_state(self, print_time):
        entry = self._entry
        params = entry["state_cmd"].send([entry["oid"]])
        invert = entry["invert"]
        return {
            "triggered": bool(params["pin_value"] ^ invert),
            "pin": params["pin_value"],
            "invert": invert,
            "armed": params["armed"],
        }


class Homing:
    def __init__(self, config):
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object("pins")

        self._axes = {}
        for axis_index, axis_name in enumerate("xyz"):
            section = "stepper_" + axis_name
            if not config.has_section(section):
                continue
            endstop_pin = config.getsection(section).get("endstop_pin", None)
            if endstop_pin is None or "virtual_endstop" in endstop_pin:
                continue
            pin_params = ppins.parse_pin(endstop_pin, can_invert=True, can_pullup=True)
            mcu = pin_params["chip"]
            entry = {
                "endstop_id": axis_index,
                "mcu": mcu,
                "oid": mcu.create_oid(),
                "pin": pin_params["pin"],
                "pullup": pin_params["pullup"],
                "invert": pin_params["invert"],
                "query_cmd": None,
            }
            self._axes[axis_index] = entry
            mcu.register_config_callback(self._make_build_config(entry))

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
                _BridgeEndstop(self._axes[axis_index]), "xyz"[axis_index]
            )

    def _make_build_config(self, entry):
        def build_config():
            entry["mcu"].add_config_cmd(
                "config_endstop oid=%d endstop_id=%d pin=%s pull_up=%d invert=%d"
                % (
                    entry["oid"],
                    entry["endstop_id"],
                    entry["pin"],
                    entry["pullup"],
                    entry["invert"],
                )
            )
            entry["query_cmd"] = entry["mcu"].lookup_command(
                "query_endstop oid=%c rest_ticks=%u"
            )
            entry["state_cmd"] = entry["mcu"].lookup_query_command(
                "endstop_query_state oid=%c",
                "endstop_state oid=%c armed=%c pin_value=%c",
                oid=entry["oid"],
            )

        return build_config

    def cmd_G28(self, gcmd):
        requested = [i for i, a in enumerate("XYZ") if gcmd.get(a, None) is not None]
        if not requested:
            requested = sorted(self._axes.keys())
        toolhead = self.printer.lookup_object("toolhead")
        bridge = self.printer.lookup_object("motion_bridge")
        kin = toolhead.get_kinematics()
        for axis in requested:
            entry = self._axes.get(axis)
            if entry is None:
                raise gcmd.error(
                    "G28: axis %s has no endstop" % ("XYZ"[axis],)
                )
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
        self._home_axis(gcmd, toolhead, bridge, kin, axis, entry, speed, max_travel)

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
        endstop_mcu = getattr(entry["mcu"], "_bridge_handle", None)
        if endstop_mcu is None:
            raise gcmd.error(
                "G28: endstop MCU for axis %s is not attached to the bridge"
                % ("XYZ"[axis],)
            )
        direction = 1.0 if hi.positive_dir else -1.0
        speed = speed_override if speed_override is not None else hi.speed
        max_travel = (
            max_travel_override
            if max_travel_override is not None
            else abs(pos_max - pos_min)
        )

        state = entry["state_cmd"].send([entry["oid"]])
        if state["pin_value"] ^ entry["invert"]:
            raise gcmd.error(
                "G28 %s: endstop already triggered — move the axis off the "
                "switch before homing" % ("XYZ"[axis],)
            )

        toolhead.wait_moves()
        stepper_enable = self.printer.lookup_object("stepper_enable")
        for s in rail.get_steppers():
            stepper_enable.motor_debug_enable(s.get_name(), True)

        rest_ticks = entry["mcu"].seconds_to_clock(HOMING_POLL_PERIOD)
        entry["query_cmd"].send([entry["oid"], rest_ticks])

        bridge.home_axis_start(
            axis, direction, speed, max_travel, entry["endstop_id"], endstop_mcu
        )
        reactor = self.printer.get_reactor()
        deadline = reactor.monotonic() + HOMING_TIMEOUT
        result = None
        while result is None:
            try:
                result = bridge.home_axis_poll()
            except Exception as e:
                bridge.home_abort()
                raise gcmd.error("G28 %s failed: %s" % ("XYZ"[axis], e))
            if result is not None:
                break
            if reactor.monotonic() > deadline:
                bridge.home_abort()
                raise gcmd.error(
                    "G28 %s: timed out waiting for endstop trip" % ("XYZ"[axis],)
                )
            reactor.pause(reactor.monotonic() + 0.010)
        trip_pos, final_pos = result

        overshoot = final_pos[axis] - trip_pos[axis]
        newpos = list(toolhead.get_position())
        newpos[axis] = hi.position_endstop + overshoot
        toolhead.set_position(newpos, homing_axes=[axis])
        logging.info(
            "homing: %s switch=%.4f overshoot=%+.4f set %s=%.4f",
            "XYZ"[axis],
            hi.position_endstop,
            overshoot,
            "XYZ"[axis],
            newpos[axis],
        )


def load_config(config):
    return Homing(config)
