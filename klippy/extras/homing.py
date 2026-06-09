import logging

# Endstop poll period while homing. The trip clock is captured at the poll that
# detects the edge, so this bounds the switch-location error (period x speed):
# 1 ms x 50 mm/s = 50 um, well under a layer. A far tighter period (e.g. 100 us)
# floods the MCU foreground timer dispatch and starves the watchdog-reload task.
HOMING_POLL_PERIOD = 0.001
HOMING_TIMEOUT = 30.0


class Homing:
    def __init__(self, config):
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object("pins")

        # axis_index -> per-axis endstop watch state.
        self._axes = {}
        for axis_index, axis_name in enumerate("xyz"):
            section = "stepper_" + axis_name
            if not config.has_section(section):
                continue
            endstop_pin = config.getsection(section).get("endstop_pin", None)
            if endstop_pin is None or "virtual_endstop" in endstop_pin:
                continue
            # parse_pin resolves chip+pin without reserving, so it coexists with
            # the rail's own endstop_pin reservation.
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
            "QUERY_ENDSTOPS", self.cmd_QUERY_ENDSTOPS, desc="Report endstop states"
        )
        gcode.register_command(
            "_HOME_TEST",
            self.cmd_HOME_TEST,
            desc="Bench only: home one axis with override SPEED/MAX_TRAVEL",
        )

    def cmd_QUERY_ENDSTOPS(self, gcmd):
        parts = []
        for axis_index in sorted(self._axes.keys()):
            entry = self._axes[axis_index]
            params = entry["state_cmd"].send([entry["oid"]])
            raw = params["pin_value"]
            triggered = raw ^ entry["invert"]
            parts.append(
                "%s:%s (pin=%d invert=%d armed=%d)"
                % (
                    "xyz"[axis_index],
                    "TRIGGERED" if triggered else "open",
                    raw,
                    entry["invert"],
                    params["armed"],
                )
            )
        gcmd.respond_info("\n".join(parts) if parts else "no endstops configured")

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
        # G28 uses the legacy G-code parser (no KEY=VALUE) — bare axis flags only.
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

        # Refuse to home from an already-triggered switch: the level-armed watch
        # would trip before home_axis_start registers the run, the trip would be
        # dropped, and the move would run to max_travel into the hard stop.
        state = entry["state_cmd"].send([entry["oid"]])
        if state["pin_value"] ^ entry["invert"]:
            raise gcmd.error(
                "G28 %s: endstop already triggered — move the axis off the "
                "switch before homing" % ("XYZ"[axis],)
            )

        # Quiesce prior motion, then arm the endstop poll before the move starts.
        toolhead.wait_moves()
        rest_ticks = entry["mcu"].seconds_to_clock(HOMING_POLL_PERIOD)
        entry["query_cmd"].send([entry["oid"], rest_ticks])

        # Dispatch the drip move, then poll cooperatively so the reactor keeps
        # draining the trip event and servicing heaters during the move.
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
