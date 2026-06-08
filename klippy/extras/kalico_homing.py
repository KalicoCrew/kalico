# Cross-MCU native homing: G28 driven by the Rust motion bridge.
#
# Reuses the existing [stepper_<axis>] homing fields (endstop_pin,
# position_endstop, homing_speed, homing_positive_dir, position_min/max). The
# endstop's MCU watches the pin and reports the trip timestamp; the bridge drips
# the homing move, broadcasts the stop to every MCU, and reconstructs — in mm,
# from the commanded trajectory — both the switch location and the overshot
# stopping location. The axis is then set to position_endstop + overshoot.
#
# v1 homes a single cartesian axis (X). Multi-axis sequencing, second-touch, and
# retract are follow-ons.

import logging

# Endstop poll period while homing. The trip clock is captured at the poll that
# detects the edge, so this bounds the switch-location error (period x speed).
HOMING_POLL_PERIOD = 0.0001
HOMING_TIMEOUT = 30.0


class KalicoHoming:
    def __init__(self, config):
        self.printer = config.get_printer()
        self._axis = 0
        self._axis_name = "x"
        self._endstop_id = 0

        stepper_cfg = config.getsection("stepper_" + self._axis_name)
        endstop_pin = stepper_cfg.get("endstop_pin")
        ppins = self.printer.lookup_object("pins")
        # parse_pin resolves chip+pin without reserving, so it coexists with the
        # rail's own endstop_pin reservation.
        pin_params = ppins.parse_pin(endstop_pin, can_invert=True, can_pullup=True)
        self._mcu = pin_params["chip"]
        self._pin = pin_params["pin"]
        self._pullup = pin_params["pullup"]
        self._invert = pin_params["invert"]

        self._oid = self._mcu.create_oid()
        self._mcu.register_config_callback(self._build_config)
        self._query_cmd = None

        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("G28", self.cmd_G28, desc="Home (kalico native)")

    def _build_config(self):
        self._mcu.add_config_cmd(
            "config_kalico_endstop oid=%d endstop_id=%d pin=%s pull_up=%d invert=%d"
            % (self._oid, self._endstop_id, self._pin, self._pullup, self._invert)
        )
        self._query_cmd = self._mcu.lookup_command(
            "query_kalico_endstop oid=%c rest_ticks=%u"
        )

    def cmd_G28(self, gcmd):
        requested = [a for a in "XYZ" if gcmd.get(a, None) is not None]
        if any(a != "X" for a in requested):
            raise gcmd.error(
                "kalico_homing v1 homes only X (requested %s)" % ("".join(requested),)
            )

        toolhead = self.printer.lookup_object("toolhead")
        bridge = self.printer.lookup_object("motion_bridge")
        kin = toolhead.get_kinematics()
        rail = kin._axis_rails().get(self._axis)
        if rail is None:
            raise gcmd.error("kalico_homing: no rail for axis X")
        hi = rail.get_homing_info()
        pos_min, pos_max = rail.get_range()

        endstop_mcu = getattr(self._mcu, "_bridge_handle", None)
        if endstop_mcu is None:
            raise gcmd.error("kalico_homing: endstop MCU is not attached to the bridge")

        direction = 1.0 if hi.positive_dir else -1.0
        max_travel = abs(pos_max - pos_min)

        # Quiesce prior motion, then arm the endstop poll before the move starts.
        toolhead.wait_moves()
        rest_ticks = self._mcu.seconds_to_clock(HOMING_POLL_PERIOD)
        self._query_cmd.send([self._oid, rest_ticks])

        # Dispatch the drip move, then wait cooperatively: reactor.pause yields so
        # the bridge event poller drains the trip (which fires the stop + recon)
        # and heaters/other tasks keep being serviced during the move.
        bridge.home_axis_start(
            self._axis, direction, hi.speed, max_travel, self._endstop_id, endstop_mcu
        )
        reactor = self.printer.get_reactor()
        deadline = reactor.monotonic() + HOMING_TIMEOUT
        result = None
        while result is None:
            try:
                result = bridge.home_axis_poll()
            except Exception as e:
                bridge.home_abort()
                raise gcmd.error("G28 X failed: %s" % (e,))
            if result is not None:
                break
            if reactor.monotonic() > deadline:
                bridge.home_abort()
                raise gcmd.error("G28 X: timed out waiting for endstop trip")
            reactor.pause(reactor.monotonic() + 0.010)
        trip_pos, final_pos = result

        # Switch lands at position_endstop; the toolhead rests one overshoot past.
        overshoot = final_pos[self._axis] - trip_pos[self._axis]
        newpos = list(toolhead.get_position())
        newpos[self._axis] = hi.position_endstop + overshoot
        toolhead.set_position(newpos, homing_axes=[self._axis])
        logging.info(
            "kalico_homing: X switch=%.4f overshoot=%+.4f set X=%.4f",
            hi.position_endstop,
            overshoot,
            newpos[self._axis],
        )


def load_config(config):
    return KalicoHoming(config)
