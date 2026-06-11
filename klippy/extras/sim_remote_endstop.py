# Sim-only virtual-endstop provider exercising the Spec B remote-trigger
# contract end to end: RemoteBridgeEndstop arming, trsync relay, terminal
# reason verification, and the measured-position override. Reference
# implementation for external-probe providers (Spec D).
import logging

from klippy import pins
from klippy.bridge_endstop import RemoteBridgeEndstop

REASON_ENDSTOP_HIT = 1
REASON_COMMS_TIMEOUT = 4


class SimRemoteEndstop:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.trigger_delay = config.getfloat("trigger_delay", 1.0, above=0.0)
        self.measured_z = config.getfloat("measured_z", None)
        self.trigger_height = config.getfloat("trigger_height", 0.0)
        mcu_name = config.get("mcu", "mcu")
        self.mcu = self.printer.lookup_object(
            "mcu" if mcu_name == "mcu" else "mcu " + mcu_name
        )
        self.oid = self.mcu.create_oid()
        self._trsync_start_cmd = None
        self._trsync_trigger_cmd = None
        self._last_reason = None
        self._trigger_timer = None
        self.mcu.register_config_callback(self._build_config)
        self.mcu.register_response(
            self._handle_trsync_state, "trsync_state", self.oid
        )
        self._endstop = RemoteBridgeEndstop(
            self.printer, self.mcu, trsync_oid=self.oid
        )
        ppins = self.printer.lookup_object("pins")
        ppins.register_chip("sim_remote_endstop", self)

    def _build_config(self):
        self.mcu.add_config_cmd("config_trsync oid=%d" % (self.oid,))
        self._trsync_start_cmd = self.mcu.lookup_command(
            "trsync_start oid=%c report_clock=%u report_ticks=%u"
            " expire_reason=%c"
        )
        self._trsync_trigger_cmd = self.mcu.lookup_command(
            "trsync_trigger oid=%c reason=%c"
        )

    def _handle_trsync_state(self, params):
        if not params["can_trigger"]:
            self._last_reason = params["trigger_reason"]

    # --- virtual endstop provider contract -------------------------------

    def setup_bridge_endstop(self, pin_params, axis):
        if pin_params["pin"] != "z_virtual_endstop" or axis != 2:
            raise pins.error(
                "sim_remote_endstop only provides z_virtual_endstop on Z"
            )
        if pin_params["invert"] or pin_params["pullup"]:
            raise pins.error(
                "Can not pullup/invert sim_remote_endstop virtual endstop"
            )
        return self._endstop

    def get_position_endstop(self):
        return self.trigger_height

    def trip_move_begin(self, entry):
        self._last_reason = None
        self._trsync_start_cmd.send([self.oid, 0, 0, REASON_COMMS_TIMEOUT])
        reactor = self.printer.get_reactor()
        self._trigger_timer = reactor.register_timer(
            self._fire_trigger, reactor.monotonic() + self.trigger_delay
        )

    def _fire_trigger(self, eventtime):
        logging.info("sim_remote_endstop: firing trsync_trigger")
        self._trsync_trigger_cmd.send([self.oid, REASON_ENDSTOP_HIT])
        return self.printer.get_reactor().NEVER

    def trip_move_end(self, entry):
        reactor = self.printer.get_reactor()
        if self._trigger_timer is not None:
            reactor.unregister_timer(self._trigger_timer)
            self._trigger_timer = None
        deadline = reactor.monotonic() + 2.0
        while self._last_reason is None:
            if reactor.monotonic() > deadline:
                raise self.printer.command_error(
                    "sim_remote_endstop: no terminal trsync_state received"
                )
            reactor.pause(reactor.monotonic() + 0.010)
        if self._last_reason != REASON_ENDSTOP_HIT:
            raise self.printer.command_error(
                "sim_remote_endstop: trsync terminated with reason %d"
                % (self._last_reason,)
            )

    def measured_trip_position(self, axis, trip_pos, final_pos):
        return self.measured_z


def load_config(config):
    return SimRemoteEndstop(config)
