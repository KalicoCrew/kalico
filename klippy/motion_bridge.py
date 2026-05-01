# Python wrapper around the PyO3 motion_bridge native module
#
# This file is part of the Kalico motion-bridge integration (Stage D).
# It wraps the Rust-built .so and provides convenience methods that
# klippy code calls during startup and MCU communication.
import logging

try:
    import motion_bridge as _native
except ImportError:
    _native = None
    logging.warning(
        "motion_bridge native module not found; "
        "build with 'make -f Makefile.kalico motion-bridge'"
    )


class MotionBridgeWrapper:
    """Thin wrapper registered as printer object 'motion_bridge'.

    All klippy code accesses the Rust bridge through this wrapper so
    that import-time failures are caught once and reported clearly.
    """

    def __init__(self, reactor):
        if _native is None:
            raise RuntimeError(
                "motion_bridge native module (.so) is not available. "
                "Run 'make -f Makefile.kalico motion-bridge' first."
            )
        self._bridge = _native.MotionBridge()
        self._reactor = reactor

    def get_bridge(self):
        return self._bridge

    # ------------------------------------------------------------------
    # MCU lifecycle
    # ------------------------------------------------------------------

    def claim_mcu(self, label, serial_path, baud):
        return self._bridge.claim_mcu(label, serial_path, baud)

    def release_mcu(self, handle):
        return self._bridge.release_mcu(handle)

    # ------------------------------------------------------------------
    # Command queues
    # ------------------------------------------------------------------

    def alloc_command_queue(self, handle):
        return self._bridge.alloc_command_queue(handle)

    # ------------------------------------------------------------------
    # Passthrough I/O
    # ------------------------------------------------------------------

    def passthrough_send(self, handle, data, minclock, reqclock, cq):
        return self._bridge.passthrough_send(handle, data, minclock, reqclock, cq)

    def passthrough_query(self, handle, data, minclock, reqclock, cq):
        return self._bridge.passthrough_query(handle, data, minclock, reqclock, cq)

    def passthrough_register_handler(self, handle, msg, oid, callback):
        return self._bridge.passthrough_register_handler(handle, msg, oid, callback)

    def passthrough_register_flush_callback(self, handle, callback):
        return self._bridge.passthrough_register_flush_callback(handle, callback)

    # ------------------------------------------------------------------
    # Event polling
    # ------------------------------------------------------------------

    def poll_event(self):
        return self._bridge.poll_event()

    # ------------------------------------------------------------------
    # Config phase
    # ------------------------------------------------------------------

    def add_config_cmd(self, handle, cmd_bytes):
        return self._bridge.add_config_cmd(handle, cmd_bytes)

    def add_init_cmd(self, handle, cmd_bytes):
        return self._bridge.add_init_cmd(handle, cmd_bytes)

    def add_restart_cmd(self, handle, cmd_bytes):
        return self._bridge.add_restart_cmd(handle, cmd_bytes)

    def begin_config_phase(self, handle):
        return self._bridge.begin_config_phase(handle)

    def next_config_entry(self, handle):
        return self._bridge.next_config_entry(handle)

    # ------------------------------------------------------------------
    # Stats / clock
    # ------------------------------------------------------------------

    def get_stats(self, handle):
        return self._bridge.get_stats(handle)

    def set_clock_est(self, handle, freq, conv_time, conv_clock, last_clock):
        return self._bridge.set_clock_est(handle, freq, conv_time, conv_clock, last_clock)

    def extract_old(self, handle, is_sent, count):
        return self._bridge.extract_old(handle, is_sent, count)
