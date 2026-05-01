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

    def passthrough_send(self, handle, cq, data, minclock=0, reqclock=0):
        return self._bridge.passthrough_send(handle, cq, data, minclock, reqclock)

    def passthrough_query(self, handle, cq, data, minclock=0, reqclock=0):
        return self._bridge.passthrough_query(handle, cq, data, minclock, reqclock)

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

    def set_clock_est(self, handle, freq, offset, last_clock):
        return self._bridge.set_clock_est(handle, freq, offset, last_clock)

    def extract_old(self, handle):
        return self._bridge.extract_old(handle)

    # ------------------------------------------------------------------
    # Phase 2: msgproto handover
    # ------------------------------------------------------------------

    def set_msgproto_dict(self, dict_json):
        return self._bridge.set_msgproto_dict(dict_json)

    # ------------------------------------------------------------------
    # Phase 2: motion submission
    # ------------------------------------------------------------------

    def init_planner(
        self,
        max_velocity,
        max_accel,
        max_z_velocity,
        max_z_accel,
        square_corner_velocity,
        shaper_type_x,
        shaper_freq_x,
        shaper_type_y,
        shaper_freq_y,
        octopus_handle,
        f446_handle,
        window_capacity=32,
        beta_max_iters=10,
    ):
        return self._bridge.init_planner(
            max_velocity,
            max_accel,
            max_z_velocity,
            max_z_accel,
            square_corner_velocity,
            shaper_type_x,
            shaper_freq_x,
            shaper_type_y,
            shaper_freq_y,
            octopus_handle,
            f446_handle,
            window_capacity,
            beta_max_iters,
        )

    def submit_move(self, dx, dy, dz, de, feedrate):
        return self._bridge.submit_move(dx, dy, dz, de, feedrate)

    def wait_moves(self):
        return self._bridge.wait_moves()

    def submit_dwell(self, duration_s):
        return self._bridge.submit_dwell(duration_s)

    def set_position(self, x, y, z):
        return self._bridge.set_position(x, y, z)

    def update_limits(self, max_velocity, max_accel):
        return self._bridge.update_limits(max_velocity, max_accel)

    def update_shaper(self, type_x, freq_x, type_y, freq_y):
        return self._bridge.update_shaper(type_x, freq_x, type_y, freq_y)

    def get_last_move_time(self):
        return self._bridge.get_last_move_time()

    def fallback_clock_conversions(self):
        return self._bridge.fallback_clock_conversions()

    def dispatched_segment_count(self):
        return self._bridge.dispatched_segment_count()
