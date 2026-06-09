try:
    from . import motion_bridge_native as _native
except ImportError:
    _native = None

from . import structured_log

# print_id is already bound when these fire, so the handler pushes the current
# session + print context.
_PRINT_ACTIVE_EVENTS = (
    "print_stats:start_printing",
    "print_stats:paused_printing",
)
# These fire BEFORE print_id is cleared, so the handler must push an explicit
# empty print_id rather than read the still-stale module global.
_PRINT_FINISH_EVENTS = (
    "print_stats:complete_printing",
    "print_stats:error_printing",
    "print_stats:cancelled_printing",
    "print_stats:reset",
)


# Methods that issue real motion/planner/MCU traffic. Under the stub bridge
# these MUST raise, not return None — a None would make MotionToolhead.move()
# compute `None - None`, hanging the test suite on a path that reached real
# motion without a real bridge.
_STUB_MOTION_METHODS = frozenset(
    {
        "init_planner",
        "submit_move",
        "submit_dwell",
        "wait_moves",
        "drain_motion",
        "motion_drain_poll",
        "motion_drain_finalize",
        "set_position",
        "get_last_move_time",
        "update_limits",
        "update_shaper",
        "fallback_clock_conversions",
        "dispatched_segment_count",
        "configure_axes",
        "register_phase_bus",
        "register_phase_motor",
        "get_mcu_capabilities",
        "ring_depth_for_axis",
        "claim_mcu",
        "claim_ethercat_node",
        "release_mcu",
        "detach_serial",
        "attach_serial",
        "alloc_command_queue",
        "set_clock_est",
        "set_msgproto_dict",
        "bridge_call",
        "bridge_send",
    }
)


def attach_structured_logging(native, printer, events_dir):
    # Must run after session_id is bound (printer.py startup) and before any MCU
    # attach/configure that could emit a Rust log. events_dir is None with no
    # logfile (--debugoutput): the jsonl sink is skipped but session context is
    # still pushed.
    if events_dir:
        native.init_logging(events_dir)
    native.set_session_context(
        structured_log.get_session(), structured_log.get_print()
    )

    def _push_ctx(*_args):
        native.set_session_context(
            structured_log.get_session(), structured_log.get_print()
        )

    def _clear_ctx(*_args):
        native.set_session_context(structured_log.get_session(), "")

    for ev in _PRINT_ACTIVE_EVENTS:
        printer.register_event_handler(ev, _push_ctx)
    for ev in _PRINT_FINISH_EVENTS:
        printer.register_event_handler(ev, _clear_ctx)


class _StubBridge:
    """Stand-in for MotionBridgeWrapper when motion_bridge_native is unavailable
    (e.g. CI without the cdylib). Usable for import/boot/config tests only: any
    motion-issuing method raises RuntimeError instead of returning None, so a
    test reaching real motion under the stub fails loud. Non-motion lifecycle
    helpers stay no-ops so config-only boots tear down cleanly.
    """

    def __getattr__(self, name):
        if name in _STUB_MOTION_METHODS:

            def _raise(*args, **kwargs):
                raise RuntimeError(
                    "motion_bridge_native not built: cannot call "
                    "%r on the stub bridge. The klippy motion path was "
                    "exercised without the real Rust engine. Build the "
                    "cdylib (e.g. `make -f Makefile.kalico motion-bridge`) "
                    "to exercise real motion, or restrict this test to "
                    "import/boot/config only." % (name,)
                )

            return _raise

        def _noop(*args, **kwargs):
            return None

        return _noop


class MotionBridgeWrapper:
    """Thin wrapper registered as printer object 'motion_bridge'."""

    def __init__(self, reactor):
        if _native is None:
            raise ImportError("motion_bridge_native not available")
        self._bridge = _native.MotionBridge()
        self._reactor = reactor

    def get_bridge(self):
        return self._bridge

    def claim_mcu(self, label, serial_path, baud):
        return self._bridge.claim_mcu(label, serial_path, baud)

    def claim_ethercat_node(
        self, label, socket_path, interface, endpoint, counts_per_mm
    ):
        return self._bridge.claim_ethercat_node(
            label, socket_path, interface, endpoint, counts_per_mm
        )

    def set_torque(self, mcu_handle, value, print_time):
        self._bridge.set_torque(mcu_handle, bool(value), print_time)

    def release_mcu(self, handle):
        return self._bridge.release_mcu(handle)

    def detach_serial(self, handle):
        return self._bridge.detach_serial(handle)

    def shutdown(self):
        return self._bridge.shutdown()

    def alloc_command_queue(self, handle):
        return self._bridge.alloc_command_queue(handle)

    def passthrough_send(self, handle, cq, data, minclock=0, reqclock=0):
        return self._bridge.passthrough_send(
            handle, cq, data, minclock, reqclock
        )

    def passthrough_query(self, handle, cq, data, minclock=0, reqclock=0):
        return self._bridge.passthrough_query(
            handle, cq, data, minclock, reqclock
        )

    def passthrough_register_handler(self, handle, msg, oid, callback):
        return self._bridge.passthrough_register_handler(
            handle, msg, oid, callback
        )

    def passthrough_register_flush_callback(self, handle, callback):
        return self._bridge.passthrough_register_flush_callback(
            handle, callback
        )

    def poll_event(self):
        return self._bridge.poll_event()

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

    def get_stats(self, handle):
        return self._bridge.get_stats(handle)

    def set_clock_est(self, handle, freq, offset, last_clock, host_now_raw):
        return self._bridge.set_clock_est(
            handle, freq, offset, last_clock, host_now_raw
        )

    def bridge_get_clock_async(self, handle):
        return self._bridge.bridge_get_clock_async(handle)

    def extract_old(self, handle):
        return self._bridge.extract_old(handle)

    def attach_serial(
        self,
        mcu_handle,
        serial_path,
        baud,
        timeout_s=30.0,
        klippy_non_critical=False,
    ):
        """klippy_non_critical feeds the per-MCU criticality gate: a
        non-critical MCU's transport drop does not abort klippy, a critical
        motion MCU's does. A Klipper-protocol-only attach (identify timed out)
        is always treated as non-critical.
        """
        return self._bridge.attach_serial(
            mcu_handle, serial_path, baud, timeout_s, klippy_non_critical
        )

    def get_identify_data(self, mcu_handle):
        return bytes(self._bridge.get_identify_data(mcu_handle))

    def get_mcu_capabilities(self, mcu_handle):
        # Bit 0 = PHASE_STEPPING_CAPABLE; 0 for stock-Klipper MCUs or before
        # attach_serial completes.
        return self._bridge.get_mcu_capabilities(mcu_handle)

    def ring_depth_for_axis(self, mcu_handle, axis_idx):
        # Requires init_planner first.
        return self._bridge.ring_depth_for_axis(mcu_handle, axis_idx)

    def configure_axes(
        self,
        mcu_handle,
        kinematics,
        present_mask,
        awd_mask,
        invert_mask,
        steps_per_mm,
        step_modes=None,
        phase_configs=None,
        timeout_s=2.0,
    ):
        """step_modes: optional [4] ints (0=Modulated, 1=StepTime); when set the
        bridge emits the 25-byte extended format, else the legacy 20-byte path.
        phase_configs: optional (bus_id, cs_pin_id, slot_idx) per phase-stepped
        motor; slot_idx is the kinematic slot driving that motor's XDIRECT. Up
        to 16. Pass None (not []) when nothing is phase stepped.
        """
        return self._bridge.configure_axes(
            mcu_handle,
            kinematics,
            present_mask,
            awd_mask,
            invert_mask,
            list(steps_per_mm),
            list(step_modes) if step_modes is not None else None,
            list(phase_configs) if phase_configs is not None else None,
            timeout_s,
        )

    def register_phase_bus(self, mcu_handle, bus_id, rate, timeout_s=5.0):
        """Call once per bus_id, BEFORE any register_phase_motor for that bus
        and BEFORE configure_axes. Per-motor CS GPIOs are registered separately
        (each TMC5160 on a shared bus needs its own CS). No-op on stock MCUs.
        """
        return self._bridge.register_phase_bus(
            mcu_handle,
            bus_id,
            rate,
            timeout_s,
        )

    def register_phase_motor(
        self, mcu_handle, motor_idx, bus_id, cs_pin_id, timeout_s=5.0
    ):
        """Call once per phase-stepped motor, AFTER register_phase_bus and
        BEFORE configure_axes. motor_idx matches the order of entries in the
        configure_axes blob's phase section.
        """
        return self._bridge.register_phase_motor(
            mcu_handle,
            motor_idx,
            bus_id,
            cs_pin_id,
            timeout_s,
        )

    def bridge_call(self, mcu_handle, msg, response, timeout_s=15.0):
        return self._bridge.bridge_call(mcu_handle, msg, response, timeout_s)

    def bridge_send(self, mcu_handle, msg):
        return self._bridge.bridge_send(mcu_handle, msg)

    def bridge_mark_expected_disconnect(self, mcu_handle):
        """Mark an imminent transport drop so the reactor's EXIT_ON_FAULT guard
        treats it as graceful instead of a wedge. Called before the firmware
        `reset` command (NVIC_SystemReset drops USB-CDC).
        """
        return self._bridge.bridge_mark_expected_disconnect(mcu_handle)

    def take_runtime_event(self, mcu_handle):
        return self._bridge.take_runtime_event(mcu_handle)

    def on_credit_freed(
        self, mcu_handle, retired_through_segment_id, free_slots
    ):
        return self._bridge.on_credit_freed(
            mcu_handle,
            retired_through_segment_id,
            free_slots,
        )

    def set_msgproto_dict(self, dict_json):
        return self._bridge.set_msgproto_dict(dict_json)

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
        mcus,
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
            mcus,
            window_capacity,
            beta_max_iters,
        )

    def submit_move(self, dx, dy, dz, de, feedrate):
        return self._bridge.submit_move(dx, dy, dz, de, feedrate)

    def wait_moves(self):
        return self._bridge.wait_moves()

    def drain_motion(self):
        return self._bridge.drain_motion()

    def motion_drain_poll(self):
        return self._bridge.motion_drain_poll()

    def motion_drain_finalize(self):
        return self._bridge.motion_drain_finalize()

    def submit_dwell(self, duration_s):
        return self._bridge.submit_dwell(duration_s)

    def set_position(self, x, y, z):
        return self._bridge.set_position(x, y, z)

    def home_axis_start(
        self, axis, direction, speed_mm_s, max_travel_mm, endstop_id, endstop_mcu
    ):
        return self._bridge.home_axis_start(
            axis, direction, speed_mm_s, max_travel_mm, endstop_id, endstop_mcu
        )

    def home_axis_poll(self):
        return self._bridge.home_axis_poll()

    def home_abort(self):
        return self._bridge.home_abort()

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
