# Python wrapper around the PyO3 motion_bridge native module
#
# This file is part of the Kalico motion-bridge integration (Stage D).
# It wraps the Rust-built .so and provides convenience methods that
# klippy code calls during startup and MCU communication.
import logging

try:
    from . import motion_bridge_native as _native
except ImportError:
    _native = None
    logging.warning(
        "motion_bridge_native module not found; "
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
        # arm_id → BridgeTriggerDispatch registry. Populated by
        # BridgeTriggerDispatch.start so the credit-freed handler can
        # fire _completion on past-end-time.
        self._homing_dispatches = {}

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
    # Phase 1: serial attach + identify
    # ------------------------------------------------------------------

    def attach_serial(self, mcu_handle, serial_path, baud, timeout_s=30.0):
        """Open serial port, run identify handshake, spawn reactor thread."""
        return self._bridge.attach_serial(mcu_handle, serial_path, baud, timeout_s)

    def get_identify_data(self, mcu_handle):
        """Return raw zlib identify bytes for process_identify."""
        return bytes(self._bridge.get_identify_data(mcu_handle))

    def get_mcu_capabilities(self, mcu_handle):
        """Return the raw capabilities bitmap from the MCU's IdentifyResponse.

        Bit 0 = PHASE_STEPPING_CAPABLE. Returns 0 for stock-Klipper MCUs that
        don't speak kalico-native, or before attach_serial has completed.
        """
        return self._bridge.get_mcu_capabilities(mcu_handle)

    def configure_axes(
        self,
        mcu_handle,
        kinematics,
        present_mask,
        awd_mask,
        invert_mask,
        steps_per_mm,
        step_modes=None,
        timeout_s=2.0,
    ):
        """Send the kalico-native ConfigureAxes message to an attached MCU.

        step_modes: optional list of 4 ints (0=Modulated/phase-stepping,
        1=StepTime/classic). When supplied the bridge emits the 25-byte
        extended format (spec §4 C1). Omit for the legacy 20-byte path.
        """
        return self._bridge.configure_axes(
            mcu_handle,
            kinematics,
            present_mask,
            awd_mask,
            invert_mask,
            list(steps_per_mm),
            list(step_modes) if step_modes is not None else None,
            timeout_s,
        )

    def bridge_call(self, mcu_handle, msg, response, timeout_s=15.0):
        """Send a msgproto command and wait for the named response dict.

        2026-05-17: bumped default 5.0s → 15.0s. With slot-pool retirement
        now working, TMC autotune's many tmcuart_send / spi_transfer
        commands race motion-bridge's push_segment traffic on the same
        USB-CDC pipe — a 5 s timeout was insufficient for the slowest
        TMC reads under sustained motion load.
        """
        return self._bridge.bridge_call(mcu_handle, msg, response, timeout_s)

    def bridge_send(self, mcu_handle, msg):
        """Send a fire-and-forget command (no response expected)."""
        return self._bridge.bridge_send(mcu_handle, msg)

    def take_runtime_event(self, mcu_handle):
        """Drain one runtime event dict, or None if nothing pending."""
        return self._bridge.take_runtime_event(mcu_handle)

    def on_credit_freed(self, mcu_handle, retired_through_segment_id,
                        free_slots):
        """Forward an MCU `kalico_credit_freed` event into the slot pool.

        Returns ``(n_released, completed_arm_id_or_None)``. The
        ``completed_arm_id`` is set when a homing segment retired without
        a trip in this credit-freed cycle; the caller is responsible for
        firing the matching dispatch's ``_completion``.
        """
        return self._bridge.on_credit_freed(
            mcu_handle, retired_through_segment_id, free_slots,
        )

    def register_homing_dispatch(self, arm_id, dispatch):
        self._homing_dispatches[int(arm_id)] = dispatch

    def unregister_homing_dispatch(self, arm_id):
        self._homing_dispatches.pop(int(arm_id), None)

    def fire_homing_completion(self, arm_id):
        """Resolve the BridgeTriggerDispatch for arm_id with
        REASON_PAST_END_TIME. No-op if no dispatch is registered (race
        with stop)."""
        dispatch = self._homing_dispatches.get(int(arm_id))
        if dispatch is not None:
            dispatch._fire_past_end_time()

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

    # ------------------------------------------------------------------
    # Step 7-D: endstop wire surface
    # ------------------------------------------------------------------

    def endstop_arm(self, mcu, queue, arm_id, arm_clock,
                    sources, stepper_oids, timeout_s=2.0):
        # `sources` is a list of 7-tuples per BridgeTriggerDispatch contract:
        # (kind, gpio, active_high, policy, sample_n, velocity_axis, v_min_q16)
        return self._bridge.endstop_arm(
            mcu, queue, arm_id, arm_clock, sources, stepper_oids, timeout_s
        )

    def endstop_disarm(self, mcu, queue, arm_id, timeout_s=2.0):
        return self._bridge.endstop_disarm(mcu, queue, arm_id, timeout_s)

    def submit_homing_move(self, newpos, speed, arm_ids):
        return self._bridge.submit_homing_move(newpos, speed, arm_ids)

    def take_trip_event(self):
        return self._bridge.take_trip_event()


# ----------------------------------------------------------------------
# Step 7-D: BridgeTriggerDispatch
# ----------------------------------------------------------------------
#
# Stand-in for klippy's legacy `TriggerDispatch` (klippy/mcu.py:336) when
# `MCU_endstop._use_bridge=True`. Owns the arm_id, sources, stepper oids
# associated with one homing operation, and bridges the (future) async
# trip event back to a reactor completion. Spec §5.2.
#
# Reason codes match `MCU_trsync` at klippy/mcu.py:155–158 exactly so
# downstream `homing.py` consumers don't see a behavior change.

REASON_ENDSTOP_HIT = 1
REASON_HOST_REQUEST = 2
REASON_PAST_END_TIME = 3
REASON_COMMS_TIMEOUT = 4

ARM_STATUS_ARMED = 0
ARM_STATUS_ALREADY_TRIPPED = 1
ARM_STATUS_REJECTED = 2

DISARM_STATUS_DISARMED = 0
DISARM_STATUS_ALREADY_TRIPPED = 1
DISARM_STATUS_UNKNOWN = 2


_ARM_ID_COUNTER = [1]


def _alloc_arm_id():
    arm_id = _ARM_ID_COUNTER[0]
    _ARM_ID_COUNTER[0] = (arm_id + 1) & 0xFFFFFFFF
    if _ARM_ID_COUNTER[0] == 0:
        _ARM_ID_COUNTER[0] = 1
    return arm_id


class BridgeTriggerDispatch:
    """Bridge-mode replacement for `klippy/mcu.py:TriggerDispatch`.

    Used by `MCU_endstop` when its underlying MCU is the kalico bridge.
    Public surface mirrors the legacy `TriggerDispatch` so existing
    `home_start` / `home_wait` callers don't change.
    """

    def __init__(self, bridge, mcu, queue, reactor):
        # `bridge` is a MotionBridgeWrapper.
        # `mcu` and `queue` are the host-side handles (u32) the bridge
        # uses to address the right MCU command queue.
        self._bridge = bridge
        self._mcu = mcu
        self._queue = queue
        self._reactor = reactor
        self._arm_id = _alloc_arm_id()
        self._completion = reactor.completion()
        self._sources = []          # list of (kind, gpio, active_high, policy,
                                    #          sample_n, velocity_axis, v_min_q16)
        self._stepper_oids = []
        self._steppers = []         # list of MCU_stepper, retained for IK lookups
        self._reason = None         # legacy-compatible reason code
        self._trip_event = None     # decoded async event payload
        self._arm_print_time = None # print_time at arm time (fallback trigger)
        self._handler_registered = False
        self._toolhead_arms = None  # set at start() for stop() cleanup

    # ── legacy TriggerDispatch surface ──────────────────────────────

    def get_oid(self):
        return self._arm_id

    def get_command_queue(self):
        return self._queue

    def get_arm_id(self):
        return self._arm_id

    def add_stepper(self, mcu_stepper):
        # MCU_stepper.get_oid() returns the per-MCU oid — exactly what
        # the runtime_arm_endstop wire format expects (spec §3.1).
        self._stepper_oids.append(mcu_stepper.get_oid())
        self._steppers.append(mcu_stepper)

    def get_steppers(self):
        return list(self._steppers)

    # ── new endstop-source binding ──────────────────────────────────

    def add_source(self, kind, gpio, active_high, policy, sample_n,
                   velocity_axis, v_min_q16):
        self._sources.append((kind, gpio, active_high, policy, sample_n,
                              velocity_axis, v_min_q16))

    # ── start / wait / stop ─────────────────────────────────────────

    def start(self, arm_print_time, mcu_obj):
        # Stash for use as fallback trigger time in AlreadyTripped path.
        self._arm_print_time = arm_print_time
        # Convert print_time → MCU clock for arm_clock.
        arm_clock = int(mcu_obj.print_time_to_clock(arm_print_time))

        # The MCU_endstop is constructed during config phase, before the
        # bridge has identified the MCU and assigned `_bridge_handle`.
        # Refresh the handle (and lazily allocate the bridge command
        # queue) here so the first homing op sees non-None values.
        if self._mcu is None:
            self._mcu = getattr(mcu_obj, "_bridge_handle", None)
            if self._mcu is None:
                raise mcu_obj.get_printer().command_error(
                    "BridgeTriggerDispatch: MCU bridge handle not yet "
                    "assigned (identify phase incomplete?)"
                )
        if self._queue is None:
            self._queue = self._bridge.alloc_command_queue(self._mcu)

        # Register in the bridge's arm_id → dispatch map so the
        # credit-freed handler can resolve past-end-time terminals.
        self._bridge.register_homing_dispatch(self._arm_id, self)

        # Register an async handler for kalico_endstop_tripped before
        # arming so we don't race the firmware emitting the event.
        if not self._handler_registered:
            mcu_obj.register_response(
                self._on_trip_message, "kalico_endstop_tripped"
            )
            self._handler_registered = True

        # Step 7-D §6.2: register arm_id with motion_toolhead so its
        # drip_move can pass the right arm_ids set to
        # bridge.submit_homing_move.
        printer = mcu_obj.get_printer()
        toolhead = printer.lookup_object("toolhead", None)
        if toolhead is not None and hasattr(toolhead, "active_homing_arms"):
            self._toolhead_arms = toolhead.active_homing_arms
            self._toolhead_arms.add(self._arm_id)

        logging.info(
            "[bridge-trace] endstop_arm arm_id=%s sources=%s steppers=%s",
            self._arm_id, self._sources, self._stepper_oids,
        )
        status = self._bridge.endstop_arm(
            self._mcu, self._queue,
            self._arm_id, arm_clock,
            self._sources, self._stepper_oids,
        )
        logging.info("[bridge-trace] endstop_arm status=%s", status)
        if status == ARM_STATUS_ALREADY_TRIPPED:
            # Pin asserted at arm time under TripImmediately. The
            # firmware published a trip snapshot in arm() itself —
            # fetch it now so home_wait can return a real trigger time.
            self._trip_event = self._bridge.take_trip_event() or {}
            self._reason = REASON_ENDSTOP_HIT
            self._completion.complete(self._reason)
        elif status == ARM_STATUS_REJECTED:
            raise printer.command_error(
                "runtime_arm_endstop rejected (status=%d)" % status
            )
        return self._completion

    def _on_trip_message(self, params):
        # params is a dict-like emitted by klippy's reactor for the
        # registered response. Filter on arm_id (multiple
        # BridgeTriggerDispatch instances may live concurrently).
        if int(params.get("arm_id", -1)) != self._arm_id:
            return
        if self._reason is not None:
            # Already terminal (e.g., from arm-time AlreadyTripped).
            return
        # Decode the payload. The msgproto layer hands this as raw fields;
        # we delegate detailed decoding to the bridge-side via
        # take_trip_event(), which pulls the runtime-Rust TripEvent.
        self._trip_event = self._bridge.take_trip_event()
        self._reason = REASON_ENDSTOP_HIT
        self._completion.complete(self._reason)

    def _fire_past_end_time(self):
        # MCU-driven no-trip terminal. Mirror _on_trip_message's
        # ownership semantics: only fire if no terminal yet.
        if self._reason is not None:
            return
        self._reason = REASON_PAST_END_TIME
        self._completion.complete(self._reason)

    def stop(self):
        # Called from MCU_endstop.home_wait. Disarm if no trip yet.
        if self._reason is None:
            try:
                status = self._bridge.endstop_disarm(
                    self._mcu, self._queue, self._arm_id
                )
            except Exception:
                status = DISARM_STATUS_UNKNOWN
            if status == DISARM_STATUS_DISARMED:
                self._reason = REASON_HOST_REQUEST
            else:
                # AlreadyTripped on race — wait briefly for the async
                # event to land.
                self._reason = REASON_ENDSTOP_HIT
            if not self._completion.test():
                self._completion.complete(self._reason)
        self._bridge.unregister_homing_dispatch(self._arm_id)
        # Unregister from the toolhead's active-arms registry. start()
        # cached the set reference; this keeps drip_move from passing a
        # stale arm_id on a subsequent unrelated move.
        if self._toolhead_arms is not None:
            self._toolhead_arms.discard(self._arm_id)
            self._toolhead_arms = None
        return self._reason

    def get_trip_event(self):
        return self._trip_event

    def get_arm_print_time(self):
        return self._arm_print_time
