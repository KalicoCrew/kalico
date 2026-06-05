# Python wrapper around the PyO3 motion_bridge native module.
import logging

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
        "submit_homing_move",
        "submit_homing_move_async",
        "endstop_arm",
        "endstop_disarm",
        "software_trip",
        "extend_homing_deadline",
        "prepare_probe_homing",
        "run_probe_homing",
        "get_homing_position_at_time",
        "take_trip_event",
        "is_homing_segment_retired",
        "get_homing_segment_reason",
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
        # arm_id → BridgeTriggerDispatch; populated by start() so the
        # credit-freed handler can fire _completion on past-end-time.
        self._homing_dispatches = {}
        self._software_trip_active = False
        self._homing_print_time_base = 0.0

    def get_bridge(self):
        return self._bridge

    def claim_mcu(self, label, serial_path, baud):
        return self._bridge.claim_mcu(label, serial_path, baud)

    def claim_ethercat_node(self, label, socket_path):
        return self._bridge.claim_ethercat_node(label, socket_path)

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

    def set_clock_est(self, handle, freq, offset, last_clock):
        return self._bridge.set_clock_est(handle, freq, offset, last_clock)

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
        """Returns (n_released, completed_arm_id_or_None); completed_arm_id is
        set when a homing segment retired without a trip — the caller fires the
        matching dispatch's _completion.
        """
        return self._bridge.on_credit_freed(
            mcu_handle,
            retired_through_segment_id,
            free_slots,
        )

    def register_homing_dispatch(self, arm_id, dispatch):
        self._homing_dispatches[int(arm_id)] = dispatch

    def unregister_homing_dispatch(self, arm_id):
        self._homing_dispatches.pop(int(arm_id), None)

    def fire_homing_completion(self, arm_id):
        # No-op if no dispatch is registered (race with stop).
        dispatch = self._homing_dispatches.get(int(arm_id))
        if dispatch is not None:
            dispatch._fire_past_end_time()

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

    def endstop_arm(
        self,
        mcu,
        queue,
        arm_id,
        arm_clock,
        sources,
        stepper_oids,
        timeout_s=2.0,
    ):
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

    def submit_homing_move_async(self, newpos, speed, arm_ids):
        return self._bridge.submit_homing_move_async(newpos, speed, arm_ids)

    def is_homing_segment_retired(self):
        return self._bridge.is_homing_segment_retired()

    def get_homing_segment_reason(self):
        return self._bridge.get_homing_segment_reason()

    def software_trip(self, mcu, arm_id, timeout_s=2.0):
        return self._bridge.software_trip(mcu, arm_id, timeout_s)

    def extend_homing_deadline(self, mcu, arm_id):
        return self._bridge.extend_homing_deadline(mcu, arm_id)

    def prepare_probe_homing(
        self,
        beacon_handle,
        beacon_trsync_oid,
        stepper_mcu_handle,
        arm_id,
        sensor_fault_timeout,
    ):
        return self._bridge.prepare_probe_homing(
            beacon_handle,
            beacon_trsync_oid,
            stepper_mcu_handle,
            arm_id,
            sensor_fault_timeout,
        )

    def run_probe_homing(self, handle_id, move_pos, speed, stepper_oids):
        return self._bridge.run_probe_homing(
            handle_id,
            move_pos,
            speed,
            stepper_oids,
        )

    def get_homing_position_at_time(self, print_time):
        return self._bridge.get_homing_position_at_time(print_time)


# Reason codes MUST match MCU_trsync (klippy/mcu.py) so homing.py consumers see
# no behavior change.
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

# Bridge-private (from get_homing_segment_reason); distinct from MCU_trsync above.
BRIDGE_REASON_PAST_END_TIME = 1
BRIDGE_REASON_TRIPPED = 2
BRIDGE_REASON_DEADLINE_EXPIRED = 3

SOURCE_KIND_SOFTWARE = 2


_ARM_ID_COUNTER = [1]


def _alloc_arm_id():
    arm_id = _ARM_ID_COUNTER[0]
    _ARM_ID_COUNTER[0] = (arm_id + 1) & 0xFFFFFFFF
    if _ARM_ID_COUNTER[0] == 0:
        _ARM_ID_COUNTER[0] = 1
    return arm_id


class BridgeTriggerDispatch:
    """Bridge-mode replacement for klippy/mcu.py:TriggerDispatch; mirrors its
    surface so home_start / home_wait callers don't change.
    """

    def __init__(self, bridge, mcu, queue, reactor):
        self._bridge = bridge
        self._mcu = mcu
        self._queue = queue
        self._reactor = reactor
        self._arm_id = _alloc_arm_id()
        self._completion = reactor.completion()
        # (kind, gpio, active_high, policy, sample_n, velocity_axis, v_min_q16)
        self._sources = []
        self._stepper_oids = []
        self._steppers = []
        self._reason = None
        self._trip_event = None
        self._arm_print_time = None
        self._handler_registered = False
        self._toolhead_arms = None

    def get_oid(self):
        return self._arm_id

    def get_command_queue(self):
        return self._queue

    def get_arm_id(self):
        return self._arm_id

    def add_stepper(self, mcu_stepper):
        self._stepper_oids.append(mcu_stepper.get_oid())
        self._steppers.append(mcu_stepper)

    def get_steppers(self):
        return list(self._steppers)

    def add_source(
        self,
        kind,
        gpio,
        active_high,
        policy,
        sample_n,
        velocity_axis,
        v_min_q16,
    ):
        self._sources.append(
            (
                kind,
                gpio,
                active_high,
                policy,
                sample_n,
                velocity_axis,
                v_min_q16,
            )
        )

    def start(self, arm_print_time, mcu_obj):
        self._arm_print_time = arm_print_time
        arm_clock = int(mcu_obj.print_time_to_clock(arm_print_time))
        # ReactorCompletion is single-shot, so allocate a fresh one (and reset
        # per-arm state) each arm; otherwise homing's second pass inherits the
        # first's completed state, drip_move early-returns without moving, and
        # check_no_movement reports "Endstop still triggered after retract".
        self._completion = self._reactor.completion()
        self._reason = None
        self._trip_event = None

        # _bridge_handle isn't assigned until after identify (the MCU_endstop is
        # built at config phase), so refresh it (and lazily alloc the queue) here.
        if self._mcu is None:
            self._mcu = getattr(mcu_obj, "_bridge_handle", None)
            if self._mcu is None:
                raise mcu_obj.get_printer().command_error(
                    "BridgeTriggerDispatch: MCU bridge handle not yet "
                    "assigned (identify phase incomplete?)"
                )
        if self._queue is None:
            self._queue = self._bridge.alloc_command_queue(self._mcu)

        self._bridge.register_homing_dispatch(self._arm_id, self)

        # Register the async handler BEFORE arming, or we race the firmware's
        # kalico_endstop_tripped event.
        if not self._handler_registered:
            mcu_obj.register_response(
                self._on_trip_message, "kalico_endstop_tripped"
            )
            self._handler_registered = True

        # Register arm_id with the toolhead so its drip_move passes the right
        # arm_ids to submit_homing_move.
        printer = mcu_obj.get_printer()
        toolhead = printer.lookup_object("toolhead", None)
        if toolhead is not None and hasattr(toolhead, "active_homing_arms"):
            self._toolhead_arms = toolhead.active_homing_arms
            self._toolhead_arms.add(self._arm_id)

        logging.info(
            "[bridge-trace] endstop_arm arm_id=%s sources=%s steppers=%s",
            self._arm_id,
            self._sources,
            self._stepper_oids,
        )
        status = self._bridge.endstop_arm(
            self._mcu,
            self._queue,
            self._arm_id,
            arm_clock,
            self._sources,
            self._stepper_oids,
        )
        logging.info("[bridge-trace] endstop_arm status=%s", status)
        if status == ARM_STATUS_ALREADY_TRIPPED:
            # Pin asserted at arm time; arm() published a trip snapshot — fetch
            # it so home_wait can return a real trigger time.
            self._trip_event = self._bridge.take_trip_event() or {}
            self._reason = REASON_ENDSTOP_HIT
            self._completion.complete(self._reason)
        elif status == ARM_STATUS_REJECTED:
            raise printer.command_error(
                "runtime_arm_endstop rejected (status=%d)" % status
            )
        return self._completion

    def _on_trip_message(self, params):
        # Filter on arm_id (concurrent dispatch instances share the response).
        if int(params.get("arm_id", -1)) != self._arm_id:
            return
        if self._reason is not None:
            return
        # Real hardware carries the trip snapshot in params; native tests queue
        # it in the bridge runtime. Prefer params so stepper snapshots survive.
        self._trip_event = dict(params)
        if "steppers" not in self._trip_event:
            self._trip_event = self._bridge.take_trip_event()
        self._reason = REASON_ENDSTOP_HIT
        self._completion.complete(self._reason)

    def _fire_past_end_time(self):
        # Only fire if no terminal yet (mirror _on_trip_message).
        if self._reason is not None:
            return
        self._reason = REASON_PAST_END_TIME
        self._completion.complete(self._reason)

    def stop(self):
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
                # AlreadyTripped on race — take the queued event.
                self._trip_event = self._bridge.take_trip_event()
                self._reason = REASON_ENDSTOP_HIT
            if not self._completion.test():
                self._completion.complete(self._reason)
        self._bridge.unregister_homing_dispatch(self._arm_id)
        # Drop the arm_id from the toolhead registry so a later unrelated move's
        # drip_move doesn't pass it.
        if self._toolhead_arms is not None:
            self._toolhead_arms.discard(self._arm_id)
            self._toolhead_arms = None
        return self._reason

    def get_trip_event(self):
        if self._trip_event is None and self._reason == REASON_ENDSTOP_HIT:
            self._trip_event = self._bridge.take_trip_event()
        return self._trip_event

    def get_arm_print_time(self):
        return self._arm_print_time
