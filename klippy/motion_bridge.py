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
        "submit_homing_move",
        "submit_homing_move_async",
        "endstop_arm",
        "endstop_disarm",
        "software_trip",
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
        "register_stepper_slot",
        "eval_motor_position_at_clock",
        "eval_motor_position_now",
        "motor_positions_to_toolhead",
        "toolhead_delta_to_motor_slots",
        "forward_motor_positions",
        "ground_origin",
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
        # MCU objects that already have the shared kalico_endstop_tripped
        # handler registered. The handler routes by arm_id (see
        # register_trip_handler) — one per MCU, never per dispatch.
        self._trip_handler_mcus = set()
        self._homing_print_time_base = 0.0

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

    def register_trip_handler(self, mcu_obj):
        # One kalico_endstop_tripped handler per MCU, routing by arm_id.
        # Per-dispatch registration collides: register_response replaces by
        # message name, so only the last dispatch's handler survives and its
        # arm_id filter drops every other arm's trip — leaving re-homed axes
        # to time out on the home_wait backstop.
        if mcu_obj in self._trip_handler_mcus:
            return
        self._trip_handler_mcus.add(mcu_obj)
        mcu_obj.register_response(
            self._route_trip_message, "kalico_endstop_tripped"
        )

    def _route_trip_message(self, params):
        dispatch = self._homing_dispatches.get(int(params.get("arm_id", -1)))
        if dispatch is not None:
            dispatch._on_trip_message(params)

    def fire_homing_completion(self, arm_id):
        # No-op if no dispatch is registered (race with stop).
        dispatch = self._homing_dispatches.get(int(arm_id))
        if dispatch is not None:
            dispatch._fire_past_end_time()

    def register_stepper_slot(self, mcu_handle, oid, slot):
        return self._bridge.register_stepper_slot(mcu_handle, oid, slot)

    def eval_motor_position_now(self, mcu_handle, oid):
        return self._bridge.eval_motor_position_now(mcu_handle, oid)

    def eval_motor_position_at_clock(self, mcu_handle, oid, trip_clock):
        return self._bridge.eval_motor_position_at_clock(
            mcu_handle, oid, trip_clock
        )

    def motor_positions_to_toolhead(self, mcu_handle, motor_a_mm, motor_b_mm):
        return self._bridge.motor_positions_to_toolhead(
            mcu_handle, motor_a_mm, motor_b_mm
        )

    def toolhead_delta_to_motor_slots(self, mcu_handle, dx, dy, dz):
        return self._bridge.toolhead_delta_to_motor_slots(
            mcu_handle, dx, dy, dz
        )

    def forward_motor_positions(self, mcu_handle, x, y, z):
        return self._bridge.forward_motor_positions(mcu_handle, x, y, z)

    def ground_origin(self):
        return self._bridge.ground_origin()

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
        # `sources`: list of 5-tuples (kind, gpio, active_high, policy, sample_n).
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

    def trip_dispatch_prepare(self, sources, sinks, participants, expire_timeout_s):
        return self._bridge.trip_dispatch_prepare(
            sources, sinks, participants, expire_timeout_s
        )

    def trip_dispatch_cleanup(self, handle_id):
        return self._bridge.trip_dispatch_cleanup(handle_id)


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
        # 5-tuples: (kind, gpio, active_high, policy, sample_n)
        self._sources = []
        self._stepper_oids = []
        self._steppers = []
        self._reason = None
        self._trip_event = None
        self._arm_print_time = None
        self._toolhead_arms = None  # set at start() for stop() cleanup
        self._sink_trsyncs = {}     # MCU object → MCU_trsync (firmware sink)
        self._classic_trsyncs = []  # MCU_trsync objects on non-bridge MCUs
        self._trip_handle_id = None # TripDispatch relay handle (None=inactive)
        # Mutable cell shared with sink trsync_state handlers; cleared in
        # stop() so late-arriving trsync_state frames after teardown no-op.
        self._sink_handler_armed = [False]

    def get_oid(self):
        return self._arm_id

    def get_command_queue(self):
        return self._queue

    def get_arm_id(self):
        return self._arm_id

    def add_stepper(self, mcu_stepper):
        self._stepper_oids.append(mcu_stepper.get_oid())
        self._steppers.append(mcu_stepper)
        # Function-level import to avoid the mcu <-> motion_bridge
        # circular import (mirrors the `from . import motion_bridge as _mb`
        # pattern used on the mcu.py side).
        from .mcu import MCU_trsync
        stepper_mcu = mcu_stepper.get_mcu()
        trsync = self._sink_trsyncs.get(stepper_mcu)
        if trsync is None:
            trsync = MCU_trsync(stepper_mcu, None)
            self._sink_trsyncs[stepper_mcu] = trsync
        trsync.add_stepper(mcu_stepper)

    def get_steppers(self):
        return list(self._steppers)

    def add_classic_trsync(self, mcu_trsync):
        """Register a non-bridge MCU_trsync (e.g. Beacon) as a classic source
        and participant. Its trsync_state can_trigger=0 fans trsync_trigger to
        all sink trsyncs; its can_trigger=1 reports keep the liveness web live.
        Call before start().
        """
        self._classic_trsyncs.append(mcu_trsync)

    def add_source(self, kind, gpio, active_high, policy, sample_n):
        self._sources.append((kind, gpio, active_high, policy, sample_n))

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

        # Register the shared (arm_id-routed) handler BEFORE arming, or we race
        # the firmware's kalico_endstop_tripped event. Idempotent per MCU.
        self._bridge.register_trip_handler(mcu_obj)

        # Register arm_id with the toolhead so its drip_move passes the right
        # arm_ids to submit_homing_move.
        printer = mcu_obj.get_printer()
        toolhead = printer.lookup_object("toolhead", None)
        if toolhead is not None and hasattr(toolhead, "active_homing_arms"):
            self._toolhead_arms = toolhead.active_homing_arms
            self._toolhead_arms.add(self._arm_id)

        # Re-arming without a prior stop() is an unexpected lifecycle
        # state — the relay handle from the previous arm would leak.
        # Fail loud rather than silently overwrite it.
        if self._trip_handle_id is not None:
            raise printer.command_error(
                "BridgeTriggerDispatch.start: stale trip handle %d — "
                "prior homing arm never stopped" % self._trip_handle_id
            )

        # A homing arm with zero sink trsyncs means nothing freezes the
        # curve evaluators on trip, so motion would not stop. This can
        # only happen if add_stepper was never called for this dispatch —
        # a real wiring bug. Fail loud.
        if not self._sink_trsyncs:
            raise printer.command_error(
                "BridgeTriggerDispatch.start: homing arm_id=%d has no "
                "sink trsyncs; nothing would stop motion on trip"
                % self._arm_id
            )

        # Sink trsyncs are armed (firmware trsync_start + runtime_stop_on_trigger)
        # and the Rust relay is prepared BEFORE endstop_arm so a GPIO trip cannot
        # race the arming window. endstop_arm can raise on ARM_STATUS_REJECTED;
        # the except block below tears the relay down so the handle never leaks.
        from .extras.danger_options import get_danger_options
        n_participants = len(self._sink_trsyncs) + len(self._classic_trsyncs)
        expire_timeout = get_danger_options().multi_mcu_trsync_timeout
        if n_participants == 1:
            expire_timeout = get_danger_options().single_mcu_trsync_timeout
        # Re-arm the handler cell so sink trsync_state callbacks are live for
        # this homing arm (cleared in stop()).
        self._sink_handler_armed[0] = True
        try:
            sink_participants = []
            for i, trsync in enumerate(self._sink_trsyncs.values()):
                trsync._bridge_arm_id = self._arm_id
                trsync.start(
                    arm_print_time, float(i) / n_participants,
                    self._completion, expire_timeout,
                )
                mcu_handle = trsync.get_mcu()._bridge_handle
                sink_participants.append((mcu_handle, trsync.get_oid()))
                # Register a trsync_state handler on this sink's MCU so a
                # REASON_COMMS_TIMEOUT expire (can_trigger=0, trigger_reason>=4)
                # surfaces as a loud completion instead of blocking home_wait.
                self._register_sink_timeout_handler(trsync)
            # Arm classic (non-bridge) trsyncs (e.g. Beacon) and add them as
            # sources and participants in TripDispatch. Their trsync_state
            # can_trigger=0 fans trsync_trigger to all sink trsyncs; their
            # can_trigger=1 reports extend the liveness window.
            # A dummy completion is passed to MCU_trsync.start() so the
            # classic trsync's _handle_trsync_state does not race our real
            # _completion. Trip notification arrives via TripDispatch relay
            # → kalico_endstop_tripped → _on_trip_message on the stepper MCU.
            n_sinks = len(self._sink_trsyncs)
            classic_participants = []
            for j, ct in enumerate(self._classic_trsyncs):
                mcu_handle = ct.get_mcu()._bridge_handle
                if mcu_handle is None:
                    raise printer.command_error(
                        "BridgeTriggerDispatch.start: classic trsync MCU "
                        "'%s' has no bridge handle (not a bridge MCU?)"
                        % ct.get_mcu().get_name()
                    )
                report_offset = float(n_sinks + j) / n_participants
                ct.start(arm_print_time, report_offset,
                         self._reactor.completion(), expire_timeout)
                classic_participants.append((mcu_handle, ct.get_oid()))
            sources = [(0, self._mcu, self._arm_id)]
            # kind=1 = SourceSpec::Trsync — listens for trsync_state
            # can_trigger=0 and fans trsync_trigger to sink trsyncs.
            for mcu_handle, trsync_oid in classic_participants:
                sources.append((1, mcu_handle, trsync_oid))
            all_participants = sink_participants + classic_participants
            self._trip_handle_id = self._bridge.trip_dispatch_prepare(
                sources, sink_participants, all_participants, expire_timeout
            )

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
            if status == ARM_STATUS_REJECTED:
                raise printer.command_error(
                    "runtime_arm_endstop rejected (status=%d)" % status
                )
        except Exception:
            # Arming failed after the relay was prepared (or while arming
            # the sinks). Tear the relay down so the handle is not leaked
            # or silently overwritten by the next start(); stop()'s disarm
            # logic drives the sinks down on the normal teardown path.
            if self._trip_handle_id is not None:
                self._bridge.trip_dispatch_cleanup(self._trip_handle_id)
                self._trip_handle_id = None
            raise

        if status == ARM_STATUS_ALREADY_TRIPPED:
            # Pin asserted at arm time under TripImmediately. The
            # firmware published a trip snapshot in arm() itself — fetch
            # it now so home_wait can return a real trigger time. This is
            # a normal early completion (not a failure): the relay stays
            # set up; stop() tears it down in home_wait as usual.
            self._trip_event = self._bridge.take_trip_event() or {}
            self._reason = REASON_ENDSTOP_HIT
            self._completion.complete(self._reason)
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

    def _register_sink_timeout_handler(self, trsync):
        trsync_oid = trsync.get_oid()
        armed_cell = self._sink_handler_armed
        completion = self._completion
        reactor = self._reactor
        dispatch = self

        def _on_sink_trsync_state(params):
            if not armed_cell[0]:
                return
            if params.get("can_trigger", 1):
                return
            reason = int(params.get("trigger_reason", 0))
            if reason < REASON_COMMS_TIMEOUT:
                return
            armed_cell[0] = False
            dispatch._reason = reason
            reactor.async_complete(completion, reason)

        trsync.get_mcu().register_response(
            _on_sink_trsync_state, "trsync_state", trsync_oid
        )

    def _fire_past_end_time(self):
        # Only fire if no terminal yet (mirror _on_trip_message).
        if self._reason is not None:
            return
        self._reason = REASON_PAST_END_TIME
        self._completion.complete(self._reason)

    def stop(self):
        # Neutralize sink trsync_state handlers before any other teardown so
        # late-arriving frames from the firmware can't race _completion after
        # we've already decided the reason.
        self._sink_handler_armed[0] = False
        # Called from MCU_endstop.home_wait. Tear down the cross-MCU trip
        # relay first so no further trsync_triggers fire on the sinks.
        # Reset the handle BEFORE the cleanup call so a raised cleanup
        # can't strand it (re-leak), and don't let a cleanup failure abort
        # the rest of teardown (disarm/unregister). Idempotent on
        # double-stop: handle is None on the second call.
        if self._trip_handle_id is not None:
            handle = self._trip_handle_id
            self._trip_handle_id = None
            try:
                self._bridge.trip_dispatch_cleanup(handle)
            except Exception:
                logging.exception(
                    "[bridge-trace] trip_dispatch_cleanup(%d) raised during "
                    "stop(); handle already cleared, continuing teardown",
                    handle,
                )
        # ALWAYS disarm — even after a trip. In the bridge engine,
        # endstop_disarm cancels the firmware poll task and restarts TIM5 (the
        # motion clock the trip halted). A trip leaves the firmware arm tripped,
        # so disarm returns AlreadyTripped — that still restarts TIM5 and clears
        # the poll task, which the post-home set_position needs to drain.
        # (Mainline can skip disarm on a trip because the tripped trsync
        # self-stops the steppers; our stop is TIM5, restarted only by disarm.)
        try:
            status = self._bridge.endstop_disarm(
                self._mcu, self._queue, self._arm_id
            )
        except Exception:
            status = DISARM_STATUS_UNKNOWN
        if self._reason is None:
            if status == DISARM_STATUS_DISARMED:
                self._reason = REASON_HOST_REQUEST
            else:
                # AlreadyTripped on race — take the queued event.
                self._trip_event = self._bridge.take_trip_event()
                self._reason = REASON_ENDSTOP_HIT
            if not self._completion.test():
                self._completion.complete(self._reason)
        # Disarm classic trsyncs (e.g. Beacon). MCU_trsync.stop() on a
        # non-bridge MCU sends trsync_trigger HOST_REQUEST and unregisters
        # the trsync_state response handler — normal mainline teardown.
        for ct in self._classic_trsyncs:
            try:
                ct.stop()
            except Exception:
                logging.exception(
                    "[bridge-trace] classic trsync stop() raised during "
                    "BridgeTriggerDispatch.stop(); continuing teardown"
                )
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
