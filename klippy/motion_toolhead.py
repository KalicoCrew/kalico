# ToolHead subclass that drives motion through the Rust bridge.
import logging
import os
import struct
from collections import defaultdict

from . import motion_kinematics, stepper
from .extras import servo_axis
from .kinematics import extruder
from .toolhead import BUFFER_TIME_START, Move, ToolHead

# Slot order must match motion_kinematics.motor_deltas and
# _configure_axes_per_mcu's slot_names. A name matching no prefix has no slot
# (e.g. corexy passthrough-Z rails) and is skipped.
_MOTOR_SLOT_PREFIXES = (
    (0, "stepper_x"),
    (1, "stepper_y"),
    (2, "stepper_z"),
    (3, "extruder"),
)


def _name_motor_slot(name):
    """Return (slot, is_primary) for a stepper section name, or None. is_primary
    is False for AWD partners (stepper_x1, stepper_z2, ...).
    """
    for slot_idx, prefix in _MOTOR_SLOT_PREFIXES:
        if not name.startswith(prefix):
            continue
        suffix = name[len(prefix) :]
        if suffix == "":
            return (slot_idx, True)
        if suffix.isdigit():
            return (slot_idx, False)
    return None


def _stepper_motor_slot(stepper_obj):
    info = _name_motor_slot(stepper_obj.get_name())
    return None if info is None else info[0]


_AXIS_X = 0
_AXIS_Y = 1

# Mirror rust KinematicTag discriminants.
_KIN_COREXY = 0
_KIN_CARTESIAN = 1


def _derive_mcu_topology(axis_to_handle, kinematics_name):
    """Map axis->MCU-handle to a list of (handle, sorted_axes, kinematics_tag),
    one per handle. An MCU's tag is COREXY iff the printer is corexy and that
    MCU carries both X and Y, else CARTESIAN.
    """
    by_handle = {}
    for axis_idx, handle in axis_to_handle.items():
        by_handle.setdefault(handle, []).append(axis_idx)
    is_corexy = (kinematics_name or "").lower() == "corexy"
    topo = []
    for handle in sorted(by_handle):
        axes = sorted(by_handle[handle])
        if is_corexy and _AXIS_X in axes and _AXIS_Y in axes:
            tag = _KIN_COREXY
        else:
            tag = _KIN_CARTESIAN
        topo.append((handle, axes, tag))
    return topo


def _open_sim_control():
    """Open the shim's control socket, or None if the shim isn't in use."""
    sock_dir = os.environ.get("KALICO_SIM_SOCK_DIR")
    if not sock_dir:
        return None
    sock_path = os.path.join(sock_dir, "sim_control")
    if not os.path.exists(sock_path):
        return None
    try:
        from tools.sim_klippy.orchestrator.sim_control_client import (
            SimControlClient,
        )
    except ImportError:
        return None
    return SimControlClient(sock_path)


class BridgeKinematics:
    """Host-side homed/range enforcement for the bridge planner. Each axis's
    (low, high) limit starts inverted (1.0, -1.0) = "unhomed" and is replaced
    by the rail range once set_position runs with that axis in homing_axes;
    check_move raises "Must home axis first" while the sentinel stands.
    """

    supports_dual_carriage = False

    def __init__(self, toolhead, config):
        self._toolhead = toolhead
        kin_name = config.get("kinematics")
        if kin_name not in ("cartesian", "corexy", "hybrid_corexy"):
            raise config.error(
                "Unsupported bridge kinematics '%s'" % (kin_name,)
            )
        self.kinematics = kin_name
        self.rails = []
        self._printer = config.get_printer()

        axes = "xy"
        if kin_name in ("cartesian", "hybrid_corexy"):
            axes = "xyz"
        for axis in axes:
            self._register_axis(config, axis, extras=("1",))
        if kin_name == "corexy" and len(self.rails) >= 2:
            x_endstop = self.rails[0].get_endstops()[0][0]
            y_endstop = self.rails[1].get_endstops()[0][0]
            for s in self.rails[1].get_steppers():
                x_endstop.add_stepper(s)
            for s in self.rails[0].get_steppers():
                y_endstop.add_stepper(s)
        # Z is independent in corexy but still dispatched to the Z MCU; register
        # its rails so config validation and homing work.
        if kin_name == "corexy" and config.has_section("stepper_z"):
            self._register_axis(config, "z", extras=("1", "2", "3"))

        # low > high means "unhomed" (the "Must home axis first" path).
        self.limits = [(1.0, -1.0)] * 3

        # Clear homed state on de-energize (M84 / shutdown) so G1s re-home.
        self._printer.register_event_handler(
            "stepper_enable:motor_off",
            self._handle_motor_off,
        )

    def _handle_motor_off(self, print_time):
        self.clear_homing_state((0, 1, 2))

    def _register_axis(self, config, axis, extras=()):
        # A [servo_<axis>] section routes the axis to an EtherCAT node. ServoRail
        # honors the rail contract but has no MCU steppers, itersolve, or
        # _bridge_drives_steppers marker.
        servo_sec = "servo_" + axis
        stepper_sec = "stepper_" + axis
        has_servo = config.has_section(servo_sec)
        has_stepper = config.has_section(stepper_sec)
        if has_servo and has_stepper:
            raise config.error(
                "axis %s has both [%s] and [%s]; pick one"
                % (axis, servo_sec, stepper_sec)
            )
        if has_servo:
            rail = servo_axis.ServoRail(config.getsection(servo_sec))
            servo_axis.register_torque_enable(self._printer, config, rail)
            self.rails.append(rail)
            return
        rail = stepper.PrinterRail(config.getsection("stepper_" + axis))
        for suffix in extras:
            extra_name = "stepper_" + axis + suffix
            if config.has_section(extra_name):
                rail.add_extra_stepper(config.getsection(extra_name))
        for mcu_stepper in rail.get_steppers():
            mcu_stepper.setup_itersolve(
                "cartesian_stepper_alloc", axis.encode()
            )
            mcu_stepper.get_mcu()._bridge_drives_steppers = True
        self.rails.append(rail)

    def _axis_rails(self):
        # Locate by name prefix (the corexy passthrough-Z rail may follow X/Y).
        out = {}
        for rail in self.rails:
            name = rail.get_name(short=True) or ""
            if name and name[0] in "xyz":
                idx = "xyz".index(name[0])
                out.setdefault(idx, rail)
        return out

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
        def rail_pos(rail):
            vals = [
                stepper_positions.get(s.get_name(), 0.0)
                for s in rail.get_steppers()
            ]
            if not vals:
                return 0.0
            return sum(vals) / len(vals)

        axis_rails = self._axis_rails()
        if self.kinematics == "corexy":
            a = rail_pos(axis_rails.get(0)) if 0 in axis_rails else 0.0
            b = rail_pos(axis_rails.get(1)) if 1 in axis_rails else 0.0
            z = rail_pos(axis_rails.get(2)) if 2 in axis_rails else 0.0
            return [0.5 * (a + b), 0.5 * (a - b), z]
        return [
            rail_pos(axis_rails.get(i)) if i in axis_rails else 0.0
            for i in (0, 1, 2)
        ]

    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if move.axes_d[i] and (
                end_pos[i] < self.limits[i][0] or end_pos[i] > self.limits[i][1]
            ):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()

    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (
            xpos < limits[0][0]
            or xpos > limits[0][1]
            or ypos < limits[1][0]
            or ypos > limits[1][1]
        ):
            self._check_endstops(move)
        if not move.axes_d[2]:
            return
        # Derate velocity/accel by the Z share.
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self._toolhead.max_z_velocity * z_ratio,
            self._toolhead.max_z_accel * z_ratio,
        )

    def home(self, homing_state):
        axis_rails = self._axis_rails()
        for axis in homing_state.get_axes():
            rail = axis_rails.get(axis)
            if rail is None:
                continue
            position_min, position_max = rail.get_range()
            hi = rail.get_homing_info()
            homepos = [None, None, None, None]
            homepos[axis] = hi.position_endstop
            forcepos = list(homepos)
            if hi.positive_dir:
                forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
            else:
                forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
            homing_state.home_rails([rail], forcepos, homepos)

    def set_position(self, newpos, homing_axes=()):
        self._toolhead.bridge._software_trip_active = False
        self._toolhead.bridge.set_position(newpos[0], newpos[1], newpos[2])
        axis_rails = self._axis_rails()
        for axis_idx, rail in axis_rails.items():
            if self.kinematics == "corexy" and axis_idx < 2:
                motor_pos = (
                    (newpos[0] + newpos[1])
                    if axis_idx == 0
                    else (newpos[0] - newpos[1])
                )
            else:
                motor_pos = newpos[axis_idx] if axis_idx < len(newpos) else 0.0
            for s in rail.get_steppers():
                step_count = int(motor_pos / s.get_step_dist() + 0.5)
                s._set_mcu_position(step_count)
        for axis in homing_axes:
            rail = axis_rails.get(axis)
            if rail is not None:
                self.limits[axis] = rail.get_range()

    def note_z_not_homed(self):
        # [beacon] prefers this over clear_homing_state("z") when present;
        # exposing it keeps clear_homing_state on the int-iterable contract.
        self.clear_homing_state([2])

    def clear_homing_state(self, axes):
        # axes is an iterable of axis indices (mainline contract).
        for i in (0, 1, 2):
            if i in axes:
                self.limits[i] = (1.0, -1.0)

    def get_status(self, eventtime):
        from . import gcode as gcode_mod

        ranges = {}
        for rail in self.rails:
            name = rail.get_name(short=True)
            if name and name[0] in "xyz":
                ranges[name[0]] = (rail.position_min, rail.position_max)
        x_min, x_max = ranges.get("x", (0.0, 0.0))
        y_min, y_max = ranges.get("y", (0.0, 0.0))
        z_min, z_max = ranges.get("z", (0.0, 0.0))
        homed = "".join(
            a
            for i, a in enumerate("xyz")
            if self.limits[i][0] <= self.limits[i][1]
        )
        return {
            "homed_axes": homed,
            "axis_minimum": gcode_mod.Coord(x_min, y_min, z_min, 0.0),
            "axis_maximum": gcode_mod.Coord(x_max, y_max, z_max, 0.0),
        }


class MotionToolhead(ToolHead):
    """Bridge-aware ToolHead subclass; overrides only the methods where the
    Rust bridge owns behavior (move issuance, timeline, velocity limits).
    """

    def __init__(self, config):
        # Pre-super: attributes BridgeKinematics / handlers reference during
        # super().__init__.
        printer = config.get_printer()
        self.bridge = printer.lookup_object("motion_bridge", None)
        if self.bridge is None:
            from . import motion_bridge

            self.bridge = motion_bridge._StubBridge()
        self.active_homing_arms = set()
        self.kinematics_name = config.get("kinematics", "")
        self.bridge._kinematics_name = self.kinematics_name
        # Projected MCU print-time of the end of the last queued bridge move.
        self._mcu_pending_end_time = 0.0

        super().__init__(config)

        # Bridge owns the timeline; silence upstream's flush machinery.
        self.reactor.update_timer(self.flush_timer, self.reactor.NEVER)
        self.do_kick_flush_timer = False

        # Bridge-only config keys (not parsed by upstream ToolHead).
        self.max_z_velocity = config.getfloat(
            "max_z_velocity", self.max_velocity, above=0.0
        )
        self.max_z_accel = config.getfloat(
            "max_z_accel", self.max_accel, above=0.0
        )

        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "KALICO_SIM_STEP_COUNT",
            self.cmd_KALICO_SIM_STEP_COUNT,
            desc="[sim] Query cumulative step count for a stepper OID",
        )
        gcode.register_command(
            "KALICO_SIM_AXIS_STEPS",
            self.cmd_KALICO_SIM_AXIS_STEPS,
            desc="[sim] Query configured steps_per_mm for an axis OID",
        )
        gcode.register_command(
            "KALICO_SIM_AXIS_ACCUM",
            self.cmd_KALICO_SIM_AXIS_ACCUM,
            desc="[sim] Query step accumulator for an axis OID",
        )
        gcode.register_command(
            "KALICO_SIM_ENDSTOP_SET_PIN",
            self.cmd_KALICO_SIM_ENDSTOP_SET_PIN,
            desc="[sim] Drive a Linux-MCU GPIO level (test fixture)",
        )
        gcode.register_command(
            "KALICO_DIAG_DUMP",
            self.cmd_KALICO_DIAG_DUMP,
            desc="Emit the live MCU diag snapshot (cause discriminators + "
            "event ring) to the structured-log store; no reset required",
        )

        self.printer.register_event_handler(
            "klippy:connect", self._init_planner
        )
        self.printer.register_event_handler(
            "klippy:disconnect", self._handle_disconnect
        )
        import signal

        def _sigterm_handler(signum, frame):
            self.printer.request_exit("exit")

        signal.signal(signal.SIGTERM, _sigterm_handler)

        logging.info("MotionToolhead: Phase 1 skeleton initialized")

    def _handle_disconnect(self):
        logging.info("MotionToolhead: _handle_disconnect called")
        if self.bridge is not None:
            logging.info("MotionToolhead: calling bridge.shutdown()")
            self.bridge.shutdown()
            logging.info("MotionToolhead: bridge.shutdown() returned")

    def _load_kinematics(self, config):
        return BridgeKinematics(self, config)

    def move(self, newpos, speed):
        # The bridge replaces the lookahead, but Move/kin/extruder validation
        # (unhomed, range checks) must still run before the move is issued.
        move = Move(self, self.commanded_pos, newpos, speed)
        if not move.move_d:
            return
        if move.is_kinematic_move:
            self.kin.check_move(move)
        if move.axes_d[3]:
            self.extruder.check_move(move)
        dx, dy, dz, de = move.axes_d
        feedrate = move.move_d / move.min_move_t
        if abs(dz) > 1e-9 and abs(dx) < 1e-9 and abs(dy) < 1e-9:
            feedrate = min(feedrate, self.max_z_velocity)
        logging.info(
            "[bridge-trace] move: newpos=%s speed=%s dx=%.4f dy=%.4f "
            "dz=%.4f de=%.4f feedrate=%.4f",
            list(newpos),
            speed,
            dx,
            dy,
            dz,
            de,
            feedrate,
        )
        enable_print_time = self.get_last_move_time()
        self._fire_active_callbacks(dx, dy, dz, de, enable_print_time)
        bridge_lmt_before = self.bridge.get_last_move_time()
        self.bridge.submit_move(dx, dy, dz, de, feedrate)
        self._bump_pending_end_time(
            self.bridge.get_last_move_time() - bridge_lmt_before
        )
        self.commanded_pos[:] = move.end_pos

    def _fire_active_callbacks(self, dx, dy, dz, de, print_time=None):
        if self.kin is None:
            return False
        kin_name = (self.kinematics_name or "").lower()
        deltas = motion_kinematics.motor_deltas(kin_name, dx, dy, dz, de)
        # Early-out is safe for the servo pass below: under cartesian the
        # motor deltas ARE the axis deltas, and under corexy a nonzero axis
        # delta always produces a nonzero a/b motor delta — so a zero-motor
        # move can never carry a nonzero servo-axis delta.
        if all(abs(d) <= 1e-9 for d in deltas):
            return False
        if print_time is None:
            print_time = self.get_last_move_time()
        fired = False
        for s in self.kin.get_steppers():
            if not s._active_callbacks:
                continue
            slot = _stepper_motor_slot(s)
            if slot is None or abs(deltas[slot]) <= 1e-9:
                continue
            cbs = s._active_callbacks
            s._active_callbacks = []
            for cb in cbs:
                cb(print_time)
            fired = True
        # Servo rails have no steppers and no motor slot; their enable gate
        # arms on the AXIS delta (under corexy the slot deltas carry a/b
        # motor values, which would be wrong for a direct-drive axis).
        for rail in getattr(self.kin, "rails", ()):
            if not isinstance(rail, servo_axis.ServoRail):
                continue
            if not rail._active_callbacks:
                continue
            axis_delta = (dx, dy, dz)["xyz".index(rail.axis)]
            if abs(axis_delta) <= 1e-9:
                continue
            cbs = rail._active_callbacks
            rail._active_callbacks = []
            for cb in cbs:
                cb(print_time)
            fired = True
        return fired

    def drip_move(self, newpos, speed, drip_completion):
        logging.info(
            "[bridge-trace] drip_move entered: newpos=%s speed=%s "
            "drip_test=%s active_homing_arms=%s",
            list(newpos),
            speed,
            (drip_completion.test() if drip_completion is not None else None),
            sorted(self.active_homing_arms),
        )
        if drip_completion is not None and drip_completion.test():
            return
        arm_ids = list(self.active_homing_arms)
        if arm_ids:
            # Bridge-native GPIO/sensorless path.
            pos3 = list(newpos[:3]) + [0.0] * max(0, 3 - len(newpos[:3]))
            dx = pos3[0] - self.commanded_pos[0]
            dy = pos3[1] - self.commanded_pos[1]
            dz = pos3[2] - self.commanded_pos[2]
            self._fire_active_callbacks(
                dx, dy, dz, 0.0, self.get_last_move_time()
            )
            self.bridge._software_trip_active = False
            bridge_lmt_before = self.bridge.get_last_move_time()
            self.bridge.submit_homing_move(pos3, speed, arm_ids)
            self.bridge.wait_moves()
            bridge_lmt_after = self.bridge.get_last_move_time()
            duration = bridge_lmt_after - bridge_lmt_before
            self._bump_pending_end_time(duration)
        elif drip_completion is not None and not drip_completion.test():
            # External probe software-trip path.
            self._drip_move_software_trip(newpos, speed, drip_completion)
        else:
            self.move(newpos, speed)

    def _prepare_probe_interceptor(self, endstops):
        """Pre-register the Rust frame interceptor for the probe's trsync, from
        homing.py BEFORE home_start(). Stores the handle id on self for
        _drip_move_software_trip.
        """
        self._probe_homing_handle_id = None
        stepper_mcus = set()
        for s in self.kin.get_steppers():
            if s.get_name().startswith("stepper_z"):
                stepper_mcus.add(s.get_mcu())
        if len(stepper_mcus) != 1:
            return
        stepper_mcu = next(iter(stepper_mcus))
        for mcu_endstop, name in endstops:
            es_mcu = mcu_endstop.get_mcu()
            if es_mcu == stepper_mcu:
                continue
            if es_mcu._bridge_handle is None:
                continue
            trsync = getattr(
                getattr(mcu_endstop, "_shared", None), "_trsync", None
            )
            if trsync is None:
                continue
            from . import motion_bridge as _mb

            arm_id = _mb._alloc_arm_id()
            nominal_dist = 250.0
            axis_rails = self.kin._axis_rails()
            z_rail = axis_rails.get(2)
            if z_rail is not None:
                z_min, z_max = z_rail.get_range()
                nominal_dist = abs(z_max - z_min)
            sensor_fault_timeout = nominal_dist / 5.0 + 5.0
            handle_id = self.bridge.prepare_probe_homing(
                es_mcu._bridge_handle,
                trsync.get_oid(),
                stepper_mcu._bridge_handle,
                arm_id,
                sensor_fault_timeout,
            )
            self._probe_homing_handle_id = handle_id
            self._probe_homing_arm_id = arm_id
            logging.info(
                "[probe-homing] interceptor registered: handle_id=%d "
                "beacon_handle=%s trsync_oid=%d arm_id=%d",
                handle_id,
                es_mcu._bridge_handle,
                trsync.get_oid(),
                arm_id,
            )
            return

    def _drip_move_software_trip(self, newpos, speed, drip_completion):
        from . import motion_bridge as _mb
        from . import motion_kinematics

        self.bridge.wait_moves()
        self._ground_pending_end_time_after_bridge_drain()

        pos3 = list(newpos[:3]) + [0.0] * max(0, 3 - len(newpos[:3]))
        dx = pos3[0] - self.commanded_pos[0]
        dy = pos3[1] - self.commanded_pos[1]
        dz = pos3[2] - self.commanded_pos[2]

        logging.info(
            "[diag] _drip_move_software_trip: "
            "commanded_pos=[%.3f,%.3f,%.3f] pos3=[%.3f,%.3f,%.3f] "
            "dx=%.6f dy=%.6f dz=%.6f",
            self.commanded_pos[0],
            self.commanded_pos[1],
            self.commanded_pos[2],
            pos3[0],
            pos3[1],
            pos3[2],
            dx,
            dy,
            dz,
        )

        kin_name = self.kinematics_name or ""
        motor_d = motion_kinematics.motor_deltas(kin_name, dx, dy, dz, 0.0)
        slot_prefixes = ["stepper_x", "stepper_y", "stepper_z", "extruder"]
        moving_steppers = []
        for slot_idx, delta in enumerate(motor_d):
            if abs(delta) < 1e-9:
                continue
            prefix = slot_prefixes[slot_idx]
            for s in self.kin.get_steppers():
                if s.get_name().startswith(prefix):
                    moving_steppers.append(s)

        if not moving_steppers:
            self.move(newpos, speed)
            return

        stepper_mcus = set(s.get_mcu() for s in moving_steppers)
        if len(stepper_mcus) > 1:
            raise self.printer.command_error(
                "External probe homing across multiple bridge MCUs "
                "is not supported"
            )
        stepper_mcu = next(iter(stepper_mcus))
        mcu_handle = stepper_mcu._bridge_handle
        queue = self.bridge.alloc_command_queue(mcu_handle)

        arm_id = getattr(self, "_probe_homing_arm_id", None)
        if arm_id is None:
            arm_id = _mb._alloc_arm_id()
        stepper_oids = [s.get_oid() for s in moving_steppers]
        source = (_mb.SOURCE_KIND_SOFTWARE, 0, False, 0, 1, 0, 0)

        # The enable callbacks do inline TMC UART reads (~300ms for 3 Z
        # steppers); pad so the last queue_digital_out clock stays in the future.
        ENABLE_HEADROOM = 2.000
        lmt = self.get_last_move_time()
        est_now = 0.0
        if self.mcu is not None:
            est_now = self.mcu.estimated_print_time(self.reactor.monotonic())
            needed = est_now + ENABLE_HEADROOM
            if lmt < needed:
                self.dwell(needed - lmt)
                lmt = self.get_last_move_time()

        logging.info(
            "[probe-homing] pre-enable: "
            "lmt=%.6f est_now=%.6f pending=%.6f stepper_mcu=%s",
            lmt,
            est_now,
            self._mcu_pending_end_time,
            stepper_mcu.get_name(),
        )

        self._fire_active_callbacks(dx, dy, dz, 0.0, lmt)

        # Recompute arm_clock after callbacks — the UART traffic consumed
        # wall-clock time, so the pre-callback lmt may now be in the past.
        if self.mcu is not None:
            est_now = self.mcu.estimated_print_time(self.reactor.monotonic())
        arm_clock = int(
            stepper_mcu.print_time_to_clock(
                max(lmt, est_now + BUFFER_TIME_START)
            )
        )
        logging.info(
            "[probe-homing] post-enable: est_now=%.6f arm_clock=%d",
            est_now,
            arm_clock,
        )

        # Arm + submit
        self.active_homing_arms.add(arm_id)
        self.bridge.register_homing_dispatch(arm_id, None)
        self.bridge._software_trip_active = True

        bridge_lmt_before = self.bridge.get_last_move_time()
        try:
            self.bridge.endstop_arm(
                mcu_handle,
                queue,
                arm_id,
                arm_clock,
                [source],
                stepper_oids,
            )
            self.bridge._homing_print_time_base = bridge_lmt_before

            handle_id = getattr(self, "_probe_homing_handle_id", None)
            if handle_id is None:
                raise self.printer.command_error(
                    "No probe homing interceptor registered "
                    "(call _prepare_probe_interceptor first)"
                )
            # arm_id from prepare — matches what the interceptor sends.
            arm_id = self._probe_homing_arm_id

            logging.info(
                "[probe-homing] calling run_probe_homing: "
                "handle_id=%d arm_id=%d speed=%.1f",
                handle_id,
                arm_id,
                speed,
            )

            result = self.bridge.run_probe_homing(
                handle_id,
                pos3,
                speed,
                stepper_oids,
            )

            PROBE_TRIGGERED = 0
            SEGMENT_RETIRED = 1
            SENSOR_FAULT = 2
            DEADLINE_EXPIRED = 3

            if result == SENSOR_FAULT:
                raise self.printer.command_error(
                    "Probe sensor fault: no trigger during full Z "
                    "travel. Check probe wiring and threshold."
                )
            if result == DEADLINE_EXPIRED:
                raise self.printer.command_error(
                    "Homing deadline expired: MCU dead-man switch "
                    "fired (host extension loop may have stalled)"
                )

            self.bridge.wait_moves()
            bridge_lmt_after = self.bridge.get_last_move_time()
            duration = bridge_lmt_after - bridge_lmt_before
            self._bump_pending_end_time(duration)
        finally:
            self.bridge._software_trip_active = False
            self.active_homing_arms.discard(arm_id)
            self.bridge.unregister_homing_dispatch(arm_id)
            try:
                self.bridge.endstop_disarm(mcu_handle, queue, arm_id)
            except Exception:
                pass  # best-effort cleanup

    def dwell(self, delay):
        self.bridge.submit_dwell(delay)
        if delay > 0.0:
            self._bump_pending_end_time(delay)

    def wait_moves(self):
        self.bridge.wait_moves()

    def wait_moves_and_mcu(self):
        self.bridge.drain_motion()
        self._ground_pending_end_time_after_bridge_drain()

    def cmd_M400(self, gcmd):
        # Overrides the upstream M400 binding so it routes through the full
        # motion drain instead of wait_moves().
        self.wait_moves_and_mcu()

    def _bridge_mcus(self):
        if not hasattr(self, "_cached_bridge_mcus"):
            mcus = set()
            if self.kin is not None:
                for s in self.kin.get_steppers():
                    mcus.add(s.get_mcu())
            self._cached_bridge_mcus = list(mcus) if mcus else [self.mcu]
        return self._cached_bridge_mcus

    def flush_step_generation(self):
        self.bridge.wait_moves()
        if self._mcu_pending_end_time > 0.0:
            for mcu in self._bridge_mcus():
                while True:
                    est = mcu.estimated_print_time(self.reactor.monotonic())
                    remaining = self._mcu_pending_end_time - est
                    if remaining <= 0.0:
                        break
                    self.reactor.pause(
                        self.reactor.monotonic() + remaining + 0.010
                    )
        self._ground_pending_end_time_after_bridge_drain()

    def get_last_move_time(self):
        # Return the MCU print-time of the end of the last queued move (callers
        # like homing.py:home_wait need it for the backstop deadline). Project
        # the pending bridge duration onto the MCU clock; if nothing is pending
        # past the current clock, the floor wins.
        est = 0.0
        if self.mcu is not None:
            est = self.mcu.estimated_print_time(self.reactor.monotonic())
        floor = est + BUFFER_TIME_START
        if self._mcu_pending_end_time > est:
            return max(self._mcu_pending_end_time, floor)
        return floor

    def note_homing_end(self):
        self._ground_pending_end_time_after_bridge_drain()

    def _ground_pending_end_time_after_bridge_drain(self):
        # wait_moves() means dispatched, not executed; ground subsequent
        # cross-MCU scheduling in the live MCU clock rather than a stale,
        # possibly-seconds-ahead projected motion end.
        if self.mcu is None:
            return
        est = self.mcu.estimated_print_time(self.reactor.monotonic())
        command_time = est + BUFFER_TIME_START
        if self._mcu_pending_end_time > command_time:
            self._mcu_pending_end_time = command_time

    def _bump_pending_end_time(self, duration_added):
        if self.mcu is None or duration_added <= 0.0:
            return
        est = self.mcu.estimated_print_time(self.reactor.monotonic())
        # Re-anchor at "now" if the prior pending-end is already in the past.
        base = max(self._mcu_pending_end_time, est)
        self._mcu_pending_end_time = base + duration_added

    def note_mcu_movequeue_activity(self, mq_time, set_step_gen_time=False):
        # No-op: upstream's body would re-arm the silenced flush_timer.
        pass

    def set_accel(self, accel):
        if accel is not None and accel > 0.0:
            self.max_accel = accel
            self.bridge.update_limits(self.max_velocity, self.max_accel)

    def reset_accel(self):
        self.bridge.update_limits(self.max_velocity, self.max_accel)

    def cmd_SET_VELOCITY_LIMIT(self, gcmd):
        super().cmd_SET_VELOCITY_LIMIT(gcmd)
        self.bridge.update_limits(self.max_velocity, self.max_accel)

    def cmd_RESET_VELOCITY_LIMIT(self, gcmd):
        super().cmd_RESET_VELOCITY_LIMIT(gcmd)
        self.bridge.update_limits(self.max_velocity, self.max_accel)

    def stats(self, eventtime):
        return False, "print_time=%.3f buffer_time=0.000 print_stall=%d" % (
            self.print_time,
            self.print_stall,
        )

    def _init_planner(self):
        # Topology is derived from config (axis->MCU from each stepper's
        # _bridge_handle, tag from the kinematics name); no MCU identity is
        # hardcoded.
        bridge_mcus = []
        for name, mcu in self.printer.lookup_objects(module="mcu"):
            handle = getattr(mcu, "_bridge_handle", None)
            if handle is None:
                continue
            bridge_mcus.append((name, mcu, handle))
        if not bridge_mcus:
            logging.warning(
                "MotionToolhead: no MCU bridge handles available; "
                "skipping init_planner"
            )
            return

        # axis -> bridge handle of the MCU driving that axis's primary stepper
        # (AWD partners share their primary's, so only primaries contribute).
        axis_to_handle = {}
        fm = self.printer.lookup_object("force_move", None)
        if fm is not None:
            for sname, s in fm.steppers.items():
                info = _name_motor_slot(sname)
                if info is None:
                    continue
                slot_idx, is_primary = info
                if not is_primary:
                    continue
                s_handle = getattr(s.get_mcu(), "_bridge_handle", None)
                if s_handle is None:
                    continue
                axis_to_handle[slot_idx] = s_handle

        # Servo axes aren't steppers (absent from force_move.steppers); map each
        # servo rail's axis to its node handle so the node joins the topology.
        servo_axis_index = {"x": _AXIS_X, "y": _AXIS_Y, "z": 2}
        for rail in getattr(self.kin, "rails", ()):
            if not isinstance(rail, servo_axis.ServoRail):
                continue
            node = self.printer.lookup_object(
                "ethercat_node " + rail.get_node_name(), None
            )
            if node is None:
                continue
            handle = node.get_bridge_handle()
            if handle is None:
                continue
            axis_idx = servo_axis_index.get(rail.axis)
            if axis_idx is not None:
                axis_to_handle[axis_idx] = handle

        topology = _derive_mcu_topology(axis_to_handle, self.kinematics_name)
        if not topology:
            logging.warning(
                "MotionToolhead: no axis->MCU assignment resolved; "
                "skipping init_planner"
            )
            return

        # Pull initial shaper params from [input_shaper] config, if present.
        shaper_type_x = "smooth_zv"
        shaper_freq_x = 0.0
        shaper_type_y = "smooth_zv"
        shaper_freq_y = 0.0
        is_obj = self.printer.lookup_object("input_shaper", None)
        if is_obj is not None:
            try:
                shapers = is_obj.get_shapers()
                for s in shapers or ():
                    if s.axis == "x":
                        shaper_type_x = s.params.shaper_type
                        shaper_freq_x = s.params.shaper_freq
                    elif s.axis == "y":
                        shaper_type_y = s.params.shaper_type
                        shaper_freq_y = s.params.shaper_freq
            except Exception:
                logging.exception(
                    "MotionToolhead: failed to read input_shaper params"
                )

        try:
            self.bridge.init_planner(
                self.max_velocity,
                self.max_accel,
                self.max_z_velocity,
                self.max_z_accel,
                self.square_corner_velocity,
                shaper_type_x,
                shaper_freq_x,
                shaper_type_y,
                shaper_freq_y,
                topology,
            )
            self._configure_axes_per_mcu(bridge_mcus)

        except Exception:
            logging.exception("MotionToolhead: init_planner failed")
            raise

    def _configure_axes_per_mcu(self, bridge_mcus):
        """Configure each MCU's axes via the per-axis kalico_configure_axis
        command. Motor slots map [0,1,2,3] = [x,y,z,e] (corexy x/y are A/B).
        Steppers not on a given MCU are omitted from its bindings.
        """
        kin = (self.kinematics_name or "").lower()
        if kin == "corexy":
            kin_tag = 0
            slot_names = ["stepper_x", "stepper_y", "stepper_z", "extruder"]
            awd_default = 0b0011
        elif kin == "cartesian":
            kin_tag = 1
            slot_names = ["stepper_x", "stepper_y", "stepper_z", "extruder"]
            awd_default = 0b0000
        else:
            logging.info(
                "MotionToolhead: kinematics=%r — skipping configure_axes",
                kin,
            )
            return

        # Primary first, then AWD partners in name order; the runtime drives all
        # steppers in a slot in lockstep.
        slot_steppers = [[], [], [], []]
        slot_primary = [None, None, None, None]
        fm = self.printer.lookup_object("force_move", None)
        if fm is not None:
            for name, s in fm.steppers.items():
                m = _name_motor_slot(name)
                if m is None:
                    continue
                slot_idx, is_primary = m
                if is_primary:
                    slot_primary[slot_idx] = (name, s)
                else:
                    slot_steppers[slot_idx].append((name, s))
        for slot_idx in range(4):
            slot_steppers[slot_idx].sort(key=lambda ns: ns[0])
            if slot_primary[slot_idx] is not None:
                slot_steppers[slot_idx].insert(0, slot_primary[slot_idx])

        PHASE_STEPPING_BIT = 0x1  # bit 0 of the IdentifyResponse caps bitmap

        for name, mcu_obj, mcu_handle in bridge_mcus:
            present_mask = 0
            invert_mask = 0
            steps_per_mm = [0.0, 0.0, 0.0, 0.0]
            # 0=Modulated (phase stepping), 1=StepTime; overridden per slot below.
            step_modes = [1, 1, 1, 1]
            # (motor_idx, name, oid, invert_dir) per bound stepper.
            bind_list = []
            for i in range(4):
                on_this_mcu = []
                for sname, s in slot_steppers[i]:
                    if len(bridge_mcus) > 1:
                        try:
                            s_mcu = s.get_mcu()
                        except AttributeError:
                            s_mcu = None
                        if s_mcu is not None and s_mcu is not mcu_obj:
                            continue
                    on_this_mcu.append((sname, s))
                if not on_this_mcu:
                    continue
                primary_name, primary = on_this_mcu[0]
                step_dist = primary.get_step_dist()
                if step_dist <= 0.0:
                    continue
                steps_per_mm[i] = 1.0 / step_dist
                present_mask |= 1 << i
                if getattr(primary, "_invert_dir", False):
                    invert_mask |= 1 << i
                if getattr(primary, "phase_stepping", False):
                    step_modes[i] = 0  # Modulated
                for sname, s in on_this_mcu:
                    inv = 1 if getattr(s, "_invert_dir", False) else 0
                    bind_list.append((i, sname, s.get_oid(), inv))
            # One (bus_id, cs_pin_id, slot_idx) per physical phase-stepped motor;
            # AWD partners share slot_idx but get their own entry so each
            # TMC5160's XDIRECT register is written. Empty = no phase stepping.
            phase_configs = []
            any_phase_stepping = False
            for i, slot in enumerate(slot_steppers):
                # step_modes[i] != 0 is the load-bearing guard (set to 0 only in
                # the on_this_mcu branch, so cross-MCU slots stay != 0).
                if step_modes[i] != 0 or not slot:
                    continue
                for stepper_name, stepper_obj in slot:
                    tmc_name = "tmc5160 " + stepper_name
                    try:
                        tmc = self.printer.lookup_object(tmc_name)
                    except Exception:
                        raise self.printer.config_error(
                            "phase_stepping=True on stepper '%s' requires "
                            "a [tmc5160 %s] section (current driver type "
                            "or absence of TMC5160 section is "
                            "incompatible with phase stepping)"
                            % (stepper_name, stepper_name)
                        )
                    if not hasattr(tmc, "get_phase_config"):
                        raise self.printer.config_error(
                            "phase_stepping=True on stepper '%s' requires "
                            "a TMC5160 driver; found driver type with no "
                            "phase-stepping support" % stepper_name
                        )
                    # If the stepper's phase_stepping flag and TMC5160's
                    # _phase_stepping ever diverge, this raises tmc5160's
                    # less-specific config_error instead of the message above.
                    bus_id, cs_pin_id = tmc.get_phase_config()
                    phase_configs.append((bus_id, cs_pin_id, i))
                    any_phase_stepping = True
            # Soft cap mirrors firmware-side MAX_STEPPER_OIDS=16 (see
            # rust/runtime/src/state.rs). Reject early with an
            # operator-friendly message rather than letting the bridge
            # call return PyRuntimeError.
            if len(phase_configs) > 16:
                raise self.printer.config_error(
                    "phase_stepping enabled on %d motors but the firmware "
                    "supports up to 16 phase-stepped motors total per MCU."
                    % len(phase_configs)
                )
            awd_mask = awd_default & present_mask
            if present_mask == 0:
                logging.info(
                    "MotionToolhead: no steppers matched MCU %s; "
                    "skipping configure_axes",
                    name,
                )
                continue
            # Capability check: any stepper requesting phase_stepping=True on
            # an MCU that does not advertise PHASE_STEPPING_CAPABLE is a
            # config error. The check happens here (connect time) because MCU
            # capabilities are only known after attach_serial / identify, which
            # runs after config-parse time.
            mcu_caps = self.bridge.get_mcu_capabilities(mcu_handle)
            for i in range(4):
                if step_modes[i] == 0 and not (mcu_caps & PHASE_STEPPING_BIT):
                    slot_name = (
                        slot_steppers[i][0][0]
                        if slot_steppers[i]
                        else "motor_%d" % i
                    )
                    raise self.printer.config_error(
                        "Stepper '%s' requests phase_stepping: 1, but MCU "
                        "'%s' did not advertise the PHASE_STEPPING capability "
                        "in its IdentifyResponse (caps=0x%x). This usually "
                        "means kalico-native identify timed out, which in "
                        "turn usually means the MCU's firmware was built "
                        "without CONFIG_KALICO_RUNTIME=y. Rebuild that MCU "
                        "with CONFIG_KALICO_RUNTIME=y (and the small or "
                        "large runtime profile for the chip family) and "
                        "reflash." % (slot_name, name, mcu_caps)
                    )
            if any_phase_stepping:
                # Two-stage registration: shared SPI cfg once per bus_id, then a
                # CS GPIO per motor (multiple drivers on one bus each need their
                # own CS). motor_idx MUST match the phase_configs list position,
                # since the configure_axes blob is parsed in the same order.
                seen_buses = set()
                for bus_id, _cs_pin_id, _slot_idx in phase_configs:
                    if bus_id == 0xFF:
                        continue
                    if bus_id in seen_buses:
                        continue
                    seen_buses.add(bus_id)
                    logging.info(
                        "register_phase_bus mcu=%s bus_id=%d", name, bus_id
                    )
                    self.bridge.register_phase_bus(
                        mcu_handle,
                        bus_id,
                        rate=2_000_000,
                    )
                for motor_idx, (bus_id, cs_pin_id, _slot_idx) in enumerate(
                    phase_configs,
                ):
                    if bus_id == 0xFF:
                        continue
                    logging.info(
                        "register_phase_motor mcu=%s motor=%d bus=%d cs=%d",
                        name,
                        motor_idx,
                        bus_id,
                        cs_pin_id,
                    )
                    self.bridge.register_phase_motor(
                        mcu_handle,
                        motor_idx,
                        bus_id,
                        cs_pin_id,
                    )
            # One kalico_configure_axis per axis carries a 4-byte-per-stepper
            # blob { stepper_oid: u8, dir_invert: u8, tmc_cs_oid: u8, flags: u8 }.
            # dir_invert (from `dir_pin: !PIN`) is forwarded because bridge mode
            # bypasses stepcompress's step-count sign flip; without it motors run
            # reversed. MCUs lacking the command skip silently.
            try:
                configure_axis_cmd = mcu_obj.lookup_command(
                    "kalico_configure_axis axis_idx=%c mode=%c"
                    " microstep_distance=%u extrusion_per_xy_mm=%u"
                    " stepper_count=%c ring_depth=%hu steppers=%*s"
                )
            except Exception:
                logging.info(
                    "MotionToolhead: mcu=%s lacks kalico_configure_axis "
                    "(no new stepping redesign command); skipping runtime "
                    "binding",
                    name,
                )
                continue

            # Reset before (re)configuring: the engine's ring allocator never
            # frees and configure_axis re-runs every klippy:connect, so a
            # reconnect without an MCU reboot would overflow the pool. Idempotent
            # on a fresh MCU; same queue, so it runs before configure_axis.
            try:
                reset_cmd = mcu_obj.lookup_command("kalico_runtime_reset")
            except Exception:
                reset_cmd = None
            if reset_cmd is not None:
                reset_cmd.send([])
                logging.info(
                    "MotionToolhead: sent kalico_runtime_reset to mcu=%s",
                    name,
                )

            axis_bindings = defaultdict(list)  # axis_idx -> [(oid, invert)]
            for motor_idx, sname, oid, inv in bind_list:
                axis_bindings[motor_idx].append((oid, inv))

            MODE_PULSE = 0
            TMC_CS_OID_NONE = 0xFF
            FLAGS_DEFAULT = 0

            for axis_idx, bindings in axis_bindings.items():
                spm = (
                    steps_per_mm[axis_idx]
                    if axis_idx < len(steps_per_mm)
                    else 0.0
                )
                if spm <= 0:  # misconfigured axis
                    continue
                microstep_distance = 1.0 / spm
                # f32 packed as u32 bits for the wire.
                microstep_bits = struct.unpack(
                    "<I", struct.pack("<f", microstep_distance)
                )[0]
                # extrusion_per_xy_mm unused by firmware; sent 0 for ABI compat.
                extrusion_bits = 0
                blob = bytearray()
                for oid, inv in bindings:
                    blob.append(oid)
                    blob.append(inv & 0x01)
                    tmc_oid = TMC_CS_OID_NONE
                    if step_modes[axis_idx] == 0:
                        for sname, s in slot_steppers[axis_idx]:
                            if s.get_oid() == oid:
                                tmc_name = "tmc5160 " + sname
                                try:
                                    tmc = self.printer.lookup_object(tmc_name)
                                    tmc_oid = tmc.get_spi_oid()
                                except Exception:
                                    pass
                                break
                    blob.append(tmc_oid)
                    blob.append(FLAGS_DEFAULT)
                ring_depth = self.bridge.ring_depth_for_axis(
                    mcu_handle, axis_idx
                )
                configure_axis_cmd.send(
                    [
                        axis_idx,
                        MODE_PULSE,
                        microstep_bits,
                        extrusion_bits,
                        len(bindings),
                        ring_depth,
                        bytes(blob),
                    ]
                )
            logging.info(
                "MotionToolhead: configure_axes mcu=%s kin=%d "
                "present=0x%x awd=0x%x invert=0x%x steps_per_mm=%s "
                "step_modes=%s mcu_caps=0x%x runtime_bindings=%s "
                "phase_configs=%s any_phase_stepping=%s "
                "phase_motor_count=%d",
                name,
                kin_tag,
                present_mask,
                awd_mask,
                invert_mask,
                steps_per_mm,
                step_modes,
                mcu_caps,
                [(m, n, o, i) for (m, n, o, i) in bind_list],
                phase_configs,
                any_phase_stepping,
                len(phase_configs),
            )
            # phase_stepping_enable_spi is sent later from
            # TMC5160._xdirect_preload, after TMC register init.

    def cmd_KALICO_DIAG_DUMP(self, gcmd):
        # Ask every kalico-capable MCU to emit its live diag snapshot; MCUs
        # lacking the command are skipped.
        sent = []
        for name, mcu_obj in self.printer.lookup_objects(module="mcu"):
            try:
                cmd = mcu_obj.lookup_command("kalico_diag_dump")
            except Exception:
                continue
            cmd.send([])
            sent.append(name)
        if sent:
            gcmd.respond_info(
                "KALICO_DIAG_DUMP: requested live diag from %s "
                "(see printer_data/logs/events/<mcu>.jsonl)"
                % (", ".join(sent),)
            )
        else:
            gcmd.respond_info(
                "KALICO_DIAG_DUMP: no MCU exposes kalico_diag_dump"
            )

    def cmd_KALICO_SIM_STEP_COUNT(self, gcmd):
        oid = gcmd.get_int("OID", 0, minval=0)
        if self.mcu is None:
            raise gcmd.error("mcu not available")
        handle = getattr(self.mcu, "_bridge_handle", None)
        if handle is None:
            raise gcmd.error("bridge handle not set")
        try:
            resp = self.bridge.bridge_call(
                handle,
                "runtime_sim_stepper_count_query oid=%d" % oid,
                "runtime_sim_stepper_count_response",
                timeout_s=5.0,
            )
            count = resp.get("count", 0)
            gcmd.respond_info(
                "[bridge-async] KALICO_SIM_STEP_COUNT oid=%d count=%d"
                % (oid, count)
            )
        except Exception as e:
            raise gcmd.error("step count query failed: %s" % e)

    def cmd_KALICO_SIM_AXIS_STEPS(self, gcmd):
        oid = gcmd.get_int("OID", 0, minval=0, maxval=3)
        if self.mcu is None:
            raise gcmd.error("mcu not available")
        handle = getattr(self.mcu, "_bridge_handle", None)
        if handle is None:
            raise gcmd.error("bridge handle not set")
        try:
            resp = self.bridge.bridge_call(
                handle,
                "runtime_sim_axis_steps_query oid=%d" % oid,
                "runtime_sim_axis_steps_response",
                timeout_s=5.0,
            )
            milli = resp.get("milli_spm", 0)
            gcmd.respond_info(
                "[bridge-async] KALICO_SIM_AXIS_STEPS oid=%d "
                "steps_per_mm=%.3f" % (oid, milli / 1000.0)
            )
        except Exception as e:
            raise gcmd.error("axis steps query failed: %s" % e)

    def cmd_KALICO_SIM_AXIS_ACCUM(self, gcmd):
        oid = gcmd.get_int("OID", 0, minval=0, maxval=3)
        if self.mcu is None:
            raise gcmd.error("mcu not available")
        handle = getattr(self.mcu, "_bridge_handle", None)
        if handle is None:
            raise gcmd.error("bridge handle not set")
        try:
            resp = self.bridge.bridge_call(
                handle,
                "runtime_sim_axis_accum_query oid=%d" % oid,
                "runtime_sim_axis_accum_response",
                timeout_s=5.0,
            )
            milli = resp.get("milli", 0)
            gcmd.respond_info(
                "[bridge-async] KALICO_SIM_AXIS_ACCUM oid=%d accum=%.3f"
                % (oid, milli / 1000.0)
            )
        except Exception as e:
            raise gcmd.error("axis accum query failed: %s" % e)

    def cmd_KALICO_SIM_ENDSTOP_SET_PIN(self, gcmd):
        gpio = gcmd.get_int("GPIO", minval=0, maxval=0xFFFF)
        level = gcmd.get_int("LEVEL", minval=0, maxval=1)
        client = _open_sim_control()
        if client is not None:
            MAX_GPIO_LINES = 288
            chip_id = gpio // MAX_GPIO_LINES
            line = gpio % MAX_GPIO_LINES
            try:
                with client:
                    client.set_gpio_input(
                        chip=chip_id,
                        line=line,
                        value=level,
                    )
                gcmd.respond_info(
                    "KALICO_SIM_ENDSTOP_SET_PIN gpio=%d level=%d -> ok (shim)"
                    % (gpio, level)
                )
                return
            except Exception as e:
                raise gcmd.error("set_gpio_input failed: %s" % e)
        # Fallback (Renode sim): drive the pin via the firmware command.
        if self.mcu is None:
            raise gcmd.error("no MCU available for sim endstop set_pin")
        handle = self.mcu._bridge_handle
        try:
            self.bridge.bridge_send(
                handle,
                "runtime_sim_endstop_set_pin gpio=%d level=%d" % (gpio, level),
            )
            gcmd.respond_info(
                "KALICO_SIM_ENDSTOP_SET_PIN gpio=%d level=%d -> ok (fw)"
                % (gpio, level)
            )
        except Exception as e:
            raise gcmd.error("runtime_sim_endstop_set_pin failed: %s" % e)


def add_printer_objects(config):
    config.get_printer().add_object("toolhead", MotionToolhead(config))
    extruder.add_printer_objects(config)
