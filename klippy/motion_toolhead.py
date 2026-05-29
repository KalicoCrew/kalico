# MotionToolhead — skeleton toolhead implementing the public API
#
# This file is part of the Kalico motion-bridge integration (Stage D).
# Move-issuing calls raise NotImplementedError; status/query methods work.
import logging
import os
import struct
from collections import defaultdict

from . import chelper
from . import motion_kinematics
from . import stepper
from .kinematics import extruder
from .toolhead import Move, ToolHead, BUFFER_TIME_START


# Per-motor-slot prefix table. Slot order matches
# `motion_kinematics.motor_deltas` and `_configure_axes_per_mcu`'s
# `slot_names`. Steppers whose name doesn't start with one of these
# prefixes have no motor slot (e.g. the corexy "passthrough Z" rails
# the runtime ignores) — their enable callbacks are skipped.
_MOTOR_SLOT_PREFIXES = (
    (0, "stepper_x"),
    (1, "stepper_y"),
    (2, "stepper_z"),
    (3, "extruder"),
)


def _name_motor_slot(name):
    """Map a stepper section name to its motor slot.

    Returns `(slot, is_primary)`:
      - `slot` is the index into `_MOTOR_SLOT_PREFIXES` / `motor_deltas`.
      - `is_primary` is True for the unsuffixed primary (`stepper_x`),
        False for AWD partners (`stepper_x1`, `stepper_z2`, ...).
    Returns None if the name doesn't match any motor slot.
    """
    for slot_idx, prefix in _MOTOR_SLOT_PREFIXES:
        if not name.startswith(prefix):
            continue
        suffix = name[len(prefix):]
        if suffix == "":
            return (slot_idx, True)
        if suffix.isdigit():
            return (slot_idx, False)
    return None


def _stepper_motor_slot(stepper_obj):
    """Return the motor slot (0..3) for an MCU_stepper, or None if it
    isn't bound to one (e.g. corexy passthrough Z rails).
    """
    info = _name_motor_slot(stepper_obj.get_name())
    return None if info is None else info[0]


def _open_sim_control():
    """Open the shim's control socket. Returns SimControlClient or None
    if shim is not in use (real hardware or vanilla MACH_LINUX).

    The launcher passes KALICO_SIM_SOCK_DIR per MCU; we connect to the
    H7's control socket since KALICO_SIM_ENDSTOP_SET_PIN is for endstops
    which are wired to the H7 in the Trident config. If the F4 ever
    needs a similar surface, add a parallel cmd_KALICO_SIM_F4_*.
    """
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
    """Kinematics shim for the motion-bridge planner.

    Mirrors mainline `klippy/kinematics/{cartesian,corexy}.py`'s host-side
    homed/range enforcement: each axis carries a `(low, high)` limit that
    starts inverted (1.0, -1.0) — meaning "unhomed" — and is replaced with
    the rail's range once `set_position` is called with that axis in
    `homing_axes`. `check_move` rejects moves that step outside the
    per-axis limit, raising "Must home axis first" when the limit is still
    in its unhomed sentinel form.
    """

    def __init__(self, toolhead, config, trapq):
        self._toolhead = toolhead
        kin_name = config.get("kinematics")
        if kin_name not in ("cartesian", "corexy", "hybrid_corexy"):
            raise config.error("Unsupported bridge kinematics '%s'" % (kin_name,))
        self.kinematics = kin_name
        self.rails = []
        self._printer = config.get_printer()

        axes = "xy"
        if kin_name in ("cartesian", "hybrid_corexy"):
            axes = "xyz"
        for axis in axes:
            self._register_axis(config, axis, trapq, extras=("1",))
        if kin_name == "corexy" and len(self.rails) >= 2:
            x_endstop = self.rails[0].get_endstops()[0][0]
            y_endstop = self.rails[1].get_endstops()[0][0]
            for s in self.rails[1].get_steppers():
                x_endstop.add_stepper(s)
            for s in self.rails[0].get_steppers():
                y_endstop.add_stepper(s)
        # Z is kinematically independent in corexy (no A/B mixing), but the
        # bridge dispatches Z curves to the Z MCU normally. Register Z rails
        # so printer.cfg validation passes and homing/move-checking works.
        if kin_name == "corexy" and config.has_section("stepper_z"):
            self._register_axis(
                config, "z", trapq, extras=("1", "2", "3")
            )

        # Per-axis (low, high) bounds. `low > high` means "unhomed" and is
        # what triggers the "Must home axis first" path in `_check_endstops`.
        self.limits = [(1.0, -1.0)] * 3

        # Mirror mainline klippy/toolhead.py + cartesian/corexy kinematics:
        # when steppers are de-energized (M84 / shutdown) the homed state
        # must clear so klippy correctly reports the axes as un-homed and
        # subsequent G1s require re-homing.
        self._printer.register_event_handler(
            "stepper_enable:motor_off",
            self._handle_motor_off,
        )

    def _handle_motor_off(self, print_time):
        self.clear_homing_state((0, 1, 2))

    def _register_axis(self, config, axis, trapq, extras=()):
        rail = stepper.PrinterRail(config.getsection("stepper_" + axis))
        for suffix in extras:
            extra_name = "stepper_" + axis + suffix
            if config.has_section(extra_name):
                rail.add_extra_stepper(config.getsection(extra_name))
        for mcu_stepper in rail.get_steppers():
            mcu_stepper.setup_itersolve(
                "cartesian_stepper_alloc", axis.encode()
            )
            # DIAG: do NOT connect steppers to trapq — bridge mode
            # generates steps via the Rust runtime, not the C trapq.
            # If Z still moves after this, the motion source is NOT trapq.
            # mcu_stepper.set_trapq(trapq)
            mcu_stepper.get_mcu()._bridge_drives_steppers = True
        self.rails.append(rail)

    def _axis_rails(self):
        # Rails are registered in axis order; the corexy "passthrough Z"
        # branch may append a Z rail after the X/Y pair. Locate by name
        # prefix so callers always get the right rail per axis.
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
                end_pos[i] < self.limits[i][0]
                or end_pos[i] > self.limits[i][1]
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
        # Move with Z — derate velocity and accel proportional to Z share.
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
                forcepos[axis] -= 1.5 * (
                    hi.position_endstop - position_min
                )
            else:
                forcepos[axis] += 1.5 * (
                    position_max - hi.position_endstop
                )
            homing_state.home_rails([rail], forcepos, homepos)

    def set_position(self, newpos, homing_axes=()):
        self._toolhead.bridge._software_trip_active = False
        self._toolhead.bridge.set_position(
            newpos[0], newpos[1], newpos[2]
        )
        axis_rails = self._axis_rails()
        for axis_idx, rail in axis_rails.items():
            if self.kinematics == "corexy" and axis_idx < 2:
                motor_pos = (newpos[0] + newpos[1]) if axis_idx == 0 \
                    else (newpos[0] - newpos[1])
            else:
                motor_pos = newpos[axis_idx] if axis_idx < len(newpos) \
                    else 0.0
            for s in rail.get_steppers():
                step_count = int(motor_pos / s.get_step_dist() + 0.5)
                s._set_mcu_position(step_count)
        for axis in homing_axes:
            rail = axis_rails.get(axis)
            if rail is not None:
                self.limits[axis] = rail.get_range()

    def note_z_not_homed(self):
        # Mirror Kalico's CartKinematics.note_z_not_homed (klippy/kinematics/
        # cartesian.py:93). `[beacon]`'s compat_kin_note_z_not_homed prefers
        # this method when present and falls back to clear_homing_state("z")
        # only when it isn't — by exposing it we keep clear_homing_state
        # on Kalico's int-iterable contract.
        self.clear_homing_state([2])

    def clear_homing_state(self, axes):
        # Kalico-mainline contract: `axes` is an iterable of axis indices
        # (e.g. `(0, 1, 2)` from _motor_off, `[2]` from note_z_not_homed /
        # safe_z_home / dockable_probe, `clear_axes` ints from force_move).
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
            a for i, a in enumerate("xyz") if self.limits[i][0] <= self.limits[i][1]
        )
        return {
            "homed_axes": homed,
            "axis_minimum": gcode_mod.Coord(x_min, y_min, z_min, 0.0),
            "axis_maximum": gcode_mod.Coord(x_max, y_max, z_max, 0.0),
        }


class MotionToolhead(ToolHead):
    """Bridge-aware ToolHead subclass.

    Inherits the upstream surface unchanged; overrides only the methods
    where the Rust motion bridge owns the behavior (move issuance,
    timeline, velocity-limit propagation, sim diagnostics).

    See docs/superpowers/specs/2026-05-07-motion-toolhead-extends-upstream-design.md.
    """

    def __init__(self, config):
        # Pre-super: attributes that BridgeKinematics or registered handlers
        # may reference during super().__init__.
        printer = config.get_printer()
        self.bridge = printer.lookup_object("motion_bridge")
        self.active_homing_arms = set()
        self.kinematics_name = config.get("kinematics", "")
        self.bridge._kinematics_name = self.kinematics_name
        # Projected MCU print-time of the END of the last queued bridge
        # move. See get_last_move_time / _bump_pending_end_time.
        self._mcu_pending_end_time = 0.0

        # Run upstream init: trapq alloc, gcode commands (G4/M400/
        # SET_VELOCITY_LIMIT/RESET_VELOCITY_LIMIT/M204), helper modules
        # (gcode_move/homing/idle_timeout/statistics/manual_probe/
        # tuning_tower/garbage_collection), lookahead, flush_timer,
        # _calc_junction_deviation, _handle_shutdown registration,
        # extruder = DummyExtruder, AND _load_kinematics → BridgeKinematics.
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

        # Planner initialization runs once all MCUs have connected.
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

    # ------------------------------------------------------------------
    # Kinematics override
    # ------------------------------------------------------------------

    def _load_kinematics(self, config):
        return BridgeKinematics(self, config, self.trapq)

    # ------------------------------------------------------------------
    # Move issuance — bridge owns these
    # ------------------------------------------------------------------

    def move(self, newpos, speed):
        # Mirror upstream `ToolHead.move`'s pre-issue validation: build a
        # Move so kin.check_move can reject unhomed / out-of-range moves
        # ("Must home axis first"), and so extruder.check_move can range-
        # check E. The bridge planner replaces the lookahead, but the
        # validation that lives on Move/kin must still run.
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
            list(newpos), speed, dx, dy, dz, de, feedrate,
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
        return fired

    def drip_move(self, newpos, speed, drip_completion):
        logging.info(
            "[bridge-trace] drip_move entered: newpos=%s speed=%s "
            "drip_test=%s active_homing_arms=%s",
            list(newpos), speed,
            (drip_completion.test()
             if drip_completion is not None else None),
            sorted(self.active_homing_arms),
        )
        if drip_completion is not None and drip_completion.test():
            return
        arm_ids = list(self.active_homing_arms)
        if arm_ids:
            # Bridge-native GPIO/sensorless path (existing)
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
            # External probe software-trip path
            self._drip_move_software_trip(newpos, speed, drip_completion)
        else:
            # No endstop armed — regular move fallback
            self.move(newpos, speed)

    def _prepare_probe_interceptor(self, endstops):
        """Pre-register the Rust frame interceptor for the external
        probe's trsync.  Called from homing.py BEFORE home_start() so
        the interceptor is in place when the probe triggers.  Stores
        the handle ID on self for _drip_move_software_trip to consume.
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
                getattr(mcu_endstop, '_shared', None), '_trsync', None
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
                handle_id, es_mcu._bridge_handle, trsync.get_oid(),
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
            self.commanded_pos[0], self.commanded_pos[1],
            self.commanded_pos[2],
            pos3[0], pos3[1], pos3[2],
            dx, dy, dz,
        )

        # Select moving steppers via kinematic motor-delta mapping
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

        # Resolve MCU handle from first stepper's MCU
        stepper_mcus = set(s.get_mcu() for s in moving_steppers)
        if len(stepper_mcus) > 1:
            raise self.printer.command_error(
                "External probe homing across multiple bridge MCUs "
                "is not supported"
            )
        stepper_mcu = next(iter(stepper_mcus))
        mcu_handle = stepper_mcu._bridge_handle
        queue = self.bridge.alloc_command_queue(mcu_handle)

        # Create virtual arm
        arm_id = getattr(self, '_probe_homing_arm_id', None)
        if arm_id is None:
            arm_id = _mb._alloc_arm_id()
        stepper_oids = [s.get_oid() for s in moving_steppers]
        source = (_mb.SOURCE_KIND_SOFTWARE, 0, False, 0, 1, 0, 0)

        # The motor-enable callbacks do inline TMC phase-offset reads
        # over UART (~100ms per stepper).  With 3 Z steppers each
        # doing ~20 UART transactions at ~5ms, total wall-clock cost
        # is ~300ms.  Pad generously so the last stepper's
        # queue_digital_out clock is still in the MCU's future.
        ENABLE_HEADROOM = 2.000
        lmt = self.get_last_move_time()
        est_now = 0.0
        if self.mcu is not None:
            est_now = self.mcu.estimated_print_time(
                self.reactor.monotonic())
            needed = est_now + ENABLE_HEADROOM
            if lmt < needed:
                self.dwell(needed - lmt)
                lmt = self.get_last_move_time()

        logging.info(
            "[probe-homing] pre-enable: "
            "lmt=%.6f est_now=%.6f pending=%.6f stepper_mcu=%s",
            lmt, est_now, self._mcu_pending_end_time,
            stepper_mcu.get_name(),
        )

        # Energize motors (TMC UART traffic runs here)
        self._fire_active_callbacks(
            dx, dy, dz, 0.0, lmt
        )

        # Recompute arm_clock AFTER callbacks — the UART traffic
        # consumed wall-clock time, so the pre-callback lmt may now
        # be in the MCU's past.
        if self.mcu is not None:
            est_now = self.mcu.estimated_print_time(
                self.reactor.monotonic())
        arm_clock = int(stepper_mcu.print_time_to_clock(
            max(lmt, est_now + BUFFER_TIME_START)
        ))
        logging.info(
            "[probe-homing] post-enable: "
            "est_now=%.6f arm_clock=%d",
            est_now, arm_clock,
        )

        # Arm + submit
        self.active_homing_arms.add(arm_id)
        self.bridge.register_homing_dispatch(arm_id, None)
        self.bridge._software_trip_active = True

        bridge_lmt_before = self.bridge.get_last_move_time()
        try:
            self.bridge.endstop_arm(
                mcu_handle, queue, arm_id, arm_clock,
                [source], stepper_oids,
            )
            self.bridge._homing_print_time_base = bridge_lmt_before

            # Use the pre-registered interceptor handle from
            # _prepare_probe_interceptor (called before home_start).
            handle_id = getattr(self, '_probe_homing_handle_id', None)
            if handle_id is None:
                raise self.printer.command_error(
                    "No probe homing interceptor registered "
                    "(call _prepare_probe_interceptor first)"
                )
            # Use the arm_id from prepare (matches what the
            # interceptor will send in software_trip).
            arm_id = self._probe_homing_arm_id

            logging.info(
                "[probe-homing] calling run_probe_homing: "
                "handle_id=%d arm_id=%d speed=%.1f",
                handle_id, arm_id, speed,
            )

            result = self.bridge.run_probe_homing(
                handle_id, pos3, speed, stepper_oids,
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
        self.flush_step_generation()

    def _bridge_mcus(self):
        if not hasattr(self, '_cached_bridge_mcus'):
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
                    est = mcu.estimated_print_time(
                        self.reactor.monotonic())
                    remaining = self._mcu_pending_end_time - est
                    if remaining <= 0.0:
                        break
                    self.reactor.pause(
                        self.reactor.monotonic() + remaining + 0.010
                    )
        self._ground_pending_end_time_after_bridge_drain()

    def get_last_move_time(self):
        # Two clocks live here:
        #   - mcu.estimated_print_time(now)   — MCU print-clock (large)
        #   - bridge.get_last_move_time()     — planner-local seconds since
        #                                       planner thread start (small)
        # We need to return MCU print-time of the END of the last queued
        # move, so callers like homing.py:home_wait can compute the
        # backstop deadline correctly.
        #
        # We track the bridge's last_move_time deltas across submits and
        # project the pending duration onto the MCU clock. If no bridge
        # work is pending past the MCU's current clock, the floor wins —
        # this preserves the legacy-MCU-command-scheduling guarantee.
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
        """Clamp stale bridge projections after a full dispatch drain.

        `bridge.wait_moves()` means the host bridge has dispatched all queued
        motion, not that every MCU has finished executing it. For subsequent
        cross-MCU command scheduling, use a print-time grounded in the live
        MCU clock plus the normal Klipper scheduling lookahead instead of a
        stale projected motion end that may be seconds ahead.
        """
        if self.mcu is None:
            return
        est = self.mcu.estimated_print_time(self.reactor.monotonic())
        command_time = est + BUFFER_TIME_START
        if self._mcu_pending_end_time > command_time:
            self._mcu_pending_end_time = command_time

    def _bump_pending_end_time(self, duration_added):
        """Extend the projected MCU end-time of the last queued move.

        Called after each bridge submit (move / drip_move / dwell) to
        keep get_last_move_time() returning a sensible MCU print-time.
        Anchors to current MCU clock when bridge has no other work
        pending.
        """
        if self.mcu is None or duration_added <= 0.0:
            return
        est = self.mcu.estimated_print_time(self.reactor.monotonic())
        # If the prior pending-end is in the past, the MCU has caught up;
        # re-anchor at "now" before adding the new duration.
        base = max(self._mcu_pending_end_time, est)
        self._mcu_pending_end_time = base + duration_added

    def note_mcu_movequeue_activity(self, mq_time, set_step_gen_time=False):
        # Bridge has its own queue; upstream's body would re-arm the
        # silenced flush_timer.
        pass

    # ------------------------------------------------------------------
    # Velocity-limit propagation — bridge mirrors host-side updates
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Stats — bridge-aware silence (see spec §"stats")
    # ------------------------------------------------------------------

    def stats(self, eventtime):
        return False, "print_time=%.3f buffer_time=0.000 print_stall=%d" % (
            self.print_time, self.print_stall,
        )

    # ------------------------------------------------------------------
    # Bridge-only: planner init, ConfigureAxes, credit-freed wiring
    # ------------------------------------------------------------------

    def _init_planner(self):
        # Locate the two MVP MCUs by name. First-print topology:
        #   "mcu" (Octopus) drives X+Y; "mcu z" drives Z.
        # If only one MCU is configured, reuse its handle for Z.
        octopus = None
        f446 = None
        bridge_mcus = []
        for name, mcu in self.printer.lookup_objects(module="mcu"):
            handle = getattr(mcu, "_bridge_handle", None)
            if handle is None:
                continue
            bridge_mcus.append((name, mcu, handle))
            mcu_name = getattr(mcu, "_name", name)
            if octopus is None or mcu_name in ("mcu", "octopus"):
                if octopus is None:
                    octopus = handle
                elif f446 is None:
                    f446 = handle
            elif f446 is None:
                f446 = handle
        if octopus is None:
            logging.warning(
                "MotionToolhead: no MCU bridge handles available; "
                "skipping init_planner"
            )
            return
        if f446 is None:
            f446 = octopus

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
                octopus,
                f446,
            )
            self._configure_axes_per_mcu(bridge_mcus)

        except Exception:
            logging.exception("MotionToolhead: init_planner failed")
            raise

    def _configure_axes_per_mcu(self, bridge_mcus):
        """Configure each bridge-attached MCU's axes via the per-axis
        `kalico_configure_axis` text command. Maps klippy `MCU_stepper`
        objects to motor slots per kinematics:
          corexy:    [A=stepper_x, B=stepper_y, Z=stepper_z, E=extruder]
          cartesian: [X=stepper_x, Y=stepper_y, Z=stepper_z, E=extruder]
        Steppers not on a given MCU are omitted from that MCU's bindings.

        The legacy batch `ConfigureAxes` (binary 0x0030) blob is no longer
        sent — see the note at its former call site below.
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

        # Build per-slot ordered stepper list. Primary stepper (no numeric
        # suffix on the section name) goes first; AWD partners (e.g.
        # stepper_x1, stepper_z2) follow in name order. The runtime path
        # drives every stepper in a slot in lockstep, so a 4-motor Voron
        # 2.4 gantry needs both stepper_x AND stepper_x1 bound to motor 0,
        # both stepper_y AND stepper_y1 bound to motor 1, etc.
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

        # Capability bit: bit 0 of the IdentifyResponse capabilities bitmap.
        PHASE_STEPPING_BIT = 0x1

        for name, mcu_obj, mcu_handle in bridge_mcus:
            present_mask = 0
            invert_mask = 0
            steps_per_mm = [0.0, 0.0, 0.0, 0.0]
            # step_modes: 0=Modulated (phase stepping), 1=StepTime (classic).
            # Default all-StepTime; overridden per motor slot below.
            step_modes = [1, 1, 1, 1]
            # Per-MCU bind list: ordered (motor_idx, name, oid, invert_dir).
            bind_list = []
            for i in range(4):
                # Filter slot steppers to those that live on this MCU.
                on_this_mcu = []
                for (sname, s) in slot_steppers[i]:
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
                # Phase-stepping mode for this motor slot: driven by primary
                # stepper's phase_stepping flag.
                if getattr(primary, "phase_stepping", False):
                    step_modes[i] = 0  # Modulated
                for (sname, s) in on_this_mcu:
                    inv = 1 if getattr(s, "_invert_dir", False) else 0
                    bind_list.append((i, sname, s.get_oid(), inv))
            # 2026-05-19 phase-stepping bridge integration (variable-length
            # rework): build a flat per-motor list of
            # (bus_id, cs_pin_id, slot_idx) triples — one entry per physical
            # phase-stepped motor, regardless of slot multiplicity. AWD
            # partners (e.g. stepper_x1) share slot_idx with their primary
            # but get their own entry so their TMC5160 chip's XDIRECT
            # register receives writes. Empty list = no phase stepping.
            phase_configs = []
            any_phase_stepping = False
            for i, slot in enumerate(slot_steppers):
                # step_modes[i] != 0 is the load-bearing guard: it's only set
                # to 0 inside the on_this_mcu branch above, so for cross-MCU
                # slots it stays != 0 and we correctly skip them. `not slot`
                # is defensive belt-and-suspenders.
                if step_modes[i] != 0 or not slot:
                    continue
                for (stepper_name, stepper) in slot:
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
                    # Invariant: stepper config's `phase_stepping` flag and
                    # TMC5160's `_phase_stepping` are read from the same
                    # [stepper_*] section field. If they ever diverge (e.g.
                    # via a refactor), get_phase_config() will raise
                    # tmc5160's less-specific config_error instead of the
                    # operator-friendly message above.
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
                    "skipping configure_axes", name,
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
                    # Find the primary stepper name for this slot to give a
                    # user-friendly error.
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
                # GCONF.direct_mode is set by _xdirect_preload at stepper
                # enable time (not at connect time) to ensure CHOPCONF
                # toff>0 is on the chip first. No barrier needed here.
                # 2026-05-19 phase-stepping two-stage registration. SPI bus
                # cfg is shared across all TMC5160s on a bus and registered
                # once per unique bus_id. Per-motor CS GPIOs are registered
                # separately so multiple drivers on one bus (e.g. dual-Y
                # b_y + b_y2 on SPI3) each get their own addressable CS —
                # the previous single-call API silently aliased every motor
                # on a bus to one cached CS. See
                # docs/superpowers/specs/2026-05-19-phase-stepping-per-motor-cs-design.md.
                # `motor_idx` MUST match the list position in phase_configs,
                # because the configure_axes blob is parsed in the same
                # order and assigns the same motor_idx to each entry
                # (rust/kalico-c-api/src/runtime_ffi.rs:1535).
                seen_buses = set()
                for (bus_id, _cs_pin_id, _slot_idx) in phase_configs:
                    if bus_id == 0xFF:
                        continue
                    if bus_id in seen_buses:
                        continue
                    seen_buses.add(bus_id)
                    logging.info("register_phase_bus mcu=%s bus_id=%d",
                                 name, bus_id)
                    self.bridge.register_phase_bus(
                        mcu_handle, bus_id, rate=2_000_000,
                    )
                for motor_idx, (bus_id, cs_pin_id, _slot_idx) in enumerate(
                    phase_configs,
                ):
                    if bus_id == 0xFF:
                        continue
                    logging.info(
                        "register_phase_motor mcu=%s motor=%d bus=%d cs=%d",
                        name, motor_idx, bus_id, cs_pin_id)
                    self.bridge.register_phase_motor(
                        mcu_handle, motor_idx, bus_id, cs_pin_id,
                    )
            # NOTE: the legacy batch `ConfigureAxes` (binary 0x0030) blob send
            # was removed here. Per the simple-MCU-contract design (§3.3,
            # docs/superpowers/specs/2026-05-27-simple-mcu-contract-design.md)
            # the batch blob — kinematics tag, present/awd/invert masks, and a
            # fixed [f32;4] steps_per_mm array — is superseded by the per-axis
            # `ConfigureAxis` whose currently-implemented realization is the
            # `kalico_configure_axis` text command issued below (microstep_
            # distance = 1/steps_per_mm, per-stepper bindings, dir-invert,
            # mode). The MCU dispatcher (src/kalico_dispatch.c) never had a
            # 0x0030 handler, so the blob always timed out; nothing it carried
            # is lost — every field is either removed-by-design (kinematics is
            # host-pre-baked; no fixed-4 axis assumption) or already sent
            # per-axis below.
            # Step 7-D: bind every stepper attached to each runtime motor
            # index to the C-side runtime emit table. config_stepper was
            # already issued during MCU config phase; the runtime binding is
            # sent post-connect as a regular klipper-protocol command.
            # Each stepper's `invert_dir` (from `dir_pin: !PIN` in printer.cfg)
            # is forwarded so the runtime can XOR it into the dir_pin level —
            # without this, mainline-style step-count sign flipping (which
            # lives in stepcompress and is bypassed in bridge mode) leaves
            # polarity unhonored and motors run in reverse.
            #
            # Replaces the legacy config_runtime_stepper per-stepper emit
            # with the new per-axis kalico_configure_axis command
            # (stepping-redesign-finish Task 20). One command per axis
            # carries a %*s blob of per-stepper bindings, 4 bytes each:
            #   { stepper_oid: u8, dir_invert: u8, tmc_cs_oid: u8, flags: u8 }
            # tmc_cs_oid = 0xFF (TMC_CS_OID_NONE) for Pulse-only steppers;
            # Phase mode is rejected at the FFI per spec §5.2, so all axes
            # are Pulse mode in this cutover.
            #
            # MCUs without the new command (no stepping-redesign runtime in
            # their data dict) raise on the lookup; skip silently — those
            # steppers stay on legacy paths (and do nothing in bridge mode).
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

            # Clean-state reset before (re)configuring this MCU's axes. The
            # engine's ring bump allocator never frees, and configure_axis is
            # re-sent on every klippy:connect; without this reset a plain
            # RESTART / systemctl restart / crash-reconnect (which does NOT
            # reboot bridge MCUs) overflows the pool -> KALICO_ERR_RING_FULL.
            # Idempotent: a no-op on a freshly-booted MCU. Same command queue,
            # so it is processed before the configure_axis commands below.
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

            # Group bind_list by axis (motor_idx). bind_list entries are
            # (motor_idx, stepper_name, stepper_oid, invert_dir) tuples.
            axis_bindings = defaultdict(list)  # axis_idx -> [(oid, invert)]
            for (motor_idx, sname, oid, inv) in bind_list:
                axis_bindings[motor_idx].append((oid, inv))

            MODE_PULSE = 0
            TMC_CS_OID_NONE = 0xFF
            FLAGS_DEFAULT = 0

            for axis_idx, bindings in axis_bindings.items():
                # microstep_distance: mm per microstep, derived from
                # steps_per_mm. Axis present in bindings with no/zero
                # steps_per_mm is misconfigured; skip.
                spm = (
                    steps_per_mm[axis_idx]
                    if axis_idx < len(steps_per_mm)
                    else 0.0
                )
                if spm <= 0:
                    continue
                microstep_distance = 1.0 / spm
                # Pack f32 as u32 bits for wire transport.
                microstep_bits = struct.unpack(
                    '<I', struct.pack('<f', microstep_distance)
                )[0]
                # extrusion_per_xy_mm is unused by the new firmware (the
                # per-segment field on push_segment is authoritative); send
                # 0.0 for ABI compatibility.
                extrusion_bits = 0
                # Build the bindings blob (4 bytes per stepper).
                blob = bytearray()
                for (oid, inv) in bindings:
                    blob.append(oid)
                    blob.append(inv & 0x01)
                    tmc_oid = TMC_CS_OID_NONE
                    if step_modes[axis_idx] == 0:
                        for (sname, s) in slot_steppers[axis_idx]:
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
                configure_axis_cmd.send([
                    axis_idx, MODE_PULSE, microstep_bits, extrusion_bits,
                    len(bindings), ring_depth, bytes(blob),
                ])
            logging.info(
                "MotionToolhead: configure_axes mcu=%s kin=%d "
                "present=0x%x awd=0x%x invert=0x%x steps_per_mm=%s "
                "step_modes=%s mcu_caps=0x%x runtime_bindings=%s "
                "phase_configs=%s any_phase_stepping=%s "
                "phase_motor_count=%d",
                name, kin_tag, present_mask, awd_mask, invert_mask,
                steps_per_mm, step_modes, mcu_caps,
                [(m, n, o, i) for (m, n, o, i) in bind_list],
                phase_configs, any_phase_stepping,
                len(phase_configs),
            )
            # phase_stepping_enable_spi is sent from TMC5160._xdirect_preload
            # after all TMC register init is complete, not here.

    # ------------------------------------------------------------------
    # Sim-only diagnostic gcode commands
    # ------------------------------------------------------------------

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
                        chip=chip_id, line=line, value=level,
                    )
                gcmd.respond_info(
                    "KALICO_SIM_ENDSTOP_SET_PIN gpio=%d level=%d -> ok (shim)"
                    % (gpio, level)
                )
                return
            except Exception as e:
                raise gcmd.error("set_gpio_input failed: %s" % e)
        # Fallback: send runtime_sim_endstop_set_pin directly to the
        # firmware (Renode sim path — CONFIG_KALICO_SIM only).
        if self.mcu is None:
            raise gcmd.error("no MCU available for sim endstop set_pin")
        handle = self.mcu._bridge_handle
        try:
            self.bridge.bridge_send(
                handle,
                "runtime_sim_endstop_set_pin gpio=%d level=%d"
                % (gpio, level),
            )
            gcmd.respond_info(
                "KALICO_SIM_ENDSTOP_SET_PIN gpio=%d level=%d -> ok (fw)"
                % (gpio, level)
            )
        except Exception as e:
            raise gcmd.error(
                "runtime_sim_endstop_set_pin failed: %s" % e
            )


def add_printer_objects(config):
    """Register the MotionToolhead (and extruder) with the printer."""
    config.get_printer().add_object("toolhead", MotionToolhead(config))
    extruder.add_printer_objects(config)
