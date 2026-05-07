# MotionToolhead — skeleton toolhead implementing the public API
#
# This file is part of the Kalico motion-bridge integration (Stage D).
# Move-issuing calls raise NotImplementedError; status/query methods work.
import logging

from . import chelper
from . import stepper
from .kinematics import extruder
from .toolhead import Move, ToolHead, BUFFER_TIME_START


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
        # corexy bridge does not drive Z, but a stable Klipper printer.cfg may
        # still declare [stepper_z]/[stepper_z1..3]. Consume them as passthrough
        # rails so option validation passes; runtime ignores them.
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
            mcu_stepper.set_trapq(trapq)
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
        return [0.0, 0.0, 0.0]

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
        # Upstream kinematics contract: this method owns runtime
        # position-state sync. For cartesian, it drives itersolve. For
        # the bridge, it pushes the new basis into the planner runtime.
        if self._toolhead.bridge is not None:
            self._toolhead.bridge.set_position(
                newpos[0], newpos[1], newpos[2]
            )
        axis_rails = self._axis_rails()
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
        self.bridge = printer.lookup_object("motion_bridge", None)
        self.active_homing_arms = set()
        self.kinematics_name = config.get("kinematics", "")
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

        # Sim-only diagnostic gcode commands (only when bridge present).
        if self.bridge is not None:
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

        logging.info("MotionToolhead: Phase 1 skeleton initialized")

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
            "dz=%.4f de=%.4f feedrate=%.4f bridge_is_none=%s",
            list(newpos), speed, dx, dy, dz, de, feedrate,
            self.bridge is None,
        )
        if self.bridge is not None:
            enable_print_time = self.get_last_move_time()
            bridge_lmt_before = self.bridge.get_last_move_time()
            self.bridge.submit_move(dx, dy, dz, de, feedrate)
            self._bump_pending_end_time(
                self.bridge.get_last_move_time() - bridge_lmt_before
            )
            # Bridge synthesizes steps in the runtime; klippy's normal
            # itersolve_check_active path doesn't fire. We trigger
            # active-stepper callbacks ourselves so motors energize
            # before the move starts.
            self._fire_active_callbacks(dx, dy, dz, de, enable_print_time)
        self.commanded_pos[:] = move.end_pos

    def _fire_active_callbacks(self, dx, dy, dz, de, print_time=None):
        if self.kin is None:
            return
        active_axes = []
        if abs(dx) > 1e-9: active_axes.append("x")
        if abs(dy) > 1e-9: active_axes.append("y")
        if abs(dz) > 1e-9: active_axes.append("z")
        if not active_axes and abs(de) <= 1e-9:
            return
        if print_time is None:
            print_time = self.get_last_move_time()
        for s in self.kin.get_steppers():
            if not s._active_callbacks:
                continue
            if not any(s.is_active_axis(a) for a in active_axes):
                continue
            cbs = s._active_callbacks
            s._active_callbacks = []
            for cb in cbs:
                cb(print_time)

    def drip_move(self, newpos, speed, drip_completion):
        # Step 7-D §6.2: bridge-aware single-segment homing.
        # Endstops were armed upstream by homing.py via
        # mcu_endstop.home_start; each BridgeTriggerDispatch.start
        # registered its arm_id with self.active_homing_arms. Submit one
        # homing-tagged segment; on trip the runtime ISR aborts and
        # freezes the curve evaluator. wait_moves() returns when the
        # segment retires (Completed or Tripped).
        logging.info(
            "[bridge-trace] drip_move entered: newpos=%s speed=%s "
            "bridge_is_none=%s drip_test=%s active_homing_arms=%s",
            list(newpos), speed, self.bridge is None,
            (drip_completion.test()
             if drip_completion is not None else None),
            sorted(self.active_homing_arms),
        )
        if self.bridge is None:
            return
        if drip_completion is not None and drip_completion.test():
            return
        arm_ids = list(self.active_homing_arms)
        if not arm_ids:
            # No bridge endstops armed — fall back to a regular move so
            # bring-up doesn't crash on file-output / legacy paths.
            self.move(newpos, speed)
            return
        pos3 = list(newpos[:3]) + [0.0] * max(0, 3 - len(newpos[:3]))
        # Energize motors before the homing move. Same reasoning as in
        # move(): the bridge runtime synthesizes steps directly, so
        # klippy's itersolve_check_active path that normally fires
        # StepperEnablePin.set_enable() never runs. Fire the active
        # callbacks here too, with deltas computed from the toolhead's
        # current commanded_pos to the homing target — that's what
        # is_active_axis(...) needs to filter the right rails.
        dx = pos3[0] - self.commanded_pos[0]
        dy = pos3[1] - self.commanded_pos[1]
        dz = pos3[2] - self.commanded_pos[2]
        self._fire_active_callbacks(dx, dy, dz, 0.0, self.get_last_move_time())
        bridge_lmt_before = self.bridge.get_last_move_time()
        self.bridge.submit_homing_move(pos3, speed, arm_ids)
        self.bridge.wait_moves()
        bridge_lmt_after = self.bridge.get_last_move_time()
        duration = bridge_lmt_after - bridge_lmt_before
        self._bump_pending_end_time(duration)
        logging.info(
            "[bridge-trace] drip_move dispatched: dx=%.4f dy=%.4f dz=%.4f "
            "speed=%.4f arms=%s bridge_last_move_time=%.6f -> %.6f "
            "delta=%.6f mcu_pending_end=%.6f",
            dx, dy, dz, speed, arm_ids,
            bridge_lmt_before, bridge_lmt_after, duration,
            self._mcu_pending_end_time,
        )

    def dwell(self, delay):
        if self.bridge is not None:
            self.bridge.submit_dwell(delay)
            if delay > 0.0:
                self._bump_pending_end_time(delay)

    def wait_moves(self):
        if self.bridge is not None:
            self.bridge.wait_moves()

    def flush_step_generation(self):
        # Bridge owns flush; upstream's body operates on lookahead +
        # trapq which we bypass.
        pass

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
            if self.bridge is not None:
                self.bridge.update_limits(self.max_velocity, self.max_accel)

    def reset_accel(self):
        if self.bridge is not None:
            self.bridge.update_limits(self.max_velocity, self.max_accel)

    def cmd_SET_VELOCITY_LIMIT(self, gcmd):
        super().cmd_SET_VELOCITY_LIMIT(gcmd)
        if self.bridge is not None:
            self.bridge.update_limits(self.max_velocity, self.max_accel)

    def cmd_RESET_VELOCITY_LIMIT(self, gcmd):
        super().cmd_RESET_VELOCITY_LIMIT(gcmd)
        if self.bridge is not None:
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
        if self.bridge is None:
            return
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
            self._register_credit_freed_handlers(bridge_mcus)
        except Exception:
            logging.exception("MotionToolhead: init_planner failed")
            raise

    def _configure_axes_per_mcu(self, bridge_mcus):
        """Send `ConfigureAxes` over the kalico-native transport for each
        bridge-attached MCU. Maps klippy `MCU_stepper` objects to motor
        slots per kinematics:
          corexy:    [A=stepper_x, B=stepper_y, Z=stepper_z, E=extruder]
          cartesian: [X=stepper_x, Y=stepper_y, Z=stepper_z, E=extruder]
        Steppers not on a given MCU are omitted from that MCU's blob.
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

        steppers_by_slot = {}
        fm = self.printer.lookup_object("force_move", None)
        if fm is not None:
            for name, s in fm.steppers.items():
                if name in slot_names and name not in steppers_by_slot:
                    steppers_by_slot[name] = s

        for name, mcu_obj, mcu_handle in bridge_mcus:
            present_mask = 0
            invert_mask = 0
            steps_per_mm = [0.0, 0.0, 0.0, 0.0]
            for i, slot in enumerate(slot_names):
                s = steppers_by_slot.get(slot)
                if s is None:
                    continue
                if len(bridge_mcus) > 1:
                    try:
                        s_mcu = s.get_mcu()
                    except AttributeError:
                        s_mcu = None
                    if s_mcu is not None and s_mcu is not mcu_obj:
                        continue
                step_dist = s.get_step_dist()
                if step_dist <= 0.0:
                    continue
                steps_per_mm[i] = 1.0 / step_dist
                present_mask |= 1 << i
            awd_mask = awd_default & present_mask
            if present_mask == 0:
                logging.info(
                    "MotionToolhead: no steppers matched MCU %s; "
                    "skipping configure_axes", name,
                )
                continue
            self.bridge.configure_axes(
                mcu_handle, kin_tag, present_mask, awd_mask,
                invert_mask, steps_per_mm,
            )
            logging.info(
                "MotionToolhead: configure_axes mcu=%s kin=%d "
                "present=0x%x awd=0x%x steps_per_mm=%s",
                name, kin_tag, present_mask, awd_mask, steps_per_mm,
            )

    def _register_credit_freed_handlers(self, bridge_mcus):
        """Register a kalico_credit_freed handler on each bridge MCU.

        Dispatch path: MCU emits kalico_credit_freed
        -> Rust host_io lifts to RuntimeEvent::CreditFreed
        -> bridge.try_recv_event surfaces dict
        -> serialhdl._bridge_event_poller renames to "kalico_credit_freed"
        -> THIS handler -> bridge.on_credit_freed
        -> Rust SlotPool.retire_through_segment + CreditCounter sync.
        """
        bridge = self.bridge
        for name, mcu_obj, mcu_handle in bridge_mcus:
            serial = getattr(mcu_obj, "_serial", None)
            if serial is None or not hasattr(serial, "register_response"):
                logging.warning(
                    "MotionToolhead: bridge MCU '%s' has no SerialReader; "
                    "kalico_credit_freed handler not registered", name,
                )
                continue
            handle = mcu_handle
            mcu_label = name

            def _on_credit_freed(params, _bridge=bridge, _handle=handle,
                                 _label=mcu_label):
                try:
                    retired = int(params.get("retired_through_segment_id", 0))
                    free_slots = int(params.get("free_slots", 0))
                    result = _bridge.on_credit_freed(
                        _handle, retired, free_slots,
                    )
                    if isinstance(result, tuple) and len(result) >= 2:
                        completed_arm = result[1]
                        if completed_arm is not None:
                            _bridge.fire_homing_completion(completed_arm)
                except Exception:
                    logging.exception(
                        "MotionToolhead: bridge.on_credit_freed failed for "
                        "MCU '%s' (handle=%s)", _label, _handle,
                    )

            serial.register_response(_on_credit_freed, "kalico_credit_freed")
            logging.info(
                "MotionToolhead: registered kalico_credit_freed handler for "
                "MCU '%s' (handle=%s)", name, mcu_handle,
            )

    # ------------------------------------------------------------------
    # Sim-only diagnostic gcode commands
    # ------------------------------------------------------------------

    def cmd_KALICO_SIM_STEP_COUNT(self, gcmd):
        oid = gcmd.get_int("OID", 0, minval=0)
        if self.bridge is None or self.mcu is None:
            raise gcmd.error("bridge not available")
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
        if self.bridge is None or self.mcu is None:
            raise gcmd.error("bridge not available")
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
        if self.bridge is None or self.mcu is None:
            raise gcmd.error("bridge not available")
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
        if self.bridge is None or self.mcu is None:
            raise gcmd.error("bridge not available")
        handle = getattr(self.mcu, "_bridge_handle", None)
        if handle is None:
            raise gcmd.error("bridge handle not set")
        try:
            resp = self.bridge.bridge_call(
                handle,
                "runtime_sim_endstop_set_pin gpio=%d level=%d" % (gpio, level),
                "runtime_sim_endstop_set_pin_response",
                timeout_s=5.0,
            )
            gcmd.respond_info(
                "[bridge-async] KALICO_SIM_ENDSTOP_SET_PIN "
                "gpio=%d level=%d result=%d"
                % (gpio, level, resp.get("result", -1))
            )
        except Exception as e:
            raise gcmd.error("endstop set_pin failed: %s" % e)


def add_printer_objects(config):
    """Register the MotionToolhead (and extruder) with the printer."""
    config.get_printer().add_object("toolhead", MotionToolhead(config))
    extruder.add_printer_objects(config)
