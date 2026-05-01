# MotionToolhead — skeleton toolhead implementing the public API
#
# This file is part of the Kalico motion-bridge integration (Stage D).
# Move-issuing calls raise NotImplementedError; status/query methods work.
import logging

from .kinematics import extruder


class MotionToolhead:
    """Phase 1 toolhead skeleton.

    Status and query methods function normally. Move-issuing calls
    raise NotImplementedError until the Rust planner is wired (Phase 2).
    """

    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.all_mcus = [
            m for n, m in self.printer.lookup_objects(module="mcu")
        ]
        self.mcu = self.all_mcus[0] if self.all_mcus else None

        # Position tracking
        self.commanded_pos = [0.0, 0.0, 0.0, 0.0]

        # Velocity / acceleration config (parsed for compat)
        self.max_velocity = config.getfloat("max_velocity", above=0.0)
        self.max_accel = config.getfloat("max_accel", above=0.0)
        min_cruise_ratio = 0.5
        if config.getfloat("minimum_cruise_ratio", None) is None:
            req_accel_to_decel = config.getfloat(
                "max_accel_to_decel", None, above=0.0
            )
            if req_accel_to_decel is not None:
                min_cruise_ratio = 1.0 - min(
                    1.0, (req_accel_to_decel / self.max_accel)
                )
        self.min_cruise_ratio = config.getfloat(
            "minimum_cruise_ratio", min_cruise_ratio, below=1.0, minval=0.0
        )
        self.square_corner_velocity = config.getfloat(
            "square_corner_velocity", 5.0, minval=0.0
        )
        self.max_accel_to_decel = self.max_accel * (1.0 - self.min_cruise_ratio)

        # Extruder placeholder
        self.extruder = extruder.DummyExtruder(self.printer)

        # Coord type from gcode module
        gcode = self.printer.lookup_object("gcode")
        self.Coord = gcode.Coord

        # Kinematics placeholder
        self.kin = None

        # Step generation stubs
        self.step_generators = []
        self.kin_flush_delay = 0.001
        self.kin_flush_times = []
        self.print_time = 0.0
        self.print_stall = 0
        self._flush_callbacks = []

        # Trapq stub (no C allocation)
        self.trapq = None

        # Register gcode commands that must exist for compat
        gcode.register_command("G4", self.cmd_G4)
        gcode.register_command("M400", self.cmd_M400)
        gcode.register_command(
            "SET_VELOCITY_LIMIT",
            self.cmd_SET_VELOCITY_LIMIT,
            desc="Set printer velocity limits",
        )
        gcode.register_command("M204", self.cmd_M204)

        # Load modules that toolhead normally loads
        for module_name in [
            "gcode_move",
            "idle_timeout",
            "statistics",
            "manual_probe",
            "tuning_tower",
            "garbage_collection",
        ]:
            self.printer.load_object(config, module_name)

        logging.info("MotionToolhead: Phase 1 skeleton initialized")

    # ------------------------------------------------------------------
    # Position tracking
    # ------------------------------------------------------------------

    def get_position(self):
        return list(self.commanded_pos)

    def set_position(self, newpos, homing_axes=()):
        self.commanded_pos[:] = newpos

    # ------------------------------------------------------------------
    # Move commands — raise until Phase 2
    # ------------------------------------------------------------------

    def move(self, newpos, speed):
        raise NotImplementedError(
            "MotionToolhead.move() not available until Phase 2 planner integration"
        )

    def manual_move(self, coord, speed):
        raise NotImplementedError(
            "MotionToolhead.manual_move() not available until Phase 2"
        )

    def dwell(self, delay):
        # Dwell is safe to no-op in Phase 1
        pass

    def wait_moves(self):
        # No queued moves in Phase 1
        pass

    def drip_move(self, newpos, speed, drip_completion):
        raise NotImplementedError(
            "MotionToolhead.drip_move() not available until Phase 2"
        )

    # ------------------------------------------------------------------
    # Extruder
    # ------------------------------------------------------------------

    def set_extruder(self, ext, extrude_pos):
        self.extruder = ext
        self.commanded_pos[3] = extrude_pos

    def get_extruder(self):
        return self.extruder

    # ------------------------------------------------------------------
    # Kinematics
    # ------------------------------------------------------------------

    def get_kinematics(self):
        return self.kin

    def get_trapq(self):
        return self.trapq

    # ------------------------------------------------------------------
    # Step generation
    # ------------------------------------------------------------------

    def register_step_generator(self, handler):
        self.step_generators.append(handler)

    def note_step_generation_scan_time(self, delay, old_delay=0.0):
        if old_delay and old_delay in self.kin_flush_times:
            self.kin_flush_times.remove(old_delay)
        if delay:
            self.kin_flush_times.append(delay)

    def register_lookahead_callback(self, callback):
        callback(self.get_last_move_time())

    def note_mcu_movequeue_activity(self, mq_time, set_step_gen_time=False):
        pass

    # ------------------------------------------------------------------
    # Flush
    # ------------------------------------------------------------------

    def flush_step_generation(self):
        pass

    def get_last_move_time(self):
        if self.mcu is not None:
            return self.mcu.estimated_print_time(self.reactor.monotonic())
        return 0.0

    # ------------------------------------------------------------------
    # Velocity limits
    # ------------------------------------------------------------------

    def get_max_velocity(self):
        return self.max_velocity, self.max_accel

    def limit_next_junction_speed(self, speed):
        pass

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def get_status(self, eventtime):
        res = {}
        if self.kin is not None and hasattr(self.kin, "get_status"):
            res.update(self.kin.get_status(eventtime))
        est = 0.0
        if self.mcu is not None:
            est = self.mcu.estimated_print_time(eventtime)
        res.update(
            {
                "print_time": self.print_time,
                "stalls": self.print_stall,
                "estimated_print_time": est,
                "extruder": self.extruder.get_name(),
                "position": self.Coord(*self.commanded_pos),
                "max_velocity": self.max_velocity,
                "max_accel": self.max_accel,
                "minimum_cruise_ratio": self.min_cruise_ratio,
                "square_corner_velocity": self.square_corner_velocity,
            },
        )
        return res

    def stats(self, eventtime):
        return False, "print_time=%.3f buffer_time=0.000 print_stall=%d" % (
            self.print_time,
            self.print_stall,
        )

    def check_busy(self, eventtime):
        est = 0.0
        if self.mcu is not None:
            est = self.mcu.estimated_print_time(eventtime)
        return self.print_time, est, True

    # ------------------------------------------------------------------
    # Misc
    # ------------------------------------------------------------------

    def motor_off(self):
        pass

    def register_move_handler(self, handler):
        pass

    # ------------------------------------------------------------------
    # G-code commands
    # ------------------------------------------------------------------

    def cmd_G4(self, gcmd):
        delay = gcmd.get_float("P", 0.0, minval=0.0) / 1000.0
        self.dwell(delay)

    def cmd_M400(self, gcmd):
        self.wait_moves()

    def cmd_SET_VELOCITY_LIMIT(self, gcmd):
        max_velocity = gcmd.get_float("VELOCITY", None, above=0.0)
        max_accel = gcmd.get_float("ACCEL", None, above=0.0)
        if max_velocity is not None:
            self.max_velocity = max_velocity
        if max_accel is not None:
            self.max_accel = max_accel

    def cmd_M204(self, gcmd):
        accel = gcmd.get_float("S", None, above=0.0)
        if accel is None:
            p = gcmd.get_float("P", None, above=0.0)
            t = gcmd.get_float("T", None, above=0.0)
            if p is not None and t is not None:
                accel = min(p, t)
        if accel is not None:
            self.max_accel = accel
