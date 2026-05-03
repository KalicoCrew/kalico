# MotionToolhead — skeleton toolhead implementing the public API
#
# This file is part of the Kalico motion-bridge integration (Stage D).
# Move-issuing calls raise NotImplementedError; status/query methods work.
import logging

from .kinematics import extruder
from . import chelper
from . import stepper


class BridgeKinematics:
    """Minimal kinematics shim for motion-bridge hardware initialization."""

    def __init__(self, toolhead, config, trapq):
        kin_name = config.get("kinematics")
        if kin_name not in ("cartesian", "corexy", "hybrid_corexy"):
            raise config.error("Unsupported bridge kinematics '%s'" % (kin_name,))
        self.kinematics = kin_name
        self.rails = []

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

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
        return [0.0, 0.0, 0.0]

    def check_move(self, move):
        pass

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
        return {
            "homed_axes": "",
            "axis_minimum": gcode_mod.Coord(x_min, y_min, z_min, 0.0),
            "axis_maximum": gcode_mod.Coord(x_max, y_max, z_max, 0.0),
        }


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

        # Phase 2: motion bridge handle (Rust planner pipeline).
        self.bridge = self.printer.lookup_object("motion_bridge", None)

        # Position tracking
        self.commanded_pos = [0.0, 0.0, 0.0, 0.0]

        # Velocity / acceleration config (parsed for compat)
        self.max_velocity = config.getfloat("max_velocity", above=0.0)
        self.max_accel = config.getfloat("max_accel", above=0.0)
        self.max_z_velocity = config.getfloat(
            "max_z_velocity", self.max_velocity, above=0.0
        )
        self.max_z_accel = config.getfloat(
            "max_z_accel", self.max_accel, above=0.0
        )
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

        # Step generation stubs
        self.step_generators = []
        self.kin_flush_delay = 0.001
        self.kin_flush_times = []
        self.print_time = 0.0
        self.print_stall = 0
        self._flush_callbacks = []

        # Allocate a real trapq so kinematics/stepper hardware init doesn't
        # crash on set_trapq(). The bridge owns trajectory; itersolve is idle.
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)

        # Load kinematics for hardware init (creates stepper objects for TMC,
        # motors_sync, autotune etc.). Bridge overrides actual motion output.
        self.kin = BridgeKinematics(self, config, self.trapq)

        # Register gcode commands that must exist for compat
        gcode.register_command("G4", self.cmd_G4)
        gcode.register_command("M400", self.cmd_M400)
        gcode.register_command(
            "SET_VELOCITY_LIMIT",
            self.cmd_SET_VELOCITY_LIMIT,
            desc="Set printer velocity limits",
        )
        gcode.register_command("M204", self.cmd_M204)
        # SET_INPUT_SHAPER is registered by the [input_shaper] module; under
        # the bridge it routes through bridge.update_shaper directly (see
        # klippy/extras/input_shaper.py::cmd_SET_INPUT_SHAPER).

        # Phase 2: initialize the Rust planner once all MCUs are connected.
        self.printer.register_event_handler(
            "klippy:connect", self._init_planner
        )

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
        if self.bridge is not None:
            self.bridge.set_position(newpos[0], newpos[1], newpos[2])

    # ------------------------------------------------------------------
    # Move commands — raise until Phase 2
    # ------------------------------------------------------------------

    def move(self, newpos, speed):
        dx = newpos[0] - self.commanded_pos[0]
        dy = newpos[1] - self.commanded_pos[1]
        dz = newpos[2] - self.commanded_pos[2]
        de = newpos[3] - self.commanded_pos[3]
        feedrate = min(speed, self.max_velocity)
        if abs(dz) > 1e-9 and abs(dx) < 1e-9 and abs(dy) < 1e-9:
            feedrate = min(feedrate, self.max_z_velocity)
        if self.bridge is not None:
            self.bridge.submit_move(dx, dy, dz, de, feedrate)
        self.commanded_pos[:] = newpos

    def manual_move(self, coord, speed):
        curpos = list(self.commanded_pos)
        for i in range(len(coord)):
            if coord[i] is not None:
                curpos[i] = coord[i]
        self.move(curpos, speed)

    def dwell(self, delay):
        if self.bridge is not None:
            self.bridge.submit_dwell(delay)

    def wait_moves(self):
        if self.bridge is not None:
            self.bridge.wait_moves()

    def drip_move(self, newpos, speed, drip_completion):
        # Phase 2: drip moves (homing) bypass the planner queue. Until the
        # bridge exposes a dedicated drip API, fall back to a normal move so
        # bring-up doesn't crash.
        self.move(newpos, speed)

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
        if self.bridge is not None:
            return self.bridge.get_last_move_time()
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
        if self.bridge is not None and (
            max_velocity is not None or max_accel is not None
        ):
            self.bridge.update_limits(self.max_velocity, self.max_accel)

    # ------------------------------------------------------------------
    # Phase 2: planner initialization
    # ------------------------------------------------------------------

    def _init_planner(self):
        if self.bridge is None:
            return
        # Locate the two MVP MCUs by name. The first-print topology is:
        #   - "mcu" (Octopus) drives X+Y
        #   - "mcu z"  (or first non-primary MCU) drives Z
        # If only one MCU is configured, reuse its handle for Z so init
        # succeeds during single-MCU bring-up.
        octopus = None
        f446 = None
        for name, mcu in self.printer.lookup_objects(module="mcu"):
            handle = getattr(mcu, "_bridge_handle", None)
            if handle is None:
                continue
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
                for s in shapers:
                    if s.axis == "x":
                        shaper_type_x = s.shaper_type
                        shaper_freq_x = s.shaper_freq
                    elif s.axis == "y":
                        shaper_type_y = s.shaper_type
                        shaper_freq_y = s.shaper_freq
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
        except Exception:
            logging.exception("MotionToolhead: init_planner failed")
            raise

    def cmd_M204(self, gcmd):
        accel = gcmd.get_float("S", None, above=0.0)
        if accel is None:
            p = gcmd.get_float("P", None, above=0.0)
            t = gcmd.get_float("T", None, above=0.0)
            if p is not None and t is not None:
                accel = min(p, t)
        if accel is not None:
            self.max_accel = accel


# ---------------------------------------------------------------------------
# Compat shim — symbols previously exported by klippy/toolhead.py
#
# trad_rack.py subclasses ToolHead and references LookAheadQueue,
# BUFFER_TIME_HIGH, and SDS_CHECK_TIME.  Provide them here so trad_rack can
# import from motion_toolhead instead of the deleted toolhead module.
# ---------------------------------------------------------------------------

LOOKAHEAD_FLUSH_TIME = 0.250
BUFFER_TIME_LOW = 1.0
BUFFER_TIME_HIGH = 2.0
BUFFER_TIME_START = 0.250
SDS_CHECK_TIME = 0.001  # step+dir+step filter in stepcompress.c


class LookAheadQueue:
    """Minimal lookahead queue used by TradRackToolHead.

    Mirrors the public interface of the original toolhead.LookAheadQueue so
    that trad_rack.py can operate without importing the deleted toolhead
    module.
    """

    def __init__(self, toolhead):
        self.toolhead = toolhead
        self.queue = []
        self.junction_flush = LOOKAHEAD_FLUSH_TIME

    def reset(self):
        del self.queue[:]
        self.junction_flush = LOOKAHEAD_FLUSH_TIME

    def set_flush_time(self, flush_time):
        self.junction_flush = flush_time

    def get_last(self):
        if self.queue:
            return self.queue[-1]
        return None

    def flush(self, lazy=False):
        # Phase-1 stub: no itersolve-based flush needed.
        pass

    def add_move(self, move):
        self.queue.append(move)


# Allow code that does ``toolhead.ToolHead`` after aliasing this module
# as ``toolhead`` to resolve correctly.
ToolHead = MotionToolhead


def add_printer_objects(config):
    """Register the MotionToolhead (and extruder) with the printer.

    Called from printer.py during printer object setup, replacing the
    equivalent function from the deleted toolhead.py.
    """
    from .kinematics import extruder

    config.get_printer().add_object("toolhead", MotionToolhead(config))
    extruder.add_printer_objects(config)
