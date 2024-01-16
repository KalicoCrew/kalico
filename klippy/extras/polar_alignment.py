# import chelper
import logging
import math


def polar_to_cartesian(r, theta):
    return (r * math.cos(theta), r * math.sin(theta))


def circle_from_3_points(p1, p2, p3):
    z1 = complex(p1[0], p1[1])
    z2 = complex(p2[0], p2[1])
    z3 = complex(p3[0], p3[1])

    if (z1 == z2) or (z2 == z3) or (z3 == z1):
        raise ValueError("Duplicate points: %s, %s, %s" % (z1, z2, z3))

    w = (z3 - z1) / (z2 - z1)

    if abs(w.imag) <= 0.0001:
        raise ValueError("Points are collinear: %s, %s, %s" % (z1, z2, z3))

    c = (z2 - z1) * (w - abs(w) ** 2) / (2j * w.imag) + z1
    # Simplified denominator
    r = abs(z1 - c)
    r = round(r, 10)
    c = (round(c.real, 10), round(c.imag, 10))

    return c, r


class PolarAlignment:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.steppers = {}
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "POLAR_ALIGN", self.cmd_POLAR_ALIGN, desc=self.cmd_POLAR_ALIGN_help
        )
        self.gcode.register_command(
            "PROBETEST", self.cmd_PROBETEST, desc=self.cmd_POLAR_ALIGN_help
        )
        self.bearing_diameter = config.getfloat(
            "bearing_diameter", 60, above=0.0
        )
        self.threads = {"CW-M3": 0.5, "CW-M4": 0.7, "CW-M5": 0.8}
        self.thread_factor = config.getchoice(
            "screw_thread", self.threads, default="CW-M5"
        )
        self.z_probe_position = config.getfloat(
            "z_probe_position", 10, above=0.0
        )
        self.z_clearance = config.getfloat("z_clearance", 50, above=0.0)
        self.center_offset = config.getfloat("center_offset", 25, above=0.0)
        self.lower_clearance = config.getfloat("lower_clearance", 1, above=0.0)
        self.bed_stepper = config.getsection("stepper_x")
        self.gcode.respond_info("bed_stepper: %s" % self.bed_stepper)
        self.probing_speed = config.getfloat("probing_speed", 10.0, above=0.0)
        self.mcu_probe = self.printer.lookup_object("probe").mcu_probe

    def register_stepper(self, config, mcu_stepper):
        self.steppers[mcu_stepper.get_name()] = mcu_stepper

    def lookup_stepper(self, name):
        if name not in self.steppers:
            raise self.printer.config_error("Unknown stepper %s" % (name,))
        return self.steppers[name]

    cmd_POLAR_ALIGN_help = "Tool to help align bed"

    def cmd_PROBETEST(self, gcmd):
        self.toolhead = self.printer.lookup_object("toolhead")
        res = self._bearing_probe()
        self.gcode.respond_info("distance is %s" % res)

    def _move(self, pos, speed):
        self.toolhead.move((pos[0], pos[1], pos[2], 0), speed)

    def _bearing_probe(self):
        phoming = self.printer.lookup_object("homing")
        move_speed = self.printer.lookup_object("gcode_move").speed
        self._move((self.center_offset, 0, self.z_clearance), move_speed)
        logging.info("moving to %s" % self.center_offset)
        self._move((self.center_offset, 0, self.z_probe_position), move_speed)
        probe = self.printer.lookup_object("probe")
        probe_gcmd = self.gcode.create_gcode_command("PROBE", "PROBE", {})
        result = probe.run_probe(probe_gcmd)
        z_probe_height = result[2]
        x_probe_height = z_probe_height + self.lower_clearance
        self._move((self.center_offset, 0, x_probe_height), move_speed)
        position = ((self.bearing_diameter / 2) + 10, 0, x_probe_height, 0)
        epos = phoming.probing_move(
            self.mcu_probe, position, self.probing_speed
        )
        dist = epos[0] + self.center_offset
        self.gcode.respond_info("probe was %s" % dist)
        self._move((self.center_offset, 0, self.z_clearance), move_speed)
        return dist

    def cmd_POLAR_ALIGN(self, gcmd):
        self.toolhead = self.printer.lookup_object("toolhead")
        move_speed = self.printer.lookup_object("gcode_move").speed
        dist_1 = self._bearing_probe()
        self.move_bed(120, 120)
        dist_2 = self._bearing_probe()
        self.move_bed(120, 120)
        dist_3 = self._bearing_probe()
        self.move_bed(120, 120)

        p1 = polar_to_cartesian(dist_1, 0)
        p2 = polar_to_cartesian(dist_2, 120)
        p3 = polar_to_cartesian(dist_3, 240)
        center, _ = circle_from_3_points(p1, p2, p3)
        x_offset = center[0]
        y_offset = center[1]
        offset_offset = 25
        self._move((x_offset + offset_offset, 0, self.z_clearance), move_speed)
        self.toolhead.set_position((offset_offset, 0, self.z_clearance, 0))
        self.gcode.respond_info("x was off by %s" % x_offset)
        self.gcode.respond_info("y is off by %s" % y_offset)
        self.gcode.respond_info("01:20 means 1 full turn and 20 minutes")
        adjust = y_offset / self.thread_factor
        adjust = abs(adjust)
        full_turns = math.trunc(adjust)
        decimal_part = adjust - full_turns
        minutes = round(decimal_part * 60, 0)
        self.gcode.respond_info("Adjust by %s:%s" % (full_turns, minutes))

    def move_bed(self, angle, speed, accel=1000):
        self.printer.lookup_object("force_move").manual_move(
            self.lookup_stepper("stepper_bed"), angle, speed, accel
        )


def load_config(config):
    return PolarAlignment(config)
