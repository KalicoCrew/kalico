from klippy import pins
from klippy.bridge_endstop import BridgeEndstop, allocate_provider_id

Z_AXIS = 2
ACCURACY_DEFAULT_SAMPLES = 10


def calc_probe_z_result(values, method):
    if method == "median":
        ordered = sorted(values)
        middle = len(ordered) // 2
        if len(ordered) % 2:
            return ordered[middle]
        return (ordered[middle - 1] + ordered[middle]) / 2.0
    if method != "average":
        raise ValueError("unknown samples_result '%s'" % (method,))
    return sum(values) / len(values)


def validate_virtual_endstop_request(pin_params, axis):
    if pin_params["pin"] != "z_virtual_endstop":
        raise pins.error(
            "probe only provides the virtual pin 'z_virtual_endstop',"
            " not '%s'" % (pin_params["pin"],)
        )
    if pin_params["invert"] or pin_params["pullup"]:
        raise pins.error("Can not pullup/invert probe virtual endstop")
    if axis != Z_AXIS:
        raise pins.error(
            "probe:z_virtual_endstop is only usable as the Z endstop"
        )


class PrinterProbe:
    cmd_PROBE_help = "Probe Z-height at the current XY position"
    cmd_QUERY_PROBE_help = "Return the current probe state"
    cmd_PROBE_ACCURACY_help = "Probe Z-height repeatedly and report statistics"

    def __init__(self, config):
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object("pins")
        pin_desc = config.get("pin")
        pin_params = ppins.lookup_pin(
            pin_desc, can_invert=True, can_pullup=True
        )
        if not hasattr(pin_params["chip"], "create_oid"):
            raise config.error(
                "[probe] pin must be a GPIO pin on an MCU, not '%s'"
                % (pin_desc,)
            )
        self._endstop = BridgeEndstop(
            pin_params, allocate_provider_id(self.printer)
        )

        self.z_offset = config.getfloat("z_offset")
        self.x_offset = config.getfloat("x_offset", 0.0)
        self.y_offset = config.getfloat("y_offset", 0.0)
        self.speed = config.getfloat("speed", 5.0, above=0.0)
        self.lift_speed = config.getfloat("lift_speed", self.speed, above=0.0)
        self.samples = config.getint("samples", 1, minval=1)
        self.sample_retract_dist = config.getfloat(
            "sample_retract_dist", 2.0, above=0.0
        )
        self.samples_result = config.getchoice(
            "samples_result", ["median", "average"], "average"
        )
        self.samples_tolerance = config.getfloat(
            "samples_tolerance", 0.100, minval=0.0
        )
        self.samples_retries = config.getint(
            "samples_tolerance_retries", 0, minval=0
        )

        self.last_query = False
        self.last_z_result = 0.0

        ppins.register_chip("probe", self)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "PROBE", self.cmd_PROBE, desc=self.cmd_PROBE_help
        )
        gcode.register_command(
            "QUERY_PROBE", self.cmd_QUERY_PROBE, desc=self.cmd_QUERY_PROBE_help
        )
        gcode.register_command(
            "PROBE_ACCURACY",
            self.cmd_PROBE_ACCURACY,
            desc=self.cmd_PROBE_ACCURACY_help,
        )
        query_endstops = self.printer.load_object(config, "query_endstops")
        query_endstops.register_endstop(self._endstop, "probe")

    def setup_bridge_endstop(self, pin_params, axis):
        validate_virtual_endstop_request(pin_params, axis)
        return self._endstop

    def get_position_endstop(self):
        return self.z_offset

    def get_offsets(self):
        return self.x_offset, self.y_offset, self.z_offset

    def get_status(self, eventtime):
        return {
            "name": "probe",
            "last_query": self.last_query,
            "last_z_result": self.last_z_result,
        }

    def _check_homed(self, gcmd, toolhead):
        curtime = self.printer.get_reactor().monotonic()
        kin_status = toolhead.get_kinematics().get_status(curtime)
        if "z" not in kin_status["homed_axes"]:
            raise gcmd.error("Must home before probe")

    def _probe_once(self, gcmd, toolhead, homing_obj, bridge, speed):
        kin = toolhead.get_kinematics()
        rail = kin._axis_rails().get(Z_AXIS)
        if rail is None:
            raise gcmd.error("PROBE: no Z rail configured")
        pos_min = rail.get_range()[0]
        current_z = toolhead.get_position()[Z_AXIS]
        max_travel = current_z - pos_min
        if max_travel <= 0.0:
            raise gcmd.error("PROBE: toolhead already at or below position_min")
        trip_pos, final_pos = homing_obj.trip_move(
            gcmd,
            toolhead,
            bridge,
            Z_AXIS,
            -1.0,
            speed,
            max_travel,
            {
                "endstop": self._endstop,
                "provider": self,
                "trigger_height": None,
            },
        )
        newpos = list(toolhead.get_position())
        newpos[Z_AXIS] = final_pos[Z_AXIS]
        toolhead.set_position(newpos)
        return trip_pos[Z_AXIS]

    def _retract(self, toolhead, target_z, lift_speed):
        newpos = list(toolhead.get_position())
        newpos[Z_AXIS] = target_z
        toolhead.move(newpos, lift_speed)
        toolhead.wait_moves()

    def run_probe(self, gcmd):
        toolhead = self.printer.lookup_object("toolhead")
        homing_obj = self.printer.lookup_object("homing")
        bridge = self.printer.lookup_object("motion_bridge")
        speed = gcmd.get_float("PROBE_SPEED", self.speed, above=0.0)
        lift_speed = gcmd.get_float("LIFT_SPEED", self.lift_speed, above=0.0)
        sample_count = gcmd.get_int("SAMPLES", self.samples, minval=1)
        retract = gcmd.get_float(
            "SAMPLE_RETRACT_DIST", self.sample_retract_dist, above=0.0
        )
        tolerance = gcmd.get_float(
            "SAMPLES_TOLERANCE", self.samples_tolerance, minval=0.0
        )
        max_retries = gcmd.get_int(
            "SAMPLES_TOLERANCE_RETRIES", self.samples_retries, minval=0
        )
        method = gcmd.get("SAMPLES_RESULT", self.samples_result)
        if method not in ("median", "average"):
            raise gcmd.error("SAMPLES_RESULT must be median or average")
        self._check_homed(gcmd, toolhead)
        retries = 0
        measured = []
        while True:
            z = self._probe_once(gcmd, toolhead, homing_obj, bridge, speed)
            measured.append(z)
            if max(measured) - min(measured) > tolerance:
                if retries >= max_retries:
                    raise gcmd.error("Probe samples exceed samples_tolerance")
                gcmd.respond_info("Probe samples exceed tolerance. Retrying...")
                retries += 1
                measured = []
            if len(measured) >= sample_count:
                break
            self._retract(toolhead, z + retract, lift_speed)
        return calc_probe_z_result(measured, method)

    def cmd_PROBE(self, gcmd):
        toolhead = self.printer.lookup_object("toolhead")
        pos = toolhead.get_position()
        z_result = self.run_probe(gcmd)
        gcmd.respond_info(
            "probe at %.3f,%.3f is z=%.6f" % (pos[0], pos[1], z_result)
        )
        self.last_z_result = z_result

    def cmd_QUERY_PROBE(self, gcmd):
        triggered = self._endstop.is_triggered()
        self.last_query = triggered
        gcmd.respond_info("probe: %s" % ("TRIGGERED" if triggered else "open"))

    def cmd_PROBE_ACCURACY(self, gcmd):
        toolhead = self.printer.lookup_object("toolhead")
        homing_obj = self.printer.lookup_object("homing")
        bridge = self.printer.lookup_object("motion_bridge")
        speed = gcmd.get_float("PROBE_SPEED", self.speed, above=0.0)
        lift_speed = gcmd.get_float("LIFT_SPEED", self.lift_speed, above=0.0)
        sample_count = gcmd.get_int(
            "SAMPLES", ACCURACY_DEFAULT_SAMPLES, minval=1
        )
        retract = gcmd.get_float(
            "SAMPLE_RETRACT_DIST", self.sample_retract_dist, above=0.0
        )
        self._check_homed(gcmd, toolhead)
        pos = toolhead.get_position()
        gcmd.respond_info(
            "PROBE_ACCURACY at X:%.3f Y:%.3f Z:%.3f"
            " (samples=%d retract=%.3f speed=%.1f lift_speed=%.1f)"
            % (pos[0], pos[1], pos[2], sample_count, retract, speed, lift_speed)
        )
        measured = []
        for _ in range(sample_count):
            z = self._probe_once(gcmd, toolhead, homing_obj, bridge, speed)
            measured.append(z)
            self._retract(toolhead, z + retract, lift_speed)
        average = calc_probe_z_result(measured, "average")
        median = calc_probe_z_result(measured, "median")
        sigma = (
            sum((v - average) ** 2 for v in measured) / len(measured)
        ) ** 0.5
        gcmd.respond_info(
            "probe accuracy results: maximum %.6f, minimum %.6f,"
            " range %.6f, average %.6f, median %.6f, standard deviation %.6f"
            % (
                max(measured),
                min(measured),
                max(measured) - min(measured),
                average,
                median,
                sigma,
            )
        )


def load_config(config):
    return PrinterProbe(config)
