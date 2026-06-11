import os
import re
import time

CAPTURE_DIR = "~/printer_data/logs/servo_captures"
NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")


class ServoCapture:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.capture_dir = os.path.expanduser(CAPTURE_DIR)
        self.active = None  # (servo_name, path) while a capture is running
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SERVO_CAPTURE_START",
            self.cmd_SERVO_CAPTURE_START,
            desc=self.cmd_SERVO_CAPTURE_START_help,
        )
        gcode.register_command(
            "SERVO_CAPTURE_STOP",
            self.cmd_SERVO_CAPTURE_STOP,
            desc=self.cmd_SERVO_CAPTURE_STOP_help,
        )

    def _nodes(self):
        return {
            name.split()[-1]: obj
            for name, obj in self.printer.lookup_objects("ethercat_node")
        }

    def _resolve_node(self, gcmd):
        servo = gcmd.get("SERVO", None)
        nodes = self._nodes()
        if not nodes:
            raise gcmd.error("SERVO_CAPTURE: no [ethercat_node] configured")
        if servo is None:
            if len(nodes) != 1:
                raise gcmd.error(
                    "SERVO_CAPTURE: multiple servos configured (%s); "
                    "SERVO= is required" % (", ".join(sorted(nodes)),)
                )
            return next(iter(nodes.items()))
        if "," in servo:
            raise gcmd.error(
                "SERVO_CAPTURE: multi-servo capture requires all drives on "
                "one endpoint and is not implemented yet"
            )
        node = nodes.get(servo)
        if node is None:
            raise gcmd.error(
                "SERVO_CAPTURE: unknown servo %r (have: %s)"
                % (servo, ", ".join(sorted(nodes)))
            )
        return servo, node

    cmd_SERVO_CAPTURE_START_help = (
        "Start a servo telemetry capture (1 kHz). Wrap test moves and finish "
        "with M400 before SERVO_CAPTURE_STOP."
    )

    def cmd_SERVO_CAPTURE_START(self, gcmd):
        if self.active is not None:
            raise gcmd.error(
                "SERVO_CAPTURE: capture already active (%s)" % (self.active[1],)
            )
        tag = gcmd.get("NAME", "capture")
        if not NAME_RE.fullmatch(tag):
            raise gcmd.error(
                "SERVO_CAPTURE: NAME must match [A-Za-z0-9_-]+, got %r" % (tag,)
            )
        servo, node = self._resolve_node(gcmd)
        handle = node.get_bridge_handle()
        if handle is None:
            raise gcmd.error(
                "SERVO_CAPTURE: servo %r has no bridge handle (node not "
                "claimed)" % (servo,)
            )
        path = os.path.join(
            self.capture_dir,
            "%s_%s.scap" % (tag, time.strftime("%Y%m%d_%H%M%S")),
        )
        started_utc = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        bridge = self.printer.lookup_object("motion_bridge")
        try:
            bridge.start_servo_capture(handle, path, started_utc, servo)
        except RuntimeError as e:
            raise gcmd.error("SERVO_CAPTURE: start failed: %s" % (e,))
        self.active = (servo, path)
        gcmd.respond_info("Servo capture started: %s" % (path,))

    cmd_SERVO_CAPTURE_STOP_help = "Stop the active servo telemetry capture."

    def cmd_SERVO_CAPTURE_STOP(self, gcmd):
        if self.active is None:
            raise gcmd.error("SERVO_CAPTURE: no capture active")
        servo, path = self.active
        self.active = None
        node = self._nodes().get(servo)
        if node is None or node.get_bridge_handle() is None:
            raise gcmd.error(
                "SERVO_CAPTURE: servo %r vanished mid-capture" % (servo,)
            )
        bridge = self.printer.lookup_object("motion_bridge")
        result, samples, overflow_cycle = bridge.stop_servo_capture(
            node.get_bridge_handle()
        )
        if result != 0:
            failed = os.path.splitext(path)[0] + ".failed.scap"
            raise gcmd.error(
                "Servo capture FAILED (endpoint code %d, overflow_cycle=%s); "
                "partial data in %s" % (result, overflow_cycle, failed)
            )
        gcmd.respond_info(
            "Servo capture stopped: %s\n"
            "samples=%d (%.2f s at the 1 kHz DC cycle)"
            % (path, samples, samples / 1000.0)
        )


def load_config(config):
    return ServoCapture(config)
