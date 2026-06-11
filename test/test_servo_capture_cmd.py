import pytest

from klippy.extras import servo_capture


class FakeGcode:
    def __init__(self):
        self.commands = {}

    def register_command(self, name, func, desc=None):
        assert name not in self.commands
        self.commands[name] = func


class FakeNode:
    def __init__(self, handle):
        self._h = handle

    def get_bridge_handle(self):
        return self._h


class FakeBridge:
    def __init__(self, stop_result=(0, 1234, None)):
        self.start_calls = []
        self.stop_calls = []
        self._stop_result = stop_result

    def start_servo_capture(self, handle, path, started_utc, drive_name):
        self.start_calls.append((handle, path, started_utc, drive_name))

    def stop_servo_capture(self, handle):
        self.stop_calls.append(handle)
        return self._stop_result


class FakePrinter:
    command_error = RuntimeError

    def __init__(self, objs):
        self._objs = objs

    def lookup_object(self, name):
        return self._objs[name]

    def lookup_objects(self, module=None):
        prefix = module + " "
        return [
            (name, obj)
            for name, obj in self._objs.items()
            if name == module or name.startswith(prefix)
        ]


class FakeConfig:
    def __init__(self, printer):
        self._printer = printer

    def get_printer(self):
        return self._printer


class FakeGcmd:
    error = RuntimeError

    def __init__(self, **params):
        self._params = params
        self.responses = []

    def get(self, name, default=None):
        return self._params.get(name, default)

    def respond_info(self, msg):
        self.responses.append(msg)


def make_capture(nodes=None, bridge=None):
    gcode = FakeGcode()
    objs = {"gcode": gcode, "motion_bridge": bridge or FakeBridge()}
    resolved_nodes = {"x": 7} if nodes is None else nodes
    for name, handle in resolved_nodes.items():
        objs["ethercat_node " + name] = FakeNode(handle)
    printer = FakePrinter(objs)
    sc = servo_capture.ServoCapture(FakeConfig(printer))
    return sc, gcode, objs["motion_bridge"]


def test_registers_both_commands():
    _, gcode, _ = make_capture()
    assert "SERVO_CAPTURE_START" in gcode.commands
    assert "SERVO_CAPTURE_STOP" in gcode.commands


def test_start_defaults_to_sole_servo_and_builds_path():
    sc, gcode, bridge = make_capture()
    gcmd = FakeGcmd(NAME="xtune")
    gcode.commands["SERVO_CAPTURE_START"](gcmd)
    assert len(bridge.start_calls) == 1
    handle, path, started_utc, drive_name = bridge.start_calls[0]
    assert handle == 7
    assert drive_name == "x"
    assert "/servo_captures/" in path
    assert path.endswith(".scap")
    assert "xtune_" in path
    assert started_utc.endswith("Z")
    assert any("started" in r for r in gcmd.responses)


def test_start_rejects_bad_name():
    sc, gcode, bridge = make_capture()
    with pytest.raises(RuntimeError):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd(NAME="../evil"))
    assert bridge.start_calls == []


def test_start_rejects_unknown_servo_and_comma_list():
    sc, gcode, bridge = make_capture()
    with pytest.raises(RuntimeError):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd(SERVO="nope"))
    with pytest.raises(RuntimeError):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd(SERVO="a,b"))
    assert bridge.start_calls == []


def test_double_start_rejected_in_klippy():
    sc, gcode, _ = make_capture()
    gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())
    with pytest.raises(RuntimeError):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())


def assert_fresh_start_possible(gcode):
    gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())


def vanish_node(sc):
    fake_node = sc.printer.lookup_objects("ethercat_node")[0][1]
    fake_node._h = None
    return fake_node


def test_stop_without_start_rejected():
    sc, gcode, bridge = make_capture()
    with pytest.raises(RuntimeError):
        gcode.commands["SERVO_CAPTURE_STOP"](FakeGcmd())
    assert bridge.stop_calls == []


def test_stop_reports_samples():
    sc, gcode, bridge = make_capture()
    gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())
    gcmd = FakeGcmd()
    gcode.commands["SERVO_CAPTURE_STOP"](gcmd)
    assert bridge.stop_calls == [7]
    assert any("1234" in r for r in gcmd.responses)


def test_stop_overflow_raises_with_failed_filename():
    bridge = FakeBridge(stop_result=(-323, 999, 4096))
    sc, gcode, _ = make_capture(bridge=bridge)
    gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())
    with pytest.raises(RuntimeError, match="failed.scap"):
        gcode.commands["SERVO_CAPTURE_STOP"](FakeGcmd())
    assert_fresh_start_possible(gcode)


def test_start_without_bridge_handle_fails_loudly():
    sc, gcode, bridge = make_capture(nodes={"x": None})
    with pytest.raises(RuntimeError, match="no bridge handle"):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())
    assert bridge.start_calls == []


def test_stop_after_node_vanished_clears_state_and_skips_bridge():
    sc, gcode, bridge = make_capture()
    gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())
    fake_node = vanish_node(sc)
    with pytest.raises(RuntimeError, match="vanished"):
        gcode.commands["SERVO_CAPTURE_STOP"](FakeGcmd())
    assert bridge.stop_calls == []
    fake_node._h = 7
    assert_fresh_start_possible(gcode)
    assert len(bridge.start_calls) == 2


def test_multiple_servos_require_servo_param():
    sc, gcode, bridge = make_capture(nodes={"a": 1, "b": 2})
    with pytest.raises(RuntimeError, match="SERVO= is required"):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())
    assert bridge.start_calls == []
    gcode.commands["SERVO_CAPTURE_START"](FakeGcmd(SERVO="b"))
    assert len(bridge.start_calls) == 1
    assert bridge.start_calls[0][0] == 2
    assert bridge.start_calls[0][3] == "b"


def test_no_nodes_configured_errors():
    sc, gcode, bridge = make_capture(nodes={})
    with pytest.raises(RuntimeError, match=r"no \[ethercat_node\]"):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())
    assert bridge.start_calls == []


def test_stop_failure_message_includes_code_and_cycle():
    bridge = FakeBridge(stop_result=(-323, 999, 4096))
    sc, gcode, _ = make_capture(bridge=bridge)
    gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())
    expected_path = bridge.start_calls[0][1]
    failed_path = expected_path[: expected_path.rfind(".scap")] + ".failed.scap"
    with pytest.raises(RuntimeError) as exc_info:
        gcode.commands["SERVO_CAPTURE_STOP"](FakeGcmd())
    msg = str(exc_info.value)
    assert "-323" in msg
    assert "4096" in msg
    assert failed_path in msg


def test_start_rejects_name_with_trailing_newline():
    sc, gcode, bridge = make_capture()
    with pytest.raises(RuntimeError):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd(NAME="evil\n"))
    assert bridge.start_calls == []


def test_start_bridge_failure_is_command_error_not_shutdown():
    class FailingBridge(FakeBridge):
        def start_servo_capture(self, handle, path, started_utc, drive_name):
            raise RuntimeError("endpoint result -322")

    sc, gcode, _ = make_capture(bridge=FailingBridge())
    with pytest.raises(RuntimeError, match="start failed.*-322"):
        gcode.commands["SERVO_CAPTURE_START"](FakeGcmd())
    assert sc.active is None
