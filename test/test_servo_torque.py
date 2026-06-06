# Unit tests for the servo torque-gate integration with stepper_enable.
# Plain fakes only — the classes under test are duck-typed.
from klippy.extras.stepper_enable import EnableTracking, StepperEnablePin


class FakeLine:
    """set_digital-shaped recorder (the BridgeTorqueLine contract)."""

    def __init__(self):
        self.calls = []

    def set_digital(self, print_time, value):
        self.calls.append((print_time, value))


class FakeMotor:
    """add_active_callback-shaped motor (MCU_stepper / ServoRail contract)."""

    def __init__(self):
        self._active_callbacks = []

    def add_active_callback(self, cb):
        self._active_callbacks.append(cb)

    def get_name(self, short=False):
        return "servo_x"


def test_enable_tracking_drives_torque_line_like_a_stepper():
    line = FakeLine()
    motor = FakeMotor()
    et = EnableTracking(motor, StepperEnablePin(line, 0))
    # construction armed the one-shot motor_enable callback
    assert len(motor._active_callbacks) == 1
    # first activity energizes at the move's print_time
    motor._active_callbacks.pop()(12.5)
    assert line.calls == [(12.5, 1)]
    assert et.is_motor_enabled()
    # M84 path de-energizes at the scheduled print_time and re-arms
    et.motor_disable(13.5)
    assert line.calls == [(12.5, 1), (13.5, 0)]
    assert not et.is_motor_enabled()
    assert len(motor._active_callbacks) == 1
    # refcount: second enable cycle goes through the same line
    motor._active_callbacks.pop()(14.5)
    assert line.calls[-1] == (14.5, 1)


from klippy.extras import servo_axis


class FakeNode:
    def __init__(self, handle):
        self._h = handle

    def get_bridge_handle(self):
        return self._h


class FakeBridge:
    def __init__(self):
        self.calls = []

    def set_torque(self, handle, value, print_time):
        self.calls.append((handle, value, print_time))


class FakePrinter:
    command_error = RuntimeError

    def __init__(self, objs):
        self._objs = objs

    def lookup_object(self, name):
        return self._objs[name]


def test_bridge_torque_line_maps_set_digital_to_set_torque():
    bridge = FakeBridge()
    printer = FakePrinter(
        {"ethercat_node node_y": FakeNode(7), "motion_bridge": bridge}
    )
    line = servo_axis.BridgeTorqueLine(printer, "node_y")
    line.set_digital(20.0, 1)
    line.set_digital(21.0, 0)
    assert bridge.calls == [(7, True, 20.0), (7, False, 21.0)]


def test_bridge_torque_line_fails_loudly_without_handle():
    printer = FakePrinter(
        {"ethercat_node node_y": FakeNode(None), "motion_bridge": FakeBridge()}
    )
    line = servo_axis.BridgeTorqueLine(printer, "node_y")
    try:
        line.set_digital(20.0, 1)
        raise AssertionError("expected command_error")
    except RuntimeError as e:
        assert "no bridge handle" in str(e)


def test_servo_rail_active_callback_contract():
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail._active_callbacks = []
    fired = []
    rail.add_active_callback(fired.append)
    assert rail._active_callbacks == [fired.append]
