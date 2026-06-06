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
