# Standalone regression tests for unified-planner ToolHead integration.
#
# Run: klippy-env/bin/python klippy/extras/test_toolhead_unified.py
import os
import sys
import types

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
KLIPPY = os.path.join(ROOT, 'klippy')
pkg = types.ModuleType('klippy')
pkg.__path__ = [KLIPPY]
sys.modules['klippy'] = pkg
sys.modules.setdefault('klippy.chelper', types.ModuleType('klippy.chelper'))
kin_pkg = types.ModuleType('klippy.kinematics')
kin_pkg.__path__ = [os.path.join(KLIPPY, 'kinematics')]
sys.modules['klippy.kinematics'] = kin_pkg
extruder = types.ModuleType('klippy.kinematics.extruder')
extruder.DummyExtruder = object
extruder.add_printer_objects = lambda config: None
sys.modules['klippy.kinematics.extruder'] = extruder

import klippy.toolhead as toolhead  # noqa: E402


class FakePrinter:
    def command_error(self, msg):
        return RuntimeError(msg)


class FakeAxis:
    def __init__(self, name="A", segmented=False):
        self.name = name
        if segmented:
            self.process_move_segment = lambda *args: None

    def get_axis_gcode_id(self):
        return self.name


def make_toolhead_without_unified_fields():
    th = types.SimpleNamespace()
    th.max_accel = 3000.0
    th.max_velocity = 200.0
    th.junction_deviation = 0.01
    th.max_accel_to_decel = 1500.0
    th.extra_axes = []
    th._move_notch_freq = lambda move: 0.0
    th._uses_unified_reach = toolhead.ToolHead._uses_unified_reach.__get__(
        th, object)
    th._pathplan_cons = toolhead.ToolHead._pathplan_cons.__get__(th, object)
    th._move_reach_v2 = toolhead.ToolHead._move_reach_v2.__get__(th, object)
    return th


def test_subclass_without_unified_fields():
    th = make_toolhead_without_unified_fields()
    prev = toolhead.Move(th, [0., 0., 0., 0.], [10., 0., 0., 0.], 100.)
    move = toolhead.Move(th, [10., 0., 0., 0.], [10., 10., 0., 0.], 100.)
    move.calc_junction(prev)
    assert move.max_start_v2 >= 0.0
    reach = th._move_reach_v2(prev, prev.max_start_v2)
    assert reach == prev.max_start_v2 + prev.delta_v2
    print("  Move.calc_junction tolerates missing unified fields OK")


def test_unified_rejects_unsegmented_extra_axis():
    th = object.__new__(toolhead.ToolHead)
    th.unified_emit = True
    th.extra_axes = [FakeAxis("A")]
    th.printer = FakePrinter()
    try:
        th._check_unified_extra_axis_support()
    except RuntimeError as e:
        assert "process_move_segment" in str(e)
        assert "'A'" in str(e)
    else:
        raise AssertionError("unsupported extra axis was accepted")
    print("  unified planner rejects unsegmented extra axes OK")


def main():
    test_subclass_without_unified_fields()
    test_unified_rejects_unsegmented_extra_axis()
    print("ALL PASS")


if __name__ == "__main__":
    main()
