# Regression tests for MotionBridgeWrapper's kalico_endstop_tripped routing.
#
# Each per-rail BridgeTriggerDispatch used to register its own _on_trip_message
# for the shared "kalico_endstop_tripped" message name. register_response
# replaces by name, so only the last-registered dispatch's handler survived;
# its arm_id filter then dropped every other arm's trip. Re-homing an axis
# after another axis had homed routed the trip to the wrong (last) dispatch and
# the correct home_wait completion never fired — it timed out on the backstop
# ("MCU silent past expected end-time"). The wrapper now registers ONE handler
# per MCU that routes by arm_id.
#
# motion_bridge.py imports the native .so at load; inject a stand-in so the
# import resolves without a build artifact.
import sys
import types

_fake_native_mod = types.ModuleType("klippy.motion_bridge_native")
_fake_native_mod.MotionBridge = object
sys.modules.setdefault("klippy.motion_bridge_native", _fake_native_mod)

from klippy.motion_bridge import MotionBridgeWrapper  # noqa: E402


class FakeMcu:
    def __init__(self):
        self.registered = []

    def register_response(self, cb, name):
        self.registered.append((name, cb))


class FakeDispatch:
    def __init__(self):
        self.seen = []

    def _on_trip_message(self, params):
        self.seen.append(params)


def _new_wrapper():
    # Bypass __init__ (which constructs the native bridge) — exercise only the
    # routing/registry surface.
    w = MotionBridgeWrapper.__new__(MotionBridgeWrapper)
    w._homing_dispatches = {}
    w._trip_handler_mcus = set()
    return w


def test_trip_routes_to_the_matching_arm_not_the_last_registered():
    w = _new_wrapper()
    d1, d2 = FakeDispatch(), FakeDispatch()
    w.register_homing_dispatch(1, d1)
    w.register_homing_dispatch(2, d2)  # registered last — must not steal arm 1

    mcu = FakeMcu()
    w.register_trip_handler(mcu)
    (name, router) = mcu.registered[0]
    assert name == "kalico_endstop_tripped"

    router({"arm_id": 1})
    router({"arm_id": 2})
    assert d1.seen == [{"arm_id": 1}]
    assert d2.seen == [{"arm_id": 2}]


def test_trip_handler_registered_once_per_mcu():
    w = _new_wrapper()
    mcu = FakeMcu()
    w.register_trip_handler(mcu)
    w.register_trip_handler(mcu)
    w.register_trip_handler(mcu)
    assert len(mcu.registered) == 1


def test_unknown_arm_id_is_dropped():
    w = _new_wrapper()
    d1 = FakeDispatch()
    w.register_homing_dispatch(1, d1)
    mcu = FakeMcu()
    w.register_trip_handler(mcu)
    (_name, router) = mcu.registered[0]

    router({"arm_id": 99})
    router({})  # missing arm_id
    assert d1.seen == []
