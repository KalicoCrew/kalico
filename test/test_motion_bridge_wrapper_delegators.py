# Every method the host calls on the bridge (the _STUB_MOTION_METHODS surface)
# must be a real method on MotionBridgeWrapper, not only on the no-bridge stub.
#
# Regression: register_stepper_slot was added to _STUB_MOTION_METHODS and called
# from motion_toolhead._configure_axes_per_mcu, but had no MotionBridgeWrapper
# delegator -> AttributeError at klippy connect. The other Python tests bypass
# the real wrapper (via __new__ / fake bridges), so nothing caught it until the
# firmware was flashed and klippy tried to connect on real hardware.
import sys
import types

_fake_native_mod = types.ModuleType("klippy.motion_bridge_native")
_fake_native_mod.MotionBridge = object
sys.modules.setdefault("klippy.motion_bridge_native", _fake_native_mod)

from klippy.motion_bridge import (  # noqa: E402
    _STUB_MOTION_METHODS,
    MotionBridgeWrapper,
)


def test_wrapper_forwards_every_stub_motion_method():
    missing = sorted(
        name
        for name in _STUB_MOTION_METHODS
        if not callable(getattr(MotionBridgeWrapper, name, None))
    )
    assert missing == [], "MotionBridgeWrapper missing delegators for: %s" % (
        missing,
    )
