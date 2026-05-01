"""Smoke test that motion_bridge imports and instantiates."""


def test_module_imports():
    import motion_bridge
    assert hasattr(motion_bridge, "MotionBridge")


def test_bridge_instantiates():
    import motion_bridge
    bridge = motion_bridge.MotionBridge()
    assert bridge.version() != ""
