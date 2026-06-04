"""Skip the motion-bridge engine-integration suite when the cdylib is absent.

These tests exercise the raw PyO3 bridge API (``motion_bridge.MotionBridge``
with ``claim_mcu`` / ``alloc_command_queue`` / ``passthrough_send`` …). That
API lives in the compiled ``motion_bridge_native`` cdylib. When it is not
built — CI today, where the extension module is not compiled — there is
nothing real to test, so every case here would fail on a missing-attribute
error. We skip them honestly (with a reason, never silent-pass) so a red CI
run always means a real defect, and the suite lights up automatically once an
engine PR builds the cdylib. See docs/kalico-rewrite/ci.md.
"""

from __future__ import annotations

import pathlib

import pytest

_THIS_DIR = pathlib.Path(__file__).resolve().parent


def _native_built() -> bool:
    """True when the ``motion_bridge_native`` cdylib is importable."""
    try:
        from klippy import motion_bridge
    except Exception:
        return False
    return motion_bridge._native is not None


_NATIVE_BUILT = _native_built()


def pytest_collection_modifyitems(config, items):
    if _NATIVE_BUILT:
        return
    skip = pytest.mark.skip(
        reason="motion_bridge_native cdylib not built — raw-bridge "
        "integration tests need it; build it to exercise the engine "
        "(see docs/kalico-rewrite/ci.md)."
    )
    for item in items:
        # collection_modifyitems sees the whole session; only gate items
        # collected from this directory's tree.
        try:
            item_path = pathlib.Path(str(item.fspath)).resolve()
        except Exception:
            continue
        if _THIS_DIR == item_path or _THIS_DIR in item_path.parents:
            item.add_marker(skip)
