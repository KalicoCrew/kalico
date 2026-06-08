from __future__ import annotations

import pathlib

import pytest

_THIS_DIR = pathlib.Path(__file__).resolve().parent


def _native_built() -> bool:
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
        try:
            item_path = pathlib.Path(str(item.fspath)).resolve()
        except Exception:
            continue
        if _THIS_DIR == item_path or _THIS_DIR in item_path.parents:
            item.add_marker(skip)
