from __future__ import annotations

import klippy.chelper
import pathlib
import os

# Ensure chelper is built
klippy.chelper.get_ffi()


ROOT = pathlib.Path(__file__).parent.parent
KLIPPY_PLUGINS = ROOT / "klippy" / "plugins"
TESTING_PLUGIN = ROOT / "test" / "klippy_testing_plugin.py"


def pytest_sessionstart(session):
    link_path = KLIPPY_PLUGINS / "testing.py"
    if link_path.exists():
        return

    os.symlink(TESTING_PLUGIN, link_path)

    @session.config.add_cleanup
    def clean_symlink():
        os.unlink(link_path)
