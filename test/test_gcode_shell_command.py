import os

import pytest

from klippy.extras import gcode_shell_command


class _GCodeError(Exception):
    pass


class _FakeGCode:
    error = _GCodeError


class _FakePrinter:
    def get_reactor(self):
        return None


def test_params_expand_env_vars_and_home(monkeypatch):
    monkeypatch.setenv("KALICO_TEST_DIR", "/tmp/kalico test")
    captured = []

    def fake_popen(args, **kwargs):
        captured.append(args)
        raise OSError("stop here")

    monkeypatch.setattr(gcode_shell_command.subprocess, "Popen", fake_popen)
    cmd = gcode_shell_command.ShellCommand.__new__(
        gcode_shell_command.ShellCommand
    )
    cmd.name = "test"
    cmd.command = ["echo"]
    cmd.gcode = _FakeGCode()
    cmd.printer = _FakePrinter()

    with pytest.raises(_GCodeError):
        cmd.cmd_RUN_SHELL_COMMAND(
            {"PARAMS": '"$KALICO_TEST_DIR/a.csv" ~/b.png plain'}
        )
    assert captured == [
        [
            "echo",
            "/tmp/kalico test/a.csv",
            os.path.expanduser("~") + "/b.png",
            "plain",
        ]
    ]
