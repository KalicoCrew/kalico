import json
import socket
import time

import pytest

from tools.sim_klippy.orchestrator.sim_control_client import SimControlClient

pytestmark = pytest.mark.needs_elf

# Must match pin-overrides.toml [stepper_*.config_set] endstop_pin lines.
X_ENDSTOP_LINE = 200
Y_ENDSTOP_LINE = 201


def _info(api_socket: str) -> dict:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(3.0)
    s.connect(api_socket)
    s.sendall(
        json.dumps({"id": 1, "method": "info", "params": {}}).encode() + b"\x03"
    )
    buf = b""
    while True:
        try:
            c = s.recv(4096)
        except Exception:
            break
        if not c:
            break
        buf += c
        if b"\x03" in buf:
            break
    s.close()
    out = buf.split(b"\x03", 1)[0]
    try:
        return json.loads(out.decode()) if out else {}
    except Exception:
        return {}


def _query_toolhead(api_socket: str) -> dict:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(3.0)
    s.connect(api_socket)
    req = {
        "id": 1,
        "method": "objects/query",
        "params": {"objects": {"toolhead": ["homed_axes", "position"]}},
    }
    s.sendall(json.dumps(req).encode() + b"\x03")
    buf = b""
    while True:
        try:
            c = s.recv(4096)
        except Exception:
            break
        if not c:
            break
        buf += c
        if b"\x03" in buf:
            break
    s.close()
    out = buf.split(b"\x03", 1)[0]
    try:
        return json.loads(out.decode()) if out else {}
    except Exception:
        return {}


def _wait_ready(sim, timeout: float = 30.0) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        r = _info(sim.api_socket)
        if (r.get("result") or {}).get("state") == "ready":
            return
        time.sleep(0.5)
    pytest.fail("klippy not ready before homing test")


def _set_pin(sim, line: int, value: int) -> None:
    with SimControlClient(sim.h7_sim_control) as c:
        c.set_gpio_input(chip=0, line=line, value=value)


@pytest.mark.parametrize(
    "axis,line,gcode",
    [("x", X_ENDSTOP_LINE, "G28 X"), ("y", Y_ENDSTOP_LINE, "G28 Y")],
)
def test_g28_already_tripped(sim, axis, line, gcode):
    _wait_ready(sim)
    _set_pin(sim, line, 1)

    t0 = time.time()
    r = sim.gcode(gcode, timeout=30.0)
    elapsed = time.time() - t0
    print(f"\n[{gcode}] elapsed={elapsed:.2f}s result={r}")

    assert "error" not in r or not r.get("error"), (
        f"{gcode} failed: {r.get('error')}"
    )

    th = _query_toolhead(sim.api_socket)
    homed = (
        th.get("result", {})
        .get("status", {})
        .get("toolhead", {})
        .get("homed_axes", "")
    )
    assert axis in homed, f"expected {axis!r} in homed_axes, got {homed!r}"


@pytest.mark.parametrize(
    "axis,line,gcode",
    [("x", X_ENDSTOP_LINE, "G28 X"), ("y", Y_ENDSTOP_LINE, "G28 Y")],
)
def test_g28_no_trigger(sim, axis, line, gcode):
    _wait_ready(sim)
    _set_pin(sim, line, 0)

    t0 = time.time()
    r = sim.gcode(gcode, timeout=30.0)
    elapsed = time.time() - t0
    print(f"\n[{gcode}] elapsed={elapsed:.2f}s result={r}")

    err = (r.get("error") or {}).get("message", "")
    assert f"No trigger on {axis}" in err, (
        f"expected 'No trigger on {axis}' error, got: {r}"
    )
