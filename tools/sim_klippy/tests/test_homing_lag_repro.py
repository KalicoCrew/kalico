import json
import socket
import threading
import time

import pytest

from tools.sim_klippy.orchestrator.sim_control_client import SimControlClient

pytestmark = pytest.mark.needs_elf

X_ENDSTOP_LINE = 200

SIM_OVERRIDES = {
    "stepper_x.config_set": {
        "endstop_pin": "^gpiochip0/gpio200",
        "use_sensorless_homing": "False",
        "homing_retract_dist": "5",
        "min_home_dist": "15",
        "position_endstop": "20",
        "position_max": "20",
    },
    "stepper_y.config_set": {
        "endstop_pin": "^gpiochip0/gpio201",
        "use_sensorless_homing": "False",
        "homing_retract_dist": "0",
        "min_home_dist": "0",
        "position_endstop": "20",
        "position_max": "20",
    },
}


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


@pytest.mark.parametrize("sim_extra_overrides", [SIM_OVERRIDES], indirect=True)
def test_homing_retract_and_rehome(sim):
    _wait_ready(sim)

    _set_pin(sim, X_ENDSTOP_LINE, 0)

    # The delay needs to be long enough for the homing move to start
    # but short enough that travel < min_home_dist.
    def delayed_trip():
        time.sleep(0.3)
        _set_pin(sim, X_ENDSTOP_LINE, 1)

    trip_thread = threading.Thread(target=delayed_trip, daemon=True)
    trip_thread.start()

    t0 = time.time()
    r = sim.gcode("G28 X", timeout=30.0)
    elapsed = time.time() - t0
    trip_thread.join(timeout=1.0)

    print(f"\n[G28 X rehome] elapsed={elapsed:.2f}s result={r}")

    log_text = sim.klippy_log.read_text()

    assert "needs rehome: True" in log_text, (
        "Expected 'needs rehome: True' in klippy.log — "
        "the trip should have happened before min_home_dist"
    )

    if "No trigger on x after full movement" in log_text:
        pytest.fail(
            "BUG REPRODUCED: Second homing attempt failed with "
            "'No trigger on x after full movement'. The retract move "
            "did not complete before the second home fired."
        )

    err = (r.get("error") or {}).get("message", "")
    assert not err, f"G28 X failed: {err}"

    # Check timing — with the bug, this takes 8-10+ seconds due to
    # _mcu_pending_end_time overshoot
    assert elapsed < 10.0, (
        f"G28 X took {elapsed:.1f}s — likely suffering from "
        f"_mcu_pending_end_time ghost-time delay (expected < 10s)"
    )


@pytest.mark.parametrize("sim_extra_overrides", [SIM_OVERRIDES], indirect=True)
def test_homing_retract_timing(sim):
    _wait_ready(sim)

    _set_pin(sim, X_ENDSTOP_LINE, 1)

    t0 = time.time()
    r = sim.gcode("G28 X", timeout=30.0)
    elapsed = time.time() - t0

    print(f"\n[G28 X retract timing] elapsed={elapsed:.2f}s result={r}")

    err = (r.get("error") or {}).get("message", "")
    assert not err, f"G28 X failed: {err}"

    # With retract_dist=5 at homing_speed=100, the retract is 0.05s
    # of physical motion plus TMC dwell. Should complete well under 5s.
    # The bug causes 8-10s delays.
    assert elapsed < 5.0, (
        f"G28 X with retract took {elapsed:.1f}s — likely suffering from "
        f"_mcu_pending_end_time ghost-time delay (expected < 5s)"
    )
