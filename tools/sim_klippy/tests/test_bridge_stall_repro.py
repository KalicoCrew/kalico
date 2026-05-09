"""Reproduce the bridge_call stall ("transport closed" / "transport timed
out") race on the sim. Mirrors what the user observed on real hardware
under G28 X / linear-move flows.

Hypothesis under test: tmc.py periodic stallguard query (1 Hz) racing
against bridge_call traffic during a stepper-enable burst causes the
reactor's awaiting_response FIFO to stall, klippy's bridge_call to time
out or see "transport closed", and IWDG to fire downstream on the MCU.

The test deliberately stresses the boundary by issuing many
back-to-back motion commands so that:

- Stepper enable fires _do_enable burst (many SPI transfers)
- Move planner emits load_curve / push_segment via kalico_call
- tmc.py periodic stallguard query has time to fire concurrently

Run with:
    docker run --rm -v $REPO:/work -w /work --tmpfs /tmp:exec kalico-sim \
        python3 -m pytest tools/sim_klippy/tests/test_bridge_stall_repro.py -v
"""
import time

import pytest


def _wait_ready(sim, timeout: float = 30.0) -> None:
    import json
    import socket as _socket
    deadline = time.time() + timeout
    while time.time() < deadline:
        s = _socket.socket(_socket.AF_UNIX, _socket.SOCK_STREAM)
        s.settimeout(3.0)
        try:
            s.connect(sim.api_socket)
            s.sendall(
                json.dumps({"id": 1, "method": "info", "params": {}}).encode()
                + b"\x03"
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
            r = json.loads(out.decode()) if out else {}
            if (r.get("result") or {}).get("state") == "ready":
                return
        except Exception:
            try:
                s.close()
            except Exception:
                pass
        time.sleep(0.5)
    pytest.fail("klippy not ready")


def test_linear_move_after_set_kinematic_position(sim):
    """Closest sim mirror of the user's last failing sequence:
        SET_KINEMATIC_POSITION X=100 Y=100 Z=10
        _CLIENT_LINEAR_MOVE X=1 F=6000

    Expected: both succeed without "bridge_call:" errors. If this
    reproduces the real-hardware crash in sim, we have a deterministic
    repro and can iterate fixes against it.
    """
    _wait_ready(sim)

    # Force-set kinematic position (no motion, no homing required).
    r = sim.gcode("SET_KINEMATIC_POSITION X=100 Y=100 Z=10", timeout=10.0)
    err = (r.get("error") or {}).get("message", "") if isinstance(r, dict) else ""
    assert "bridge_call" not in err, (
        f"SET_KINEMATIC_POSITION already failed with bridge_call error: {r}"
    )

    # Now issue a small linear move. This triggers stepper enable
    # (-> tmc.py _do_enable burst) and a planner push (-> 2 load_curves
    # + 1 push_segment via kalico_call).
    r = sim.gcode("G1 X101 F6000", timeout=15.0)
    err = (r.get("error") or {}).get("message", "") if isinstance(r, dict) else ""
    print(f"\n[G1] result={r}")
    assert "transport closed" not in err, (
        f"REPRO: bridge_call: transport closed during linear move\n  result={r}"
    )
    assert "transport timed out" not in err, (
        f"REPRO: bridge_call: transport timed out during linear move\n  result={r}"
    )


def test_burst_of_linear_moves(sim):
    """Stress-test: many linear moves back to back. If the bridge-call
    race exists, this will hit it eventually — accumulating concurrent
    kalico_call traffic against tmc periodic stallguard queries.
    """
    _wait_ready(sim)

    # Force position so we don't need homing.
    r = sim.gcode("SET_KINEMATIC_POSITION X=100 Y=100 Z=10", timeout=10.0)
    err = (r.get("error") or {}).get("message", "") if isinstance(r, dict) else ""
    assert "bridge_call" not in err

    # ~1.5 seconds of motion across multiple moves at moderate speed.
    # Long enough that the 1Hz tmc-poll timer fires at least once.
    moves = [
        "G1 X110 F6000",  # +10 mm
        "G1 X90 F6000",   # -20 mm
        "G1 Y110 F6000",
        "G1 Y90 F6000",
        "G1 X100 Y100 F6000",
    ]
    for i, mv in enumerate(moves):
        r = sim.gcode(mv, timeout=15.0)
        err = (r.get("error") or {}).get("message", "") if isinstance(r, dict) else ""
        if "transport closed" in err or "transport timed out" in err:
            pytest.fail(
                f"REPRO at move {i} ({mv!r}): {err}\n"
                f"  full result: {r}"
            )


def test_long_move_during_tmc_poll(sim):
    """Single move long enough to span at least one tmc.py periodic
    stallguard query (~1 Hz cadence). If the stallguard query races the
    motion-bridge load_curve traffic, this will reproduce the race.

    Move duration target: ~2 s. At 6000 mm/min (100 mm/s) → 200 mm.
    With position_max=20 from pin-overrides this would over-travel; we
    do a Z move instead which has more headroom.
    """
    _wait_ready(sim)

    r = sim.gcode("SET_KINEMATIC_POSITION X=100 Y=100 Z=10", timeout=10.0)
    err = (r.get("error") or {}).get("message", "") if isinstance(r, dict) else ""
    assert "bridge_call" not in err

    # Z at moderate speed: 50 mm/min on default Z config is slow enough
    # to give the 1Hz timer multiple chances to fire.
    r = sim.gcode("G1 Z40 F300", timeout=30.0)
    err = (r.get("error") or {}).get("message", "") if isinstance(r, dict) else ""
    print(f"\n[long Z move] result={r}")
    assert "transport closed" not in err, (
        f"REPRO: transport closed during long Z move\n  result={r}"
    )
    assert "transport timed out" not in err, (
        f"REPRO: transport timed out during long Z move\n  result={r}"
    )
