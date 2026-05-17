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


@pytest.mark.parametrize(
    "sim_extra_overrides",
    [
        {
            # Force phase-stepping mode on every Z stepper. The bench's
            # printer.cfg has `phase_stepping: 1` on stepper_z / z1 / z2,
            # which sets step_modes[2]=0 (Modulated) in
            # MotionToolhead.configure_axes. Bench evidence (klippy.log
            # L1522556): `step_modes=[1, 1, 0, 1] mcu_caps=0x1`.
            #
            # Without this override the sim's vendored printer.cfg leaves
            # Z in StepTime mode (step_modes[2]=1), the F4 engine
            # advances current_segment_id normally, and credit_freed
            # retires the slot pool — no deadlock, no repro.
            "stepper_z.config_set": {"phase_stepping": "1"},
            "stepper_z1.config_set": {"phase_stepping": "1"},
            "stepper_z2.config_set": {"phase_stepping": "1"},
        },
    ],
    indirect=True,
)
def test_same_direction_jogs_reproduce_slot_pool_exhaustion(sim):
    """REPRO for the live-printer jogging crash captured on trident
    in klippy.log session banner_time='Sun May 17 18:47:58 2026'.

    User sequence (bench, real Trident config):
        SET_KINEMATIC_POSITION X=100 Y=100 Z=10
        _CLIENT_LINEAR_MOVE X=-25 F=6000   # expands to G91 + G1 X-25 F6000
        _CLIENT_LINEAR_MOVE X=-25 F=6000
        _CLIENT_LINEAR_MOVE X=-25 F=6000
        _CLIENT_LINEAR_MOVE X=-25 F=6000   # this is the one that raises

    Observed (klippy.log L1617041):
        RuntimeError: dispatch error: slot pool exhausted for mcu=1
        (capacity=4, in_flight=4); awaiting kalico_credit_freed
        retirement events

    Bridge transitions to shutdown; H7 'mcu' and F4 'bottom' both get
    'Command request' shutdown reasons.

    Root cause (from the live log):
      - F4 'bottom' MCU (owns Z, CONFIG_RUNTIME_CURVE_POOL_N=4) never
        advances its runtime engine past segment_id=0 across the entire
        session — verified across ~105k log lines after configure_axes,
        F4 status frames only ever report current_segment_id=0.
      - F4 therefore never emits kalico_credit_freed, so the host's
        F4 slot pool never retires.
      - Per the 2026-05-11 dispatch fix (rust/motion-bridge/src/dispatch.rs:
        ``x_move_sends_curves_for_every_kinematic_axis_on_each_mcu``), every
        ShapedSegment dispatches a Z curve to F4 even on pure-XY jogs
        because Z is in the kinematics bitmask.
      - 4-jog sequence fills the F4 slot pool; the 5th try_alloc raises
        SlotPoolExhausted{mcu_id=1, capacity=4, in_flight=4}.

    Sim adaptation: pin-overrides shrinks position_max to 20, so we
    cannot start at X=100 like the bench. Use small jogs near origin to
    stay within the shrunken bounds; the slot-pool accounting is
    independent of dx magnitude (1 F4 slot per ShapedSegment regardless
    of geometry).

    Asserted behavior: with the 2026-05-17 fix in place, all 10 jogs
    must succeed. The fix has two parts in rust/runtime/src/engine.rs:
      - Engine::push_segment no longer CASes producer_pending. Previously
        it raced the C-side `kalico_runtime_kick_producer` CAS; Engine
        always won, leaving the C-side's `arm_producer_timer_if_kicked`
        with a failed CAS and no scheduled producer timer. On pure-
        Modulated MCUs (F4 phase-stepped Z with no StepTime motor to
        drive step_time_event re-kicks) this left the producer timer
        un-armed forever.
      - producer_step unconditionally dequeues at the top so Modulated-
        only configs see segment cursor advance. The per-motor StepTime
        loop below `continue`s on every non-StepTime motor, so without
        the top-of-function dequeue `fetch_segment_for_motor` was never
        called for pure-Modulated configs and `producer_current` stayed
        None → `runtime_modulated_tick` no-op'd → segments never retired.
    Both fixes are required: the first re-arms the timer so producer_step
    actually runs; the second makes producer_step useful for Modulated-
    only MCUs.
    """
    _wait_ready(sim)

    # Position inside the sim's shrunken range (pin-overrides:
    # position_max=20). Z is unconstrained for our purposes here.
    r = sim.gcode(
        "SET_KINEMATIC_POSITION X=15 Y=15 Z=10",
        timeout=10.0,
    )
    err = (r.get("error") or {}).get("message", "") if isinstance(r, dict) else ""
    assert not err, f"SET_KINEMATIC_POSITION failed: {r}"

    # Switch to relative coords once; subsequent G1s are 1mm-X jogs.
    # Each G1 = one ShapedSegment to F4 = one F4 slot allocation.
    r = sim.gcode("G91", timeout=5.0)
    err = (r.get("error") or {}).get("message", "") if isinstance(r, dict) else ""
    assert not err, f"G91 failed: {r}"

    # Loop generously — bench reproduces on the 5th, but a sim bring-up
    # might consume an extra slot somewhere (e.g. a position-anchor on
    # first dispatch), so allow up to 10. The bug deadlocks the pool
    # regardless of jog count once retirements never arrive.
    # Pace the loop so each jog crosses the planner's 50 ms T_COMMIT
    # quiescence window (rust/motion-bridge/src/planner.rs:53). Otherwise
    # back-to-back submits keep extending the held-back speculative tail
    # and never actually dispatch a new segment — masking the F4 deadlock
    # because no further try_alloc fires after the first commit. Bench
    # reproduces the bug because the user clicks jog buttons ~1 s apart,
    # which is comfortably past T_COMMIT.
    fail_idx = None
    fail_msg = ""
    responses = []
    for i in range(1, 11):
        r = sim.gcode("G1 X-1 F6000", timeout=15.0)
        responses.append((i, r))
        err = (
            (r.get("error") or {}).get("message", "")
            if isinstance(r, dict)
            else ""
        )
        if "slot pool exhausted" in err:
            fail_idx = i
            fail_msg = err
            break
        # T_COMMIT is 50 ms; 150 ms gives the timer plenty of room.
        time.sleep(0.15)

    # Print the per-jog trace whether we passed or failed, so a successful
    # repro leaves an obvious audit trail in the test output.
    print(f"[repro] jog responses (fail_idx={fail_idx}):")
    for i, r in responses:
        print(f"  jog {i}: {r}")
    if fail_idx is not None:
        print(f"[repro] REPRODUCED on jog {fail_idx}: {fail_msg}")

    assert fail_idx is None, (
        f"expected all 10 sequential G1 X-1 F6000 jogs to succeed under "
        f"the post-2026-05-17 fix, but jog {fail_idx} failed: {fail_msg!r}\n"
        f"Responses: {responses!r}\n"
        "Regression in producer_step's top-of-function dequeue OR "
        "Engine::push_segment's removed-CAS may have come back. Check "
        "rust/runtime/src/engine.rs::push_segment and ::producer_step."
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
