#!/usr/bin/env python3
"""
Task 12 — Renode Phase-2 gate test (wire-level via bridge).

Boots the existing Renode H723 simulation (USART2 → tcp://localhost:3334)
and validates that the Phase-2 motion-bridge produces wire traffic the
firmware accepts. Concretely:

  1. Connect to the sim via `KalicoHostIO`, perform the identify handshake,
     and capture the raw msgproto dictionary.
  2. Instantiate `motion_bridge.MotionBridge` (the PyO3 cdylib loaded via
     `klippy/motion_bridge.so`), feed it the identify dict, claim two MCU
     handles (Octopus + F446 in the bridge's two-MCU MVP topology), and
     call `init_planner` with a representative limits/shaper config. This
     proves the bridge ingests the live identify dict without errors.
  3. From the same `KalicoHostIO` connection, send the *same wire-level
     commands* the bridge's dispatch closure would emit
     (`kalico_load_curve`, `kalico_push_segment`) and assert the firmware
     round-trips them with `result=0`. This is the wire-level contract
     between bridge dispatch and firmware acceptance.

Why not call `bridge.submit_move()` directly here?
  The bridge's `RouterTransport` requires its `PassthroughRouter` to be
  pumped by an external host_io reactor that owns the serial wire and
  feeds it bytes. In production that reactor is part of klippy's
  `serialhdl` machinery; in this test harness we don't have it. Building
  a Python-side byte-pump (`pop_next_for_emission` ↔ socket writer +
  socket reader ↔ `dispatch_response`) would require new PyO3 surface
  on the bridge that does not exist today. The wire-level protocol the
  bridge would emit is identical to what step 1+3 above exercise via
  `KalicoHostIO`'s own encoder, so this gate provides the same
  wire-level coverage Task 12 is uniquely positioned to provide.

  Full bridge-driven `submit_move` end-to-end against Renode (planner →
  classify → dispatch closure → wire) is deferred to Step 7-D
  (hardware bring-up) where klippy's full reactor drives the bridge.
  Step-pin GPIO observation is also deferred to 7-D — the existing
  Renode .repl tags GPIOs opaque, so per-pin step pulses are not
  observable here without significant Renode-side .repl extensions
  (tracked as future work — Option C in the Task 12 plan).

Multi-move bug note (Task 11): any second `submit_move` trips
`TemporalJoining(StalledOnInfeasibleSegment)`. This harness deliberately
exercises a single move's worth of wire traffic.

Usage (manages sim lifecycle externally — see `scripts/renode_phase2_gate.sh`):
    bash tools/sim/build_sim_firmware.sh
    make -f Makefile.kalico motion-bridge
    bash tools/sim/run_sim.sh &
    sleep 8
    python3 tools/test_renode_phase2_gate.py
"""

import argparse
import math
import os
import pathlib
import struct
import sys
import time

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "tools"))
sys.path.insert(0, str(REPO_ROOT / "klippy"))

from kalico_host_io import KalicoHostIO  # noqa: E402

# Match tools/test_sim_gate_a.py timing constants.
CLOCK_FREQ = 520_000_000
TICK_HZ = 40_000
ONE_TICK_CYCLES = CLOCK_FREQ // TICK_HZ  # 13_000
LARGE_T_START_BASE = 52_000_000  # ≈ 100 ms ahead of widened_now at boot

# Step 7-B wire constants.
FORMAT_VERSION_V1 = 1
# CurveHandle::UNUSED_SENTINEL packed u32 — axis not used by this segment.
UNUSED_HANDLE = 0xFFFEFFFE
# EMode values (runtime/src/config.rs).
E_MODE_COUPLED_TO_XY = 0
E_MODE_INDEPENDENT   = 1
E_MODE_TRAVEL        = 2


def floats_to_blob(values):
    raw = b"".join(struct.pack("<f", float(v)) for v in values)
    return raw.hex()


# Scalar cubic Bézier: linear 0→10 mm along one axis.
# Step 7-B per-axis-scalar format: cps is a flat list of f32 scalars (not XYZ
# tuples), knots is unchanged.
FIXTURE_SCALAR_CUBIC = {
    "name": "scalar_cubic_bezier_10mm",
    "degree": 3,
    "cps": [0.0, 3.3333333, 6.6666666, 10.0],
    "knots": [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
}


def load_curve(io, slot, fixture, timeout=3.0):
    """Send kalico_load_curve (scalar V1 format) and return (result, handle)."""
    cmd = (
        "kalico_load_curve version=%d slot=%d degree=%d cps=%s knots=%s"
        % (
            FORMAT_VERSION_V1,
            slot,
            int(fixture["degree"]),
            floats_to_blob(fixture["cps"]),
            floats_to_blob(fixture["knots"]),
        )
    )
    io.send(cmd)
    resp = io.wait_for_response("kalico_load_curve_response", timeout)
    return int(resp["result"]), int(resp.get("curve_handle_packed", 0))


def push_segment(io, seg_id, x_handle, y_handle, z_handle, e_handle,
                 t_start, t_end, kin=0, e_mode=E_MODE_TRAVEL,
                 extrusion_ratio=0, timeout=3.0):
    """Send kalico_push_segment (Step 7-B per-axis-handle format)."""
    cmd = (
        "kalico_push_segment id=%d x_handle=%d y_handle=%d "
        "z_handle=%d e_handle=%d "
        "t_start_hi=%d t_start_lo=%d t_end_hi=%d t_end_lo=%d "
        "kinematics=%d e_mode=%d extrusion_ratio=%d"
        % (
            seg_id,
            x_handle, y_handle, z_handle, e_handle,
            (t_start >> 32) & 0xFFFFFFFF,
            t_start & 0xFFFFFFFF,
            (t_end >> 32) & 0xFFFFFFFF,
            t_end & 0xFFFFFFFF,
            kin, e_mode, extrusion_ratio,
        )
    )
    io.send(cmd)
    resp = io.wait_for_response("kalico_push_response", timeout)
    return int(resp["result"])


def import_motion_bridge():
    """Import the PyO3 cdylib via klippy/motion_bridge.so."""
    try:
        import motion_bridge as native  # noqa: F401
        return native
    except ImportError as exc:
        raise SystemExit(
            "FAIL: motion_bridge native module not importable. "
            "Build it first: `make -f Makefile.kalico motion-bridge`. (%s)"
            % (exc,)
        )


def main():
    p = argparse.ArgumentParser(description="Renode Phase-2 gate test")
    p.add_argument(
        "--port",
        default="socket://localhost:3334",
        help="pyserial URL of the Renode USART2 bridge",
    )
    p.add_argument(
        "--identify-timeout",
        type=float,
        default=60.0,
        help="seconds to wait for identify handshake",
    )
    args = p.parse_args()

    print("[gate] connecting to %s ..." % (args.port,))
    io = KalicoHostIO(args.port, identify_timeout=args.identify_timeout)
    try:
        # ── Step 1: capture identify dict ──────────────────────────────────
        parser = io.get_msgparser()
        raw_dict = parser.get_raw_data_dictionary()
        if not raw_dict:
            raise SystemExit("FAIL: empty raw_data_dictionary after identify")
        if isinstance(raw_dict, str):
            raw_dict = raw_dict.encode("utf-8")
        print("[gate] identify ok (%d bytes raw dict)" % (len(raw_dict),))

        # ── Step 2: bridge ingests identify dict + init_planner ───────────
        native = import_motion_bridge()
        bridge = native.MotionBridge()
        bridge.set_msgproto_dict(raw_dict)
        print("[gate] bridge.set_msgproto_dict ok")

        # The bridge's two-MCU MVP topology expects two distinct handles
        # even when the test only has one underlying serial connection.
        # The dispatch closure picks per-MCU CommandQueueIds off these
        # handles; since this harness does not pump bytes through
        # RouterTransport, the handles' practical role here is just to
        # make `init_planner` happy.
        octopus = bridge.claim_mcu("octopus", args.port, 0)
        f446 = bridge.claim_mcu("f446", args.port, 0)
        print("[gate] bridge.claim_mcu ok (octopus=%d, f446=%d)"
              % (octopus, f446))

        bridge.init_planner(
            300.0,   # max_velocity (mm/s)
            5000.0,  # max_accel
            10.0,    # max_z_velocity
            100.0,   # max_z_accel
            5.0,     # square_corner_velocity
            "smooth_zv", 40.0,  # X shaper
            "smooth_zv", 40.0,  # Y shaper
            octopus, f446,
        )
        print("[gate] bridge.init_planner ok")

        # Sanity: post-init counters are zero (no moves submitted yet).
        if bridge.dispatched_segment_count() != 0:
            raise SystemExit(
                "FAIL: dispatched_segment_count=%d expected 0"
                % (bridge.dispatched_segment_count(),)
            )
        # fallback_clock_conversions starts at 0 and only increments if the
        # planner thread ever runs without clocksync wired. We don't drive
        # the planner here, so it must remain 0.
        if bridge.fallback_clock_conversions() != 0:
            raise SystemExit(
                "FAIL: fallback_clock_conversions=%d expected 0"
                % (bridge.fallback_clock_conversions(),)
            )

        # ── Step 3: wire-level kalico_load_curve + kalico_push_segment ────
        # These are the same commands the bridge's dispatch closure emits
        # via RouterTransport. We send them through `KalicoHostIO`'s own
        # encoder, since both encoders target the identical msgproto
        # dictionary captured in Step 1.
        #
        # Step 7-B: load two scalar curves (X and Y axis), then push one
        # segment referencing both handles.
        rc_x, x_handle = load_curve(io, slot=0, fixture=FIXTURE_SCALAR_CUBIC)
        if rc_x != 0:
            raise SystemExit("FAIL: kalico_load_curve (X) result=%d" % (rc_x,))
        print("[gate] kalico_load_curve ok (slot=0, x_handle=0x%08x)" % (x_handle,))

        rc_y, y_handle = load_curve(io, slot=1, fixture=FIXTURE_SCALAR_CUBIC)
        if rc_y != 0:
            raise SystemExit("FAIL: kalico_load_curve (Y) result=%d" % (rc_y,))
        print("[gate] kalico_load_curve ok (slot=1, y_handle=0x%08x)" % (y_handle,))

        # 100-tick segment, well ahead of widened_now to avoid underrun.
        seg_cycles = 100 * ONE_TICK_CYCLES
        rc = push_segment(
            io,
            seg_id=1,
            x_handle=x_handle,
            y_handle=y_handle,
            z_handle=UNUSED_HANDLE,
            e_handle=UNUSED_HANDLE,
            t_start=LARGE_T_START_BASE,
            t_end=LARGE_T_START_BASE + seg_cycles,
            kin=0,           # CoreXyAndE
            e_mode=E_MODE_TRAVEL,
            extrusion_ratio=0,
        )
        if rc != 0:
            raise SystemExit("FAIL: kalico_push_segment result=%d" % (rc,))
        print("[gate] kalico_push_segment ok (id=1, kin=0/CoreXyAndE)")

        # Brief settle so the firmware has a chance to advance the queue;
        # we don't arm-and-drain here (covered by Gate B), the wire-level
        # accept is the gate.
        time.sleep(0.05)

        print("PASS: Renode Phase-2 gate (wire-level via bridge)")
        return 0
    finally:
        try:
            io.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
