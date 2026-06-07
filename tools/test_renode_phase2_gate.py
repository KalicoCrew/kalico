#!/usr/bin/env python3
import argparse
import pathlib
import struct
import sys
import time

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "tools"))
sys.path.insert(0, str(REPO_ROOT / "klippy"))

from kalico_host_io import KalicoHostIO  # noqa: E402

pytestmark = pytest.mark.needs_renode

CLOCK_FREQ = 520_000_000
TICK_HZ = 40_000
ONE_TICK_CYCLES = CLOCK_FREQ // TICK_HZ
LARGE_T_START_BASE = 52_000_000

FORMAT_VERSION_V1 = 1
# Wire-stable CurveHandle::UNUSED_SENTINEL packed u32 — NOT 0xFFFFFFFF.
UNUSED_HANDLE = 0xFFFEFFFE
E_MODE_COUPLED_TO_XY = 0
E_MODE_INDEPENDENT = 1
E_MODE_TRAVEL = 2


def floats_to_blob(values):
    raw = b"".join(struct.pack("<f", float(v)) for v in values)
    return raw.hex()


FIXTURE_SCALAR_CUBIC = {
    "name": "scalar_cubic_bezier_10mm",
    "degree": 3,
    "cps": [0.0, 3.3333333, 6.6666666, 10.0],
    "knots": [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
}


def load_curve(io, slot, fixture, timeout=3.0):
    cmd = "kalico_load_curve version=%d slot=%d degree=%d cps=%s knots=%s" % (
        FORMAT_VERSION_V1,
        slot,
        int(fixture["degree"]),
        floats_to_blob(fixture["cps"]),
        floats_to_blob(fixture["knots"]),
    )
    io.send(cmd)
    resp = io.wait_for_response("kalico_load_curve_response", timeout)
    return int(resp["result"]), int(resp.get("curve_handle_packed", 0))


def push_segment(
    io,
    seg_id,
    x_handle,
    y_handle,
    z_handle,
    e_handle,
    t_start,
    t_end,
    kin=0,
    e_mode=E_MODE_TRAVEL,
    extrusion_ratio=0,
    timeout=3.0,
):
    cmd = (
        "kalico_push_segment id=%d x_handle=%d y_handle=%d "
        "z_handle=%d e_handle=%d "
        "t_start_hi=%d t_start_lo=%d t_end_hi=%d t_end_lo=%d "
        "kinematics=%d e_mode=%d extrusion_ratio=%d"
        % (
            seg_id,
            x_handle,
            y_handle,
            z_handle,
            e_handle,
            (t_start >> 32) & 0xFFFFFFFF,
            t_start & 0xFFFFFFFF,
            (t_end >> 32) & 0xFFFFFFFF,
            t_end & 0xFFFFFFFF,
            kin,
            e_mode,
            extrusion_ratio,
        )
    )
    io.send(cmd)
    resp = io.wait_for_response("kalico_push_response", timeout)
    return int(resp["result"])


def import_motion_bridge():
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
        parser = io.get_msgparser()
        raw_dict = parser.get_raw_data_dictionary()
        if not raw_dict:
            raise SystemExit("FAIL: empty raw_data_dictionary after identify")
        if isinstance(raw_dict, str):
            raw_dict = raw_dict.encode("utf-8")
        print("[gate] identify ok (%d bytes raw dict)" % (len(raw_dict),))

        native = import_motion_bridge()
        bridge = native.MotionBridge()
        bridge.set_msgproto_dict(raw_dict)
        print("[gate] bridge.set_msgproto_dict ok")

        octopus = bridge.claim_mcu("octopus", args.port, 0)
        f446 = bridge.claim_mcu("f446", args.port, 0)
        print(
            "[gate] bridge.claim_mcu ok (octopus=%d, f446=%d)" % (octopus, f446)
        )

        topology = [
            (octopus, [0, 1, 3], 0),
            (f446, [2], 1),
        ]
        bridge.init_planner(
            300.0,
            5000.0,
            10.0,
            100.0,
            5.0,
            "smooth_zv",
            40.0,
            "smooth_zv",
            40.0,
            topology,
        )
        print("[gate] bridge.init_planner ok")

        if bridge.dispatched_segment_count() != 0:
            raise SystemExit(
                "FAIL: dispatched_segment_count=%d expected 0"
                % (bridge.dispatched_segment_count(),)
            )
        if bridge.fallback_clock_conversions() != 0:
            raise SystemExit(
                "FAIL: fallback_clock_conversions=%d expected 0"
                % (bridge.fallback_clock_conversions(),)
            )

        rc_x, x_handle = load_curve(io, slot=0, fixture=FIXTURE_SCALAR_CUBIC)
        if rc_x != 0:
            raise SystemExit("FAIL: kalico_load_curve (X) result=%d" % (rc_x,))
        print(
            "[gate] kalico_load_curve ok (slot=0, x_handle=0x%08x)"
            % (x_handle,)
        )

        rc_y, y_handle = load_curve(io, slot=1, fixture=FIXTURE_SCALAR_CUBIC)
        if rc_y != 0:
            raise SystemExit("FAIL: kalico_load_curve (Y) result=%d" % (rc_y,))
        print(
            "[gate] kalico_load_curve ok (slot=1, y_handle=0x%08x)"
            % (y_handle,)
        )

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
            kin=0,
            e_mode=E_MODE_TRAVEL,
            extrusion_ratio=0,
        )
        if rc != 0:
            raise SystemExit("FAIL: kalico_push_segment result=%d" % (rc,))
        print("[gate] kalico_push_segment ok (id=1, kin=0/CoreXyAndE)")

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
