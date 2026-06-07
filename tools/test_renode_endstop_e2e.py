#!/usr/bin/env python3
import argparse
import pathlib
import struct
import sys

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "tools"))

from kalico_host_io import KalicoHostIO  # noqa: E402
from test_renode_gpio_injection import RenodeMonitor  # noqa: E402

pytestmark = pytest.mark.needs_renode


def encode_source(
    kind, gpio, polarity, policy, sample_n, velocity_axis, v_min_q16
):
    return struct.pack(
        "<BHBBBBI",
        int(kind),
        int(gpio),
        int(polarity),
        int(policy),
        int(sample_n),
        int(velocity_axis),
        int(v_min_q16),
    )


def encode_steppers(oids):
    return bytes(int(o) & 0xFF for o in oids)


def send_arm_endstop(io, arm_id, arm_clock, sources, stepper_oids):
    parser = io.get_msgparser()
    sources_blob = b"".join(encode_source(*s) for s in sources)
    steppers_blob = encode_steppers(stepper_oids)
    arm_clock_lo = arm_clock & 0xFFFFFFFF
    arm_clock_hi = (arm_clock >> 32) & 0xFFFFFFFF
    msg = None
    for fmt in parser.messages_by_id.values():
        if getattr(fmt, "name", None) == "runtime_arm_endstop":
            msg = fmt
            break
    if msg is None:
        raise RuntimeError("runtime_arm_endstop missing from data dict")
    params = {
        "arm_id": int(arm_id),
        "arm_clock_lo": int(arm_clock_lo),
        "arm_clock_hi": int(arm_clock_hi),
        "source_count": len(sources),
        "sources": sources_blob,
        "stepper_count": len(stepper_oids),
        "steppers": steppers_blob,
    }
    cmd_bytes = msg.encode_by_name(**params)
    _send_encoded(io, cmd_bytes)


def _send_encoded(io, cmd_bytes):
    import msgproto

    seq = io._next_seq()  # noqa: SLF001
    msglen = msgproto.MESSAGE_MIN + len(cmd_bytes)
    seq_byte = (seq & msgproto.MESSAGE_SEQ_MASK) | msgproto.MESSAGE_DEST
    payload = [msglen, seq_byte] + list(cmd_bytes)
    crc = msgproto.crc16_ccitt(payload)
    payload.extend(crc)
    payload.append(msgproto.MESSAGE_SYNC)
    io._ser.write(bytes(payload))  # noqa: SLF001
    io._ser.flush()  # noqa: SLF001


def send_disarm_endstop(io, arm_id):
    io.send("runtime_disarm_endstop arm_id=%d" % (int(arm_id),))


def send_sim_set_endstop_pin(io, gpio, level):
    io.send(
        "runtime_sim_endstop_set_pin gpio=%d level=%d"
        % (int(gpio), int(bool(level)))
    )


def _wait_for_response_with_id(io, name, arm_id, timeout=5.0):
    import time

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = max(0.05, deadline - time.monotonic())
        resp = io.wait_for_response(name, timeout=remaining)
        if int(resp.get("arm_id", -1)) == int(arm_id):
            return resp
    raise AssertionError(
        "did not see %s for arm_id=%d within %.2fs" % (name, arm_id, timeout)
    )


def test_arm_trip_disarm(io, renode):
    io.send("runtime_sim_engine_tick_start")
    renode.advance_time(0.005)
    tick_start_resp = io.wait_for_response(
        "runtime_sim_engine_tick_start_response", timeout=3.0
    )
    assert int(tick_start_resp["result"]) == 0
    print("[arm_trip] TIM5 enabled")

    arm_id = 0xA5A50001
    test_gpio = 17
    sample_n = 1
    sources = [
        (0, test_gpio, 1, 0, sample_n, 0x01, 0),
    ]
    stepper_oids = [1, 2]

    send_arm_endstop(
        io,
        arm_id=arm_id,
        arm_clock=0,
        sources=sources,
        stepper_oids=stepper_oids,
    )
    renode.advance_time(0.010)
    arm_resp = _wait_for_response_with_id(
        io, "kalico_arm_endstop_response", arm_id, timeout=5.0
    )
    assert int(arm_resp["status"]) == 0, "expected status=0 (Armed), got %r" % (
        arm_resp,
    )
    print("[arm_trip] arm acked: %r" % (arm_resp,))

    send_sim_set_endstop_pin(io, gpio=test_gpio, level=1)
    renode.advance_time(0.010)
    pin_resp = io.wait_for_response(
        "runtime_sim_endstop_set_pin_response", timeout=3.0
    )
    assert int(pin_resp["result"]) == 0, "set_pin failed: %s" % (pin_resp,)
    print("[arm_trip] pin set high: %r" % (pin_resp,))
    renode.advance_time(0.020)

    trip = _wait_for_response_with_id(
        io, "kalico_endstop_tripped", arm_id, timeout=5.0
    )
    print("[arm_trip] trip event: %r" % (trip,))

    assert int(trip["fmt_version"]) == 1, "expected fmt_version=1, got %r" % (
        trip,
    )
    assert int(trip["trip_source_idx"]) == 0, (
        "expected trip_source_idx=0, got %r" % (trip,)
    )
    assert int(trip["stepper_count"]) == len(stepper_oids), (
        "expected stepper_count=%d, got %r" % (len(stepper_oids), trip)
    )
    blob = trip.get("stepper_data", b"")
    if isinstance(blob, str):
        import ast

        try:
            blob = ast.literal_eval(blob)
        except (ValueError, SyntaxError):
            blob = blob.encode("latin-1")
    if not isinstance(blob, (bytes, bytearray)):
        raise AssertionError("stepper_data not bytes-like: %r" % (blob,))
    expected_blob_len = 5 * len(stepper_oids)
    assert len(blob) == expected_blob_len, "stepper_data len=%d expected %d" % (
        len(blob),
        expected_blob_len,
    )
    for i, oid in enumerate(stepper_oids):
        rec = blob[i * 5 : (i + 1) * 5]
        got_oid = rec[0]
        got_count = struct.unpack("<i", rec[1:5])[0]
        assert got_oid == oid, "rec[%d] oid=%d expected %d" % (i, got_oid, oid)
        assert got_count == 0, (
            "rec[%d] step_count=%d expected 0 (no motion in test)"
            % (i, got_count)
        )
    trip_clock = (int(trip["trip_clock_hi"]) << 32) | int(trip["trip_clock_lo"])
    assert trip_clock > 0, "trip_clock=%d expected > 0" % (trip_clock,)
    print("[arm_trip] trip event shape OK; trip_clock=%d" % (trip_clock,))

    send_disarm_endstop(io, arm_id=arm_id)
    renode.advance_time(0.010)
    disarm_resp = _wait_for_response_with_id(
        io, "kalico_disarm_endstop_response", arm_id, timeout=3.0
    )
    assert int(disarm_resp["status"]) == 1, (
        "expected status=1 (AlreadyTripped), got %r" % (disarm_resp,)
    )
    print("[arm_trip] disarm-after-trip → AlreadyTripped (status=1)")


def test_arm_disarm_clean(io, renode):
    arm_id = 0xA5A50002
    test_gpio = 18
    sources = [(0, test_gpio, 1, 0, 1, 0x01, 0)]
    stepper_oids = [3]
    send_sim_set_endstop_pin(io, gpio=test_gpio, level=0)
    renode.advance_time(0.005)
    io.wait_for_response("runtime_sim_endstop_set_pin_response", timeout=3.0)

    send_arm_endstop(
        io,
        arm_id=arm_id,
        arm_clock=0,
        sources=sources,
        stepper_oids=stepper_oids,
    )
    renode.advance_time(0.010)
    arm_resp = _wait_for_response_with_id(
        io, "kalico_arm_endstop_response", arm_id, timeout=5.0
    )
    assert int(arm_resp["status"]) == 0, "expected status=0 (Armed), got %r" % (
        arm_resp,
    )
    print("[arm_disarm] arm acked: %r" % (arm_resp,))

    send_disarm_endstop(io, arm_id=arm_id)
    renode.advance_time(0.010)
    disarm_resp = _wait_for_response_with_id(
        io, "kalico_disarm_endstop_response", arm_id, timeout=3.0
    )
    assert int(disarm_resp["status"]) == 0, (
        "expected status=0 (Disarmed), got %r" % (disarm_resp,)
    )
    print("[arm_disarm] clean disarm OK (status=0)")


def run(args):
    renode = RenodeMonitor(
        uart_port=args.uart_tcp_port,
        gdb_port=args.gdb_port,
        monitor_port=args.monitor_tcp_port,
        log_path=args.renode_log,
    )
    io = None
    try:
        print("[e2e] launching Renode ...")
        renode.start()
        port_url = "socket://localhost:%d" % (args.uart_tcp_port,)
        print("[e2e] connecting host I/O on %s ..." % (port_url,))
        io = KalicoHostIO(port_url, identify_timeout=args.identify_timeout)
        parser = io.get_msgparser()
        messages = parser.get_messages()
        names = set()
        for m in messages:
            if isinstance(m, str):
                names.add(m.split(None, 1)[0])
            elif isinstance(m, (tuple, list)) and len(m) >= 3:
                fmt = m[2]
                if isinstance(fmt, str):
                    names.add(fmt.split(None, 1)[0])
        for need in (
            "runtime_arm_endstop",
            "runtime_disarm_endstop",
            "runtime_sim_endstop_set_pin",
            "runtime_sim_engine_tick_start",
        ):
            assert need in names, (
                "%s missing from data dict; rebuild with CONFIG_KALICO_SIM=y"
                % (need,)
            )
        print("[e2e] data dict OK")

        renode.pause()

        test_arm_trip_disarm(io, renode)
        test_arm_disarm_clean(io, renode)

        print("PASS: Renode endstop arm/trip/disarm e2e")
        return 0
    finally:
        if io is not None:
            try:
                io.disconnect()
            except Exception:
                pass
        if not args.keep_renode:
            renode.stop()


def main():
    p = argparse.ArgumentParser(description="Renode endstop e2e test")
    p.add_argument("--uart-tcp-port", type=int, default=3334)
    p.add_argument("--monitor-tcp-port", type=int, default=3335)
    p.add_argument("--gdb-port", type=int, default=3333)
    p.add_argument("--identify-timeout", type=float, default=60.0)
    p.add_argument("--renode-log", default="/tmp/kalico-renode-endstop.log")
    p.add_argument("--keep-renode", action="store_true")
    args = p.parse_args()
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
