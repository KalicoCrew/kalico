#!/usr/bin/env python3
"""
Renode end-to-end test for the Step 7-D endstop arm/disarm/trip wire path.

This is the §10 spec acceptance test: it boots the H723 Renode sim, exercises
the host→MCU `runtime_arm_endstop` and `runtime_disarm_endstop` commands and
the MCU→host `kalico_arm_endstop_response` / `kalico_disarm_endstop_response`
sync responses + async `kalico_endstop_tripped` output, all via real
msgproto framing through the existing `KalicoHostIO` client. The test wires
together every layer of Steps 1–4:

    Step 1  rust/runtime/src/endstop.rs  arm/disarm/poll_trip + tick ISR hook
    Step 2  rust/runtime/src/engine.rs   modulation-ISR endstop::tick + AbortNow
    Step 3  rust/runtime/src/state.rs    per-stepper counter atomics
    Step 4  src/runtime_tick.c           DECL_COMMAND wire surface +
            rust/kalico-c-api/src/runtime_ffi.rs  C-API exports +
            rust/kalico-host-rt/src/endstop.rs    host-side wire codec

Why a sim-only scaffold command is needed
=========================================

The Step-1 `endstop::set_pin_level()` abstraction (rust/runtime/src/endstop.rs:
311) reads from a `PIN_LEVELS: [AtomicBool; MAX_GPIO_PINS]` table internal
to the runtime crate. That table is *not* yet wired to real MCU GPIO in
the Step-2 modulation-ISR integration — the Step-1 commit message
(`e5026de05`) explicitly calls this out as deferred to a separate
"set_pin_level abstraction stub for the Step-2 ISR integration to wire in
real GPIO reads" workstream that has not landed.

Until that wiring is in, the e2e test cannot trip an endstop by driving
PC13 in Renode (the existing `tools/test_renode_gpio_injection.py`
fixture demonstrates real-GPIO injection working — but the runtime never
samples real GPIOs on its own). The fix here is the minimal
CONFIG_KALICO_SIM-only scaffold:

    src/runtime_tick.c       command_runtime_sim_endstop_set_pin
    rust/kalico-c-api/src/runtime_ffi.rs
                             pub extern "C" kalico_endstop_set_pin_level
                             (gated `#[cfg(feature = "kalico-sim")]`)

The shim pokes the abstract pin table directly. Production firmware
never includes it. When the real-GPIO sampler lands, this test should
be updated to drive PC13 via the existing `RenodeMonitor.set_gpio`
fixture (the pin-table abstraction stays useful for unit tests).

Hard prerequisites the test handles explicitly
==============================================

A. Endstop ISR must run. `Engine::tick` calls `poll_endstop_trip` from
   the empty-queue path (rust/runtime/src/engine.rs:323). This fires
   every modulation tick once `homed=1` (otherwise the homed gate
   short-circuits before reaching the queue check). So the test issues
   `runtime_set_homed_state homed=1` after identify and never opens a
   stream — the runtime ticks, sees no segments, and runs the endstop
   poll each tick.

B. CONFIG_KALICO_SIM build: the new commands require the sim build
   (`tools/sim/build_sim_firmware.sh`). The endstop production wire
   surface is NOT gated on CONFIG_KALICO_SIM and is present in any
   build; only `runtime_sim_endstop_set_pin` is sim-only.

C. arm_clock=0 makes the arm immediately effective (clock comparison in
   `endstop::tick` is `clock < ARM.arm_clock`).

D. Stepper oids are arbitrary u8s; the runtime indexes its
   `stepper_counts` array by oid. Since the test never actually steps
   the steppers, the counters stay at 0 in the trip snapshot — the test
   only asserts the snapshot *shape* (count, length), not non-zero
   values.

Invocation:
    tools/sim/build_sim_firmware.sh
    python3 tools/test_renode_endstop_e2e.py
"""

import argparse
import pathlib
import struct
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "tools"))

from kalico_host_io import KalicoHostIO  # noqa: E402
from test_renode_gpio_injection import RenodeMonitor  # noqa: E402

# Wire encoding: source record (11 bytes LE) per spec §3.1.
#   kind u8 | gpio u16 | polarity u8 | arm_policy u8 | sample_n u8
#   | velocity_axis u8 | v_min_q16 u32
def encode_source(kind, gpio, polarity, policy, sample_n, velocity_axis,
                  v_min_q16):
    return struct.pack(
        "<BHBBBBI",
        int(kind), int(gpio), int(polarity),
        int(policy), int(sample_n), int(velocity_axis),
        int(v_min_q16),
    )


# Stepper record: 1 byte (oid u8) per spec §3.1.
def encode_steppers(oids):
    return bytes(int(o) & 0xFF for o in oids)


def send_arm_endstop(io, arm_id, arm_clock, sources, stepper_oids):
    """Issue `runtime_arm_endstop` with raw blob params via msgproto.

    KalicoHostIO.send takes a string command that
    msgproto.MessageParser.create_command parses by splitting on
    whitespace; that path mishandles raw blobs containing arbitrary
    bytes. We use MessageFormat.encode_by_name directly (klippy/
    msgproto.py:223) to keep `%*s` blobs as raw bytes, then frame the
    encoded command bytes with the same low-level framing
    KalicoHostIO._send_raw uses.
    """
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
    # _send_raw expects a string; we sidestep it and use the same low-
    # level framing it does.
    _send_encoded(io, cmd_bytes)


def _send_encoded(io, cmd_bytes):
    """Frame and write raw msgproto command bytes (post-encode_by_name).

    Replicates KalicoHostIO._send_raw's framing without re-encoding the
    string. Maintains the 4-bit seq counter the same way.
    """
    import msgproto
    seq = io._next_seq()  # noqa: SLF001 — fixture-internal API.
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


def send_set_homed(io, homed):
    io.send("runtime_set_homed_state homed=%d" % (int(bool(homed)),))


def send_sim_set_endstop_pin(io, gpio, level):
    io.send(
        "runtime_sim_endstop_set_pin gpio=%d level=%d" %
        (int(gpio), int(bool(level)))
    )


# ---- Test cases ------------------------------------------------------------


def _wait_for_response_with_id(io, name, arm_id, timeout=5.0):
    """Drain `name` responses until one matches `arm_id`."""
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
    """Arm → assert pin → expect trip event → disarm and expect AlreadyTripped."""
    # Make the runtime tick the endstop poll path. With homed=1 and no
    # stream open, Engine::tick reaches poll_endstop_trip every period
    # (rust/runtime/src/engine.rs:323).
    send_set_homed(io, True)
    renode.advance_time(0.005)
    set_homed_resp = io.wait_for_response("kalico_set_homed_response",
                                          timeout=3.0)
    assert int(set_homed_resp["result"]) == 0, (
        "runtime_set_homed_state homed=1 failed: %s" % (set_homed_resp,)
    )
    print("[arm_trip] homed=1 acked")

    # TIM5 (40 kHz modulation timer) is gated until first-segment push in
    # production. The e2e test never pushes a segment, so we use the
    # sim-only `runtime_sim_engine_tick_start` shim to start TIM5 directly
    # so endstop::tick runs each modulation period.
    io.send("runtime_sim_engine_tick_start")
    renode.advance_time(0.005)
    tick_start_resp = io.wait_for_response(
        "runtime_sim_engine_tick_start_response", timeout=3.0
    )
    assert int(tick_start_resp["result"]) == 0
    print("[arm_trip] TIM5 enabled")

    arm_id = 0xA5A50001
    test_gpio = 17                       # arbitrary slot in PIN_LEVELS table
    sample_n = 1                         # trip on first asserted sample
    sources = [
        # (kind, gpio, polarity, policy, sample_n, velocity_axis, v_min_q16)
        # kind=0 Physical, polarity=1 active-high, policy=0 TripImmediately,
        # velocity_axis=0x01 X (irrelevant for TripImmediately).
        (0, test_gpio, 1, 0, sample_n, 0x01, 0),
    ]
    stepper_oids = [1, 2]                # arbitrary u8s; runtime indexes by oid.

    send_arm_endstop(io, arm_id=arm_id, arm_clock=0,
                     sources=sources, stepper_oids=stepper_oids)
    # Run for a few ms to let the MCU process the command and the
    # engine tick a few times in the Armed state with the pin still
    # low (proves no spurious trip).
    renode.advance_time(0.010)
    arm_resp = _wait_for_response_with_id(
        io, "kalico_arm_endstop_response", arm_id, timeout=5.0
    )
    assert int(arm_resp["status"]) == 0, (
        "expected status=0 (Armed), got %r" % (arm_resp,)
    )
    print("[arm_trip] arm acked: %r" % (arm_resp,))

    # Assert pin high through the sim shim, then advance time so the
    # modulation ISR observes it (TIM5 runs at 40 kHz → 25 µs / period;
    # 5 ms = 200 periods, plenty of margin).
    send_sim_set_endstop_pin(io, gpio=test_gpio, level=1)
    renode.advance_time(0.010)
    pin_resp = io.wait_for_response("runtime_sim_endstop_set_pin_response",
                                    timeout=3.0)
    assert int(pin_resp["result"]) == 0, (
        "set_pin failed: %s" % (pin_resp,)
    )
    print("[arm_trip] pin set high: %r" % (pin_resp,))
    # Give the trip-drain task a few cycles to publish the event.
    renode.advance_time(0.020)

    trip = _wait_for_response_with_id(
        io, "kalico_endstop_tripped", arm_id, timeout=5.0
    )
    print("[arm_trip] trip event: %r" % (trip,))

    assert int(trip["fmt_version"]) == 1, (
        "expected fmt_version=1, got %r" % (trip,)
    )
    assert int(trip["trip_source_idx"]) == 0, (
        "expected trip_source_idx=0, got %r" % (trip,)
    )
    assert int(trip["stepper_count"]) == len(stepper_oids), (
        "expected stepper_count=%d, got %r" %
        (len(stepper_oids), trip)
    )
    blob = trip.get("stepper_data", b"")
    # Output-frame parsing path renders the blob as a Python repr-style
    # string (`"b'\\x01...\\x00'"`) when re-emitting from the OutputFormat
    # tokenizer in kalico_host_io.py. Recover the raw bytes via ast.
    if isinstance(blob, str):
        import ast
        try:
            blob = ast.literal_eval(blob)
        except (ValueError, SyntaxError):
            blob = blob.encode("latin-1")
    if not isinstance(blob, (bytes, bytearray)):
        raise AssertionError("stepper_data not bytes-like: %r" % (blob,))
    expected_blob_len = 5 * len(stepper_oids)
    assert len(blob) == expected_blob_len, (
        "stepper_data len=%d expected %d" %
        (len(blob), expected_blob_len)
    )
    # Every per-stepper record is `oid u8 | step_count i32 LE`. The
    # runtime never stepped, so step_count must be 0; the oid must
    # match what we bound.
    for i, oid in enumerate(stepper_oids):
        rec = blob[i * 5:(i + 1) * 5]
        got_oid = rec[0]
        got_count = struct.unpack("<i", rec[1:5])[0]
        assert got_oid == oid, (
            "rec[%d] oid=%d expected %d" % (i, got_oid, oid)
        )
        assert got_count == 0, (
            "rec[%d] step_count=%d expected 0 (no motion in test)" %
            (i, got_count)
        )
    trip_clock = (int(trip["trip_clock_hi"]) << 32) | int(trip["trip_clock_lo"])
    assert trip_clock > 0, (
        "trip_clock=%d expected > 0" % (trip_clock,)
    )
    print("[arm_trip] trip event shape OK; trip_clock=%d" % (trip_clock,))

    # Disarm-after-trip must report AlreadyTripped (status=1).
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
    """Arm → disarm before any assertion → expect status=0 (Disarmed)."""
    arm_id = 0xA5A50002
    test_gpio = 18
    sources = [(0, test_gpio, 1, 0, 1, 0x01, 0)]
    stepper_oids = [3]
    # Make sure the pin is low before arming (we never set it high in
    # this case).
    send_sim_set_endstop_pin(io, gpio=test_gpio, level=0)
    renode.advance_time(0.005)
    io.wait_for_response("runtime_sim_endstop_set_pin_response", timeout=3.0)

    send_arm_endstop(io, arm_id=arm_id, arm_clock=0,
                     sources=sources, stepper_oids=stepper_oids)
    renode.advance_time(0.010)
    arm_resp = _wait_for_response_with_id(
        io, "kalico_arm_endstop_response", arm_id, timeout=5.0
    )
    assert int(arm_resp["status"]) == 0, (
        "expected status=0 (Armed), got %r" % (arm_resp,)
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


# ---- Driver ----------------------------------------------------------------


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
        # Verify both the production endstop wire surface and the sim
        # scaffold are present.
        names = set()
        for m in messages:
            # `get_messages` returns either a list of format strings or
            # `(msgid, msgtype, msgformat)` tuples depending on how the
            # MessageParser was populated; handle both shapes.
            if isinstance(m, str):
                names.add(m.split(None, 1)[0])
            elif isinstance(m, (tuple, list)) and len(m) >= 3:
                fmt = m[2]
                if isinstance(fmt, str):
                    names.add(fmt.split(None, 1)[0])
        for need in (
            "runtime_arm_endstop",
            "runtime_disarm_endstop",
            "runtime_set_homed_state",
            "runtime_sim_endstop_set_pin",
            "runtime_sim_engine_tick_start",
        ):
            assert need in names, (
                "%s missing from data dict; rebuild with CONFIG_KALICO_SIM=y" %
                (need,)
            )
        print("[e2e] data dict OK")

        # Pause emulation; we drive virtual time deterministically.
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
    # Use a dedicated TCP monitor port by default. With stdin transport
    # (default in test_renode_gpio_injection.py), Renode's [INFO] log
    # lines interleave with the `(h723)` prompt during back-to-back
    # `emulation RunFor` calls and break prompt-detection. The TCP
    # monitor channel keeps logs (which go to stdout) separate from
    # monitor command output (which goes to the TCP socket).
    p.add_argument("--monitor-tcp-port", type=int, default=3335)
    p.add_argument("--gdb-port", type=int, default=3333)
    p.add_argument("--identify-timeout", type=float, default=60.0)
    p.add_argument("--renode-log", default="/tmp/kalico-renode-endstop.log")
    p.add_argument("--keep-renode", action="store_true")
    args = p.parse_args()
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
