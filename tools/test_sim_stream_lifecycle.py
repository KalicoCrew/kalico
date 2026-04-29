#!/usr/bin/env python3
"""
Step-6 Phase 7 integration test — exercises the §8.3 stream lifecycle
commands (open / arm / terminal / flush) end-to-end against the Renode
sim firmware.

Validates the WIRE PROTOCOL for each lifecycle command. The actual
engine-drain behaviour after `terminal` (which depends on Renode's
nondeterministic pacing relative to segment t_start clocks) is left to
the dedicated `test_sim_gate_a.py` engine-progress check; this test's
job is to confirm:

    - open(stream_id) returns OK + a credit_epoch.
    - open with same stream_id is idempotent (returns OK again).
    - push during StreamOpenPriming auto-tracks first_priming_t_start.
    - arm with first_t_start ≥ now+lead returns OK + echoes armed_t_start.
    - arm with same t_start_t0 is idempotent.
    - terminal in Running/Armed state returns OK; idempotent on same id.
    - flush returns OK and bumps credit_epoch.
    - clock_sync_request returns the widened-now MCU clock value.

Usage:
    bash tools/sim/build_sim_firmware.sh
    bash tools/sim/run_sim.sh &
    sleep 8
    python3 tools/test_sim_stream_lifecycle.py
"""
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from kalico_host_io import KalicoHostIO  # noqa: E402

CLOCK_FREQ = 520_000_000
TICK_HZ = 40_000
ONE_TICK_CYCLES = CLOCK_FREQ // TICK_HZ  # 13_000
# Short segment so trace traffic stays under USART2's drain capacity.
SEG_CYCLES = 100 * ONE_TICK_CYCLES

STREAM_ID = 42
ARM_LEAD_CYCLES = 2 * ONE_TICK_CYCLES


def push_segment(io, seg_id, curve_handle_packed, t_start_ticks, t_end_ticks,
                 timeout=5.0):
    cmd = (
        f"kalico_push_segment id={seg_id} curve_handle={curve_handle_packed} "
        f"t_start_hi={(t_start_ticks >> 32) & 0xFFFFFFFF} "
        f"t_start_lo={t_start_ticks & 0xFFFFFFFF} "
        f"t_end_hi={(t_end_ticks >> 32) & 0xFFFFFFFF} "
        f"t_end_lo={t_end_ticks & 0xFFFFFFFF} "
        f"kinematics=0"
    )
    io.send(cmd)
    r = io.wait_for_response("kalico_push_response", timeout)
    return int(r["result"])


def stream_open(io, stream_id, timeout=5.0):
    io.send(f"kalico_stream_open stream_id={stream_id}")
    r = io.wait_for_response("kalico_stream_open_response", timeout)
    return int(r["result"]), int(r.get("credit_epoch", 0))


def stream_arm(io, t_start_t0, arm_lead_cycles, timeout=5.0):
    cmd = (
        f"kalico_stream_arm "
        f"t_start_t0_lo={t_start_t0 & 0xFFFFFFFF} "
        f"t_start_t0_hi={(t_start_t0 >> 32) & 0xFFFFFFFF} "
        f"arm_lead_cycles={arm_lead_cycles}"
    )
    io.send(cmd)
    r = io.wait_for_response("kalico_stream_arm_response", timeout)
    armed_t = (int(r.get("armed_t_start_hi", 0)) << 32) | int(
        r.get("armed_t_start_lo", 0))
    return int(r["result"]), armed_t


def stream_terminal(io, segment_id, timeout=5.0):
    io.send(f"kalico_stream_terminal segment_id={segment_id}")
    r = io.wait_for_response("kalico_stream_terminal_response", timeout)
    return int(r["result"])


def stream_flush(io, timeout=5.0):
    io.send("kalico_stream_flush")
    r = io.wait_for_response("kalico_stream_flush_response", timeout)
    return int(r["result"]), int(r.get("credit_epoch", 0))


def clock_sync_request(io, request_id, timeout=5.0):
    io.send(
        f"kalico_clock_sync_request request_id={request_id} "
        f"host_send_time_lo=0 host_send_time_hi=0"
    )
    r = io.wait_for_response("kalico_clock_sync_response", timeout)
    return (int(r.get("mcu_clock_hi", 0)) << 32) | int(r.get("mcu_clock_lo", 0))


def query_diag(io, timeout=10.0):
    io.send("kalico_sim_diag")
    return io.wait_for_response("kalico_sim_diag_response", timeout)


def load_fixture(io, slot, fixture_id, timeout=5.0):
    io.send(f"kalico_load_fixture_curve slot={slot} fixture_id={fixture_id}")
    r = io.wait_for_response("kalico_load_fixture_response", timeout)
    return int(r["result"]), int(r.get("curve_handle_packed", 0))


def run_test():
    t0 = time.monotonic()
    io = KalicoHostIO("socket://localhost:3334", identify_timeout=60.0)
    try:
        diag = query_diag(io, timeout=10.0)
        print(
            f"  initial: status={diag.get('status')} "
            f"tick_counter={diag.get('tick_counter')}"
        )
        if int(diag.get("status", 255)) != 0:
            print(f"FAIL: initial status not IDLE: {diag}")
            return 1

        # Load 1 fixture (single curve reused across all 6 segments).
        rc, handle = load_fixture(io, 0, 0)
        if rc != 0:
            print(f"FAIL: load fixture → rc={rc}")
            return 1

        # 1. clock_sync_request before any push: widened_now should be 0
        #    (TIM5 not yet enabled).
        mcu_clock_pre = clock_sync_request(io, request_id=1)
        print(f"  clock_sync (pre-push): mcu_clock={mcu_clock_pre}")

        # 2. stream_open.
        rc, epoch_initial = stream_open(io, STREAM_ID)
        if rc != 0:
            print(f"FAIL: stream_open → rc={rc}")
            return 1
        print(f"  stream_open: rc=0 credit_epoch={epoch_initial}")

        # 3. Idempotent stream_open with same id.
        rc2, _ = stream_open(io, STREAM_ID)
        if rc2 != 0:
            print(f"FAIL: idempotent stream_open(same id) → rc={rc2}")
            return 1
        # Idempotent stream_open with DIFFERENT id should return -140.
        rc3, _ = stream_open(io, STREAM_ID + 1)
        if rc3 != -140:
            print(
                f"FAIL: stream_open(different id) expected -140 (state "
                f"violation), got {rc3}"
            )
            return 1
        print(f"  stream_open idempotency: same-id OK, diff-id rejected")

        # 4. Push 1 priming segment (large t_start so the engine doesn't
        #    drain it during this test). Renode pacing makes wall-clock
        #    elapsed seconds correspond to fewer MCU cycles, so a
        #    1-billion-cycle t_start sits comfortably ahead of mcu_now
        #    even after the test runs. The push auto-transitions
        #    StreamOpening → StreamOpenPriming and records
        #    fg.first_priming_segment_t_start.
        first_t_start = 1_000_000_000
        rc = push_segment(io, 1, handle, first_t_start, first_t_start + SEG_CYCLES)
        if rc != 0:
            print(f"FAIL: priming push → rc={rc}")
            return 1
        print(f"  pushed priming seg 1 t_start={first_t_start}")

        # 5. arm — first_t_start (1G) far ahead of mcu_now (small or zero
        #    initially); arm validation should pass.
        rc, armed_t = stream_arm(io, first_t_start, ARM_LEAD_CYCLES)
        if rc != 0:
            print(f"FAIL: arm → rc={rc} armed_t={armed_t}")
            return 1
        if armed_t != first_t_start:
            print(f"FAIL: armed_t={armed_t} != first_t_start={first_t_start}")
            return 1
        print(f"  arm: rc=0 armed_t_start={armed_t}")

        # 6. Idempotent arm with same t_start_t0.
        rc2, _ = stream_arm(io, first_t_start, ARM_LEAD_CYCLES)
        if rc2 != 0:
            print(f"FAIL: idempotent arm → rc={rc2}")
            return 1
        # Different t_start while armed → -140.
        rc3, _ = stream_arm(io, first_t_start + 1, ARM_LEAD_CYCLES)
        if rc3 != -140:
            print(f"FAIL: arm(different t) expected -140, got {rc3}")
            return 1
        print(f"  arm idempotency: same-t OK, diff-t rejected")

        # 7. terminal — set the terminal segment id while in Armed state.
        rc = stream_terminal(io, 1)
        if rc != 0:
            print(f"FAIL: terminal → rc={rc}")
            return 1
        print(f"  terminal: rc=0 (id=1)")

        # Idempotent terminal.
        rc2 = stream_terminal(io, 1)
        if rc2 != 0:
            print(f"FAIL: idempotent terminal → rc={rc2}")
            return 1
        # Different segment id → -140.
        rc3 = stream_terminal(io, 99)
        if rc3 != -140:
            print(f"FAIL: terminal(different id) expected -140, got {rc3}")
            return 1
        print(f"  terminal idempotency: same-id OK, diff-id rejected")

        # 8. clock_sync_request again — TIM5 has been running since the push,
        #    so widened_now should be > 0 now.
        mcu_clock_post = clock_sync_request(io, request_id=2)
        print(f"  clock_sync (post-push): mcu_clock={mcu_clock_post}")
        if mcu_clock_post <= mcu_clock_pre:
            print(
                f"FAIL: mcu_clock did not advance "
                f"(pre={mcu_clock_pre} post={mcu_clock_post})"
            )
            return 1

        # 9. flush — clears stream-machine state and bumps credit_epoch.
        rc, epoch_after = stream_flush(io)
        if rc != 0:
            print(f"FAIL: flush → rc={rc}")
            return 1
        if epoch_after <= epoch_initial:
            print(
                f"FAIL: credit_epoch did not advance "
                f"(before={epoch_initial} after={epoch_after})"
            )
            return 1
        print(
            f"  flush: rc=0 credit_epoch {epoch_initial} → {epoch_after}"
        )

        # 10. Post-flush sanity: a fresh stream_open should succeed (the
        #     flush should have reset internal state).
        rc, _ = stream_open(io, STREAM_ID + 100)
        if rc != 0:
            print(f"FAIL: post-flush stream_open → rc={rc}")
            return 1
        print(f"  post-flush stream_open(new id): rc=0")

        elapsed = time.monotonic() - t0
        print(f"PASS: stream_lifecycle ({elapsed:.1f}s wall-clock)")
        return 0
    finally:
        io.disconnect()


def main():
    sys.exit(run_test())


if __name__ == "__main__":
    main()
