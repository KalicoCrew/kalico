#!/usr/bin/env python3
# Surface C — H723 soak test.
#
# Per Step-5 plan Task 29 (referenced by `Makefile.kalico:test-h723` recipe).
# Polls kalico_query_status every 1 second for `--minutes` minutes; FAILs
# immediately on any FAULT status. Idempotent — can be killed and restarted.
#
# Pre-flight: requires flashed H723 hardware with CONFIG_KALICO_RUNTIME=y
# and a representative segment chain pushed to the runtime (e.g. by running
# test_h723_first_light.py beforehand, or by a parallel slicer-driven workload).
import argparse
import logging
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from kalico_host_io import HostIoError, KalicoHostIO  # noqa: E402

STATUS_NAMES = {0: "IDLE", 1: "LOADED", 2: "RUNNING", 3: "FAULT"}


def main():
    p = argparse.ArgumentParser(description="kalico H723 soak test")
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=250000)
    p.add_argument(
        "--minutes",
        type=float,
        default=30.0,
        help="duration in minutes (default 30)",
    )
    p.add_argument(
        "--poll-interval",
        type=float,
        default=1.0,
        help="poll period in seconds",
    )
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    end_at = time.monotonic() + args.minutes * 60.0
    print(
        "Soaking %s for %.1f min (poll every %.1fs) ..."
        % (args.port, args.minutes, args.poll_interval)
    )
    io = KalicoHostIO(args.port, args.baud)
    poll_count = 0
    fault_count = 0
    try:
        while time.monotonic() < end_at:
            io.send("kalico_query_status")
            try:
                resp = io.wait_for_response("kalico_status", timeout=2.0)
            except HostIoError as exc:
                fault_count += 1
                raise SystemExit("FAIL: %s after %d polls" % (exc, poll_count))
            status = int(resp["status"])
            last_err = int(resp["last_err"])
            poll_count += 1
            if status == 3:  # FAULT
                fault_count += 1
                raise SystemExit(
                    "FAIL: FAULT detected after %d polls (last_err=%d)"
                    % (poll_count, last_err)
                )
            if poll_count % 60 == 0:
                print(
                    "  %d polls; last status=%s last_err=%d"
                    % (poll_count, STATUS_NAMES.get(status, status), last_err)
                )
            time.sleep(args.poll_interval)
        print(
            "PASS — soaked %.1f min, %d polls, no FAULT"
            % (args.minutes, poll_count)
        )
    finally:
        io.disconnect()


if __name__ == "__main__":
    main()
