"""Beacon stub scaffolding tests.

These don't validate a faithful beacon protocol — they validate that the
PTY is reachable, the sample stream produces output at approximately the
configured rate, and that received bytes are logged.

Real protocol fidelity (msgproto framing, beacon_data encoding, NVM-read
responses) is deferred to Phase 7 once we have the actual traffic capture
from a real boot run.

Design note on PTY reads in tests
----------------------------------
Reading from a PTY slave fd opened externally (via os.open) does NOT
reliably see data written to the master on macOS — the tty line discipline
only routes master→slave for the fd created by pty.openpty(), not for a
fresh os.open() on the same device name.  Tests therefore validate
observable side-effects (log file contents, stub.tx_sample_count counter,
stub.rx_byte_count counter) rather than reading raw bytes from the PTY
slave.  The counters are written only by the loop thread so they're
effectively monotonic once the loop is running.
"""

import os
import time

from tools.sim_klippy.orchestrator.beacon_serial_stub import BeaconSerialStub


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _wait_for_path(path: str, timeout: float = 1.0) -> bool:
    """Poll until *path* exists (symlink or regular file), or timeout."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if os.path.exists(path):
            return True
        time.sleep(0.01)
    return False


def _wait_for(condition, timeout: float = 2.0, poll: float = 0.02) -> bool:
    """Poll *condition()* until it returns truthy, or timeout."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(poll)
    return False


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_pty_appears(tmp_path):
    """start() creates the PTY symlink at the configured path."""
    pty_path = str(tmp_path / "beacon_pty")
    stub = BeaconSerialStub(pty_path)
    stub.start()
    try:
        assert _wait_for_path(pty_path), (
            f"PTY symlink {pty_path!r} did not appear within 1 s"
        )
        # Confirm it points to a real tty device.
        assert os.path.islink(pty_path)
        target = os.readlink(pty_path)
        assert target.startswith("/dev/pts/") or target.startswith("/dev/tty"), (
            f"symlink target {target!r} doesn't look like a PTY device"
        )
    finally:
        stub.stop()


def test_pty_removed_after_stop(tmp_path):
    """stop() removes the symlink at the configured path."""
    pty_path = str(tmp_path / "beacon_stop_test")
    stub = BeaconSerialStub(pty_path)
    stub.start()
    assert _wait_for_path(pty_path)
    stub.stop()
    assert not os.path.exists(pty_path), (
        "PTY symlink should be removed after stop()"
    )


def test_sample_stream_emits_at_rate(tmp_path):
    """start_sample_stream() increments tx_sample_count at roughly rate_hz."""
    pty_path = str(tmp_path / "beacon_samples")
    log_path = str(tmp_path / "beacon.log")
    rate_hz = 200
    stub = BeaconSerialStub(pty_path, log_path=log_path)
    stub.start_sample_stream(z_target_mm=10.0, rate_hz=rate_hz)
    try:
        assert _wait_for_path(pty_path), "PTY symlink did not appear"
        # Wait until at least 50 samples have been emitted (should take ~250ms
        # at 200 Hz, with generous 3 s timeout for slow CI).
        assert _wait_for(lambda: stub.tx_sample_count >= 50, timeout=3.0), (
            f"expected ≥50 samples in 3 s at {rate_hz} Hz, "
            f"got {stub.tx_sample_count}"
        )
        # Log should also contain [tx] entries for the emitted frames.
        assert os.path.exists(log_path), "log file was not created"
        log_contents = open(log_path, "r").read()
        assert "[tx]" in log_contents, "no [tx] entry in log for emitted samples"
        assert "beacon_data" in log_contents, (
            "log should contain placeholder beacon_data frame text"
        )
    finally:
        stub.stop()


def test_received_bytes_logged(tmp_path):
    """Bytes written to the PTY slave appear as [rx] lines in the log."""
    pty_path = str(tmp_path / "beacon_log_test")
    log_path = str(tmp_path / "beacon_traffic.log")
    stub = BeaconSerialStub(pty_path, log_path=log_path)
    stub.start()
    try:
        assert _wait_for_path(pty_path), "PTY symlink did not appear"
        # Write bytes to the slave side of the PTY.  The slave fd is kept
        # open inside the stub (self._slave_fd) so we write to the same
        # device name; that write enters the PTY line discipline and becomes
        # readable on the master fd that the loop polls.
        slave_fd = os.open(
            pty_path,
            os.O_WRONLY | os.O_NOCTTY | os.O_NONBLOCK,
        )
        try:
            os.write(slave_fd, b"\x01\x02\x03BEACON_TEST\n")
        finally:
            os.close(slave_fd)
        # Wait until the loop has processed the bytes.
        assert _wait_for(lambda: stub.rx_byte_count > 0, timeout=2.0), (
            "stub rx_byte_count stayed 0 after writing to PTY slave"
        )
        assert os.path.exists(log_path), "log file was not created"
        log_contents = open(log_path, "r").read()
        assert "[rx]" in log_contents, (
            "no [rx] entry in log after writing bytes to PTY slave"
        )
    finally:
        stub.stop()


def test_set_z_reflected_in_log(tmp_path):
    """set_z() changes the Z value reported in logged tx frames."""
    pty_path = str(tmp_path / "beacon_setz")
    log_path = str(tmp_path / "beacon_setz.log")
    stub = BeaconSerialStub(pty_path, log_path=log_path)
    stub.start_sample_stream(z_target_mm=1.0, rate_hz=100)
    try:
        assert _wait_for_path(pty_path)
        # Wait for a few samples near z=1.0.
        assert _wait_for(lambda: stub.tx_sample_count >= 5, timeout=2.0)
        count_before = stub.tx_sample_count
        stub.set_z(5.0)
        # Wait for at least 5 more samples after the change.
        assert _wait_for(
            lambda: stub.tx_sample_count >= count_before + 5, timeout=2.0
        )
        log_contents = open(log_path, "r").read()
        # Frames at both target values should appear in the log.
        assert "z=1." in log_contents or "z=0." in log_contents, (
            "expected frames near z=1.0 in log before set_z()"
        )
        assert "z=5." in log_contents or "z=4." in log_contents, (
            "expected frames near z=5.0 in log after set_z(5.0)"
        )
    finally:
        stub.stop()


def test_start_is_idempotent(tmp_path):
    """Calling start() twice does not crash or create duplicate symlinks."""
    pty_path = str(tmp_path / "beacon_idem")
    stub = BeaconSerialStub(pty_path)
    stub.start()
    stub.start()  # second call; should be a no-op
    try:
        assert _wait_for_path(pty_path)
        assert os.path.islink(pty_path)
    finally:
        stub.stop()
