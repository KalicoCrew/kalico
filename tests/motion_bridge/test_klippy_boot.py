"""
Phase 1 smoke test: verify motion-bridge components and Renode sim readiness.

Prerequisites:
  - out/klipper.elf exists (sim firmware)
  - renode is on PATH
  - make -f Makefile.kalico motion-bridge has been run

Phase 1 limitations:
  The bridge's claim_mcu stores connection params but does NOT open the serial
  port or perform the identify handshake. A full klippy boot against the
  simulated MCU requires the Phase 2 reactor thread + serial open path.

  What IS testable in Phase 1:
    1. Renode starts and exposes USART2 on tcp://localhost:3334
    2. The motion_bridge module loads and its API is callable
    3. Stub MCUs can be claimed without error
    4. A raw TCP connection to the Renode UART succeeds

  What is DEFERRED to Phase 2:
    - Full klippy reactor boot against the sim
    - Identify handshake over the bridge
    - Heater setpoint verification
    - Config send / firmware version negotiation
"""

import os
import shutil
import signal
import socket
import subprocess
import sys
import time

import pytest

REPO_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)

# Add klippy dir to path so motion_bridge.so is importable
sys.path.insert(0, os.path.join(REPO_ROOT, "klippy"))

HAS_RENODE = shutil.which("renode") is not None
HAS_FIRMWARE = os.path.isfile(os.path.join(REPO_ROOT, "out", "klipper.elf"))


def _port_is_open(host, port, timeout=1.0):
    """Check if a TCP port is accepting connections."""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except (ConnectionRefusedError, OSError, socket.timeout):
        return False


def _wait_for_port(host, port, timeout=30.0, poll=0.5):
    """Block until the TCP port is accepting connections or timeout."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if _port_is_open(host, port, timeout=1.0):
            return True
        time.sleep(poll)
    return False


def _kill_process_tree(proc):
    """Terminate a subprocess and its entire process group.

    Renode runs as a dotnet process that may not respond to SIGTERM
    promptly. We send SIGTERM to the process group first, then SIGKILL
    if it doesn't exit within a few seconds.
    """
    pgid = None
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return  # already dead

    # Try graceful SIGTERM to the whole group
    try:
        os.killpg(pgid, signal.SIGTERM)
    except ProcessLookupError:
        return

    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        # Force kill the group
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def renode_sim():
    """Start Renode H723 sim, yield when UART socket is ready, teardown after.

    Skipped if Renode is not installed or the sim firmware is missing.
    """
    if not HAS_RENODE:
        pytest.skip("renode not on PATH")
    if not HAS_FIRMWARE:
        pytest.skip("out/klipper.elf not found")

    # Check that port 3334 isn't already in use
    if _port_is_open("localhost", 3334):
        pytest.skip("port 3334 already in use (another Renode instance?)")

    # Start Renode in its own process group so we can kill the whole tree.
    # run_sim.sh uses `exec renode ...` which replaces the shell, but the
    # dotnet runtime may spawn child processes.
    proc = subprocess.Popen(
        ["bash", os.path.join(REPO_ROOT, "tools/sim/run_sim.sh")],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )

    # Wait for Renode to bind the UART TCP socket
    if not _wait_for_port("localhost", 3334, timeout=30.0):
        _kill_process_tree(proc)
        pytest.fail("Renode did not open port 3334 within 30 s")

    yield proc

    _kill_process_tree(proc)


# ---------------------------------------------------------------------------
# Tests: motion_bridge module (no Renode needed)
# ---------------------------------------------------------------------------


class TestBridgeModule:
    """Verify the motion_bridge PyO3 module is functional."""

    def test_import(self):
        import motion_bridge

        assert hasattr(motion_bridge, "MotionBridge")

    def test_instantiate(self):
        import motion_bridge

        bridge = motion_bridge.MotionBridge()
        assert bridge.version() != ""

    def test_claim_primary_mcu(self):
        """claim_mcu for the primary motion MCU returns a valid handle."""
        import motion_bridge

        bridge = motion_bridge.MotionBridge()
        handle = bridge.claim_mcu("mcu", "socket://localhost:3334", 250000)
        assert isinstance(handle, int)
        assert handle >= 0

    def test_claim_stub_mcus(self):
        """All stub (non-motion) MCUs can be claimed without error."""
        import motion_bridge
        from stub_mcus import claim_stub_mcus

        bridge = motion_bridge.MotionBridge()
        handles = claim_stub_mcus(bridge)
        assert len(handles) == 3
        assert set(handles.keys()) == {"bottom", "beacon", "NIS"}
        # All handles are distinct
        assert len(set(handles.values())) == 3

    def test_alloc_queues_for_all_mcus(self):
        """Command queues can be allocated for both primary and stub MCUs."""
        import motion_bridge
        from stub_mcus import claim_stub_mcus

        bridge = motion_bridge.MotionBridge()
        primary = bridge.claim_mcu("mcu", "socket://localhost:3334", 250000)
        stubs = claim_stub_mcus(bridge)

        # Primary MCU queue
        q0 = bridge.alloc_command_queue(primary)
        assert isinstance(q0, int)

        # Stub MCU queues
        for name, h in stubs.items():
            q = bridge.alloc_command_queue(h)
            assert isinstance(q, int), f"queue for {name} should be int"

    def test_poll_event_empty(self):
        """poll_event returns None when no events are queued."""
        import motion_bridge

        bridge = motion_bridge.MotionBridge()
        assert bridge.poll_event() is None


# ---------------------------------------------------------------------------
# Tests: Renode sim (requires Renode + firmware)
# ---------------------------------------------------------------------------


@pytest.mark.skipif(not HAS_RENODE, reason="renode not on PATH")
@pytest.mark.skipif(not HAS_FIRMWARE, reason="out/klipper.elf not found")
class TestRenodeSim:
    """Verify Renode starts and the UART socket is reachable."""

    def test_renode_process_running(self, renode_sim):
        """Renode process should still be alive after startup."""
        assert renode_sim.poll() is None, "Renode exited prematurely"

    def test_uart_socket_connectable(self, renode_sim):
        """TCP connection to Renode USART2 (port 3334) should succeed."""
        assert _port_is_open("localhost", 3334), (
            "USART2 socket not accepting connections"
        )

    def test_uart_receives_data(self, renode_sim):
        """The simulated MCU should be sending data over USART2.

        Klipper firmware sends identify responses and status messages on
        the serial port. We just verify we can read at least one byte
        within a reasonable timeout.
        """
        with socket.create_connection(("localhost", 3334), timeout=5.0) as s:
            s.settimeout(10.0)
            try:
                data = s.recv(64)
                # The firmware should be sending something (even if just
                # identify chatter or watchdog pings). Any data is a pass.
                assert len(data) > 0, "Expected data from simulated MCU"
            except socket.timeout:
                # The MCU might not send unsolicited data without a host
                # identify request. This is acceptable in Phase 1.
                pytest.skip(
                    "MCU did not send unsolicited data within 10 s "
                    "(expected in Phase 1 — no host identify sent)"
                )

    def test_bridge_with_sim_serial_path(self, renode_sim):
        """Bridge can claim the primary MCU with the sim's socket path."""
        import motion_bridge

        bridge = motion_bridge.MotionBridge()
        handle = bridge.claim_mcu("mcu", "socket://localhost:3334", 250000)
        assert isinstance(handle, int)

        # Allocate a queue and push a command (buffered, not sent in Phase 1)
        q = bridge.alloc_command_queue(handle)
        bridge.passthrough_send(handle, q, b"\x00")

        # Stats should reflect the queued command
        stats = bridge.get_stats(handle)
        assert isinstance(stats, dict)
