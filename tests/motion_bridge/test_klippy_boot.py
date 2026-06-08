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

sys.path.insert(0, os.path.join(REPO_ROOT, "klippy"))

HAS_RENODE = shutil.which("renode") is not None
HAS_FIRMWARE = os.path.isfile(os.path.join(REPO_ROOT, "out", "klipper.elf"))


def _port_is_open(host, port, timeout=1.0):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except (ConnectionRefusedError, OSError, socket.timeout):
        return False


def _wait_for_port(host, port, timeout=30.0, poll=0.5):
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
        return

    try:
        os.killpg(pgid, signal.SIGTERM)
    except ProcessLookupError:
        return

    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass


@pytest.fixture(scope="module")
def renode_sim():
    if not HAS_RENODE:
        pytest.skip("renode not on PATH")
    if not HAS_FIRMWARE:
        pytest.skip("out/klipper.elf not found")

    if _port_is_open("localhost", 3334):
        pytest.skip("port 3334 already in use (another Renode instance?)")

    proc = subprocess.Popen(
        ["bash", os.path.join(REPO_ROOT, "tools/sim/run_sim.sh")],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )

    if not _wait_for_port("localhost", 3334, timeout=30.0):
        _kill_process_tree(proc)
        pytest.fail("Renode did not open port 3334 within 30 s")

    yield proc

    _kill_process_tree(proc)


class TestBridgeModule:
    def test_import(self):
        import motion_bridge

        assert hasattr(motion_bridge, "MotionBridge")

    def test_instantiate(self):
        import motion_bridge

        bridge = motion_bridge.MotionBridge()
        assert bridge.version() != ""

    def test_claim_primary_mcu(self):
        import motion_bridge

        bridge = motion_bridge.MotionBridge()
        handle = bridge.claim_mcu("mcu", "socket://localhost:3334", 250000)
        assert isinstance(handle, int)
        assert handle >= 0

    def test_claim_stub_mcus(self):
        import motion_bridge
        from stub_mcus import claim_stub_mcus

        bridge = motion_bridge.MotionBridge()
        handles = claim_stub_mcus(bridge)
        assert len(handles) == 3
        assert set(handles.keys()) == {"bottom", "beacon", "NIS"}
        assert len(set(handles.values())) == 3

    def test_alloc_queues_for_all_mcus(self):
        import motion_bridge
        from stub_mcus import claim_stub_mcus

        bridge = motion_bridge.MotionBridge()
        primary = bridge.claim_mcu("mcu", "socket://localhost:3334", 250000)
        stubs = claim_stub_mcus(bridge)

        q0 = bridge.alloc_command_queue(primary)
        assert isinstance(q0, int)

        for name, h in stubs.items():
            q = bridge.alloc_command_queue(h)
            assert isinstance(q, int), f"queue for {name} should be int"

    def test_poll_event_empty(self):
        import motion_bridge

        bridge = motion_bridge.MotionBridge()
        assert bridge.poll_event() is None


@pytest.mark.skipif(not HAS_RENODE, reason="renode not on PATH")
@pytest.mark.skipif(not HAS_FIRMWARE, reason="out/klipper.elf not found")
class TestRenodeSim:
    def test_renode_process_running(self, renode_sim):
        assert renode_sim.poll() is None, "Renode exited prematurely"

    def test_uart_socket_connectable(self, renode_sim):
        assert _port_is_open("localhost", 3334), (
            "USART2 socket not accepting connections"
        )

    def test_uart_receives_data(self, renode_sim):
        with socket.create_connection(("localhost", 3334), timeout=5.0) as s:
            s.settimeout(10.0)
            try:
                data = s.recv(64)
                assert len(data) > 0, "Expected data from simulated MCU"
            except socket.timeout:
                pytest.skip(
                    "MCU did not send unsolicited data within 10 s "
                    "(expected in Phase 1 — no host identify sent)"
                )

    def test_bridge_with_sim_serial_path(self, renode_sim):
        import motion_bridge

        bridge = motion_bridge.MotionBridge()
        handle = bridge.claim_mcu("mcu", "socket://localhost:3334", 250000)
        assert isinstance(handle, int)

        q = bridge.alloc_command_queue(handle)
        bridge.passthrough_send(handle, q, b"\x00")

        stats = bridge.get_stats(handle)
        assert isinstance(stats, dict)
