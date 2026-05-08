"""End-to-end: spin up TMC5160 / TMC2209 emulators behind a
ChipSocketServer, connect a Unix socket, exchange bytes, assert the
chip latched the value.

Catches: ChipSocketServer chunk size mismatches with the chip framing,
threading-loop quirks, the round-trip path TMC firmware will use."""
import os
import socket
import time
import pytest

from tools.sim_klippy.orchestrator.chip_socket_server import ChipSocketServer
from tools.sim_klippy.orchestrator.tmc5160_emulator import TMC5160Emulator
from tools.sim_klippy.orchestrator.tmc2209_emulator import (
    TMC2209Emulator, crc8, _decode_uart_bits, _encode_uart_bits,
)


def _wait_for_socket(path, timeout=0.5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if os.path.exists(path):
            return True
        time.sleep(0.005)
    return False


def test_tmc5160_via_socket():
    """5-byte SPI: write GLOBALSCALER=200, double-read returns 200."""
    sock_path = "/tmp/test_tmc5160_via_socket"
    if os.path.exists(sock_path):
        os.unlink(sock_path)
    chip = TMC5160Emulator()
    server = ChipSocketServer(sock_path, chip.transfer, chunk=5)
    server.start()
    try:
        assert _wait_for_socket(sock_path)
        client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        client.connect(sock_path)
        # Write GLOBALSCALER (0x0B) = 200
        client.sendall(bytes([0x80 | 0x0B, 0, 0, 0, 200]))
        client.recv(5)
        # Latched read: first read fetches 200 into latch
        client.sendall(bytes([0x0B, 0, 0, 0, 0]))
        client.recv(5)
        # Second read returns it
        client.sendall(bytes([0x0B, 0, 0, 0, 0]))
        reply = client.recv(5)
        assert reply[4] == 200
    finally:
        server.stop()
        if os.path.exists(sock_path):
            os.unlink(sock_path)


def test_tmc2209_via_socket_read_request():
    """5-byte wire-form read request → 10-byte wire-form reply (UART
    start/stop framing on each logical byte)."""
    sock_path = "/tmp/test_tmc2209_via_socket_read"
    if os.path.exists(sock_path):
        os.unlink(sock_path)
    chip = TMC2209Emulator(slave_addr=0)
    # chunk=5 matches the wire-format read-request size (4 logical bytes
    # × 10 wire bits / 8 = 5 bytes). Writes are 10 wire bytes; this test
    # only exercises reads to keep the chunk constant.
    server = ChipSocketServer(sock_path, chip.handle, chunk=5)
    server.start()
    try:
        assert _wait_for_socket(sock_path)
        client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        client.connect(sock_path)
        body = bytes([0x05, 0x00, 0x00])  # GCONF read from slave 0
        logical = body + bytes([crc8(body)])
        client.sendall(_encode_uart_bits(logical))
        reply_wire = client.recv(10)
        assert len(reply_wire) == 10
        reply = _decode_uart_bits(reply_wire)
        assert len(reply) == 8
        assert reply[0] == 0x05
        assert reply[1] == 0xFF
        assert reply[2] == 0x00
        assert reply[7] == crc8(reply[:7])
    finally:
        server.stop()
        if os.path.exists(sock_path):
            os.unlink(sock_path)
