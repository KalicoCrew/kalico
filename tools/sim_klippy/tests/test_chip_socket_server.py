import os
import socket
import time

import pytest

from tools.sim_klippy.orchestrator.chip_socket_server import (
    ChipSocketServer,
)

pytestmark = pytest.mark.sim_unit


def test_echo_handler_round_trips():
    sock_path = "/tmp/test_chip_socket_echo"
    if os.path.exists(sock_path):
        os.unlink(sock_path)

    def echo_handler(req: bytes) -> bytes:
        return req[::-1]

    server = ChipSocketServer(sock_path, echo_handler, chunk=4)
    server.start()
    try:
        for _ in range(50):
            if os.path.exists(sock_path):
                break
            time.sleep(0.01)
        client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        client.connect(sock_path)
        client.sendall(b"\x01\x02\x03\x04")
        reply = client.recv(4)
        assert reply == b"\x04\x03\x02\x01"
        client.close()
    finally:
        server.stop()
        if os.path.exists(sock_path):
            os.unlink(sock_path)


def test_framed_dispatches_by_cs_byte():
    sock_path = "/tmp/test_chip_socket_framed_dispatch"
    if os.path.exists(sock_path):
        os.unlink(sock_path)

    seen = []

    def handler(cs: int, payload: bytes) -> bytes:
        seen.append((cs, payload))
        return bytes([cs]) + payload[1:]

    server = ChipSocketServer(sock_path, handler, framed=True)
    server.start()
    try:
        for _ in range(50):
            if os.path.exists(sock_path):
                break
            time.sleep(0.01)
        client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        client.connect(sock_path)
        # Frame 1: cs=5, payload 5 bytes (TMC5160-style read).
        client.sendall(bytes([5, 5, 0x80, 0, 0, 0, 1]))
        reply_len = client.recv(1)
        assert reply_len == bytes([5])
        reply = client.recv(5)
        assert reply == bytes([5, 0, 0, 0, 1])
        # Frame 2: cs=3, payload 2 bytes (MAX31865-style read).
        client.sendall(bytes([3, 2, 0xC0, 0]))
        reply_len = client.recv(1)
        assert reply_len == bytes([2])
        reply = client.recv(2)
        assert reply == bytes([3, 0])
        client.close()
        time.sleep(0.05)
        assert seen == [(5, b"\x80\x00\x00\x00\x01"), (3, b"\xc0\x00")]
    finally:
        server.stop()
        if os.path.exists(sock_path):
            os.unlink(sock_path)


def test_framed_partial_read_recovery():
    sock_path = "/tmp/test_chip_socket_framed_partial"
    if os.path.exists(sock_path):
        os.unlink(sock_path)

    def handler(cs: int, payload: bytes) -> bytes:
        return payload

    server = ChipSocketServer(sock_path, handler, framed=True)
    server.start()
    try:
        for _ in range(50):
            if os.path.exists(sock_path):
                break
            time.sleep(0.01)
        client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        client.connect(sock_path)
        client.sendall(bytes([7]))
        time.sleep(0.02)
        client.sendall(bytes([3, 0xAA]))
        time.sleep(0.02)
        client.sendall(bytes([0xBB, 0xCC]))
        reply_len = client.recv(1)
        assert reply_len == bytes([3])
        reply = client.recv(3)
        assert reply == bytes([0xAA, 0xBB, 0xCC])
        client.close()
    finally:
        server.stop()
        if os.path.exists(sock_path):
            os.unlink(sock_path)


def test_handler_with_empty_reply_does_not_send():
    sock_path = "/tmp/test_chip_socket_no_reply"
    if os.path.exists(sock_path):
        os.unlink(sock_path)

    def silent_handler(req: bytes) -> bytes:
        return b""

    server = ChipSocketServer(sock_path, silent_handler, chunk=8)
    server.start()
    try:
        for _ in range(50):
            if os.path.exists(sock_path):
                break
            time.sleep(0.01)
        client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        client.connect(sock_path)
        client.sendall(b"\x05\x00\x80\x00\x00\x00\x05\x42")
        client.settimeout(0.2)
        try:
            data = client.recv(1)
            assert data == b"", f"unexpected reply: {data!r}"
        except socket.timeout:
            pass  # expected — no reply
        client.close()
    finally:
        server.stop()
        if os.path.exists(sock_path):
            os.unlink(sock_path)
