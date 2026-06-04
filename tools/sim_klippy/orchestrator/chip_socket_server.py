"""Async-ish Unix-socket server for chip emulators.

Two wire modes:

- Default (``framed=False``): each client message is a fixed-length
  ``chunk`` byte sequence; the handler returns a reply of equal length.
  Used by tmcuart's bit-banged 10-byte-per-logical-byte path.

- Framed (``framed=True``): each client message is
  ``[cs:1][len:1][payload:len]``; the handler is called with
  ``(cs, payload)`` and returns the reply payload. The server frames the
  reply as ``[len:1][reply:len]``. Used by sim SPI buses that multiplex
  multiple chips behind a single socket — the firmware-side spidev sim
  path emits the CS byte from the active config_spi pin, and the
  orchestrator dispatches to the right per-chip emulator.

Threaded model: one accept thread, one worker thread per connection.
Sufficient for our handful of chip stubs."""

from __future__ import annotations

import os
import socket
import threading
from typing import Callable, Union

UnframedHandler = Callable[[bytes], bytes]
FramedHandler = Callable[[int, bytes], bytes]


class ChipSocketServer:
    def __init__(
        self,
        path: str,
        handler: Union[UnframedHandler, FramedHandler],
        chunk: int = 16,
        framed: bool = False,
    ):
        self._path = path
        self._handler = handler
        self._chunk = chunk
        self._framed = framed
        self._sock = None
        self._accept_thread = None
        self._stop = threading.Event()

    def start(self) -> None:
        if os.path.exists(self._path):
            os.unlink(self._path)
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._sock.bind(self._path)
        self._sock.listen(4)
        self._sock.settimeout(0.1)
        self._accept_thread = threading.Thread(
            target=self._accept_loop, daemon=True
        )
        self._accept_thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        if self._accept_thread:
            self._accept_thread.join(timeout=1.0)

    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                client, _ = self._sock.accept()
            except (socket.timeout, OSError):
                continue
            t = threading.Thread(
                target=self._serve,
                args=(client,),
                daemon=True,
            )
            t.start()

    def _serve(self, client: socket.socket):
        client.settimeout(1.0)
        try:
            if self._framed:
                self._serve_framed(client)
            else:
                self._serve_unframed(client)
        except (socket.timeout, ConnectionResetError, OSError):
            pass
        finally:
            try:
                client.close()
            except OSError:
                pass

    def _serve_unframed(self, client: socket.socket):
        # client.settimeout(1.0) is set in _serve so a stopped server can
        # exit promptly. recv() raising socket.timeout is NOT a fatal
        # condition — clients legitimately go idle between requests
        # (e.g. tmc.py's drv_status periodic queries run every 1.0 s,
        # exactly racing the timeout). Re-loop on timeout; only break on
        # EOF or genuine I/O error.
        while not self._stop.is_set():
            try:
                data = client.recv(self._chunk)
            except socket.timeout:
                continue
            if not data:
                break
            reply = self._handler(data)
            if reply:
                client.sendall(reply)

    def _recv_exactly(self, client: socket.socket, n: int) -> bytes:
        """Receive exactly n bytes or return b'' on EOF.

        Robust to TCP-style partial reads: keeps recv()'ing until n
        bytes are accumulated. Returns b'' if the peer closes.
        """
        buf = bytearray()
        while len(buf) < n:
            if self._stop.is_set():
                return b""
            try:
                chunk = client.recv(n - len(buf))
            except socket.timeout:
                # Allow the stop event to interrupt long-idle connections.
                continue
            if not chunk:
                return b""
            buf.extend(chunk)
        return bytes(buf)

    def _serve_framed(self, client: socket.socket):
        while not self._stop.is_set():
            hdr = self._recv_exactly(client, 2)
            if len(hdr) < 2:
                break
            cs = hdr[0]
            length = hdr[1]
            if length == 0:
                # Zero-length payload is invalid — the firmware never sends it.
                break
            payload = self._recv_exactly(client, length)
            if len(payload) < length:
                break
            reply = self._handler(cs, bytes(payload))
            # SPI is symmetric: reply length must equal request length.
            if len(reply) != length:
                raise ValueError(
                    f"framed handler returned {len(reply)}B reply for "
                    f"{length}B request (cs={cs:#x})"
                )
            client.sendall(bytes([len(reply)]) + reply)
