#!/usr/bin/env python3
"""Minimal TMC5160 SPI register emulator for sim tests.

Listens on a Unix socket at $KALICO_SIM_SOCK_DIR/spi_cs_<chip>_<line>.
The sim intercept shim (libsim_intercept.so) connects when a CS pin is
asserted and an SPI transfer occurs. Protocol: raw byte relay — the shim
writes N bytes, the emulator reads them (TMC5160 request datagram) and
replies with N bytes (TMC5160 response datagram).

TMC5160 SPI protocol (40-bit, MSB-first):
  Request:  [RW:1 | ADDR:7] [DATA:32]
  Response: [STATUS:8] [DATA:32]

For reads (bit 7 = 0): returns the shadow register value.
For writes (bit 7 = 1): stores the value in the shadow register.
"""

import os
import socket
import struct
import sys
import threading

TMC5160_DEFAULTS = {
    0x00: 0x00000009,  # GCONF
    0x01: 0x00000000,  # GSTAT
    0x04: 0x00000000,  # IOIN
    0x06: 0x00000000,  # FACTORY_CONF — unused but sometimes read
    0x10: 0x00061F0A,  # IHOLD_IRUN
    0x11: 0x0000000A,  # TPOWERDOWN
    0x12: 0x00000000,  # TSTEP
    0x13: 0x00000000,  # TPWMTHRS
    0x14: 0x00000000,  # TCOOLTHRS
    0x15: 0x00000000,  # THIGH
    0x2D: 0x00000000,  # XDIRECT
    0x60: 0x00000000,  # MSLUT0
    0x69: 0x00000000,  # MSLUTSTART
    0x6A: 0x00000000,  # MSCNT
    0x6B: 0x00000000,  # MSCURACT
    0x6C: 0x10410150,  # CHOPCONF
    0x6D: 0x00000000,  # COOLCONF
    0x6E: 0x00000000,  # DCCTRL
    0x6F: 0x00000000,  # DRV_STATUS
    0x70: 0xC40C001E,  # PWMCONF
    0x71: 0x00000000,  # PWM_SCALE
    0x72: 0x00000000,  # PWM_AUTO
}


def handle_client(conn, regs):
    """Serve one SPI connection (one CS assertion).

    The shim opens a new connection per SPI operation. For ioctl-based
    transfers it writes N bytes then reads N bytes; for write()-based
    sends it writes N bytes then drops the connection. We respond
    immediately per 5-byte datagram so the ioctl path gets its data
    without waiting for EOF.
    """
    conn.settimeout(0.5)
    try:
        buf = b""
        while True:
            try:
                chunk = conn.recv(4096)
            except socket.timeout:
                break
            if not chunk:
                break
            buf += chunk
            while len(buf) >= 5:
                frame = buf[:5]
                buf = buf[5:]
                addr_byte = frame[0]
                is_write = bool(addr_byte & 0x80)
                reg_addr = addr_byte & 0x7F
                value = struct.unpack(">I", frame[1:5])[0]
                if is_write:
                    regs[reg_addr] = value
                resp = struct.pack(">BI", 0x00, regs.get(reg_addr, 0))
                try:
                    conn.sendall(resp)
                except (BrokenPipeError, ConnectionResetError):
                    return
    except (BrokenPipeError, ConnectionResetError, socket.timeout):
        pass
    finally:
        conn.close()


def run_emulator(sock_path):
    regs = dict(TMC5160_DEFAULTS)
    if os.path.exists(sock_path):
        os.unlink(sock_path)
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(sock_path)
    srv.listen(8)
    srv.settimeout(0.5)
    while True:
        try:
            conn, _ = srv.accept()
            t = threading.Thread(target=handle_client, args=(conn, regs),
                                 daemon=True)
            t.start()
        except socket.timeout:
            continue
        except Exception:
            break


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: tmc5160_emu.py <socket_path>")
        sys.exit(1)
    run_emulator(sys.argv[1])
