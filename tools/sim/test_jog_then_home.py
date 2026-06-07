#!/usr/bin/env python3
import argparse
import json
import socket
import sys
import threading
import time

import pytest

# Renode-driven __main__ script (drives a Renode monitor); no pytest test
# functions. Tagged needs_renode so it is honestly excluded from CI. Run
# directly: `python3 <this file> --api ... --renode-monitor ...`.
pytestmark = pytest.mark.needs_renode


def gcode(api_socket, script, timeout=60.0):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(timeout)
    s.connect(api_socket)
    s.sendall(
        json.dumps(
            {
                "id": 1,
                "method": "gcode/script",
                "params": {"script": script},
            }
        ).encode()
        + b"\x03"
    )
    buf = b""
    while True:
        c = s.recv(4096)
        if not c:
            break
        buf += c
        if b"\x03" in buf:
            break
    s.close()
    out = buf.split(b"\x03", 1)[0]
    return json.loads(out.decode()) if out else {}


class RenodeMonitor:
    def __init__(self, addr):
        host, port = addr.rsplit(":", 1)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, int(port)))
        self.sock.settimeout(5.0)
        time.sleep(0.5)
        try:
            self.sock.recv(4096)
        except socket.timeout:
            pass

    def cmd(self, command):
        self.sock.sendall((command + "\n").encode())
        time.sleep(0.3)
        try:
            return self.sock.recv(8192).decode(errors="replace")
        except socket.timeout:
            return ""

    def inject_gpio(self, machine, port_addr, bit):
        self.cmd(f'mach set "{machine}"')
        cur = self.cmd(f"sysbus ReadDoubleWord {port_addr}")
        try:
            val = int(cur.strip().split("\n")[-1].strip(), 0)
        except (ValueError, IndexError):
            val = 0
        val |= 1 << bit
        self.cmd(f"sysbus WriteDoubleWord {port_addr} {val}")
        check = self.cmd(f"sysbus ReadDoubleWord {hex(port_addr + 0x10)}")
        print(f"  [inject] IDR after write: {check.strip()}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--api", required=True)
    parser.add_argument("--renode-monitor", required=True)
    args = parser.parse_args()

    mon = RenodeMonitor(args.renode_monitor)

    # GPIOC ODR = 0x58020814, IDR = 0x58020810, bit 6 = PC6
    GPIOC_ODR = 0x58020814
    PC6_BIT = 6

    print("=== Test: SET_KINEMATIC_POSITION + jog + G28 X ===")

    print("\n1. SET_KINEMATIC_POSITION X=150 Y=150 Z=0")
    t0 = time.time()
    r = gcode(args.api, "SET_KINEMATIC_POSITION X=150 Y=150 Z=0")
    err = r.get("error", {}).get("message", "")
    print(f"   elapsed={time.time() - t0:.3f}s error={err or 'none'}")

    print("\n2. G1 X151 F3000 (1mm jog)")
    t0 = time.time()
    r = gcode(args.api, "G1 X151 F3000")
    err = r.get("error", {}).get("message", "")
    print(f"   elapsed={time.time() - t0:.3f}s error={err or 'none'}")

    print("\n3. G28 X (with GPIO injection after 2s)")

    def inject_after_delay():
        time.sleep(2.0)
        print("  [inject] Setting PC6 high on h723")
        mon.inject_gpio("h723", GPIOC_ODR, PC6_BIT)

    inject_thread = threading.Thread(target=inject_after_delay, daemon=True)
    inject_thread.start()

    t0 = time.time()
    r = gcode(args.api, "G28 X", timeout=30.0)
    elapsed = time.time() - t0
    err = r.get("error", {}).get("message", "")
    print(f"   elapsed={elapsed:.3f}s error={err or 'none'}")

    if err:
        print(f"\n  FAIL: {err}")
        sys.exit(1)
    elif elapsed < 1.0:
        print(f"\n  SUSPICIOUS: homed too fast ({elapsed:.1f}s)")
        sys.exit(1)
    else:
        print(f"\n  OK: homed in {elapsed:.1f}s")
        sys.exit(0)


if __name__ == "__main__":
    main()
