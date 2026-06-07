#!/usr/bin/env python3
import argparse
import json
import socket
import sys
import time

import pytest

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
        self._sock = socket.create_connection((host, int(port)), timeout=10)
        self._sock.settimeout(5)
        self._buf = b""
        self._read_until_prompt()
        self.cmd("mach set 0")

    def _read_until_prompt(self):
        while True:
            try:
                chunk = self._sock.recv(4096)
            except socket.timeout:
                break
            if not chunk:
                break
            self._buf += chunk
            text = self._buf.decode(errors="replace")
            import re

            clean = re.sub(r"\x1b\[[0-9;]*m", "", text)
            if re.search(r"\((monitor|h723)\)\s*$", clean):
                result = clean.strip()
                self._buf = b""
                return result
        result = self._buf.decode(errors="replace").strip()
        self._buf = b""
        return result

    def cmd(self, cmd):
        self._sock.sendall((cmd + "\n").encode())
        return self._read_until_prompt()

    def close(self):
        self._sock.close()


def renode_cmd(monitor_addr, cmd):
    mon = RenodeMonitor(monitor_addr)
    r = mon.cmd(cmd)
    mon.close()
    return r


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--api", required=True, help="klippy API socket path")
    parser.add_argument(
        "--renode-monitor", required=True, help="host:port for Renode monitor"
    )
    args = parser.parse_args()

    failures = []

    print("\n=== Test 1: Pre-tripped endstop (retract timing) ===")
    import threading

    def delayed_gpio_inject():
        time.sleep(2.0)
        m = RenodeMonitor(args.renode_monitor)
        m.cmd("sysbus.gpioPortC OnGPIO 6 true")
        r = m.cmd("sysbus ReadDoubleWord 0x58020810")
        print(f"  [inject thread] IDR after OnGPIO: {r[:60]}")
        m.close()

    inject_thread = threading.Thread(target=delayed_gpio_inject, daemon=True)
    inject_thread.start()

    print("Sending G28 X (PC6 will be injected after ~3s)...")
    t0 = time.time()
    r = gcode(args.api, "G28 X", timeout=120.0)
    elapsed = time.time() - t0
    inject_thread.join(timeout=5.0)

    err = (r.get("error") or {}).get("message", "")

    print(f"  Elapsed: {elapsed:.2f}s")
    if err:
        print(f"  Error: {err[:200]}")
    if "still triggered after retract" in err:
        print("  (expected — pin stays HIGH through retract)")
    elif err:
        failures.append(f"Test 1 unexpected error: {err[:100]}")
    else:
        print("  Homing succeeded")
    if elapsed > 15.0:
        print(f"  BUG: {elapsed:.1f}s total — likely ghost delay")
        failures.append(f"Test 1: {elapsed:.1f}s delay")
    else:
        print(f"  Timing acceptable: {elapsed:.1f}s")

    print("\n=== Test 2: Post-homing response time ===")
    t1 = time.time()
    r2 = gcode(args.api, "SET_KINEMATIC_POSITION X=0 Y=0 Z=0", timeout=30.0)
    post_homing_elapsed = time.time() - t1
    err2 = (r2.get("error") or {}).get("message", "")
    print(f"  SET_KINEMATIC_POSITION elapsed: {post_homing_elapsed:.3f}s")
    if err2:
        print(f"  Error: {err2[:200]}")

    t2 = time.time()
    r3 = gcode(args.api, "G1 X10 F3000", timeout=30.0)
    move_elapsed = time.time() - t2
    err3 = (r3.get("error") or {}).get("message", "")
    print(f"  G1 X10 elapsed: {move_elapsed:.3f}s")
    if err3:
        print(f"  Error: {err3[:200]}")

    if post_homing_elapsed > 3.0:
        print(
            f"  BUG: {post_homing_elapsed:.1f}s post-homing delay on SET_KINEMATIC_POSITION"
        )
        failures.append(f"Test 2: {post_homing_elapsed:.1f}s post-homing delay")
    else:
        print(f"  Post-homing response OK: {post_homing_elapsed:.3f}s")

    print("\n=== Summary ===")
    if failures:
        for f in failures:
            print(f"  FAIL: {f}")
        return 1
    print("  All tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
