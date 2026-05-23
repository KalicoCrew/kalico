#!/usr/bin/env python3
"""Reproduce the homing-lag timing bug.

Usage inside Docker:
    python3 tools/sim/test_homing_lag.py --api /tmp/klippy_api --renode-monitor localhost:3335

Usage standalone (if klippy + Renode already running):
    python3 tools/sim/test_homing_lag.py --api /tmp/klippy_api --renode-monitor localhost:3335
"""
import argparse
import json
import socket
import time
import sys


def gcode(api_socket, script, timeout=60.0):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(timeout)
    s.connect(api_socket)
    s.sendall(json.dumps({
        "id": 1, "method": "gcode/script",
        "params": {"script": script},
    }).encode() + b"\x03")
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
        # Drain banner until first prompt
        self._read_until_prompt()
        # Select machine
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
            # Renode prompt: (monitor) or (h723) followed by space
            text = self._buf.decode(errors="replace")
            # Strip ANSI escape codes for matching
            import re
            clean = re.sub(r'\x1b\[[0-9;]*m', '', text)
            if re.search(r'\((monitor|h723)\)\s*$', clean):
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
    parser.add_argument("--renode-monitor", required=True, help="host:port for Renode monitor")
    args = parser.parse_args()

    failures = []

    # Test 1: Pre-tripped endstop with retract — measures post-homing delay
    print("\n=== Test 1: Pre-tripped endstop (retract timing) ===")
    # Don't pause Renode — klippy needs the MCU responsive to send
    # arm commands. Instead, inject the pin via Renode monitor from a
    # background thread after a wall-clock delay, while G28 is in flight.
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
    print(f"  Elapsed: {elapsed:.2f}s")
    if err:
        print(f"  Error: {err[:200]}")
    # "Endstop still triggered after retract" is expected when the pin
    # stays HIGH through the retract.
    if "still triggered after retract" in err:
        print(f"  (expected — pin stays HIGH through retract)")
    elif err:
        failures.append(f"Test 1 unexpected error: {err[:100]}")
    else:
        print(f"  Homing succeeded")
    # Renode runs at ~0.3-0.7x real time. A 50mm/50mm/s = 1s firmware
    # homing move should take < 10s wall-clock even at 0.1x. The bug
    # adds 5-10s of ghost delay on top of that. Threshold at 15s to
    # separate Renode slowness from the actual bug.
    if elapsed > 15.0:
        print(f"  BUG: {elapsed:.1f}s total — likely ghost delay")
        failures.append(f"Test 1: {elapsed:.1f}s delay")
    else:
        print(f"  Timing acceptable: {elapsed:.1f}s")

    # Summary
    print("\n=== Summary ===")
    if failures:
        for f in failures:
            print(f"  FAIL: {f}")
        return 1
    print("  All tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
