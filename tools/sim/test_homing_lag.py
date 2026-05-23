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
        self._sock.settimeout(2)
        # Drain banner
        try:
            self._sock.recv(8192)
        except socket.timeout:
            pass
        # Select machine
        self._send("mach set 0")

    def _send(self, cmd):
        self._sock.sendall((cmd + "\n").encode())
        time.sleep(0.3)
        resp = ""
        try:
            resp = self._sock.recv(8192).decode(errors="replace")
        except socket.timeout:
            pass
        return resp.strip()

    def cmd(self, cmd):
        return self._send(cmd)

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
    # PB8 = gpio_id 24 (port B * 16 + pin 8).
    # Set via firmware's runtime_sim_endstop_set_pin command, which
    # directly writes the runtime's PIN_LEVELS array — bypasses GPIO
    # hardware that Renode can't inject into.
    GPIO_PB8 = 1 * 16 + 8  # 24
    print(f"Setting PB8 HIGH (gpio_id={GPIO_PB8})...")
    r = gcode(args.api,
              f"KALICO_SIM_ENDSTOP_SET_PIN GPIO={GPIO_PB8} LEVEL=1",
              timeout=5.0)
    print(f"  Result: {r}")
    time.sleep(0.5)

    t0 = time.time()
    r = gcode(args.api, "G28 X", timeout=60.0)
    elapsed = time.time() - t0
    err = (r.get("error") or {}).get("message", "")

    print(f"  Elapsed: {elapsed:.2f}s")
    if err:
        print(f"  Error: {err[:200]}")
    # "Endstop still triggered after retract" is expected when the pin
    # stays HIGH through the retract — the test only checks timing.
    if "still triggered after retract" in err:
        if elapsed > 5.0:
            print(f"  BUG: {elapsed:.1f}s delay before 'still triggered' error")
            failures.append(f"Test 1: {elapsed:.1f}s delay")
        else:
            print(f"  OK: retract + re-check completed in {elapsed:.1f}s (no ghost delay)")
    elif err:
        failures.append(f"Test 1 unexpected error: {err[:100]}")
    elif elapsed > 5.0:
        print(f"  BUG: {elapsed:.1f}s delay (expected < 5s)")
        failures.append(f"Test 1: {elapsed:.1f}s delay")
    else:
        print(f"  OK: {elapsed:.1f}s")

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
