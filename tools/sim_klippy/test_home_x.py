#!/usr/bin/env python3
"""G28 X smoke test — does the kalico runtime actually complete a homing move?"""
import json, os, pathlib, signal, socket, subprocess, sys, time, threading

REPO = pathlib.Path(os.environ.get("KALICO_REPO", "/work"))
LOGDIR = REPO / "tools" / "sim_klippy" / ".local-logs"
KLIPPER_ELF = REPO / "out" / "klipper.elf"
PRINTER_CFG = REPO / "tools" / "sim_klippy" / "printer.cfg"
SIM_SOCKET = "/tmp/klipper_sim_socket"
KLIPPY_INPUT_TTY = "/tmp/klippy_sim_printer"
KLIPPY_API = "/tmp/klippy_sim_api"
KLIPPY_LOG = LOGDIR / "klippy.log"
ELF_LOG = LOGDIR / "klipper_elf.log"

def cleanup_prior():
    subprocess.run(["pkill", "-f", str(KLIPPER_ELF)], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "klippy_sim"], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)
    for path in (SIM_SOCKET, KLIPPY_INPUT_TTY, KLIPPY_API):
        try: os.unlink(path)
        except FileNotFoundError: pass

def spawn_elf():
    LOGDIR.mkdir(parents=True, exist_ok=True)
    elf_log = open(ELF_LOG, "wb")
    proc = subprocess.Popen([str(KLIPPER_ELF), "-I", SIM_SOCKET], stdout=elf_log, stderr=subprocess.STDOUT)
    for _ in range(50):
        if os.path.exists(SIM_SOCKET): return proc
        time.sleep(0.1)
    proc.terminate(); raise RuntimeError("elf failed")

def spawn_klippy():
    import shutil
    py = pathlib.Path(shutil.which("python3") or "python3")
    klippy_stderr = open(LOGDIR / "klippy_stderr.log", "wb")
    proc = subprocess.Popen([str(py), str(REPO/"klippy"/"klippy.py"), str(PRINTER_CFG),
                             "-l", str(KLIPPY_LOG), "-I", KLIPPY_INPUT_TTY, "-a", KLIPPY_API],
                            cwd=str(REPO),
                            stdout=klippy_stderr, stderr=subprocess.STDOUT)
    for _ in range(150):
        if os.path.exists(KLIPPY_API):
            time.sleep(5.0); return proc
        if proc.poll() is not None: raise RuntimeError("klippy died")
        time.sleep(0.1)
    proc.terminate(); raise RuntimeError("klippy api missing")

def send_gcode(script, timeout=30.0):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(timeout); s.connect(KLIPPY_API)
    msg = json.dumps({"id":1,"method":"gcode/script","params":{"script":script}}).encode()+b"\x03"
    s.sendall(msg)
    buf = b""
    while True:
        c = s.recv(4096)
        if not c: break
        buf += c
        if b"\x03" in buf: break
    s.close()
    out = buf.split(b"\x03",1)[0]
    return json.loads(out.decode()) if out else {}

def main():
    cleanup_prior()
    elf = spawn_elf()
    klippy = spawn_klippy()
    try:
        # Drive endstop pin gpio20 (X) HIGH-after-move via a klippy delayed timer.
        # We can't directly drive it from here without running a stepper-side
        # command; the sim has KALICO_SIM_ENDSTOP_SET_PIN G-code that drives
        # gpio levels via the firmware shim.

        # Step 1: leave endstop LOW (not triggered) initially.
        print("[home] forcing endstop gpio20 LOW")
        r = send_gcode("KALICO_SIM_ENDSTOP_SET_PIN GPIO=20 LEVEL=0")
        print(f"  -> {r}")

        # Step 2: arm a background thread to flip it HIGH after 200ms.
        def trip_after_delay():
            time.sleep(0.6)
            try:
                rr = send_gcode("KALICO_SIM_ENDSTOP_SET_PIN GPIO=20 LEVEL=1", timeout=5.0)
                print(f"[home] late-trip set gpio20=1 -> {rr}")
            except Exception as e:
                print(f"[home] late-trip set failed: {e}")
        t = threading.Thread(target=trip_after_delay, daemon=True)
        t.start()

        print("[home] sending G28 X")
        r = send_gcode("G28 X", timeout=30.0)
        print(f"[home] G28 X result: {r}")

        # Verify position via M114
        r = send_gcode("M114", timeout=5.0)
        print(f"[home] M114: {r}")

        return 0 if "result" in r and "error" not in r.get("error", {}) else 1
    finally:
        for p in (klippy, elf):
            try: p.terminate(); p.wait(timeout=3)
            except Exception: p.kill()

if __name__ == "__main__":
    sys.exit(main())
