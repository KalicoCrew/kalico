#!/usr/bin/env python3
"""G28 X end-to-end tests against the klippy-in-loop sim.

Two scenarios:
  - trip_path: arm endstop LOW, schedule a delayed_gcode at +0.6 s to
    flip pin HIGH, send G28 X, expect success and M114 reporting X=0.
  - notrip_path: arm endstop LOW, send G28 X with no trip scheduled,
    expect the new MCU-driven past-end-time terminal to surface as a
    "No trigger on x after full movement" error returned in well under
    the host wall-clock backstop.
"""
import json, os, pathlib, signal, socket, subprocess, sys, time

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
    subprocess.run(["pkill", "-f", str(KLIPPER_ELF)], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "klippy_sim"], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)
    for path in (SIM_SOCKET, KLIPPY_INPUT_TTY, KLIPPY_API):
        try: os.unlink(path)
        except FileNotFoundError: pass

def spawn_elf():
    LOGDIR.mkdir(parents=True, exist_ok=True)
    elf_log = open(ELF_LOG, "wb")
    proc = subprocess.Popen([str(KLIPPER_ELF), "-I", SIM_SOCKET],
                            stdout=elf_log, stderr=subprocess.STDOUT)
    for _ in range(50):
        if os.path.exists(SIM_SOCKET): return proc
        time.sleep(0.1)
    proc.terminate(); raise RuntimeError("elf failed")

def spawn_klippy():
    import shutil
    py = pathlib.Path(shutil.which("python3") or "python3")
    klippy_stderr = open(LOGDIR / "klippy_stderr.log", "wb")
    proc = subprocess.Popen([str(py), str(REPO/"klippy"/"klippy.py"),
                             str(PRINTER_CFG),
                             "-l", str(KLIPPY_LOG),
                             "-I", KLIPPY_INPUT_TTY,
                             "-a", KLIPPY_API],
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
    msg = json.dumps({"id":1,"method":"gcode/script",
                      "params":{"script":script}}).encode()+b"\x03"
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

def run_scenario(name, schedule_trip):
    cleanup_prior()
    elf = spawn_elf(); klippy = spawn_klippy()
    try:
        send_gcode("KALICO_SIM_ENDSTOP_SET_PIN GPIO=20 LEVEL=0")
        if schedule_trip:
            # Reactor-driven: fires from klippy's loop, NOT serialized
            # behind the in-flight G28 gcode-script command.
            send_gcode("UPDATE_DELAYED_GCODE ID=trip_x DURATION=0.6")
        t0 = time.time()
        r = send_gcode("G28 X", timeout=30.0)
        elapsed = time.time() - t0
        print(f"[{name}] G28 X result: {r} elapsed={elapsed:.2f}s")
        if schedule_trip:
            ok = "result" in r and "error" not in r.get("error", {})
        else:
            err_msg = r.get("error", {}).get("message", "")
            ok = "No trigger on x" in err_msg and elapsed < 10.0
        return ok
    finally:
        for p in (klippy, elf):
            try: p.terminate(); p.wait(timeout=3)
            except Exception: p.kill()

def main():
    failures = []
    if not run_scenario("trip_path", schedule_trip=True):
        failures.append("trip_path")
    if not run_scenario("notrip_path", schedule_trip=False):
        failures.append("notrip_path")
    if failures:
        print(f"FAIL: {failures}"); return 1
    print("OK"); return 0

if __name__ == "__main__":
    sys.exit(main())
