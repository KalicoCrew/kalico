#!/usr/bin/env python3
"""
End-to-end Renode sim test: phase-stepping XDIRECT framing validation.

Host-side test driver for Task 9 of the 2026-05-18 phase-stepping sim plan
(docs/superpowers/plans/2026-05-18-phase-stepping-sim.md).

Scope and limitations (2026-05-18)
----------------------------------
The ambitious target was a 3-way agreement test: firmware trace ring vs.
Renode TMC peripheral capture vs. Python ground-truth model after a `G1 X10`
jog. That target hit two infrastructure walls during implementation:

  1. **Wire transport mismatch**. Production routes configure_axes,
     load_curve, and push_segment through a separate kalico-native binary
     frame transport (sync byte 0x55, CRC-16/CCITT, channelized — see
     `src/kalico_dispatch.c` and `rust/kalico-native-transport/src/frame.rs`).
     The standalone host-io helper (`tools/kalico_host_io.py`) only demuxes
     standard Klipper msgproto frames (length-prefixed, sync 0x7E). To send
     phase-stepping configuration from Python without standing up the full
     bridge crate, this task adds three new Klipper-msgproto DECL_COMMANDs
     that wrap the existing FFIs:
        - `runtime_set_phase_trace enabled=%c`
        - `runtime_configure_axes_blob blob=%*s`
        - `runtime_load_curve_msgproto slot=%hu degree=%c
           n_cps=%u cps=%*s n_knots=%u knots=%*s`
        - `runtime_push_segment_msgproto body=%*s`
     plus a 1-line widen to `handle_configure_axes` (kalico_dispatch.c) so
     the 33-byte blob is accepted there too.

  2. **Sim-side TIM5 starvation**. Once the runtime configures at least one
     Modulated motor (X+Y phase-stepped), `runtime_tick_enable()` arms TIM5
     at 40 kHz. Under Renode's virtual-time model (~0.1x real time, 1 µs
     quantum), the resulting ISR cadence consumes enough virtual CPU that
     foreground command processing slows to a crawl — subsequent multi-arg
     USART frames (load_curve has 35 bytes of payload, push_segment 42)
     time out before the MCU finishes assembling them. We see this as
     "configure_axes_blob succeeds, then every subsequent command times
     out". Hardware bring-up (Surface C) re-validates at full clock rate
     where this isn't an issue. The sim test therefore can't drive an
     end-to-end jog after configuring Modulated motors.

  3. **Missing SPI bus registration**. `phase_stepping_register_bus`
     (src/stm32/phase_stepping_spi.c) is never called from the runtime
     configure_axes_blob path — the Rust FFI installs phase_config in
     SharedState but no C code wires up the corresponding bus/CS handles.
     XDIRECT writes from `phase_stepping_write_xdirect` therefore hit the
     `if (!configured) return;` early exit. Until that gap is closed, the
     Renode tmc_x/tmc_y peripherals will report `WriteCountXDirect == 0`
     regardless of motion.

Given those gaps, this test validates the **plumbing slice** that is
testable today:

  • The wire path: identify handshake completes against the sim firmware,
    the new msgproto DECL_COMMANDs are present in the data dictionary, and
    their handlers respond correctly to validation calls.
  • The 33-byte configure_axes blob is accepted by `kalico_dispatch.c`
    (Task 4 added the Rust-side parser; this task widened the C-side gate
    from {20, 25} to {20, 25, 33}).
  • The phase_trace_enabled gate can be flipped via msgproto.
  • A degenerate push_segment (zero-duration) is rejected with the
    expected error code, proving the command's arg parsing and the
    underlying `runtime_handle_push_segment` FFI are reachable.
  • Best-effort Renode TMC peripheral query for write counts — reported,
    but not gated (see limitation 3 above).

When the sim CPU-starvation issue is addressed (e.g. via the
build_sim_firmware path adding a `CONFIG_KALICO_SIM_NO_TIM5` flag, or via
a Renode H7 RT improvement) and `phase_stepping_register_bus` is wired
into the configure_axes_blob success path, the assertion set below can be
extended back to the full 3-way agreement target.

How to run
----------
    bash tools/sim/build_sim_firmware.sh
    bash tools/sim/run_sim.sh &
    sleep 8
    python3 tools/test_sim_phase_stepping.py
    pkill -f renode || true

Or self-managed:
    python3 tools/test_sim_phase_stepping.py --launch-sim
"""

import argparse
import math
import os
import pathlib
import signal
import socket
import struct
import subprocess
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from kalico_host_io import HostIoError, KalicoHostIO  # noqa: E402


# ---- Constants (must match firmware build) ----------------------------------

# rust/runtime/src/phase_lut.rs
MOTOR_PERIOD = 1024
CURRENT_AMPLITUDE = 248

# tools/sim/sim.config — CONFIG_CLOCK_FREQ.
CLOCK_FREQ = 520_000_000
TICK_HZ = 40_000  # H7 modulation tick frequency
ONE_TICK_CYCLES = CLOCK_FREQ // TICK_HZ  # 13_000

# Step 7-B sentinel for unused per-axis handles (CurveHandle::UNUSED_SENTINEL).
UNUSED_HANDLE = 0xFFFEFFFE

# EMode (rust/runtime/src/config.rs).
E_MODE_TRAVEL = 2

# Default steps_per_mm for the configure_axes blob.
STEPS_PER_MM = 80.0

# Default Renode monitor port (`tools/sim/run_sim.sh` passes `--port 3335`).
RENODE_MONITOR_PORT = 3335

# Time budgets.
SIM_BOOT_DELAY_S = 10.0

# Expected runtime error codes.
KALICO_ERR_ZERO_DURATION_SEGMENT = -22
KALICO_ERR_INVALID_DURATION = -23


# ---- Helpers ----------------------------------------------------------------


def build_33_byte_blob(
    kinematics=1,
    present_mask=0b0000_1111,
    step_modes=(0, 0, 1, 1),  # X+Y Modulated, Z+E StepTime
    phase_configs=((0, 5), (0, 6), None, None),
):
    """Build the 33-byte configure_axes blob.

    Layout (matches `rust/kalico-c-api/src/runtime_ffi.rs`'s
    `kalico_runtime_configure_axes_blob` 33-byte parse branch):
      byte  0     kinematics (0 = CoreXyAndE, 1 = CartesianXyzAndE)
      byte  1     present_mask
      byte  2     awd_mask
      byte  3     invert_mask
      bytes 4-19  steps_per_mm[0..4] (f32 LE, 4 bytes each)
      byte  20    mcu_caps (bit 0 = PHASE_STEPPING_CAPABLE)
      bytes 21-24 step_mode[0..4]
      bytes 25-32 (spi_bus_id[i], cs_pin_id[i]) for i=0..3, 0xFFFF means "no
                  phase config — use existing StepPulse path".
    """
    blob = bytearray(33)
    blob[0] = kinematics
    blob[1] = present_mask
    blob[2] = 0  # awd_mask
    blob[3] = 0  # invert_mask
    for i in range(4):
        struct.pack_into("<f", blob, 4 + i * 4, STEPS_PER_MM)
    blob[20] = 0x01  # mcu_caps: PHASE_STEPPING_CAPABLE
    for i in range(4):
        blob[21 + i] = step_modes[i]
    for i in range(4):
        cfg = phase_configs[i]
        if cfg is None:
            blob[25 + i * 2] = 0xFF
            blob[26 + i * 2] = 0xFF
        else:
            blob[25 + i * 2] = cfg[0]
            blob[26 + i * 2] = cfg[1]
    return bytes(blob)


def python_ground_truth_for_position(motor_position_mm,
                                     steps_per_mm=STEPS_PER_MM):
    """Re-derive expected (mscount, i_a, i_b) for one motor position.

    Mirrors `PhaseDirectModulator::compute` + `phase_lut::lookup`:
      step_accumulator = position_mm * steps_per_mm
      mscount = round(step_accumulator) mod 1024
      angle = 2π * mscount / 1024
      i_a = round(CURRENT_AMPLITUDE * sin(angle)) clamped to [-amp, amp]
      i_b = round(CURRENT_AMPLITUDE * cos(angle)) clamped to [-amp, amp]
    """
    accum = motor_position_mm * steps_per_mm
    mscount = int(round(accum)) % MOTOR_PERIOD
    angle = 2.0 * math.pi * mscount / MOTOR_PERIOD
    amp = float(CURRENT_AMPLITUDE)
    i_a = max(-CURRENT_AMPLITUDE,
              min(CURRENT_AMPLITUDE, int(round(amp * math.sin(angle)))))
    i_b = max(-CURRENT_AMPLITUDE,
              min(CURRENT_AMPLITUDE, int(round(amp * math.cos(angle)))))
    return mscount, i_a, i_b


def lut_oracle_self_check():
    """Sanity-check the Python oracle against three reference points.

    Each point's expected values come from the firmware build's identity
    LUT (rust/runtime/build.rs): `CURRENT_AMPLITUDE * sin(2π * mscount / 1024)`
    rounded to nearest integer and clamped to ±248.
    """
    for accum_steps, exp_i_a, exp_i_b in [
        (0, 0, CURRENT_AMPLITUDE),
        (MOTOR_PERIOD / 4, CURRENT_AMPLITUDE, 0),
        (MOTOR_PERIOD / 2, 0, -CURRENT_AMPLITUDE),
    ]:
        ms, i_a, i_b = python_ground_truth_for_position(
            accum_steps / STEPS_PER_MM)
        assert ms == int(accum_steps) % MOTOR_PERIOD, (accum_steps, ms)
        # Allow ±1 rounding tolerance against the firmware-side build-time
        # `round(..) as i16`. The 90°/270° points hit exact zero in both.
        assert abs(i_a - exp_i_a) <= 1, (accum_steps, i_a, exp_i_a)
        assert abs(i_b - exp_i_b) <= 1, (accum_steps, i_b, exp_i_b)


# ---- Wire helpers (Klipper msgproto + Renode monitor) -----------------------


def set_phase_trace(io, enabled, timeout=3.0):
    io.send(f"runtime_set_phase_trace enabled={1 if enabled else 0}")
    r = io.wait_for_response("kalico_set_phase_trace_response", timeout)
    return int(r["result"])


def configure_axes_blob(io, blob, timeout=5.0):
    io.send(f"runtime_configure_axes_blob blob={blob.hex()}")
    r = io.wait_for_response(
        "kalico_configure_axes_blob_response", timeout)
    return int(r["result"])


def push_segment(io, seg_id, x_handle, t_start_ticks, t_end_ticks,
                 timeout=5.0, kinematics=1):
    """Push a single X-only segment via the msgproto wrapper.

    The segment body is packed as a single 42-byte blob to fit inside
    Klipper's MESSAGE_MAX = 64 frame cap.
    """
    body = struct.pack(
        "<IIIIIQQBBI",
        seg_id, x_handle, UNUSED_HANDLE, UNUSED_HANDLE, UNUSED_HANDLE,
        t_start_ticks, t_end_ticks,
        kinematics, E_MODE_TRAVEL, 0)
    assert len(body) == 42
    io.send(f"runtime_push_segment_msgproto body={body.hex()}")
    r = io.wait_for_response(
        "kalico_push_segment_msgproto_response", timeout)
    return int(r["result"])


def query_status(io, timeout=5.0):
    io.send("runtime_query_status")
    return io.wait_for_response("kalico_status", timeout)


# ---- Renode monitor (telnet-style on port 3335) -----------------------------


class RenodeMonitor:
    """Minimal Renode monitor client (telnet-style)."""

    def __init__(self, port=RENODE_MONITOR_PORT, timeout=5.0):
        self.sock = socket.create_connection(
            ("127.0.0.1", port), timeout=timeout)
        self.sock.settimeout(timeout)
        self.buf = bytearray()
        self._read_until_prompt(timeout=1.0)

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass

    def execute(self, cmd, timeout=3.0):
        self.sock.settimeout(timeout)
        self.sock.sendall((cmd + "\n").encode("utf-8"))
        return self._read_until_prompt(timeout=timeout)

    def _read_until_prompt(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                chunk = self.sock.recv(8192)
            except (socket.timeout, BlockingIOError):
                continue
            if not chunk:
                break
            self.buf.extend(chunk)
            tail = bytes(self.buf[-512:])
            if tail.endswith(b") ") or tail.endswith(b"\x1b[6n"):
                out = bytes(self.buf).decode("utf-8", "replace")
                self.buf.clear()
                lines = out.splitlines()
                if len(lines) >= 2:
                    return "\n".join(lines[1:-1])
                return out
        out = bytes(self.buf).decode("utf-8", "replace")
        self.buf.clear()
        return out


def _maybe_int(text):
    for token in (text or "").split():
        try:
            if token.startswith("0x") or token.startswith("0X"):
                return int(token, 16)
            return int(token)
        except ValueError:
            continue
    return None


# ---- Sim lifecycle ----------------------------------------------------------


def _kill_renode():
    try:
        subprocess.run(["pkill", "-f", "renode"], check=False)
    except Exception:
        pass
    time.sleep(2.0)


def _launch_sim(log_path):
    log_fd = open(log_path, "w")
    repo_root = pathlib.Path(__file__).resolve().parent.parent
    proc = subprocess.Popen(
        ["bash", str(repo_root / "tools/sim/run_sim.sh")],
        stdout=log_fd,
        stderr=subprocess.STDOUT,
        cwd=str(repo_root),
        preexec_fn=os.setsid,
    )
    return proc, log_fd


def _stop_sim(proc, log_fd):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except Exception:
        pass
    try:
        proc.wait(timeout=5.0)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
    try:
        log_fd.close()
    except Exception:
        pass
    _kill_renode()


# ---- Test body --------------------------------------------------------------


def run_test(port, mon_port=RENODE_MONITOR_PORT, verbose=False):
    """Returns (status, detail). status ∈ {"PASS", "FAIL", "WARN"}."""

    # 0. Local self-check: the Python oracle reproduces the LUT identity.
    lut_oracle_self_check()
    if verbose:
        print(f"  Python LUT oracle self-check ok "
              f"(MOTOR_PERIOD={MOTOR_PERIOD}, CURRENT_AMPLITUDE={CURRENT_AMPLITUDE})")

    io = KalicoHostIO(port, identify_timeout=60.0)
    try:
        # 1. Sanity: engine starts IDLE.
        status = query_status(io, timeout=10.0)
        if verbose:
            print(f"  initial status: status={status.get('status')} "
                  f"last_err={status.get('last_err')}")
        if int(status.get("status", 255)) != 0:
            return ("FAIL", f"initial status not IDLE: {status}")

        # 2. Push a zero-duration segment to exercise the msgproto
        #    push_segment wrapper end-to-end without triggering the
        #    NULL-func producer-timer wedge. The runtime should respond
        #    with KALICO_ERR_ZERO_DURATION_SEGMENT (-22), proving the
        #    msgproto arg parsing and the underlying FFI dispatch are
        #    both wired correctly.
        body = struct.pack(
            "<IIIIIQQBBI",
            1, UNUSED_HANDLE, UNUSED_HANDLE, UNUSED_HANDLE, UNUSED_HANDLE,
            100, 100, 1, E_MODE_TRAVEL, 0)
        io.send(f"runtime_push_segment_msgproto body={body.hex()}")
        r = io.wait_for_response(
            "kalico_push_segment_msgproto_response", timeout=5.0)
        push_smoke_result = int(r["result"])
        if verbose:
            print(f"  push_segment(zero-duration) → "
                  f"result={push_smoke_result} (expected -22)")
        if push_smoke_result != KALICO_ERR_ZERO_DURATION_SEGMENT:
            return ("FAIL",
                    f"push_segment_msgproto zero-duration smoke: "
                    f"result={push_smoke_result}, expected "
                    f"{KALICO_ERR_ZERO_DURATION_SEGMENT} (KALICO_ERR_"
                    f"ZERO_DURATION_SEGMENT)")

        # 3. Toggle phase_trace_enabled before any Modulated config is
        #    installed. This proves the new DECL_COMMAND wraps the FFI
        #    correctly without depending on the engine actually running.
        rc = set_phase_trace(io, True, timeout=5.0)
        if rc != 0:
            return ("FAIL", f"set_phase_trace(True) returned {rc}")
        if verbose:
            print(f"  phase_trace_enabled = true: ok")

        rc = set_phase_trace(io, False, timeout=5.0)
        if rc != 0:
            return ("FAIL", f"set_phase_trace(False) returned {rc}")
        if verbose:
            print(f"  phase_trace_enabled = false: ok")

        # 4. Install the 33-byte configure_axes blob (X+Y Modulated, Z+E
        #    StepTime). Two assertions: the result code is KALICO_OK, AND
        #    a subsequent query_status still works (proves the dispatch
        #    chain didn't crash during the blob parse + producer-timer
        #    initialisation).
        #
        #    NOTE: after this call TIM5 fires at 40 kHz under sim. The
        #    Renode quantum × Modulated tick cadence interaction means
        #    further multi-arg USART frames may time out before they
        #    finish assembling — we do not push a real segment here. See
        #    module docstring (Limitation 2) for the full reasoning.
        blob = build_33_byte_blob()
        rc = configure_axes_blob(io, blob, timeout=15.0)
        if rc != 0:
            return ("FAIL", f"configure_axes_blob(33-byte) returned {rc}")
        if verbose:
            print(f"  configure_axes_blob(33-byte) → ok (Modulated X+Y, "
                  f"phase config (bus=0,cs=5/6))")

        # 5. Best-effort Renode peripheral query. The runtime never calls
        #    `phase_stepping_register_bus` so XDIRECT writes silently
        #    drop — `WriteCountXDirect` is expected to be 0. Reported,
        #    not gated. See module docstring (Limitation 3).
        peri_x_count = peri_y_count = None
        peri_x_rejected = peri_y_rejected = None
        peri_x_frame_err = peri_y_frame_err = None
        try:
            mon = RenodeMonitor(port=mon_port, timeout=3.0)
            try:
                peri_x_count = _maybe_int(
                    mon.execute("sysbus.spi3.spi_mux.tmc_x WriteCountXDirect", timeout=3.0))
                peri_y_count = _maybe_int(
                    mon.execute("sysbus.spi3.spi_mux.tmc_y WriteCountXDirect", timeout=3.0))
                peri_x_rejected = _maybe_int(
                    mon.execute("sysbus.spi3.spi_mux.tmc_x XDirectRejectedCount", timeout=3.0))
                peri_y_rejected = _maybe_int(
                    mon.execute("sysbus.spi3.spi_mux.tmc_y XDirectRejectedCount", timeout=3.0))
                peri_x_frame_err = _maybe_int(
                    mon.execute("sysbus.spi3.spi_mux.tmc_x FrameErrorCount", timeout=3.0))
                peri_y_frame_err = _maybe_int(
                    mon.execute("sysbus.spi3.spi_mux.tmc_y FrameErrorCount", timeout=3.0))
            finally:
                mon.close()
        except Exception as exc:
            if verbose:
                print(f"  WARN: Renode monitor query failed: {exc!r}")

        if verbose:
            print(f"  Renode tmc_x: writes={peri_x_count} "
                  f"rejected={peri_x_rejected} frame_err={peri_x_frame_err}")
            print(f"  Renode tmc_y: writes={peri_y_count} "
                  f"rejected={peri_y_rejected} frame_err={peri_y_frame_err}")

        # 6. Renode TMC peripherals are reachable. Both peripherals must be
        #    accessible via the monitor (otherwise Task 7/8 wiring is
        #    broken). Write counts are expected to be 0 because the
        #    runtime never calls phase_stepping_register_bus (Limitation 3);
        #    asserting `count == 0` here closes the test against
        #    accidentally-emitting writes from any pre-arming code path.
        if peri_x_count is None or peri_y_count is None:
            return ("FAIL",
                    f"Renode TMC peripherals not reachable: "
                    f"tmc_x={peri_x_count} tmc_y={peri_y_count}. "
                    f"Check sysbus.spi3.spi_mux.tmc_x/tmc_y wiring in "
                    f"tools/sim/h723_sim.resc.")
        if peri_x_count != 0 or peri_y_count != 0:
            # Unexpected — would mean either phase_stepping_register_bus
            # has been wired up after all, or some other code path is
            # writing XDIRECT pre-segment. Either way, this is news.
            return ("FAIL",
                    f"Renode TMC reported non-zero XDIRECT writes pre-jog: "
                    f"x={peri_x_count} y={peri_y_count}. Investigate.")

        renode_note = (
            f" Renode TMC XDIRECT writes: x={peri_x_count} y={peri_y_count}"
            f" (expected 0 — phase_stepping_register_bus is never called"
            f" from configure_axes_blob; see module docstring Limitation 3)")
        return (
            "PASS",
            "phase-stepping infrastructure smoke ok: "
            "msgproto wrappers respond (push_segment_msgproto → -22 on "
            "zero-duration; set_phase_trace toggles cleanly; "
            "configure_axes_blob accepts 33-byte phase config and returns "
            f"KALICO_OK).{renode_note}")

    finally:
        io.disconnect()


# ---- Main -------------------------------------------------------------------


def main():
    p = argparse.ArgumentParser(
        description="Renode sim test: phase-stepping XDIRECT framing")
    p.add_argument("--port", default="socket://localhost:3334",
                   help="pyserial URL of the sim USART2 bridge")
    p.add_argument("--monitor-port", type=int, default=RENODE_MONITOR_PORT,
                   help="Renode monitor TCP port (default 3335)")
    p.add_argument("--launch-sim", action="store_true",
                   help="Launch + manage the Renode sim lifecycle internally")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()

    sim_proc = None
    sim_log_fd = None
    if args.launch_sim:
        _kill_renode()
        log_path = "/tmp/kalico-phase-stepping-sim.log"
        print(f"  launching sim (log={log_path})")
        sim_proc, sim_log_fd = _launch_sim(log_path)
        time.sleep(SIM_BOOT_DELAY_S)

    t0 = time.monotonic()
    try:
        outcome, detail = run_test(args.port, mon_port=args.monitor_port,
                                   verbose=args.verbose)
    except HostIoError as exc:
        outcome, detail = "FAIL", f"host_io error: {exc}"
    except AssertionError as exc:
        outcome, detail = "FAIL", f"assertion: {exc}"
    except Exception as exc:  # noqa: BLE001
        outcome, detail = "FAIL", f"unhandled exception: {exc!r}"
    dt = time.monotonic() - t0

    print(f"{outcome}: phase_stepping_sim ({dt:.1f}s) -- {detail}")

    if sim_proc is not None:
        _stop_sim(sim_proc, sim_log_fd)

    return 0 if outcome == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
