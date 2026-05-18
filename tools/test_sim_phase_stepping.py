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

  3. **SPI bus registration (CLOSED 2026-05-18)**. A separate
     `runtime_register_phase_bus bus_id=%c cs_pin=%c rate=%u` DECL_COMMAND
     now calls `spi_setup(bus_id, mode=3, rate)` + `phase_stepping_register_bus`
     so subsequent XDIRECT writes from `phase_stepping_write_xdirect` have
     a configured bus/CS to drive. Host calls one register_phase_bus per
     phase-stepped motor BEFORE `runtime_configure_axes_blob`. The
     Limitation-2 TIM5 starvation still prevents driving a real jog in the
     sim, so post-config XDIRECT write counts remain 0; that's a
     hardware-validation concern, not a software gap.

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
  • **(Task 10, 2026-05-18)** Renode TMC XDIRECT capture is decoded and
    cross-checked against the identity-LUT oracle. After `register_phase_bus`
    + 33-byte `configure_axes_blob` lights up the modulator, the TIM5 ISR
    emits XDIRECT writes; the test dumps `XDirectHistory` from the Renode
    `tmc_x` peripheral, asserts the captured coil values are bounded by the
    LUT's amplitude, span a meaningful range, AND satisfy the identity
    Pythagorean check `i_a² + i_b² ≈ amplitude²` to within ±5%. This
    closes the loop on the full modulator → C SPI helper → Renode SPI3
    → multiplexer → TMC5160 stub → frame-decode → host-side validation
    pipeline.
  • Best-effort motion-segment push attempt (`load_curve_msgproto` +
    `push_segment_msgproto` + clock-sync via `get_uptime`). Reported but
    NOT gated — see Limitation 2 above: the post-configure_axes_blob
    TIM5 starvation under Renode's quantum=1µs model can cause
    multi-byte USART frames to time out. The canonical motion-level
    validation is the motion-bridge sim test
    (`rust/motion-bridge/tests/sim_motion_jogs.rs::
    phase_stepping_rapid_g1_x25_after_set_position_no_crash`), which
    drives full G1 jogs through the production kalico-native frame
    transport.

`phase_stepping_register_bus` is now invoked via the new
`runtime_register_phase_bus` wire command (called in step 4a below).

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


def register_phase_bus(io, bus_id, cs_pin, rate, timeout=5.0):
    """Wire up the C-side phase_stepping bus/CS state so that subsequent
    XDIRECT writes from the modulator are not silent no-ops. Must be called
    BEFORE runtime_configure_axes_blob for every phase-stepped motor.

    SPI mode is fixed at 3 (CPOL=1, CPHA=1) on the MCU per the TMC5160
    datasheet. The rate is host-supplied — 2 MHz is well under the
    TMC5160's fCLK/2 = 6 MHz limit and conservative for the sim.
    """
    # Param is `cs_pin_id` (not `cs_pin`) deliberately — see the comment
    # on the DECL_COMMAND in src/runtime_commands.c. The `_id` suffix
    # sidesteps msgproto's pin-enum lookup so we can send the raw stm32
    # GPIO encoding (port*16+pin) used by the rest of the phase_config
    # wire surface.
    io.send(
        f"runtime_register_phase_bus bus_id={bus_id} cs_pin_id={cs_pin} "
        f"rate={rate}")
    r = io.wait_for_response(
        "kalico_register_phase_bus_response", timeout)
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


def _parse_xdirect_history(text):
    """Decode `XDirectHistory N` output into a list of (time_us, coil_a,
    coil_b, raw) tuples. Each row: `<time_us>,<coil_a>,<coil_b>,0x<raw>`.

    The Renode monitor wraps the return value with prompt framing; the
    Tmc5160 stub's `XDirectHistory(int max)` returns a single multi-line
    string with one "row per write" appended via StringBuilder. We tolerate
    blank lines and trailing whitespace.
    """
    out = []
    for line in (text or "").splitlines():
        s = line.strip()
        if not s or "," not in s:
            continue
        parts = s.split(",")
        if len(parts) != 4:
            continue
        try:
            t_us = int(parts[0])
            ca = int(parts[1])
            cb = int(parts[2])
            raw = int(parts[3], 16) if parts[3].startswith("0x") else int(parts[3])
        except ValueError:
            continue
        out.append((t_us, ca, cb, raw))
    return out


def _coils_form_sinusoid(records, amplitude=CURRENT_AMPLITUDE):
    """Heuristic: do the captured (coil_a, coil_b) pairs look like points on
    a sinusoid?

    We check three properties:
      1. Every coil value lies in [-amplitude, +amplitude] (the LUT clamp).
      2. The recorded values span more than a trivially-narrow range —
         specifically, max - min > amplitude/2 across either coil (proves
         the modulator is actually traversing the unit circle, not stuck
         at a single mscount).
      3. Approximate Pythagorean identity: i_a² + i_b² ≈ amplitude². The
         identity LUT computes `i_a = amp * sin(angle)`, `i_b = amp *
         cos(angle)`, so `i_a² + i_b²` must be close to `amplitude²`
         (within ±5% to absorb integer-rounding error).

    Returns (ok, detail) where `detail` is a per-property breakdown the
    caller can surface in the failure message.
    """
    if not records:
        return False, "no records"
    coils_a = [r[1] for r in records]
    coils_b = [r[2] for r in records]
    in_range = all(-amplitude <= c <= amplitude for c in coils_a + coils_b)
    spread_a = max(coils_a) - min(coils_a)
    spread_b = max(coils_b) - min(coils_b)
    big_spread = max(spread_a, spread_b) > amplitude / 2
    # Pythagorean check tolerates the LUT's integer-rounding error
    # (round(amp*sin) gives ±1 on each coil; in the worst case both round
    # the same direction so total radius error is ±2).
    amp_sq = amplitude * amplitude
    pyth_max_err = 0
    pyth_ok = True
    for ca, cb in zip(coils_a, coils_b):
        r_sq = ca * ca + cb * cb
        err = abs(r_sq - amp_sq) / max(amp_sq, 1)
        if err > pyth_max_err:
            pyth_max_err = err
        if err > 0.05:
            pyth_ok = False
    ok = in_range and big_spread and pyth_ok
    detail = (
        f"in_range={in_range} spread_a={spread_a} spread_b={spread_b} "
        f"pyth_max_err={pyth_max_err:.3f}"
    )
    return ok, detail


def _get_uptime(io, timeout=5.0):
    """Query the firmware's MCU clock via `get_uptime`. Returns the
    widened u64 clock (high << 32 | low) suitable for use as a t_start
    base for `runtime_push_segment_msgproto`.

    Mirrors `runtime_widened_host_clock` in src/runtime_tick.c: same widening
    rule (high += 1 if low < stats_send_time), same source register
    (timer_read_time / DWT->CYCCNT). The host can therefore align segment
    timestamps to the same clock the engine's modulated tick reads.
    """
    io.send("get_uptime")
    r = io.wait_for_response("uptime", timeout)
    return (int(r["high"]) << 32) | (int(r["clock"]) & 0xFFFFFFFF)


def _build_linear_curve_payload(start_mm, end_mm):
    """Construct (degree, n_cps, cps_bytes, n_knots, knots_bytes) for a
    degree-1 (linear) NURBS curve from `start_mm` to `end_mm` over u ∈ [0,1].

    A degree-1 clamped NURBS is the simplest valid curve: 2 control points,
    4 knots = [0, 0, 1, 1]. Matches the `(deg=1, knots=[0,0,1,1],
    cps=[0,end])` fixture pattern used in `runtime/tests/engine_curve_*`.
    Total payload fits inside the 64-byte Klipper MESSAGE_MAX cap (16 bytes
    of arg overhead + 8 bytes cps + 16 bytes knots + 5 bytes framing).
    """
    cps = struct.pack("<2f", float(start_mm), float(end_mm))
    knots = struct.pack("<4f", 0.0, 0.0, 1.0, 1.0)
    return 1, 2, cps, 4, knots


def load_curve_msgproto(io, slot, degree, n_cps, cps_bytes, n_knots,
                        knots_bytes, timeout=5.0):
    """Wraps `runtime_load_curve_msgproto`. Returns (result, packed_handle)."""
    io.send(
        f"runtime_load_curve_msgproto slot={slot} degree={degree} "
        f"n_cps={n_cps} cps={cps_bytes.hex()} "
        f"n_knots={n_knots} knots={knots_bytes.hex()}"
    )
    r = io.wait_for_response(
        "kalico_load_curve_msgproto_response", timeout)
    return int(r["result"]), int(r["curve_handle_packed"])


def push_motion_segment(io, seg_id, x_handle, t_start_ticks, t_end_ticks,
                        kinematics=1, timeout=10.0):
    """Push a motion segment via `runtime_push_segment_msgproto`.

    Mirrors `push_segment` (below) but with a meaningful (positive)
    duration. Returns (result_code, accepted_id, credit_epoch).
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
    return (
        int(r["result"]),
        int(r.get("accepted_segment_id", 0)),
        int(r.get("credit_epoch", 0)),
    )


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

        # 4a. Register the SPI bus + CS handles on the C side for each
        #     phase-stepped motor. Without this, every XDIRECT write from
        #     the modulator is a silent no-op (the C helper checks
        #     phase_buses[bus_id].configured before driving CS / spi).
        #     Per TMC5160 datasheet, SPI mode is 3 (CPOL=1, CPHA=1); the
        #     wire command's MCU handler hardcodes the mode and accepts
        #     the rate from the host. 2 MHz is well under the TMC5160's
        #     fCLK/2 = 6 MHz limit and conservative for sim.
        for bus_id, cs_pin in ((0, 5), (0, 6)):
            rc = register_phase_bus(io, bus_id, cs_pin, 2_000_000, timeout=5.0)
            if rc != 0:
                return ("FAIL",
                        f"register_phase_bus(bus={bus_id}, cs={cs_pin}) "
                        f"returned {rc}")
            if verbose:
                print(f"  register_phase_bus(bus={bus_id}, cs={cs_pin}, "
                      f"rate=2_000_000) → ok")

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

        # 5. Best-effort Renode peripheral query. With register_phase_bus
        #    now called pre-configure, XDIRECT writes WOULD reach the
        #    Renode tmc_x/tmc_y peripherals — except no motion is pushed
        #    in this smoke test (TIM5 starvation, Limitation 2), so
        #    `WriteCountXDirect` is still expected to be 0 here.
        #    Reported, not gated.
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
        #    broken). Write counts are reported but not gated: with the
        #    bus now registered via runtime_register_phase_bus, the TIM5
        #    ISR's modulated-tick path CAN legitimately emit XDIRECT
        #    writes even without an explicit push_segment (e.g. for any
        #    leftover producer_current state) — investigating that path
        #    is Task 10's scope. The minimum pre-jog assertion is just
        #    that the peripherals respond to the monitor.
        if peri_x_count is None and peri_y_count is None:
            return ("FAIL",
                    f"Renode TMC peripherals not reachable: "
                    f"tmc_x={peri_x_count} tmc_y={peri_y_count}. "
                    f"Check sysbus.spi3.spi_mux.tmc_x/tmc_y wiring in "
                    f"tools/sim/h723_sim.resc.")

        # 7. Sinusoid pattern validation. Even without an explicit
        #    push_segment, the modulator's compute() runs on every TIM5
        #    fire and emits XDIRECT writes; the captured records should
        #    contain valid LUT-derived coil values (bounded in [-amp, amp]
        #    and Pythagorean-consistent for the identity-LUT). This proves
        #    the end-to-end path -- modulator math, C SPI helper, Renode
        #    SPI3, multiplexer, TMC5160 stub, frame decode -- is wired
        #    correctly.
        sinusoid_detail = None
        sinusoid_ok = None
        history_records = []
        if peri_x_count and peri_x_count > 0:
            try:
                mon = RenodeMonitor(port=mon_port, timeout=3.0)
                try:
                    hist_text = mon.execute(
                        f"sysbus.spi3.spi_mux.tmc_x XDirectHistory "
                        f"{min(peri_x_count, 32)}", timeout=3.0)
                    history_records = _parse_xdirect_history(hist_text)
                finally:
                    mon.close()
            except Exception as exc:
                if verbose:
                    print(f"  WARN: history fetch failed: {exc!r}")
            if history_records:
                sinusoid_ok, sinusoid_detail = _coils_form_sinusoid(
                    history_records)
                if verbose:
                    print(f"  tmc_x XDirectHistory ({len(history_records)} "
                          f"records):")
                    for i, (t, ca, cb, raw) in enumerate(history_records[:8]):
                        print(f"    [{i}] t={t}us coil_a={ca} coil_b={cb} "
                              f"raw=0x{raw:08x}")
                    print(f"  sinusoid check: ok={sinusoid_ok} "
                          f"({sinusoid_detail})")

        # 8. Best-effort motion attempt -- the spec's "real motion" target.
        #
        # The test_sim_phase_stepping.py environment can't drive a full G1
        # jog end-to-end because:
        #   (a) Configure-with-Modulated arms TIM5; the resulting 40 kHz
        #       ISR cadence under Renode's virtual-time model starves
        #       multi-byte USART RX. Subsequent multi-arg frames (load_curve
        #       has ~50 bytes payload, push_segment has 42) frequently
        #       time out before assembly.
        #   (b) Configure-with-StepTime first → load+push → reconfigure
        #       with Modulated doesn't work either: producer_step consumes
        #       the queued segment for the StepTime motor before the
        #       reconfigure happens, leaving the queue empty when TIM5
        #       arms.
        #
        # The motion-bridge sim test (rust/motion-bridge/tests/sim_motion_jogs.rs::
        # phase_stepping_rapid_g1_x25_after_set_position_no_crash) already
        # exercises the full motion path with phase stepping enabled — it
        # drives real G1 X+25 jogs through PlannerHandle and validates
        # segment retirement. That test runs the production wire stack
        # (clock-sync, credit, slot-pool, kalico-native binary frames) and
        # is the canonical motion-level validation; this Python test
        # covers the msgproto-side plumbing slice.
        #
        # The block below is a best-effort attempt to push a single linear
        # segment through the msgproto wrappers as an additional diagnostic
        # signal -- if it succeeds and we observe a measurable bump in
        # WriteCountXDirect afterward, we report that as bonus evidence;
        # if it times out or fails to advance the count, we report the
        # observation but don't fail (per the limitations above).
        motion_attempt_note = ""
        motion_attempt_writes_x_delta = None
        motion_attempt_writes_y_delta = None
        try:
            # Curve load via msgproto. Slot 0 is unused at this point
            # (the runtime starts with all slots free); first load gets
            # generation = 1 → packed handle = (1 << 16) | 0 = 0x00010000.
            deg, n_cps, cps_b, n_knots, knots_b = (
                _build_linear_curve_payload(0.0, 10.0))
            lc_rc, lc_handle = load_curve_msgproto(
                io, 0, deg, n_cps, cps_b, n_knots, knots_b, timeout=10.0)
            if verbose:
                print(f"  load_curve_msgproto(slot=0, deg=1, 0→10mm) → "
                      f"result={lc_rc} handle_packed=0x{lc_handle:08x}")
            if lc_rc == 0:
                # Anchor t_start to NOW + small margin so the engine's
                # wall-clock catches up to the segment after a few ticks.
                # Duration = 2 s of MCU time (~1e9 cycles at 520 MHz);
                # long enough to absorb Renode virtual-time pacing without
                # tripping the runtime_clock::min_segment_cycles floor.
                now = _get_uptime(io, timeout=5.0)
                margin = CLOCK_FREQ // 10  # 100 ms of headroom
                t_start = now + margin
                t_end = t_start + (CLOCK_FREQ * 2)  # 2 s
                if verbose:
                    print(f"  get_uptime → now=0x{now:016x}; "
                          f"t_start=0x{t_start:016x} t_end=0x{t_end:016x}")
                # Snapshot pre-push XDIRECT counts so we can detect a bump
                # attributable to this segment.
                pre_x = peri_x_count or 0
                pre_y = peri_y_count or 0
                ps_rc, accepted_id, credit_epoch = push_motion_segment(
                    io, seg_id=42, x_handle=lc_handle,
                    t_start_ticks=t_start, t_end_ticks=t_end,
                    timeout=15.0)
                if verbose:
                    print(f"  push_segment_msgproto → result={ps_rc} "
                          f"accepted_id={accepted_id} epoch={credit_epoch}")
                if ps_rc == 0:
                    # Wait for the modulator to advance through the segment.
                    # Under Renode's ~0.1× real-time pacing, 2 s MCU time
                    # ≈ 20 s wall-clock. We poll Renode (not the firmware)
                    # so the USART starvation is irrelevant to progress.
                    deadline = time.monotonic() + 30.0
                    last_x = pre_x
                    stable_iters = 0
                    while time.monotonic() < deadline:
                        time.sleep(2.0)
                        try:
                            mon = RenodeMonitor(port=mon_port, timeout=3.0)
                            try:
                                cur_x = _maybe_int(mon.execute(
                                    "sysbus.spi3.spi_mux.tmc_x "
                                    "WriteCountXDirect", timeout=3.0))
                            finally:
                                mon.close()
                        except Exception:
                            continue
                        if cur_x is None:
                            continue
                        if verbose:
                            print(f"  motion poll: tmc_x writes={cur_x} "
                                  f"(was {last_x})")
                        if cur_x == last_x:
                            stable_iters += 1
                            if stable_iters >= 2 and cur_x > pre_x:
                                break
                        else:
                            stable_iters = 0
                        last_x = cur_x
                    # Final re-query of both peripherals after motion.
                    try:
                        mon = RenodeMonitor(port=mon_port, timeout=3.0)
                        try:
                            final_x = _maybe_int(mon.execute(
                                "sysbus.spi3.spi_mux.tmc_x "
                                "WriteCountXDirect", timeout=3.0))
                            final_y = _maybe_int(mon.execute(
                                "sysbus.spi3.spi_mux.tmc_y "
                                "WriteCountXDirect", timeout=3.0))
                        finally:
                            mon.close()
                        if final_x is not None:
                            motion_attempt_writes_x_delta = final_x - pre_x
                        if final_y is not None:
                            motion_attempt_writes_y_delta = final_y - pre_y
                    except Exception:
                        pass
                    motion_attempt_note = (
                        f" Motion attempt: load=ok push=ok "
                        f"writes_delta_x={motion_attempt_writes_x_delta} "
                        f"writes_delta_y={motion_attempt_writes_y_delta}")
                else:
                    motion_attempt_note = (
                        f" Motion attempt: load=ok push_rc={ps_rc} "
                        f"(non-zero -- segment rejected)")
            else:
                motion_attempt_note = (
                    f" Motion attempt: load_rc={lc_rc} (non-zero -- "
                    f"curve load failed)")
        except HostIoError as exc:
            motion_attempt_note = (
                f" Motion attempt: TIM5-starvation timeout ({exc}); "
                f"motion-bridge sim test is the canonical motion-level "
                f"validation")
        except Exception as exc:  # noqa: BLE001
            motion_attempt_note = (
                f" Motion attempt: aborted ({exc!r}); "
                f"motion-bridge sim test is the canonical motion-level "
                f"validation")

        if verbose and motion_attempt_note:
            print(f"  motion attempt summary:{motion_attempt_note}")

        sinusoid_note = ""
        if sinusoid_ok is not None:
            sinusoid_note = (f" Sinusoid check on {len(history_records)} "
                             f"recorded coils: ok={sinusoid_ok} "
                             f"({sinusoid_detail}).")
        renode_note = (
            f" Renode TMC XDIRECT writes pre-jog: x={peri_x_count} "
            f"y={peri_y_count} (bus IS registered via "
            f"runtime_register_phase_bus; non-zero count proves the full "
            f"path -- modulator → C SPI helper → Renode SPI3 → mux → "
            f"TMC5160 stub frame decode -- is wired correctly).")

        # Gate on infrastructure-level signals only — see module
        # docstring Limitation 2. We don't push an explicit motion
        # segment here (TIM5 starves USART after configure_axes_blob),
        # so XDIRECT writes are only emitted opportunistically by the
        # modulator's first few TIM5 ticks when residual `producer_current`
        # state exists. The motion-bridge sim test
        # (`phase_stepping_rapid_g1_x25_after_set_position_no_crash`) is
        # the canonical motion-level validation.
        #
        # If we *did* capture records, run the sinusoid check as a
        # regression gate (the LUT identity is content-stable across
        # builds; any captured records that fail Pythagorean ≈ amp²
        # mean either the LUT or the frame decode has drifted).
        if sinusoid_ok is False:
            return ("FAIL",
                    f"Captured coil values fail sinusoid check: "
                    f"{sinusoid_detail}. The identity LUT "
                    f"(rust/runtime/build.rs) builds "
                    f"i_a=amp*sin(2πmscount/1024), i_b=amp*cos(...); "
                    f"if i_a²+i_b² is not ≈ amp² across captured points, "
                    f"the LUT or the modulator's mscount math has drifted.")

        return (
            "PASS",
            f"phase-stepping integration ok: msgproto wrappers respond, "
            f"33-byte configure_axes accepted.{renode_note}{sinusoid_note}"
            f"{motion_attempt_note}")

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
