# EtherCAT servo bring-up (bench checklist)

> Practical guide for bringing the EtherCAT servo axis up at the bench on the
> `motion-node-unification` branch. Companion to
> [`motion-node-unification.md`](motion-node-unification.md) (the design).
> The trajectory math, fault handling, streaming, and stepper-path safety are
> already verified off-bench (see that doc); this is the on-hardware sequence.

## What's already proven without hardware
- MCU stepper hot-path codegen is **byte-identical** to pristine sota-motion (disasm-verified). Flashing this branch will not change stepper behavior.
- The servo runs the **same hardened walker** as the MCU (`runtime::motion_core`); its trajectory eval, origin/no-jump mapping, piece-boundary continuity, and the `PieceStartInPast` fault boundary are unit-tested.
- Sustained streaming past one ring depth works over the real `UnixNativeConn ↔ FrameServer` socket (no stall — the "stopped after first move" class is covered).
- `klippy → motion-bridge → endpoint` host wiring is ported and the stepper-path tests still pass.

## Stub validation: results (2026-06-01, no second MCU)

Validated the whole host path on the Pi 3B with **no second STM32** — a
Linux-process MCU (`klipper_mcu`, MACH_LINUX build) as the primary clock + Y/Z
steppers, and the EtherCAT servo on X talking to the `kalico-ethercat-rt-stub`.

**Proven end-to-end (servo path):**
- klippy reaches `ready`; `[ethercat_node]` + `[servo_x]` parse; the servo axis
  binds correctly (X excluded from stepper `runtime_bindings`: `present=0x6`,
  `steps_per_mm[0]=0`); the bridge claims the node (`claimed handle=1`) and
  `UnixNativeConn` connects to the stub (`client connected`).
- `SET_KINEMATIC_POSITION` → position updates, axes home (`xyz`), no crash.
- `G1 X…` streams `PushPieces` to the endpoint; the stub's `retired_count`
  advances steadily; `M400` drain completes for servo-only moves; the endpoint
  **never faults** (`engine_state` stays running).

**Bug found + fixed during this validation** (commit `5ad6e3568`): the
`set_position` seed loop assumed every motion node is a serial MCU with a
`host_io` and aborted the klippy host with **SIGABRT** (`bridge.rs` panic
`set_position seed: mcu_id N has no host_io`) the moment an EtherCAT node was
present. EtherCAT endpoints self-seed their origin from the encoder at first
sample and were already re-seeded by `kalico_stream_open`, so the stepper-only
serial `runtime_seed_position` must be skipped for them (`build_serial_seed_sends`
filters EtherCAT `mcu_id`s; fail-loud panics preserved for genuine invariants).
This bug was **MCU-independent** — it would have crashed the real-drive bench
too, so catching it without the drive was the point of the stub step.

**Linux soft-MCU `TickIntervalExceeded` (-311) — root-caused and fixed.** The
Y/Z stepper path on the Pi-3B Linux MCU faulted `-311` (retired 0 pieces) because
the host tick loop ran at a hardwired `HOST_TICK_HZ=1000` while the MACH_LINUX
sample-rate default was 10000 — a 10× mismatch between the engine's expected
`sample_period` and the actual inter-tick gap, which trips the guard on the first
active tick. The MACH_LINUX first-class-MCU work fixes this: `HOST_TICK_HZ` now
derives from `CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ`, whose MACH_LINUX default is
1000 (the stock-kernel `clock_nanosleep` floor), and the tick thread inherits the
process SCHED_FIFO (`klipper_mcu -r`) instead of self-demoting to nice+19. A real
STM32 (hardware TIM5) never had this issue.

**Verified on the Pi 3B (2026-06-01)** with the real non-sim build (`mcu-linux`,
`KALICO_SIM=n`): klippy reaches `ready` driving real `/dev/gpiochip0`, and a
stepper move now *executes* (`G1 Y110` moved Y 100→110) where the old binary
retired 0 pieces and instant-faulted `-311`. The runtime `-311` is resolved. A
**separate** residual remains: *sustained* motion still trips Klipper's **base**
scheduler guard (`Rescheduled timer in the past`, `src/linux/timer.c` / `sched.c`
— NOT the runtime `-311`) because the soft timer jitters under load on a
non-PREEMPT_RT kernel. Reliable sustained Linux-MCU stepping therefore needs a
PREEMPT_RT kernel (or the parallel soft-MCU timing work); the EtherCAT servo
path — the actual bench target — is unaffected and streams cleanly. See
[`docs/superpowers/specs/2026-06-01-mach-linux-first-class-mcu-design.md`](../superpowers/specs/2026-06-01-mach-linux-first-class-mcu-design.md).

**Building a real (non-sim) Linux MCU:** `make` with `CONFIG_MACH_LINUX=y` and
`CONFIG_KALICO_SIM` **unset** drives real `/dev/gpiochip` / `/dev/spidev`
(`test/configs/kalico-linux.config`). `CONFIG_KALICO_SIM=y` (`.config.linux`,
used by `tools/sim_klippy`) selects the in-memory sim shims. The Rust `mcu-linux`
feature carries the f64 host numeric profile plus the real-firmware marker that
links the C step/SPI FFI. Note: raw STEP/DIR GPIO pulse emission on a Linux MCU
is a follow-on; TMC phase-stepping over SPI is the supported real-hardware
stepping path today.

## Sample config

```ini
# The EtherCAT motion endpoint, reached over a Unix socket (NOT a Klipper MCU).
[ethercat_node node_x]
socket: /tmp/kalico-ethercat.sock

# A position-commanded servo presented as the X axis. No step/dir, no microsteps.
[servo_x]
protocol: ethercat            # only 'ethercat' is supported
node: node_x                  # must match an [ethercat_node <name>]
rotation_distance: 40         # mm of axis travel per motor revolution (your mechanics)
position_min: 0
position_max: 300
```

`rotation_distance` + the drive's encoder counts/rev determine `counts_per_mm` (the `CountMap` gain on the endpoint). Get this right before the drive moves.

## Bring-up sequence

### 1. Deploy + flash (low risk — stepper path unchanged)
- Commit/push the branch; pull on the Pi; build there (never cross-compile + scp).
- Flash **both** MCUs with their respective configs (H7 from `.config.h7.bak`, F446 from `.config.f446.test`); `make clean` between them. `make -j$(nproc)`.
- `CONFIG_MOTION_MODULE_STEPPER=y` is the default on STM32 — leave it on.

### 2. Stub validation FIRST (no drive — zero hardware risk)
Run the no-hardware endpoint and confirm the whole host path before energizing anything:
```sh
# on the Pi, drive OFF / disconnected:
cargo run -p kalico-ethercat-rt --bin kalico-ethercat-rt-stub -- --socket /tmp/kalico-ethercat.sock
```
- Start klippy with the config above. Confirm it reaches **`ready`**.
- `SET_KINEMATIC_POSITION X=100`, then a small `G1 X105 F600`, `M400`.
- Watch the stub's stderr: `PushPieces` frames arrive, `retired` counts advance, `engine_state` stays running (1), never `Fault` (3). This proves planner → bridge → transport → endpoint end-to-end with no servo.

### 3. Real drive (supervised)
- Swap to the hardware endpoint: `cargo run -p kalico-ethercat-rt --features hw --bin kalico-ethercat-rt -- ...` (needs libecrt/SOEM on the Pi).
- **Before the first move:** the endpoint captures the rotor's current count as the origin at first sample, so the first commanded position maps to the actual rotor position — there should be **no startup jump**. If the axis lurches on the first command, stop and check `counts_per_mm` / origin capture.
- Do a small supervised jog. Watch for:
  - `engine_state == Fault (3)` in the `StatusHeartbeat` → the host pump fell behind >2 ms (`PieceStartInPast`). The endpoint latches the fault and propagates it so the host can shut down; the hw binary also disables the drive. This is expected on a gross stall, not on a healthy stream.
  - `wkc != 3` → EtherCAT bus working-counter fault (drive comms), the endpoint halts.

## Fault-response reference
- **`PieceStartInPast`** (a piece adopted >2 ms late = 2× the 1 ms DC period): the walker faults, the endpoint latches it (allocation-free atomic) and reports `engine_state=Fault` to the host. Primary response is host-coordinated shutdown (mirrors the MCU model); the hw binary additionally disables the drive as a local backstop. It does **not** silently hold the last position.
- If `engine_state=Fault` fires during a *healthy* stream, the 2 ms tolerance may be too tight for your RT scheduling — that's a tuning knob (`EC_DC_PERIOD_NS` in `curves.rs`), not a logic change.

## If something's off
- Re-run `cargo test -p kalico-ethercat-rt -p motion-bridge` on the Pi — these are the host-path regression tests.
- The stub-level path (step 2) isolates host bugs from drive/EtherCAT bugs — always confirm it green before blaming the drive.
