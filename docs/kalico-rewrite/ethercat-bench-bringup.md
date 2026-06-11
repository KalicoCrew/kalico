# EtherCAT servo bring-up (bench checklist)

> Practical guide for bringing the EtherCAT servo axis up at the bench.
> Companion to
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
# klippy SPAWNS the endpoint binary itself at claim time — you do not launch it.
[ethercat_node node_x]
socket: /tmp/kalico-ethercat.sock   # required; the Unix socket klippy connects on
interface: eth0                     # required; NIC the drive is wired to (raw EtherCAT)
# endpoint: optional. Absolute or repo-relative path to the binary klippy spawns.
#   Default: rust/target/release/kalico-ethercat-rt (the hw binary). Point this at
#   the stub for drive-off validation (see below).
#endpoint: rust/target/release/kalico-ethercat-rt-stub

# A position-commanded servo presented as the X axis. No step/dir, no microsteps.
[servo_x]
protocol: ethercat            # only 'ethercat' is supported
node: node_x                  # must match an [ethercat_node <name>]
rotation_distance: 40         # mm of axis travel per motor revolution (your mechanics)
encoder_counts_per_rev: 131072  # required; drive encoder counts per motor rev (A6-EC: 131072)
position_min: 0
position_max: 300
# Feedforward (optional; see servo-feedforward.md):
#velocity_ff: True              # stream 60B1h velocity feedforward
#dynamics_profile: dynamics_x.toml  # enables 60B2h torque feedforward
#ff_torque_clamp: 30.0          # torque-offset clamp, % of rated
# Homing (optional). With these set, G28 homes the servo axis against a GPIO
# endstop on any bridge MCU; without endstop_pin the axis has no endstop and
# G28 on it fails loudly.
#endstop_pin: PA13             # pin on the MCU that carries the switch
#position_endstop: 0           # must equal position_min or position_max
#homing_speed: 50
#homing_retract_dist: 5        # back-off after endstop contact (default 5, 0 disables)
#homing_retract_speed: 50      # back-off speed (default: homing_speed)
# Drive protection (homing-scoped: written to 6065h/6072h around each G28,
# restored after; a trip de-energizes the drive and fails the G28 loudly):
#homing_following_error: 2.5   # mm of commanded-vs-actual deviation (default 2.5)
#homing_max_torque: 50         # % of rated torque during homing (default 50)
# Session-wide variants (written once at bringup; unset = drive defaults):
#following_error: 10
#max_torque: 150
```

Bring-up now performs the **variable PDO remap** (1600h/1A00h via SDO in
PRE-OP; exit rc -6 on failure) and the **FF-routing SDO writes** (C01.13/16 =
5, C01.14/17 = 1000; exit rc -10 on failure) before the DC stabilize loop.
Both are rewritten on every claim — they are not EEPROM-retained. See
[`servo-feedforward.md`](servo-feedforward.md) for the FF config keys and
identification workflow.

`counts_per_mm = encoder_counts_per_rev / rotation_distance` — the `CountMap` gain
the endpoint uses to convert host millimetres to drive counts. klippy derives it at
claim time from `[servo_x]` and hands it to the spawned endpoint. Get both keys right
before the drive moves.

## Drive parameters (SDO)

Drive tuning lives in config, not drive EEPROM. `params:` entries on `[servo_*]`
are raw CoE object addresses pushed to drive RAM (never EEPROM) on every claim,
after bringup succeeds and before the claim is reported healthy. Each write is
read back; a mismatch (drive clamped or rejected the value) fails the claim with
the offending address, the value written, and what the drive settled on.

```ini
[servo_x]
# ... options above ...
params:
    0x2002.0: 100          # size probed via SDO upload (one extra mailbox round-trip)
    0x2003.0: u16 250      # explicit type (u8/u16/u32/i8/i16/i32) skips the probe
    0x2010.1: i32 -4096
```

Ad-hoc access while tuning:

```
SERVO_PARAM SERVO=servo_x GET=0x2002.0
SERVO_PARAM SERVO=servo_x GET=0x2002.0 TYPE=i16
SERVO_PARAM SERVO=servo_x SET=0x2002.0 VALUE=100 TYPE=u16
```

GET without `TYPE=` prints raw hex plus both unsigned and signed decimal
interpretations. SET reads the value back and reports what the drive settled on.
Objects wider than 4 bytes (strings, segmented transfers) are unsupported and
fail loudly. SDO traffic is mailbox traffic — it rides between DC cycles, fast
but not deterministic; anything needing hard-real-time parameter changes gets
mapped into the PDO instead.

To deliberately persist parameters to drive EEPROM, SET the drive's
store-parameters object (CiA 301 object 0x1010 — check the A6-EC manual for the
magic value) — kalico never does this implicitly.

The stub endpoint serves a small fake object dictionary (0x2002.0, 0x2003.0
clamping at 500, 0x2010.1, 0x6041.0 read-only), so the whole path — claim-push,
verify-mismatch claim failure, SERVO_PARAM — can be validated drive-off.

## Bring-up sequence

The endpoint is **spawned by klippy** at node-claim time (mcu-identify), using the
`endpoint:` path and the derived `counts_per_mm`. There is no manual endpoint launch
and no pre-flight script — you choose stub vs hw by setting `endpoint:`, then start
klippy.

### 1. Deploy + flash (low risk — stepper path unchanged)
- Commit/push the branch; pull on the Pi; build there (never cross-compile + scp).
- Flash **both** MCUs with their respective configs (H7 from `.config.h7.bak`, F446 from `.config.f446.test`); `make clean` between them. `make -j$(nproc)`.
- `CONFIG_MOTION_MODULE_STEPPER=y` is the default on STM32 — leave it on.

### 2. Build the endpoint binaries (on the Pi)
```sh
# hw endpoint — links bench/libecrt.a + SOEM. Build on the Pi (never in CI):
make -f Makefile.kalico ethercat-endpoint-hw
# Grant capabilities so it runs unprivileged. sudo, ONCE PER REBUILD of the binary:
make -f Makefile.kalico setcap-ethercat   # cap_net_raw, cap_sys_nice, cap_ipc_lock
# no-hardware stub (no FFI, no setcap needed):
make -f Makefile.kalico ethercat-stub
```

### 3. Stub validation FIRST (no drive — zero hardware risk)
Confirm the whole host path before energizing anything. Point the node at the stub:
```ini
[ethercat_node node_x]
socket: /tmp/kalico-ethercat.sock
interface: eth0
endpoint: rust/target/release/kalico-ethercat-rt-stub
```
- Start klippy. It **spawns the stub itself** at claim time — you do not launch it.
  Confirm klippy reaches **`ready`**.
- `SET_KINEMATIC_POSITION X=100`, then a small `G1 X105 F600`, `M400`.
- Watch the stub's stderr (klippy redirects it): `PushPieces` frames arrive, `retired`
  counts advance, `engine_state` stays running (1), never `Fault` (3). This proves
  planner → bridge → transport → endpoint end-to-end with no servo.

### 4. Real drive (supervised)
- Switch `endpoint:` back to the hw binary (or drop the key to use the default
  `rust/target/release/kalico-ethercat-rt`) and restart klippy.
- **Dark drive (powered off / disconnected):** with the drive as the only slave on
  the bus, a powered-off drive means SOEM finds no slaves at all (rc=-2); klippy
  fails the claim loudly with:

  > `ethercat node_x: EtherCAT bus on eth0: no slaves responding (bringup rc=-2) — check cable and drive power, then FIRMWARE_RESTART`

  If the drive IS found but fails the SAFE-OP/OP/CiA402-enable walk (rc=-3..-5),
  you get the per-drive variant instead:

  > `ethercat node_x: drive (slave 1) offline (bringup rc=-{N}) — check drive power, then FIRMWARE_RESTART`

  Power the drive on (and/or fix the cable), then `FIRMWARE_RESTART` — klippy re-spawns
  the endpoint and the claim succeeds, reaching `ready`.
- **Before the first move:** the endpoint captures the rotor's current count as the
  origin at first sample, so the first commanded position maps to the actual rotor
  position — there should be **no startup jump**. If the axis lurches on the first
  command, stop and check `encoder_counts_per_rev` / `rotation_distance` / origin
  capture.
- Do a small supervised jog. Watch for:
  - `engine_state == Fault (3)` in the `StatusHeartbeat` → the host pump fell behind >2 ms (`PieceStartInPast`). The endpoint latches the fault and propagates it so the host can shut down; the hw binary also disables the drive. This is expected on a gross stall, not on a healthy stream.
  - `wkc != 3` → EtherCAT bus working-counter fault (drive comms), the endpoint
    halts and dumps `al=0x…`. `al=0x001a` is a DC sync loss (ErC1.1) — see the
    real-time scheduling section; the usual cause is the loop not running
    `SCHED_FIFO` on the isolated core.

### 5. Recovery
- Any fault or claim failure is recovered the same way: fix the cause, then
  **`FIRMWARE_RESTART`**. klippy SIGTERMs the old endpoint (which cleanly disables the
  drive), re-spawns it, and re-runs the claim. There is no manual pre-launch, socket
  cleanup, or endpoint restart to do by hand.
- The old `bench-hw-up.sh` choreography is **obsolete** — klippy owns the endpoint
  lifecycle now. Delete that script from the bench host if it's still there.
- **A latched `0x8700` / ErC1.1 is the exception:** `FIRMWARE_RESTART` re-spawns
  the endpoint but does **not** clear the drive's stored sync-loss fault (the
  EtherCAT INIT bounce resets the network state machine, not the CiA402 fault).
  With the drive on its own supply it stays faulted across host restarts —
  **power-cycle the drive** to clear it, then fix the root cause (almost always
  RT scheduling; see that section) so it does not re-latch on the next boot.

## Real-time scheduling — mandatory (the ErC1.1 / "ErC11" trap)

The endpoint's 1 kHz DC loop **must** run `SCHED_FIFO` on an isolated CPU. This
is not best-effort. If it runs `SCHED_OTHER`, the loop keeps cadence on a warm,
idle Pi but misses SYNC0 under boot load — and the drive latches **ErC1.1
"synchronization loss"** (panel reads `ErC11`; CoE error register `0x8700`;
EtherCAT AL status `0x001a`, visible in the endpoint's `ec_rt: slave1 …
al=0x001a` dump on a working-counter halt). Because the drive is usually on its
own always-on supply, that latch **survives every host reboot**, so klippy's
auto-restart keeps re-claiming an already-faulted drive — only a **drive
power-cycle** clears `0x8700`. Classic signature: fails on cold boot / right
after a flash, "works once it's connected."

All three of the following are required, or `go_realtime()` aborts the claim
loudly — `rc=-10` (mlockall / `CAP_IPC_LOCK`), `rc=-11` (CPU pin), `rc=-12`
(`SCHED_FIFO` / `CAP_SYS_NICE`), each naming the missing capability. There is no
silent `SCHED_OTHER` fallback any more; that fallback was the original ErC11
heisenbug.

1. **`cap_sys_nice`** (for `SCHED_FIFO`) and **`cap_ipc_lock`** (for `mlockall`)
   on the endpoint binary: `make -f Makefile.kalico setcap-ethercat`. **A
   `cargo build` writes a fresh inode and drops file-caps**, so re-run setcap
   after *every* endpoint rebuild — the flash script re-applies it, a bare
   rebuild does not. Skipping it is the direct cause of "ErC11 after flashing".
2. **An isolated core to pin to.** The bench reserves CPUs 2-3 on the kernel
   cmdline (`isolcpus=domain,managed_irq,2-3 nohz_full=2-3 rcu_nocbs=2-3`); the
   endpoint pins to CPU `--rt-cpu` (default 3). An isolated core stays
   contention-free even while the rest of the Pi is saturated booting — that is
   what holds cadence through the cold-boot window that used to fault.
3. **`SCHED_FIFO`** at priority `--rt-prio` (default 80).

**Robust alternative to per-rebuild setcap** (ambient caps survive rebuilds; the
file-cap does not): grant the caps on the systemd service via a drop-in
`/etc/systemd/system/klipper.service.d/10-ethercat-rt.conf`, then
`systemctl daemon-reload`:

    [Service]
    AmbientCapabilities=CAP_SYS_NICE CAP_IPC_LOCK
    LimitRTPRIO=infinity
    LimitMEMLOCK=infinity

The spawned endpoint inherits the ambient caps from klippy. (If the binary also
carries file-caps, those take precedence and the ambient set reads back empty —
harmless, since file-caps already include `cap_sys_nice`.)

**Verify it is actually in force** — a green *warm* restart only proves the cap
took; only a **cold reboot** proves the loop holds cadence under boot load:

    pid=$(pgrep -f release/kalico-ethercat-rt)
    chrt -p $pid                                     # want: SCHED_FIFO priority 80
    grep Cpus_allowed_list /proc/$pid/status         # want: 3 (the isolated core)
    /usr/sbin/getcap rust/target/release/kalico-ethercat-rt   # want: ...cap_sys_nice=ep
    sudo journalctl -b | grep -c 'al=0x001a'         # want: 0

`SCHED_OTHER` + `cpus 0-1` on the live endpoint is the bug, not health — the cap
is not reaching it. (Endpoints sampled in their first ~200 ms read `SCHED_OTHER`
because `go_realtime()` runs just after `main()` startup; sample the
steady-state pid.)

## Variable RxPDO 0x1600 remap (rc=-6 / rc=-7)

`out_t` is the 18-byte variable RxPDO **`0x1600`** (`6040` controlword, `607A`
target position, `60B8` touch-probe function, `60FE:01` forced-DO, plus the two
feedforward offsets `60B1` velocity and `60B2` torque). The fixed `0x1701` the
bench used before carried only the first four — it physically can't hold the FF
offsets — so feedforward requires the variable map.

The drive can power up with a *different* RxPDO assigned to `0x1C12`, whose byte
count disagrees with `out_t` and aborts bringup at `EC_RT_ERR_PDO_SIZE` (`rc=-7`,
`ec_rt: PDO size mismatch — mapped out=… …`). Bringup now **forces** `0x1C12 →
0x1600` and rewrites the whole `0x1600` entry table before mapping (mirroring the
existing `0x1C13 → 0x1A00` TxPDO remap), so a clean 18-byte map no longer depends
on the drive's retained state. A failure to write that remap is `rc=-6`
(`EC_RT_ERR_PDO_REMAP`), the same code as the `0x1A00` TxPDO remap. See
[`servo-feedforward.md`](servo-feedforward.md) for the FF routing (C01 group →
60B1h/60B2h) that the FF entries feed.

## Fault-response reference
- **`PieceStartInPast`** (a piece adopted >2 ms late = 2× the 1 ms DC period): the walker faults, the endpoint latches it (allocation-free atomic) and reports `engine_state=Fault` to the host. Primary response is host-coordinated shutdown (mirrors the MCU model); the hw binary additionally disables the drive as a local backstop. It does **not** silently hold the last position.
- If `engine_state=Fault` fires during a *healthy* stream, the 2 ms tolerance may be too tight for your RT scheduling — that's a tuning knob (`EC_DC_PERIOD_NS` in `curves.rs`), not a logic change.

## If something's off
- Re-run `cargo test -p kalico-ethercat-rt -p motion-bridge` on the Pi — these are the host-path regression tests.
- The stub-level path (step 2) isolates host bugs from drive/EtherCAT bugs — always confirm it green before blaming the drive.
- Per-piece dispatch projection diagnostics (`[dispatch-margin]` and `[project]`) are emitted at **trace** level to avoid flooding production logs. Enable them with `RUST_LOG=trace` (or a targeted filter such as `RUST_LOG=motion_bridge=trace,kalico_host_rt=trace`). `RUST_LOG` is read by the `EnvFilter` in `rust/motion-bridge/src/logging/mod.rs` at bridge startup.
