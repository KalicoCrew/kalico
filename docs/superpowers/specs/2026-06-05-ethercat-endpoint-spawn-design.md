# EtherCAT endpoint: spawn-on-claim lifecycle

## Problem

Bringing the EtherCAT servo bench up requires a hand-rolled shell script
(`bench-hw-up.sh`) that stops klipper, launches the endpoint as root in a
retry loop until the drive reaches the DC loop, then starts klipper. The
ordering is manual, the endpoint needs sudo, a drive-off start produces a
silent `exit(1)` instead of an actionable error, and a stale endpoint binary
can keep running across rebuilds. None of this matches how the rest of the
system behaves: a missing serial MCU produces a clear klippy error, gets
fixed, and `FIRMWARE_RESTART` recovers.

## Decision

klippy owns the endpoint's lifecycle. No systemd unit, no wrapper script.

- **Spawn at claim.** During klippy's connect phase, the bridge's
  `claim_ethercat_node` launches the endpoint process, waits for its Unix
  socket, connects, and handshakes. The endpoint binds the socket, runs SOEM
  bringup, enables the drive (CiA402 operation-enabled), and enters the DC
  loop — the existing flow, but triggered by the claim instead of preceding
  it.
- **Named errors, not connect errors.** Bringup failure answers the
  handshake with a structured per-drive outcome, then the endpoint exits.
  klippy surfaces a message naming the drive, not the node:
  - endpoint failed to start (binary missing / caps not set) — infra error
    naming the binary path;
  - bus dead — `ethercat node_x: no slaves responding on eth0`;
  - drive offline/faulted — `ethercat node_x: drive 'x' (slave 1) offline —
    check drive power, then FIRMWARE_RESTART`.
- **Enable at claim, disable at release.** The drive holds torque only while
  a klippy session owns it. On klippy shutdown, `FIRMWARE_RESTART`,
  disconnect, or SIGTERM the endpoint disables the drive (controlword
  0x0006) and exits. The clean disable also eliminates the `ErC1.1`
  sync-loss latch from abrupt SYNC0 loss.
- **Recovery = cold start.** After any error or fault shutdown,
  `FIRMWARE_RESTART` spawns a fresh endpoint and re-runs bringup. There is
  exactly one bring-up path.

## Components

### Endpoint (`rust/kalico-ethercat-rt`)

- Keep socket-bind-before-bringup order. Replace `exit(1)` on bringup
  failure with: serve the handshake, report the per-drive outcome, exit.
- SIGTERM handler and socket-disconnect detection both run the same
  shutdown: `ec_rt_disable()` → `ec_rt_shutdown()` → exit. A SIGKILLed
  klippy therefore cannot leave a live DC loop behind (socket drop reaps
  it).
- Runs unprivileged. The build applies
  `setcap cap_net_raw,cap_sys_nice,cap_ipc_lock+ep` to the binary (raw
  EtherCAT socket, `SCHED_FIFO`/affinity, `mlockall`).

### Wire protocol (`kalico-native-transport` / handshake)

- The claim handshake reply gains a per-slave status list:
  `[(slave_idx, state)]` where state ∈ ok / offline / fault(code). The shim
  serving exactly slave 1 is an implementation detail behind the format —
  multi-drive buses later do not change the wire shape.
- Fail loudly: an unknown state value or a reply missing the status list is
  a hard protocol error, not a default-to-ok.

### Bridge (`rust/motion-bridge`)

- `claim_ethercat_node` gains spawn duties: launch the configured binary
  with `<interface>`, `--socket <path>`, and `--counts-per-mm` computed from
  the servo config (rotation_distance + encoder counts/rev) instead of the
  binary's hardcoded default;
  poll for the socket with a bounded deadline; connect; handshake; map the
  per-slave statuses to typed errors for klippy. Bounded deadlines on both
  socket appearance and handshake reply — a hung endpoint is killed and
  reported, never waited on indefinitely.
- On release/restart: close the socket, send SIGTERM, reap with a bounded
  wait, SIGKILL as backstop.

### klippy (`klippy/extras/ethercat_node.py`)

- `[ethercat_node]` config gains `interface` (required) and `endpoint`
  (binary path; default: the repo-relative release binary). The existing
  `socket` option stays.
- Claim errors raise klippy's standard startup error with the named-drive
  message verbatim.

### Bench flow

- `bench-hw-up.sh` is deleted from the bench host.
- New flow: power anything in any order; start klipper. Dark drive →
  named error → power the drive → `FIRMWARE_RESTART`.
- The stub rides the same machinery: a bench config pointing `endpoint:` at
  `kalico-ethercat-rt-stub` gives drive-off testing with the identical
  lifecycle. No special-casing in the bridge.
- `docs/kalico-rewrite/ethercat-bench-bringup.md` updated to the new flow.

## Mid-session faults (unchanged)

`wkc != 3` halts the endpoint; the `PieceStartInPast` fault latch propagates
`engine_state=Fault` to the host, which shuts down. Both stay exactly as
implemented. The endpoint's local drive-disable backstop on fault stays.

## Out of scope (recorded for later)

De-energize-and-track: disabling servo torque while continuing to read
`position_actual`, so a paused print's toolhead can be hand-moved and the
print resumed with a re-anchor. The disable primitive added here is the
building block; the host-side flow is future work.

## Testing

- Existing wire/walker/streaming tests untouched.
- New integration test (stub): claim spawns the process, handshake
  succeeds, disconnect terminates it (no orphan).
- New failure test: a bringup-failure outcome propagates the named-drive
  error into the claim reply (stub gains a `--fail-bringup slave=1` switch
  to simulate).
- Hardware validation per the updated bring-up doc: dark-drive start shows
  the named error; power + `FIRMWARE_RESTART` reaches ready; SIGTERM leaves
  the drive disabled with no `ErC1.1` on the next session.
