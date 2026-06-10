# SERVO_PARAM — SDO access and declarative drive parameters

Date: 2026-06-10
Branch: servo-param

## Goal

Give the host raw CoE SDO read/write access to EtherCAT servo drives, through
two surfaces:

1. **Declarative param block** (the priority): per-servo drive parameters in
   `printer.cfg`, pushed to the drive at claim time. Drive tuning (gains,
   inertia ratio, filters) becomes version-controlled config — reproducible
   after a drive swap or factory reset instead of living in drive EEPROM as
   undocumented panel pokes.
2. **Console command**: `SERVO_PARAM` for ad-hoc GET/SET while tuning.

This is the foundation layer; future tools (autotuning, mode switching,
diagnostics) drive parameters through it.

## Decisions made during brainstorming

- **Raw addressing only** (`0xINDEX.SUB`), no named-parameter table. Generic
  across CoE drives, zero manual-derived tables to maintain.
- **Size is optional.** Without a type token, the endpoint probes the object
  with an SDO upload first (the CoE response carries the size), then writes —
  one extra mailbox round-trip. With an explicit type token
  (`u8/u16/u32/i8/i16/i32`) the probe is skipped — for latency-sensitive
  runtime pokes. Type tokens also pin signedness for display.
- **Read-back verify on every write.** After each write (claim-push or console
  SET), read the object back. Mismatch (drive clamped or rejected the value)
  → hard error carrying index, written value, read value. At claim time a
  mismatch is a **claim failure** — a drive that didn't accept the configured
  gains never reports healthy. (Fail-loudly constraint.)
- **EEPROM never touched.** Params land in drive RAM on every claim; that is
  the reproducibility model and avoids per-boot EEPROM wear. A deliberate
  save-to-EEPROM is just a documented SET to the drive's store-parameters
  object.
- SDO traffic is mailbox traffic: it rides between DC cycles — fast but not
  deterministic. Anything needing hard-real-time parameter changes gets mapped
  into the PDO instead (out of scope here).

## Config surface

A `params:` multi-line option in the existing `[servo_x]` section (params are
per-drive; per-drive config already lives there):

```ini
[servo_x]
protocol: ethercat
node: node_x
rotation_distance: 40
encoder_counts_per_rev: 131072
position_min: 0
position_max: 300
params:
    0x2002.0: 100          # probed size
    0x2003.0: u16 250      # explicit type, no probe
    0x2010.1: i32 -4096
```

- One `0xINDEX.SUB: [type] value` entry per line. Index is 16-bit hex,
  subindex decimal (or hex with `0x`), value decimal or hex.
- Pushed in file order after `ec_rt_bringup()` succeeds, before the claim is
  reported healthy.
- Negative values are two's-complemented into the discovered/declared width.
  A negative value with an unsigned type token is a config error.
- Any parse error, SDO abort, or verify mismatch → claim failure with the
  offending line and drive abort code.

## Console surface

One mux command registered per servo axis:

```
SERVO_PARAM SERVO=servo_x GET=0x2002.0 [TYPE=i16]
SERVO_PARAM SERVO=servo_x SET=0x2002.0 VALUE=100 [TYPE=u16]
```

- GET prints raw hex bytes plus unsigned and signed decimal interpretations
  (signedness unknown without a table); `TYPE=` picks a single interpretation.
- SET without `TYPE=` probes for size; with `TYPE=` writes directly. Both
  read back and report the settled value; mismatch → command error.

## Architecture

New SDO path through the four existing layers:

```
klippy/extras/servo_param.py       SERVO_PARAM command
klippy/extras/ethercat_node.py     parses params:, pushes at claim
        │ (cffi)
rust/motion-bridge/src/bridge.rs   sdo_read / sdo_write entry points
rust/motion-bridge/src/servo_sdo.rs  marshals over Unix socket
        │ (wire frames)
rust/kalico-protocol/src/messages.rs  SdoRead / SdoWrite (+ responses)
rust/kalico-ethercat-rt/src/wire.rs   codec entries
rust/kalico-ethercat-rt (endpoint)    executes in command-poll path
        │ (FFI)
bench/libecrt.c                    ec_rt_sdo_read / ec_rt_sdo_write
        │
SOEM ec_SDOread / ec_SDOwrite      CoE mailbox, serialized between
                                   process-data exchanges
```

### Protocol messages (`kalico-protocol`)

- `SdoRead { index: u16, subindex: u8 }`
  → `SdoReadResponse { result: i32, size: u8, data: [u8; 4] }`
- `SdoWrite { index: u16, subindex: u8, size: u8, data: [u8; 4] }`
  → `SdoWriteResponse { result: i32 }`
  (`size == 0` on SdoWrite = endpoint probes first.)
- `result` carries the CoE abort code on failure (0 = success). Objects
  larger than 4 bytes are out of scope (no strings/segmented transfers);
  endpoint returns a distinct error for them.

Probe-then-write and write-then-verify both execute **endpoint-side** as one
command, so each host request is a single socket round-trip and the
read-modify sequence can't interleave with another host command.

### FFI (`bench/libecrt.c`)

- `int ec_rt_sdo_read(uint16_t index, uint8_t sub, uint8_t *buf, int *size)`
- `int ec_rt_sdo_write(uint16_t index, uint8_t sub, const uint8_t *buf, int size)`

Thin wrappers over SOEM, fixed timeout, returning SOEM/abort status. Safe to
call while the DC loop runs (SOEM mailbox traffic coexists with cyclic data);
calls are made from the endpoint's single command-poll thread, so no new
locking.

### Endpoint behavior

- `SdoWrite` handler: optional probe → encode → `ec_rt_sdo_write` → read-back
  → compare → respond (success, or abort code + observed bytes).
- `SdoRead` handler: `ec_rt_sdo_read` → respond with size + bytes.
- Stub endpoint (`-stub` binary) grows an in-memory fake object dictionary
  (a few preloaded objects with fixed sizes, one read-only object, one
  value-clamping object) so claim-push, verify, and failure paths run in the
  sim without hardware.

## Error handling

Fail loudly, everywhere:

- SDO abort during claim-push → claim failure, log structured event with
  index/subindex/abort code via `kalico_log_emit`.
- Verify mismatch → error with wrote/read values (claim: fatal; console:
  command error).
- Probe on a write-only object → abort code surfaces with a hint to add an
  explicit type token.
- Object size > 4 bytes → explicit "unsupported object size" error.
- No retries, no silent clamping, no skip-and-continue.

## Testing

- Wire codec round-trip tests for the four new messages (`kalico-protocol` /
  `wire.rs`), run via `cargo nextest run`.
- Stub-endpoint integration: claim with a `params:` block → verify dictionary
  state; clamping object → claim fails; read-only object → claim fails;
  typed write skips probe (assert via stub's probe counter).
- klippy-side parse tests for the `params:` block grammar (bad index, bad
  type token, negative-with-unsigned).

## Out of scope

- Named parameter tables (revisit if raw addressing proves painful).
- Objects > 4 bytes / segmented SDO transfers / strings.
- PDO remapping for hard-real-time parameter changes.
- EEPROM store orchestration.
