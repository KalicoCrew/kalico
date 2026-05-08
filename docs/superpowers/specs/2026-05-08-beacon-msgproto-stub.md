# Faithful Beacon msgproto stub — design

**Status:** approved 2026-05-08
**Parent spec:** `docs/superpowers/specs/2026-05-08-faithful-klippy-sim-design.md`
**Replaces:** the ASCII-passthrough scaffold in `tools/sim_klippy/orchestrator/beacon_serial_stub.py`

## Goal

Replace the ASCII-passthrough beacon stub with a faithful msgproto-speaking
endpoint so klippy's beacon plugin can complete identify-handshake,
`_build_config()`, NVM reads, threshold setup, sample streaming, and
proximity-homing trip events. Test gates: `test_boot` reaches "ready"
with the `[beacon]` section live; downstream `test_g28_full` runs
`G28 Z` via beacon proximity homing without changes to this stub.

## Why faithful, not bypass

The `[beacon]` section dropping out of the rendered printer.cfg was the
original signal-hiding workaround that the `f6c5d7d09` revert removed.
A faithful stub is the only way the sim catches host-side beacon-API
regressions of the bricking class (`clear_homing_state("z")` from
beacon's compat layer) before they reach hardware.

## Architecture

```
┌─────────── orchestrator process (Python) ─────────────┐
│                                                       │
│  BeaconMcuStub                                        │
│  ├── klippy.msgproto.MessageParser                    │
│  │     • identify_dict (zlib-compressed JSON)         │
│  │     • encode_msgblock / parse / check_packet       │
│  ├── reactor thread (select on PTY master fd)         │
│  │     • drain inbound bytes → parser.check_packet    │
│  │     • dispatch by #name                            │
│  ├── outbound seq counter; ack/reply via              │
│  │   parser.encode_msgblock                           │
│  ├── command handlers                                 │
│  │     • core: identify, get_uptime, get_clock,       │
│  │       get_config, allocate_oids, finalize_config,  │
│  │       emergency_stop, debug_*                      │
│  │     • beacon: stream, set_threshold, home,         │
│  │       stop_home, nvm_read, contact_home,           │
│  │       contact_query, contact_stop_home,            │
│  │       contact_set_latency_min,                     │
│  │       contact_set_sensitivity                      │
│  ├── 1.6 kHz sample-stream thread                     │
│  │     • emits beacon_data + beacon_status            │
│  │     • z_target driven by orchestrator.set_z()      │
│  ├── proximity homing trip                            │
│  │     • when streaming + beacon_home_active +        │
│  │       z ≤ trigger_distance → emit beacon_status    │
│  │       with last_trigger_clock to fire trsync       │
│  └── PTY master fd (existing — keep)                  │
│                                                       │
│  /tmp/klipper_sim_beacon  ← symlinked PTY slave       │
└───────────────────────────────────────────────────────┘
```

## Components

### 1. Override-layer addition

`tools/sim_klippy/orchestrator/overrides.py` and `pin-overrides.toml`:
inject `skip_firmware_version_check = True` into the `[beacon]` section
of the rendered printer.cfg. Without this, beacon spawns a subprocess
running `update_firmware.py` which tries to talk DFU to the PTY and
hangs the boot for 30 s before failing identify.

The override is a per-section key insert (not a string replacement) so
it survives if the user updates printer.cfg structure. Mechanism:
extend `apply_overrides` with a `[<section>.config_inject]` table type:

```toml
[beacon.config_inject]
skip_firmware_version_check = "True"
```

Implementation: parse the rendered config with `configparser`, set the
key on the matching section, write back. Keep ordering stable (load
into ordered dict, modify, render).

### 2. Identify dictionary

A JSON blob shipped as a Python const string in
`orchestrator/beacon_identify_dict.py`. Top-level keys:

- `app`: `"BeaconStub"`
- `version`: `"v0.0.0-sim"`
- `build_versions`: `""`
- `enumerations`:
  - `pin`: `{"gpio0": [0, 32]}`  (klippy needs at least one)
  - `static_string_id`: minimal set
- `commands`: every command beacon.py looks up via
  `_mcu.lookup_command` or `_mcu.lookup_query_command`, plus klippy
  core commands MCU initialization needs:
  - `identify offset=%u count=%c`
  - `get_uptime`
  - `get_clock`
  - `get_config`
  - `allocate_oids count=%c`
  - `finalize_config crc=%u`
  - `emergency_stop`
  - `debug_nop`, `debug_ping data=%*s`, `debug_read order=%c addr=%u`,
    `debug_write order=%c addr=%u val=%u`, `clear_shutdown`
  - `beacon_stream en=%u`
  - `beacon_set_threshold trigger=%u untrigger=%u`
  - `beacon_home trsync_oid=%c trigger_reason=%c trigger_invert=%c`
  - `beacon_stop_home`
  - `beacon_nvm_read len=%c offset=%hu`
  - `beacon_contact_home trsync_oid=%c trigger_reason=%c trigger_type=%c`
  - `beacon_contact_query`
  - `beacon_contact_stop_home`
  - `beacon_contact_set_latency_min latency_min=%c`
  - `beacon_contact_set_sensitivity sensitivity=%c`
  - (Accel commands omitted — `BEACON_HAS_ACCEL=0` in config.)
- `responses`: every response beacon.py registers, plus klippy core:
  - `identify_response offset=%u data=%.*s`
  - `uptime high=%u clock=%u`
  - `clock clock=%u`
  - `config is_config=%c crc=%u is_shutdown=%c move_count=%hu`
  - `is_shutdown static_string_id=%hu`
  - `stats count=%u sum=%u sumsq=%u`
  - `shutdown clock=%u static_string_id=%hu`
  - `pong data=%*s`
  - `debug_result val=%u`
  - `beacon_data bytes=%*s`
  - `beacon_status clock=%u sample=%i frequency=%u temp=%hi`
  - `beacon_contact triggered=%c clock=%u sample=%i frequency=%u`
  - `beacon_nvm_data bytes=%*s offset=%hu`
  - `beacon_contact_state triggered=%c detect_clock=%u`
- `output`: `[]`
- `config`:
  - `MCU`: `"beacon"`
  - `CLOCK_FREQ`: `20000000`
  - `STATS_SUMSQ_BASE`: `1`
  - `ADC_MAX`: `4095`
  - `BEACON_ADC_SMOOTH_COUNT`: `8`
  - `BEACON_HAS_ACCEL`: `0`

The dict is constructed in code (cleaner diffs than a JSON literal),
serialized to JSON, then zlib-compressed. The compressed bytes are
served in chunks via `identify` requests.

The exact `commands`/`responses` keys must be **byte-identical** to
what beacon.py looks up — extra whitespace or arg-name differences
fail `lookup_command`. Source of truth: grep beacon.py for
`lookup_command(` and `lookup_query_command(`.

### 3. Frame plumbing

`BeaconMcuStub`:

```python
class BeaconMcuStub:
    def __init__(self, pty_path, log_path=None):
        self._parser = MessageParser(warn_prefix="beacon-stub: ")
        self._parser.process_identify(IDENTIFY_BLOB, decompress=True)
        self._send_seq = 1
        self._recv_seq = 1
        self._inbuf = bytearray()
        # … pty fd setup unchanged …

    def _on_recv(self, data: bytes):
        self._inbuf.extend(data)
        while True:
            msglen = self._parser.check_packet(self._inbuf)
            if msglen <= 0:
                if msglen < 0:
                    # Resync to next sync byte
                    sync_idx = self._inbuf.find(MESSAGE_SYNC)
                    self._inbuf = self._inbuf[sync_idx + 1:] if sync_idx >= 0 else bytearray()
                return
            frame = bytes(self._inbuf[:msglen])
            del self._inbuf[:msglen]
            params = self._parser.parse(list(frame))
            self._dispatch(params)

    def _send(self, msgformat: str, **kwargs):
        msg = self._parser.lookup_command(msgformat).encode_by_name(**kwargs)
        block = self._parser.encode_msgblock(self._send_seq, msg)
        self._send_seq += 1
        os.write(self._master_fd, bytes(block))
```

### 4. Command handlers

| Inbound | Handler |
|---|---|
| `identify offset=N count=K` | reply chunk of compressed dict bytes |
| `get_uptime` | reply `uptime high=H clock=C` from monotonic clock |
| `get_clock` | reply `clock clock=C` |
| `get_config` | reply `config is_config=1 crc=0 is_shutdown=0 move_count=0` |
| `allocate_oids` / `finalize_config` | no reply (commands, not queries) |
| `beacon_stream en=1` | start sample thread |
| `beacon_stream en=0` | stop sample thread |
| `beacon_set_threshold trigger untrigger` | store thresholds |
| `beacon_home trsync_oid trigger_reason trigger_invert` | enter homing state |
| `beacon_stop_home` | exit homing state |
| `beacon_nvm_read len offset` | reply from NVM blob |
| `beacon_contact_*` | ack only (proximity path used for first print) |
| `emergency_stop` | latch is_shutdown=1, reply via stats |
| `debug_*` | ack |

### 5. Sample streaming

When `_stream_en`:
- 1.6 kHz timer (Python threading.Timer or async loop)
- emits `beacon_status` with the packed sample
- `frequency` field: derived from `z_target` via inverse of beacon's
  model_coef polynomial — or, simpler, a constant in the model's
  expected range that decodes back to ≈ z_target. The first-print path
  doesn't run model calibration so a constant frequency is fine.
- `temp` = 25 °C in beacon's fixed-point format
- `sample` = monotonic counter

### 6. NVM blob

Hand-crafted byte string mirroring beacon firmware's NVM layout:

- bytes 0..15: serial string `"BEACONSIM       "` (16 bytes)
- bytes 16..23: model_temp (4 bytes), reserved
- bytes 24..N: packed model coefficients from
  `printer_real/config/printer.cfg` `[beacon model default]`

Source of truth for layout: read `BeaconMCUTempHelper.build_with_nvm`
and `BeaconModelHelper.build_with_nvm` in beacon.py; whatever offsets
those parse, our blob writes the same. Verified by an integration test
that loads beacon and asserts `beacon.models["default"]` equals the
config-file model.

### 7. Proximity homing

On `beacon_home`, store `(trsync_oid, trigger_reason)` and enter
homing-active state. Each sample-stream tick checks
`z_target ≤ trigger_distance`:
- if true and homing-active: emit `beacon_contact triggered=1 …`,
  clear homing-active, klippy's trsync sees the trip via the response
  callback and stops the homing move.

Orchestrator drives `z_target` via `BeaconMcuStub.set_z(z_mm)`. The
sensorless trigger mechanism in the existing `sensorless_trigger.py`
already polls `runtime_handle_step_count` for X/Y; this stub adds the
analogous Z poll, converting Z step count to mm via Z `rotation_distance`
from the printer.cfg.

## Test plan

### A. Unit: identify-handshake

`tools/sim_klippy/tests/test_beacon_msgproto_stub.py`:

1. `test_identify_chunked_returns_full_dict` — open PTY, send a
   sequence of `identify offset count` requests; assemble replies;
   zlib-decompress; assert keys and beacon command names present.
2. `test_msgproto_frame_roundtrip` — send a `get_uptime` frame, assert
   `uptime` reply.
3. `test_beacon_nvm_read_returns_model_bytes` — drive `beacon_nvm_read`
   for the model region; assert bytes match the configured model.

### B. Integration: boot

`test_boot.py` reaches "ready" with `[beacon]` live. Asserts:
- klippy.log contains `mcu 'beacon': Loaded MCU`
- no `Unhandled exception during connect` mentioning beacon
- no `Unable to communicate with beacon`

### C. Integration: G28 Z proximity (deferred to follow-up commit)

`test_g28_full.py` runs `G28 Z`. Expected behavior: beacon enters
homing, orchestrator-driven Z position decreases past `trigger_distance`,
trsync trips, `homed_axes` includes z. Out of scope for the beacon
commit — depends on TMC CS-pin discrimination + sensorless trigger
extension to Z.

## Failure modes covered

- klippy's beacon `_build_config()` `lookup_command` mismatch (any
  drift between beacon.py's expected commands and the stub dict)
- bricking-class `clear_homing_state("z")` regression (today's case
  that motivated the sim) — beacon `_handle_connect` runs against the
  faithful stub
- klippy MCU clock-sync / stats-frame regressions on a non-trivial
  msgproto endpoint (the H7 sim ELF isn't enough — beacon is its own
  endpoint shape)

## Out of scope

- accel data path (`BEACON_HAS_ACCEL=0` in config; no
  `beacon_accel_*` traffic)
- contact homing semantics (proximity is the configured method; contact
  commands are acked-only)
- model auto-calibration sequence
- axis-twist-compensation interaction
- temperature model accuracy (constant 25 °C is fine for boot+G28)
- mcu_temp / supply_voltage extended stats accuracy

## Implementation order

1. Override-layer `config_inject` mechanism + `[beacon].skip_firmware_version_check = True`
2. Identify dictionary (no streaming, no NVM)
3. `BeaconMcuStub` frame plumbing + `identify` chunked reply
4. Core MCU command handlers (`get_uptime`, `get_clock`, `get_config`, allocate/finalize)
5. Beacon command stubs (ack-only) — boot reaches "ready" gate
6. NVM blob + `beacon_nvm_read` — beacon `_build_config` reaches end
7. Sample stream + `beacon_status` — beacon `_stream_en=1` gate
8. Proximity homing trip — G28 Z follow-up
9. Unit tests
10. Integration test (`test_boot.py`) green

Each step is a self-contained sub-commit if scope warrants; minimum
shipping unit is steps 1–6 + 9 (boot reaches "ready" with full beacon).
Step 7 enables sustained streaming. Step 8 lands separately with the
G28 Z work.
