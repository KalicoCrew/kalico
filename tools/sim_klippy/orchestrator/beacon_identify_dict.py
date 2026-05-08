"""Identify dictionary served by the BeaconMcuStub.

Klippy's MCU connect path runs `identify offset count` queries against
every serial endpoint and parses the resulting zlib-compressed JSON to
discover the command/response inventory of that endpoint. This module
constructs a dictionary that is byte-equal to what the beacon firmware
would advertise — i.e. every command/response format string beacon.py
calls `lookup_command()` / `lookup_query_command()` for, plus the core
klippy MCU bring-up commands (identify, get_uptime, get_clock,
get_config, allocate_oids, finalize_config, debug_*, emergency_stop).

The exact format strings must match beacon.py byte-for-byte: msgproto
matches by full ``"name arg1=%type arg2=%type"`` string, not by name
alone. Source of truth is `printer_real/third_party_repos/beacon_klipper
/beacon.py`. Drift in either direction (extra whitespace, renamed args)
breaks `lookup_command` at klippy connect time.

Layout:

* ``commands`` — anything klippy or beacon will *send* to the MCU.
  msgid space starts at 1 (id 0 is reserved by msgproto for
  ``identify_response`` per ``DefaultMessages``; identify itself
  is id 1).
* ``responses`` — anything the MCU will send back.
* ``output`` — debug ``output()`` formats (none used here).
* ``enumerations`` — at minimum a ``pin`` enumeration (klippy core
  expects one; we expose a single dummy gpio range so the parser is
  happy without us emulating real beacon GPIO).
* ``config`` — firmware constants beacon.py reads at ``_build_config``:
  ``CLOCK_FREQ``, ``ADC_MAX``, ``BEACON_ADC_SMOOTH_COUNT``,
  ``BEACON_HAS_ACCEL``, plus the ``MCU`` identifier klippy logs.
"""

from __future__ import annotations

import json
import zlib

# ---------------------------------------------------------------------------
# Wire format strings — keep byte-exact with beacon.py / klippy core.
# ---------------------------------------------------------------------------

# msgid 0 = identify_response, msgid 1 = identify (per msgproto.DefaultMessages).
# Everything below uses a contiguous id range starting at 2.

CORE_COMMANDS = [
    # Klippy core MCU bring-up.
    "get_uptime",
    "get_clock",
    "get_config",
    "allocate_oids count=%c",
    "finalize_config crc=%u",
    "emergency_stop",
    "clear_shutdown",
    "debug_nop",
    "debug_ping data=%*s",
    "debug_read order=%c addr=%u",
    "debug_write order=%c addr=%u val=%u",
    # trsync — every klippy MCU object instantiates a `MCU_trsync`
    # (klippy/mcu.py:208) which looks up these formats verbatim. The
    # firmware-side authoritative source for the strings is
    # src/trsync.c (DECL_COMMAND lines for config_trsync, trsync_start,
    # trsync_set_timeout, trsync_trigger) plus src/stepper.c
    # (stepper_stop_on_trigger). Beacon presents itself as a full MCU
    # so this surface must be exposed even though beacon's homing path
    # uses its own beacon_home / beacon_contact_home commands — klippy
    # still allocates a trsync OID per MCU at config time.
    "config_trsync oid=%c",
    "trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c",
    "trsync_set_timeout oid=%c clock=%u",
    "trsync_trigger oid=%c reason=%c",
    "stepper_stop_on_trigger oid=%c trsync_oid=%c",
]

BEACON_COMMANDS = [
    "beacon_stream en=%u",
    "beacon_set_threshold trigger=%u untrigger=%u",
    "beacon_home trsync_oid=%c trigger_reason=%c trigger_invert=%c",
    "beacon_stop_home",
    "beacon_nvm_read len=%c offset=%hu",
    "beacon_contact_home trsync_oid=%c trigger_reason=%c trigger_type=%c",
    "beacon_contact_query",
    "beacon_contact_stop_home",
    "beacon_contact_set_latency_min latency_min=%c",
    "beacon_contact_set_sensitivity sensitivity=%c",
]

CORE_RESPONSES = [
    # `identify_response` is the default-message id 0; we still list it so
    # MessageParser._init_messages re-installs it into the by-id table
    # alongside the others.
    "identify_response offset=%u data=%.*s",
    "uptime high=%u clock=%u",
    "clock clock=%u",
    "config is_config=%c crc=%u is_shutdown=%c move_count=%hu",
    "stats count=%u sum=%u sumsq=%u",
    "shutdown clock=%u static_string_id=%hu",
    "is_shutdown static_string_id=%hu",
    "pong data=%*s",
    "debug_result val=%u",
    # trsync_state — sent by firmware on every report tick and on
    # trigger. klippy's MCU_trsync._handle_trsync_state is the consumer.
    "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
]

BEACON_RESPONSES = [
    "beacon_data bytes=%*s",
    "beacon_status clock=%u sample=%i frequency=%u temp=%hi",
    "beacon_contact triggered=%c clock=%u sample=%i frequency=%u",
    "beacon_nvm_data bytes=%*s offset=%hu",
    "beacon_contact_state triggered=%c detect_clock=%u",
]


# ---------------------------------------------------------------------------
# Constants reported via msgparser.get_constants() and friends.
# ---------------------------------------------------------------------------

# Beacon uses a 20 MHz tick rate; the real firmware reports CLOCK_FREQ=20000000.
CLOCK_FREQ = 20_000_000

CONFIG = {
    "MCU": "beacon",
    "CLOCK_FREQ": CLOCK_FREQ,
    "STATS_SUMSQ_BASE": 1,
    "ADC_MAX": 4095,
    "BEACON_ADC_SMOOTH_COUNT": 8,
    "BEACON_HAS_ACCEL": 0,
}


def build_identify_dict() -> dict:
    """Return the un-compressed identify dictionary as a Python dict.

    Klippy's MessageParser.process_identify zlib-decompresses the bytes
    we ship via identify_response, then json.loads them. The structure
    here mirrors what the firmware build pipeline produces.

    Klippy's _init_messages also re-runs lookup_params with these
    enumerations, so any pin-typed argument needs the ``pin`` enum to
    resolve. Beacon firmware has no pin-typed arguments, but we expose
    a single trivial ``pin`` range to keep the schema valid for the
    common klippy code path.
    """
    # Reserve msgid 0 / 1 for the default messages baked into msgproto.
    # Build the commands / responses dictionaries with stable ascending ids.
    next_id = 2
    commands: dict = {}
    responses: dict = {}

    for fmt in CORE_COMMANDS + BEACON_COMMANDS:
        commands[fmt] = next_id
        next_id += 1
    for fmt in CORE_RESPONSES + BEACON_RESPONSES:
        # identify_response is already id 0 in msgproto.DefaultMessages, but
        # listing it here too is harmless — _init_messages just rewrites
        # the by-id table with our id, which we keep at 0.
        if fmt.startswith("identify_response"):
            responses[fmt] = 0
            continue
        responses[fmt] = next_id
        next_id += 1

    return {
        "app": "BeaconStub",
        "version": "v0.0.0-sim",
        "build_versions": "sim",
        "license": "GPL-3.0-or-later",
        "enumerations": {
            "pin": {"gpio0": [0, 32]},
            "static_string_id": {"shutdown": 0},
        },
        "commands": commands,
        "responses": responses,
        "output": {},
        "config": CONFIG,
    }


def build_identify_blob() -> bytes:
    """Return the wire-format identify blob (zlib-compressed JSON)."""
    raw = json.dumps(build_identify_dict()).encode("utf-8")
    return zlib.compress(raw)


# Cached at import time so the stub can serve identify chunks without
# rebuilding the dict on every fixture instantiation.
IDENTIFY_BLOB = build_identify_blob()
