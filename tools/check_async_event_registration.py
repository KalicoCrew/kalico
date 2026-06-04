#!/usr/bin/env python3
"""
Build-time invariant: kalico_* async event format strings must register
via _DECL_OUTPUT exclusively.

Reads out/klipper.dict and:
  1. Errors (exit 1) if any KNOWN_ASYNC_EVENTS appear in 'responses'.
  2. Warns (exit 0) if any other kalico_* names (non-_response) appear
     in 'responses' — unexpected but not build-breaking.
"""

import json
import sys

# These four are the only kalico_* async events (spec §1.1 item 11).
# They MUST appear in the 'output' category, never 'responses'.
KNOWN_ASYNC_EVENTS = {
    "kalico_credit_freed",
    "kalico_fault",
    "kalico_status_v6",
    "kalico_trace",
}

ASYNC_EVENT_PREFIX = "kalico_"
RESPONSE_SUFFIX = "_response"


def main(dict_path: str) -> int:
    with open(dict_path) as f:
        d = json.load(f)

    responses_names = {fmt.split()[0] for fmt in d.get("responses", {})}

    violations = KNOWN_ASYNC_EVENTS & responses_names
    if violations:
        print(
            "ERROR: known kalico_* async events found in 'responses' category:",
            file=sys.stderr,
        )
        for v in sorted(violations):
            print(f"  - {v}", file=sys.stderr)
        print(
            "\nThese must use output(...) / _DECL_OUTPUT in firmware source.",
            file=sys.stderr,
        )
        return 1

    # Warn about unexpected kalico_* names in responses (not in known-async list,
    # not _response-suffixed) — may indicate a new async event was added without
    # updating KNOWN_ASYNC_EVENTS.
    unexpected = {
        n
        for n in responses_names
        if n.startswith(ASYNC_EVENT_PREFIX)
        and not n.endswith(RESPONSE_SUFFIX)
        and n not in KNOWN_ASYNC_EVENTS
    }
    if unexpected:
        print(
            "WARNING: unexpected kalico_* names in 'responses' (not in KNOWN_ASYNC_EVENTS):",
            file=sys.stderr,
        )
        for n in sorted(unexpected):
            print(
                f"  - {n} (add to KNOWN_ASYNC_EVENTS if async, or verify it's a real response)",
                file=sys.stderr,
            )

    print(f"OK: no known async events miscategorized in {dict_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "out/klipper.dict"))
