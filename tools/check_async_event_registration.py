#!/usr/bin/env python3
import json
import sys

# spec §1.1 item 11
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
