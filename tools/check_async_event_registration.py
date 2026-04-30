#!/usr/bin/env python3
"""
Build-time invariant: all kalico_* async event format strings must
register via _DECL_OUTPUT exclusively (no sendf-emitted kalico_* events).

Reads out/klipper.dict and asserts no msg name starting with 'kalico_'
appears in the 'responses' category UNLESS suffixed with '_response'
(those ARE responses, not async events).
"""
import json
import sys

ASYNC_EVENT_PREFIX = "kalico_"
RESPONSE_SUFFIX = "_response"

def main(dict_path: str) -> int:
    with open(dict_path) as f:
        d = json.load(f)

    violations = []
    for fmt in d.get("responses", {}):
        name = fmt.split()[0]
        if name.startswith(ASYNC_EVENT_PREFIX) and not name.endswith(RESPONSE_SUFFIX):
            violations.append(name)

    if violations:
        print("ERROR: kalico_* async events found in 'responses' category:", file=sys.stderr)
        for v in sorted(set(violations)):
            print(f"  - {v}", file=sys.stderr)
        print("\nAll kalico_* async events must use output(...) / _DECL_OUTPUT.", file=sys.stderr)
        return 1

    print(f"OK: no kalico_* async events miscategorized in {dict_path}")
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "out/klipper.dict"))
