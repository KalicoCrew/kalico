#!/usr/bin/env bash
# klippy-log reader preprocessor — stub. Tasks 2+ flesh this out.
set -u

SRC="${KLOG_LOCAL_OVERRIDE_PATH:-}"
if [[ -z "$SRC" ]]; then
  SRC="<ssh:dderg@trident.local:~/printer_data/logs/klippy.log>"
  echo "klippy.log empty or unreadable at $SRC (stub; ssh path not yet implemented)" >&2
  exit 1
fi

if [[ ! -s "$SRC" ]]; then
  echo "klippy.log empty or unreadable at $SRC" >&2
  exit 1
fi

echo "STUB: would process $SRC"
