#!/usr/bin/env bash
# klippy-log reader preprocessor.
# Stages: 1 fetch, 2 banner index, 3 session pick, 4 slice, 5 collapse, 6 pass-through.
# Honors KLOG_LOCAL_OVERRIDE_PATH for tests/fixtures.
set -u

usage() {
  echo "usage: filter.sh [--session=latest|previous|N]" >&2
}

session_arg="latest"
for arg in "$@"; do
  case "$arg" in
    --session=*) session_arg="${arg#--session=}" ;;
    -h|--help) usage; exit 0 ;;
    *) usage; exit 2 ;;
  esac
done

# Stage 1: Fetch source into a temp file.
TMPLOG="$(mktemp -t klog.XXXXXX)"
trap 'rm -f "$TMPLOG"' EXIT

SRC_LABEL=""
if [[ -n "${KLOG_LOCAL_OVERRIDE_PATH:-}" ]]; then
  SRC_LABEL="$KLOG_LOCAL_OVERRIDE_PATH"
  if [[ ! -e "$KLOG_LOCAL_OVERRIDE_PATH" ]]; then
    echo "klippy.log empty or unreadable at $SRC_LABEL" >&2
    exit 1
  fi
  cat "$KLOG_LOCAL_OVERRIDE_PATH" > "$TMPLOG"
else
  SRC_LABEL="dderg@trident.local:~/printer_data/logs/klippy.log"
  if ! ssh dderg@trident.local 'cat ~/printer_data/logs/klippy.log' > "$TMPLOG" 2>/tmp/klog_ssh_err.$$; then
    err="$(cat /tmp/klog_ssh_err.$$)"; rm -f /tmp/klog_ssh_err.$$
    echo "ssh dderg@trident.local failed: $err" >&2
    exit 1
  fi
  rm -f /tmp/klog_ssh_err.$$
fi

if [[ ! -s "$TMPLOG" ]]; then
  echo "klippy.log empty or unreadable at $SRC_LABEL" >&2
  exit 1
fi

# TODO Task 3+: banner indexing, session pick, slice, collapse.
# For now emit a placeholder so the test driver detects this stage works.
echo "FETCHED: $(wc -l < "$TMPLOG") lines from $SRC_LABEL"
