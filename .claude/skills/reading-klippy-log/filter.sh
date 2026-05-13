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

# Stage 2: Index boot banners.
# Each banner line: "Start printer at <date> (<host_epoch> <internal_clock>)"
mapfile -t BANNER_LINES < <(grep -n "^Start printer at " "$TMPLOG" | cut -d: -f1)

emit_full_file_warning() {
  total_lines=$(awk 'END{print NR}' "$TMPLOG")
  non_status=$(grep -cv "kalico_status_v6" "$TMPLOG")
  echo "SESSION: WARNING no boot banner found, using entire file (slice=L1-L${total_lines} non_status_lines=${non_status})"
  awk '{ printf("L%d\t%s\n", NR, $0) }' "$TMPLOG"
}

if (( ${#BANNER_LINES[@]} == 0 )); then
  emit_full_file_warning
  exit 0
fi

# TODO Task 4+: session pick, slice, collapse.
# For now emit a placeholder showing banner-index worked.
echo "BANNERS_FOUND: ${#BANNER_LINES[@]} at lines: ${BANNER_LINES[*]}"
