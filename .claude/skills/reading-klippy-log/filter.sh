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
  local total_lines non_status
  total_lines=$(awk 'END{print NR}' "$TMPLOG")
  non_status=$(grep -cv "kalico_status_v6" "$TMPLOG")
  echo "SESSION: WARNING no boot banner found, using entire file (slice=L1-L${total_lines} non_status_lines=${non_status})"
  awk '{ printf("L%d\t%s\n", NR, $0) }' "$TMPLOG"
}

if (( ${#BANNER_LINES[@]} == 0 )); then
  emit_full_file_warning
  exit 0
fi

# Stage 3: Pick a session.
# session_arg is one of: latest (default), previous, N (integer 1-based from start).
SESSION_INDEX=""  # 1-based index into BANNER_LINES
FALLBACK_REASON="none"

case "$session_arg" in
  latest)
    SESSION_INDEX=${#BANNER_LINES[@]}
    ;;
  previous)
    if (( ${#BANNER_LINES[@]} >= 2 )); then
      SESSION_INDEX=$(( ${#BANNER_LINES[@]} - 1 ))
    else
      echo "requested session 'previous' but only ${#BANNER_LINES[@]} session(s) found" >&2
      exit 1
    fi
    ;;
  *)
    # Numeric? 1-based.
    if [[ "$session_arg" =~ ^[0-9]+$ ]]; then
      if (( session_arg >= 1 && session_arg <= ${#BANNER_LINES[@]} )); then
        SESSION_INDEX=$session_arg
      else
        echo "requested session $session_arg not found (found ${#BANNER_LINES[@]} sessions)" >&2
        exit 1
      fi
    else
      echo "invalid --session value: $session_arg (expected latest, previous, or integer)" >&2
      exit 2
    fi
    ;;
esac

# Resolve to actual banner line number (1-based into TMPLOG).
BANNER_LINE=${BANNER_LINES[$((SESSION_INDEX - 1))]}

# TODO Task 5+: fresh-restart fallback heuristic refines BANNER_LINE.
# TODO Task 6+: slice + collapse from BANNER_LINE to EOF.
echo "PICKED_SESSION: index=$SESSION_INDEX banner_line=$BANNER_LINE"
