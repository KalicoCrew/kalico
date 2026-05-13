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

# Compute slice metadata.
TOTAL_LINES=$(awk 'END{print NR}' "$TMPLOG")
SLICE_END=$TOTAL_LINES
NON_STATUS_LINES=$(awk -v start="$BANNER_LINE" 'NR >= start && $0 !~ /kalico_status_v6/' "$TMPLOG" | wc -l | tr -d ' ')
BANNER_TIME=$(awk -v ln="$BANNER_LINE" 'NR == ln { sub(/^Start printer at /, ""); sub(/ \(.*$/, ""); print; exit }' "$TMPLOG")

# Stage 4: emit SESSION header.
echo "SESSION: banner_line=$BANNER_LINE banner_time='$BANNER_TIME' slice=L${BANNER_LINE}-L${SLICE_END} non_status_lines=$NON_STATUS_LINES fallback=$FALLBACK_REASON"

# Stage 5: collapse-aware slice — per-MCU run tracking for kalico_status_v6 frames.
awk -v start="$BANNER_LINE" '
function extract_field(line, field,   pat, val) {
  pat = "'\''" field "'\'': [0-9]+"
  if (match(line, pat)) {
    val = substr(line, RSTART + length(field) + 4, RLENGTH - length(field) - 4)
    return val + 0
  }
  return -1
}
function extract_mcu(line,   s) {
  if (match(line, /^mcu '\''[^'\'']+'\''/)) {
    s = substr(line, RSTART + 5, RLENGTH - 6)
    return s
  }
  return "?"
}
function extract_recv_time(line,   val) {
  if (match(line, /'\''#receive_time'\'': [0-9.]+/)) {
    # prefix "'#receive_time': " is 17 chars
    val = substr(line, RSTART + 17, RLENGTH - 17)
    return val + 0
  }
  return 0
}
function close_run(mcu,   dur) {
  if (active[mcu]) {
    dur = run_end_time[mcu] - run_start_time[mcu]
    printf("[L%d-L%d] mcu='\''%s'\'' status unchanged: engine_status=%d segment_id=%d last_fault=%d fault_detail=%d (%d frames, %.1fs)\n",
      run_start_line[mcu], run_end_line[mcu], mcu,
      run_es[mcu], run_seg[mcu], run_fault[mcu], run_detail[mcu],
      run_count[mcu], dur)
    active[mcu] = 0
  }
}
function close_all_sorted(   names, n, i, j, tmp) {
  n = 0
  for (m in active) if (active[m]) { names[++n] = m }
  for (i = 1; i < n; i++) for (j = i+1; j <= n; j++) if (names[i] > names[j]) { tmp = names[i]; names[i] = names[j]; names[j] = tmp }
  for (i = 1; i <= n; i++) close_run(names[i])
}
NR < start { next }
{
  if ($0 ~ /kalico_status_v6/) {
    mcu = extract_mcu($0)
    es = extract_field($0, "engine_status")
    seg = extract_field($0, "current_segment_id")
    fault = extract_field($0, "last_fault")
    detail = extract_field($0, "fault_detail")
    t = extract_recv_time($0)
    if (active[mcu] && es == run_es[mcu] && seg == run_seg[mcu] && fault == run_fault[mcu] && detail == run_detail[mcu]) {
      run_end_line[mcu] = NR
      run_end_time[mcu] = t
      run_count[mcu]++
    } else {
      close_run(mcu)
      printf("L%d\t%s\n", NR, $0)
      run_es[mcu] = es; run_seg[mcu] = seg; run_fault[mcu] = fault; run_detail[mcu] = detail
      run_start_line[mcu] = NR; run_end_line[mcu] = NR
      run_start_time[mcu] = t; run_end_time[mcu] = t
      run_count[mcu] = 1
      active[mcu] = 1
    }
  } else {
    close_all_sorted()
    printf("L%d\t%s\n", NR, $0)
  }
}
END {
  close_all_sorted()
}
' "$TMPLOG"
