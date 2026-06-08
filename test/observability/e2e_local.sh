#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
RIG="$ROOT/.tools/observability"
EVENTS="$RIG/events"
VL_BIN="$RIG/bin/victoria-logs-prod"
VECTOR_BIN="$RIG/vector-arm64-apple-darwin/bin/vector"
VL="http://127.0.0.1:9428"

git -C "$ROOT" check-ignore "$RIG" >/dev/null || { echo "ABORT: $RIG not gitignored"; exit 1; }
mkdir -p "$EVENTS"
rm -f "$EVENTS/host-py.jsonl" "$EVENTS/host-rust.jsonl"

q(){ curl -s "$VL/select/logsql/query" --data-urlencode "query=$1" --data-urlencode "limit=${2:-50}"; }

echo "== 1. start VictoriaLogs =="
"$VL_BIN" -storageDataPath="$RIG/vl-data" -httpListenAddr=127.0.0.1:9428 -retentionPeriod=30d >"$RIG/vl.log" 2>&1 &
VL_PID=$!; sleep 3
curl -s -m3 "$VL/health" >/dev/null && echo "  VL up (pid $VL_PID)"

echo "== 2. emit host-py.jsonl via the real Stage 1 JsonlSink =="
PYTHONPATH="$ROOT:$ROOT/klippy" python3 - "$EVENTS" <<'PY'
import logging, sys
from klippy import structured_log, log_sinks
rig = sys.argv[1]
structured_log.bind_session("k-rig-now"); structured_log.bind_print("print-rig")
sink = log_sinks.JsonlSink(rig + "/host-py.jsonl"); cf = structured_log.ContextFilter()
def emit(name, lvl, msg, **ex):
    r = logging.LogRecord(name, lvl, __file__, 1, msg, (), None)
    for k, v in ex.items(): setattr(r, k, v)
    r.message = r.getMessage(); cf.filter(r); sink.emit_record(r)
emit("gcode.GCodeDispatch", logging.INFO, "G1 move queued", subsystem="motion", event="motion.move_queued", gcode_line=1843)
emit("homing.Probe", logging.WARNING, 'needs rehome\nwith a "quoted" tail', subsystem="homing", event="homing.retry")
emit("log_observability", logging.INFO, "pipeline heartbeat", subsystem="observability", event="heartbeat")
sink.close()
PY
echo "  host-py.jsonl: $(wc -l < "$EVENTS/host-py.jsonl") lines"

echo "== 3. emit host-rust.jsonl via the real Rust serializer (integration test) =="
( cd "$ROOT/rust" && cargo test -p motion-bridge --test logging_integration >/dev/null 2>&1 )
RUST_OUT="$(find "${TMPDIR:-/tmp}" -name host-rust.jsonl -path '*kalico-log-it*' 2>/dev/null | xargs ls -t 2>/dev/null | head -1)"
cp "$RUST_OUT" "$EVENTS/host-rust.jsonl"
echo "  host-rust.jsonl: $(wc -l < "$EVENTS/host-rust.jsonl") lines"

echo "== 4. start Vector (tail -> parse_json -> ship) =="
"$VECTOR_BIN" --config "$RIG/vector.dev.toml" >"$RIG/vector.log" 2>&1 &
VECTOR_PID=$!; sleep 8
echo "  vector up (pid $VECTOR_PID)"

echo "== 5. query via query-logs recipes =="
echo "-- both sources present:"; q 'session_id:in(k-rig-now,k-1748700131-4412) _time:1h | stats by (source) count() as n | sort by (n) desc'
echo "-- sanitization (embedded newline+quote = ONE record):"; q 'source:=host-py subsystem:=homing _time:1h'
echo "-- numeric range:"; q 'source:=host-rust trigger_mm:>10 _time:1h'
echo "-- heartbeat self-check:"; q 'subsystem:=observability event:=heartbeat _time:1h' 1

echo "== 6. durability: VL down -> append -> restart -> backfill =="
kill "$VL_PID" 2>/dev/null || true; sleep 1
PYTHONPATH="$ROOT:$ROOT/klippy" python3 - "$EVENTS" <<'PY'
import logging, sys
from klippy import structured_log, log_sinks
rig = sys.argv[1]
structured_log.bind_session("k-rig-now"); structured_log.bind_print("print-rig")
sink = log_sinks.JsonlSink(rig + "/host-py.jsonl"); cf = structured_log.ContextFilter()
r = logging.LogRecord("durability.test", logging.ERROR, __file__, 1, "written while VL was DOWN", (), None)
r.subsystem="durability"; r.event="vl_down_marker"; r.message=r.getMessage(); cf.filter(r)
sink.emit_record(r); sink.close()
PY
"$VL_BIN" -storageDataPath="$RIG/vl-data" -httpListenAddr=127.0.0.1:9428 -retentionPeriod=30d >>"$RIG/vl.log" 2>&1 &
VL_PID=$!; sleep 3; curl -s -m3 "$VL/health" >/dev/null && echo "  VL back up"
sleep 8
echo "-- backfilled down-marker (must be present = no loss):"; q 'event:=vl_down_marker _time:1h'

echo "== teardown =="
kill "$VECTOR_PID" "$VL_PID" 2>/dev/null || true
echo "done."
