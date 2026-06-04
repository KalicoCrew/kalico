# Local observability test rig (no printer required)

This rig runs VictoriaLogs (and, optionally, Vector) on the dev host so the
whole logging pipeline — schema, ingest, LogsQL queries, the `query-logs`
skill, sanitization, numeric range filters, durability — can be built and
verified **without Trident**. Binaries + scratch data live under the
gitignored `.tools/observability/`.

## VictoriaLogs (validated: v1.50.0, darwin-arm64)

```bash
mkdir -p .tools/observability/bin .tools/observability/vl-data
curl -sL -o /tmp/vl.tar.gz \
  https://github.com/VictoriaMetrics/VictoriaLogs/releases/download/v1.50.0/victoria-logs-darwin-arm64-v1.50.0.tar.gz
tar xzf /tmp/vl.tar.gz -C .tools/observability/bin/   # -> victoria-logs-prod

# run (localhost, 30d retention):
.tools/observability/bin/victoria-logs-prod \
  -storageDataPath=.tools/observability/vl-data \
  -httpListenAddr=127.0.0.1:9428 \
  -retentionPeriod=30d &

curl -s http://127.0.0.1:9428/health     # -> OK
```

### Direct ingest + query (no Vector needed)

The VL ingest URL params below are the ones the deployed `vector.toml` uses;
they were validated end-to-end against v1.50.0.

```bash
# ingest one schema-shaped NDJSON line:
printf '%s\n' '{"_time":"...Z","_msg":"m","level":"info","source":"host-py","subsystem":"motion","session_id":"k-1","print_id":"","event":"x","target":"t"}' \
| curl -s -X POST -H 'Content-Type: application/stream+json' --data-binary @- \
  'http://127.0.0.1:9428/insert/jsonline?_stream_fields=source,subsystem&_time_field=_time&_msg_field=_msg'

# query (NDJSON out, one object per line). NOTE: relative _time:Nh is anchored
# to NOW, so test data must be timestamped near the current time; otherwise use
# an absolute range _time:[2026-06-01T00:00:00Z, 2026-06-02T00:00:00Z].
curl -s 'http://127.0.0.1:9428/select/logsql/query' \
  --data-urlencode 'query=subsystem:=motion _time:1h' --data-urlencode 'limit=10'
```

Validated against v1.50.0: `_stream={source,subsystem}` mapping, field indexing,
numeric range filters (`trigger_mm:>10`), `| stats by (f) count() as n | sort by
(n) desc` (aggregates must be aliased — `sort by (count(*))` does not parse),
and `_msg` sanitization (an embedded newline/quote stays exactly one record).

## Vector (the shipper) — validated: v0.55.0, arm64-apple-darwin

The darwin asset is a direct GitHub release tarball — note Vector names it
`arm64` (not `aarch64`):

```bash
curl -sL -o /tmp/vector.tar.gz \
  https://github.com/vectordotdev/vector/releases/download/v0.55.0/vector-0.55.0-arm64-apple-darwin.tar.gz
tar xzf /tmp/vector.tar.gz -C .tools/observability/   # -> vector-arm64-apple-darwin/bin/vector
.tools/observability/vector-arm64-apple-darwin/bin/vector --version
```

Run it against the scratch events dir using the dev-override config (same
structure as `config/observability/vector.toml`, with dev paths and a writable
`data_dir`). The committed config uses a `remap` transform (`. =
parse_json!(.message)`) because Vector's `file` source emits the raw line in
`message` — it has no per-line JSON decoder. Validated:

```bash
.tools/observability/vector-arm64-apple-darwin/bin/vector validate \
  .tools/observability/vector.dev.toml          # -> Validated, health check OK
```

The full round-trip (emit → JSONL → Vector → VL → query) plus the VL-down /
backfill durability check are scripted in `test/observability/e2e_local.sh`.

## Teardown

```bash
pkill -f victoria-logs-prod
rm -rf .tools/   # all rig state is gitignored and disposable
```
