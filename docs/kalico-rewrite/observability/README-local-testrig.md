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

## Vector (the shipper) — fetch for the full e2e

Vector v0.55.0 no longer publishes a plain static tarball on GitHub releases /
packages.timber.io. Two clean options to complete the shipper round-trip:

```bash
# Option A — official installer into a contained prefix (no system change):
curl --proto '=https' --tlsv1.2 -sSfL https://sh.vector.dev \
  | bash -s -- -y --prefix .tools/observability/vector-home

# Option B — Homebrew (adds a third-party tap to the system):
brew tap vectordotdev/brew && brew install vector
```

Then run it against the scratch events dir:

```bash
# dev override of config/observability/vector.toml:
#   include  -> .tools/observability/events/*.jsonl
#   data_dir -> .tools/observability/vector-data
<vector-bin> --config .tools/observability/vector.dev.toml
```

Until Vector is fetched, the pipeline is verified up to the VL boundary via the
direct-ingest path above (which uses the identical ingest params), plus the
host-side emit/JSONL/schema/sanitization is covered by the Python + Rust unit
and integration suites. The remaining Vector-in-the-loop checks (config
`validate`, checkpoint resume on restart) run with Option A/B or on the printer.

## Teardown

```bash
pkill -f victoria-logs-prod
rm -rf .tools/   # all rig state is gitignored and disposable
```
