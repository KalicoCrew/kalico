#!/usr/bin/env bash
# Test driver for filter.sh. Iterates fixtures, runs filter.sh under
# KLOG_LOCAL_OVERRIDE_PATH, diffs stdout against tests/expected/<name>.txt OR
# checks exit code + stderr substring against tests/expected/<name>.exit_code /
# .stderr_grep when those exist instead.

set -u
SKILL_DIR="$(cd "$(dirname "$0")/.." && pwd)"
FIXTURES_DIR="$SKILL_DIR/tests/fixtures"
EXPECTED_DIR="$SKILL_DIR/tests/expected"
FILTER="$SKILL_DIR/filter.sh"

pass=0
fail=0
failed_names=()

for fixture in "$FIXTURES_DIR"/*.log; do
  name="$(basename "$fixture" .log)"
  exit_code_file="$EXPECTED_DIR/$name.exit_code"
  stderr_grep_file="$EXPECTED_DIR/$name.stderr_grep"
  expected_txt="$EXPECTED_DIR/$name.txt"

  args_file="$EXPECTED_DIR/$name.args"
  if [[ -f "$args_file" ]]; then
    # shellcheck disable=SC2046
    actual_stdout="$(KLOG_LOCAL_OVERRIDE_PATH="$fixture" bash "$FILTER" $(cat "$args_file") 2>/tmp/run_sh_stderr.$$)"
  else
    actual_stdout="$(KLOG_LOCAL_OVERRIDE_PATH="$fixture" bash "$FILTER" 2>/tmp/run_sh_stderr.$$)"
  fi
  actual_exit=$?
  actual_stderr="$(cat /tmp/run_sh_stderr.$$)"
  rm -f /tmp/run_sh_stderr.$$

  ok=1

  if [[ -f "$exit_code_file" ]]; then
    expected_exit="$(cat "$exit_code_file")"
    if [[ "$actual_exit" != "$expected_exit" ]]; then
      echo "FAIL $name: exit code expected=$expected_exit actual=$actual_exit"
      ok=0
    fi
  fi

  if [[ -f "$stderr_grep_file" ]]; then
    needle="$(cat "$stderr_grep_file")"
    if ! grep -qF -- "$needle" <<<"$actual_stderr"; then
      echo "FAIL $name: stderr missing substring: $needle"
      echo "  actual stderr: $actual_stderr"
      ok=0
    fi
  fi

  if [[ -f "$expected_txt" ]]; then
    if ! diff -u "$expected_txt" <(echo "$actual_stdout") > /tmp/run_sh_diff.$$ 2>&1; then
      echo "FAIL $name: stdout differs from $expected_txt"
      cat /tmp/run_sh_diff.$$ | head -40
      ok=0
    fi
    rm -f /tmp/run_sh_diff.$$
  fi

  if (( ok )); then
    pass=$((pass + 1))
    echo "PASS $name"
  else
    fail=$((fail + 1))
    failed_names+=("$name")
  fi
done

echo
echo "Results: $pass passed, $fail failed"
if (( fail > 0 )); then
  echo "Failed: ${failed_names[*]}"
  exit 1
fi
