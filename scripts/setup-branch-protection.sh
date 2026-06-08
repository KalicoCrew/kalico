#!/usr/bin/env bash
set -euo pipefail

REPO="${1:-$(gh repo view --json nameWithOwner -q .nameWithOwner)}"
BRANCH="${2:-sota-motion}"

REQUIRED_CHECKS=(
  # ci-rust-runtime.yaml
  changes
  rust-host
  rust-loom
  rust-mcu-h7
  rust-mcu-f4
  rust-cbindgen-drift
  rust-c-smoke
  rust-deny
  rust-miri
  rust-panic-symbol-grep
  watchdog-canary
  # ci-build_test.yaml
  build
  sim
  # ci-lintformat.yaml
  ruff
)

contexts_json="$(printf '%s\n' "${REQUIRED_CHECKS[@]}" | jq -R . | jq -s .)"

echo "Applying branch protection to ${REPO}@${BRANCH} with required checks:"
printf '  - %s\n' "${REQUIRED_CHECKS[@]}"

gh api -X PUT "repos/${REPO}/branches/${BRANCH}/protection" \
  --input - <<JSON
{
  "required_status_checks": {
    "strict": true,
    "contexts": ${contexts_json}
  },
  "enforce_admins": false,
  "required_pull_request_reviews": null,
  "restrictions": null,
  "allow_force_pushes": false,
  "allow_deletions": false
}
JSON

echo "Done. Required checks must pass (or be skipped) before merge into ${BRANCH}."
