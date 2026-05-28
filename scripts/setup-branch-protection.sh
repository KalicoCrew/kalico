#!/usr/bin/env bash
# scripts/setup-branch-protection.sh — OPTIONAL: require these checks on PRs.
#
# Not needed for a solo direct-commit workflow (sota-motion is committed to
# directly; CI-on-push + the pre-push hook keep it honest — see
# docs/kalico-rewrite/ci.md). This only adds value once OTHER contributors open
# PRs: it gates their merges on the checks. It sets `enforce_admins: false`, so
# it never blocks direct maintainer pushes to sota-motion.
#
# Requires admin on the repo and `gh auth login`. Run once (re-running is safe;
# it overwrites the protection config). Review the REQUIRED_CHECKS list below
# before running.
#
#   ./scripts/setup-branch-protection.sh [owner/repo] [branch]
#
# Why these checks: only jobs that are ALWAYS created on a PR are listed. The
# rust-* jobs are gated by a paths-filter `if:` and report "skipped" on
# non-rust PRs — GitHub treats a skipped required check as passing, so they do
# not block docs-only PRs. The docs ("test") job uses a top-level paths filter
# and is therefore NOT always created, so it is intentionally NOT required.
set -euo pipefail

REPO="${1:-$(gh repo view --json nameWithOwner -q .nameWithOwner)}"
BRANCH="${2:-sota-motion}"

# Status check contexts = workflow job ids.
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
