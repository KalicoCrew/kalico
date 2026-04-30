#!/usr/bin/env bash
# scripts/ci-local.sh — Run CI checks locally before pushing.
#
# Usage:
#   ./scripts/ci-local.sh          # run all checks
#   ./scripts/ci-local.sh --quick  # skip slow jobs (loom, miri, mcu builds)
#
# Prerequisites (one-time):
#   rustup target add thumbv7em-none-eabi
#   rustup component add --toolchain nightly miri
#   cargo install cargo-deny        # optional, skipped if missing
set -euo pipefail

QUICK=false
if [[ "${1:-}" == "--quick" ]]; then
    QUICK=true
fi

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUST="$ROOT/rust"
PASS=0
FAIL=0
SKIP=0

red()    { printf '\033[1;31m%s\033[0m\n' "$*"; }
green()  { printf '\033[1;32m%s\033[0m\n' "$*"; }
yellow() { printf '\033[1;33m%s\033[0m\n' "$*"; }

run_check() {
    local name="$1"
    shift
    printf '%-40s ' "$name"
    if "$@" > /tmp/ci-local-$$.log 2>&1; then
        green "PASS"
        PASS=$((PASS + 1))
    else
        red "FAIL"
        cat /tmp/ci-local-$$.log
        FAIL=$((FAIL + 1))
    fi
    rm -f /tmp/ci-local-$$.log
}

skip_check() {
    printf '%-40s ' "$1"
    yellow "SKIP"
    SKIP=$((SKIP + 1))
}

# ── Python / Ruff ──────────────────────────────────────────────────
if command -v uvx &>/dev/null; then
    run_check "ruff check" uvx ruff check "$ROOT"
    run_check "ruff format" uvx ruff format --check "$ROOT"
elif command -v ruff &>/dev/null; then
    run_check "ruff check" ruff check "$ROOT"
    run_check "ruff format" ruff format --check "$ROOT"
else
    skip_check "ruff (not installed)"
fi

# ── Rust: build + test + clippy + fmt ──────────────────────────────
cd "$RUST"

run_check "cargo build --workspace" \
    cargo build --workspace

run_check "cargo test --workspace" \
    cargo test --workspace --features runtime/test-injection

run_check "cargo clippy" \
    cargo clippy --workspace --all-targets -- -D warnings

run_check "cargo fmt" \
    cargo fmt --all -- --check

# ── Rust: MCU builds ──────────────────────────────────────────────
if $QUICK; then
    skip_check "mcu-h7 (--quick)"
    skip_check "mcu-f4 (--quick)"
else
    if rustup target list --installed 2>/dev/null | grep -q '^thumbv7em-none-eabi$'; then
        run_check "cargo build mcu-h7" \
            cargo build -p kalico-c-api --no-default-features \
            --features mcu-h7,header-nurbs,header-runtime \
            --target thumbv7em-none-eabi

        run_check "cargo build mcu-f4" \
            cargo build -p kalico-c-api --no-default-features \
            --features mcu-f4,header-nurbs,header-runtime \
            --target thumbv7em-none-eabi
    else
        skip_check "mcu-h7 (target thumbv7em-none-eabi not installed)"
        skip_check "mcu-f4 (target thumbv7em-none-eabi not installed)"
    fi
fi

# ── Rust: cbindgen drift ──────────────────────────────────────────
if [[ -x "$ROOT/tools/regen_headers.sh" ]]; then
    run_check "cbindgen drift" bash -c \
        "$ROOT/tools/regen_headers.sh && git diff --exit-code $RUST/kalico-c-api/include/"
else
    skip_check "cbindgen drift (regen_headers.sh not found)"
fi

# ── Rust: c-smoke ─────────────────────────────────────────────────
run_check "c-smoke staticlib" \
    cargo build -p kalico-c-api --no-default-features \
    --features host,header-nurbs,header-runtime --release

run_check "c-smoke test" \
    cargo test -p kalico-c-api --no-default-features \
    --features host,header-nurbs,header-runtime \
    --test c_smoke_build

# ── Rust: cargo-deny ──────────────────────────────────────────────
if command -v cargo-deny &>/dev/null; then
    run_check "cargo deny" cargo deny check
else
    skip_check "cargo deny (not installed; cargo install cargo-deny)"
fi

# ── Rust: miri ────────────────────────────────────────────────────
if $QUICK; then
    skip_check "miri (--quick)"
else
    if rustup component list --toolchain nightly 2>/dev/null | grep -q 'miri.*installed'; then
        run_check "miri (runtime)" \
            env MIRIFLAGS="-Zmiri-ignore-leaks" \
            cargo +nightly miri test -p runtime --features host
    else
        skip_check "miri (nightly miri component not installed)"
    fi
fi

# ── Rust: loom ────────────────────────────────────────────────────
if $QUICK; then
    skip_check "loom (--quick)"
else
    run_check "loom (runtime)" \
        env RUSTFLAGS="--cfg loom" \
        cargo test -p runtime --release \
        --test loom_seqlock \
        --test loom_spsc_split \
        --test loom_force_idle \
        --test loom_curve_pool_alloc
fi

# ── Klipper C: watchdog canary ────────────────────────────────────
if [[ -f "$ROOT/src/stm32/watchdog.c" ]]; then
    run_check "watchdog canary" bash -c \
        "grep -qF 'kalico_liveness_ok' '$ROOT/src/stm32/watchdog.c' && grep -qF 'CONFIG_KALICO_RUNTIME' '$ROOT/src/stm32/watchdog.c'"
else
    skip_check "watchdog canary (watchdog.c not found)"
fi

# ── Summary ───────────────────────────────────────────────────────
echo ""
echo "────────────────────────────────────────"
printf "  %s  %s  %s\n" \
    "$(green "$PASS pass")" \
    "$(red "$FAIL fail")" \
    "$(yellow "$SKIP skip")"
echo "────────────────────────────────────────"

exit $FAIL
