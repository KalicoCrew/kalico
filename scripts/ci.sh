#!/usr/bin/env bash
# Usage:
#   ./scripts/ci.sh                 # run all gates with a summary (local)
#   ./scripts/ci.sh quick           # fast subset: ruff + rust test/clippy/fmt
#   ./scripts/ci.sh install-hooks   # enable the pre-push hook (runs `quick` per push)
#   ./scripts/ci.sh <job>           # run one gate, exit with its status (CI)
#
# Jobs: ruff rust-host rust-build rust-test rust-clippy rust-fmt rust-loom
#       rust-mcu-h7 rust-mcu-f4 rust-mcu-g0 rust-no-stepper cbindgen-drift
#       c-smoke deny miri panic-grep watchdog-canary py docs sim
#
# Prerequisites (one-time, for the full local run):
#   rustup target add thumbv7em-none-eabi
#   rustup component add --toolchain nightly miri
#   cargo install cargo-nextest --locked        # or: curl -LsSf https://get.nexte.st/latest/<os> | tar zxf - -C ~/.cargo/bin
#   cargo install cargo-deny                     # optional
set -uo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUST="$ROOT/rust"
DOCKER_IMAGE="dangerklippers/klipper-build:latest"

# Linux-only: env RUSTFLAGS replaces (does not merge with) the per-target
# rustflags in rust/.cargo/config.toml, so widening this past the host target
# would drop the macOS cdylib `-undefined dynamic_lookup` and the cross-build
# target-cpu/--nmagic flags.
host_cargo() {
    if [ "$(uname -s)" = Linux ] && command -v ld.lld >/dev/null 2>&1; then
        RUSTFLAGS="-Clink-arg=-fuse-ld=lld" cargo "$@"
    else
        cargo "$@"
    fi
}

job_rust_build()  { cd "$RUST" && cargo build --workspace; }

job_rust_test() {
    cd "$RUST"
    host_cargo nextest run --workspace --profile ci
    host_cargo test --workspace --doc
}

job_rust_clippy() { cd "$RUST" && cargo clippy --workspace --all-targets -- -D warnings; }
job_rust_fmt()    { cd "$RUST" && cargo fmt --all -- --check; }

job_rust_host()   { job_rust_test && job_rust_clippy && job_rust_fmt; }

job_rust_loom() {
    cd "$RUST"
    RUSTFLAGS="--cfg loom" cargo test -p runtime --release \
        --test loom_seqlock \
        --test loom_force_idle \
        --test loom
}

KALICO_MCU_ENV=(KALICO_RUNTIME_STORAGE_SIZE=122880 KALICO_RUNTIME_PIECE_RING_SIZE=63488)

job_rust_mcu_h7() {
    cd "$RUST"
    env "${KALICO_MCU_ENV[@]}" \
        cargo build -p kalico-c-api --no-default-features \
        --features mcu-h7,header-nurbs,header-runtime,motion-module-stepper \
        --target thumbv7em-none-eabi
}

job_rust_mcu_f4() {
    cd "$RUST"
    env "${KALICO_MCU_ENV[@]}" \
        cargo build -p kalico-c-api --no-default-features \
        --features mcu-f4,header-nurbs,header-runtime,motion-module-stepper \
        --target thumbv7em-none-eabi
}

job_rust_mcu_g0() {
    cd "$RUST"
    env "${KALICO_MCU_ENV[@]}" \
        cargo build -p kalico-c-api --no-default-features \
        --features mcu-g0,header-nurbs,header-runtime,motion-module-stepper \
        --target thumbv6m-none-eabi
}

job_rust_no_stepper() {
    cd "$RUST"
    cargo build -p runtime --no-default-features --features host
    cargo test -p runtime --no-default-features --features host --no-run
}

job_cbindgen_drift() {
    "$ROOT/tools/regen_headers.sh"
    git -C "$ROOT" diff --exit-code rust/kalico-c-api/include/
}

job_c_smoke() {
    cd "$RUST"
    cargo build -p kalico-c-api --no-default-features \
        --features host,header-nurbs,header-runtime --release
    cargo test -p kalico-c-api --no-default-features \
        --features host,header-nurbs,header-runtime \
        --test c_smoke_build
}

job_deny() {
    if command -v cargo-deny >/dev/null 2>&1; then
        cargo deny --manifest-path "$RUST/Cargo.toml" check
    else
        echo "cargo-deny not installed (cargo install cargo-deny) — CI runs it via cargo-deny-action; skipping locally"
    fi
}

job_miri() {
    cd "$RUST"
    MIRIFLAGS="-Zmiri-ignore-leaks" cargo +nightly miri test -p runtime --features host \
        --test fault_encoding \
        --test monomial_eval \
        --test phase_lut_anchors \
        --test seqlock_unit
}

job_panic_grep() {
    cd "$RUST"
    env "${KALICO_MCU_ENV[@]}" \
        cargo rustc -p kalico-c-api --release \
        --no-default-features \
        --features mcu-h7,header-nurbs,header-runtime,motion-module-stepper \
        --target thumbv7em-none-eabi -- --emit=llvm-ir
    shopt -s nullglob
    local ll_files=(target/thumbv7em-none-eabi/release/deps/*.ll)
    if [ ${#ll_files[@]} -eq 0 ]; then
        echo "No LLVM-IR files emitted; build step likely failed silently"
        return 1
    fi

    local total
    total=$(grep -hc 'panic_bounds_check' "${ll_files[@]}" 2>/dev/null | awk '{s+=$1} END{print s+0}')
    echo "panic_bounds_check total in MCU release build: ${total}"
    echo "  by function:"
    awk '/^define/{fn=$0} /panic_bounds_check/{print fn}' "${ll_files[@]}" \
        | grep -oE 'kalico_[a-z0-9_]+' | sort | uniq -c | sed 's/^/    /' || true

    local nurbs_panics
    nurbs_panics=$(awk '
        /^define/   { infn=1; fn=$0; hp=0 }
        /panic_bounds_check/ { if (infn) hp=1 }
        /^}/        { if (infn && hp && fn ~ /nurbs/) print fn; infn=0 }
    ' "${ll_files[@]}")
    if [ -n "$nurbs_panics" ]; then
        echo "REGRESSION: panic path(s) in NURBS de Boor evaluator:"
        echo "$nurbs_panics" | sed 's/^/    /'
        return 1
    fi
    echo "NURBS de Boor evaluator is panic-free. OK."
}

job_watchdog_canary() {
    grep -qF 'runtime_liveness_ok' "$ROOT/src/stm32/watchdog.c"
}

job_ruff() {
    if command -v uvx >/dev/null 2>&1; then
        uvx ruff check "$ROOT" && uvx ruff format --check "$ROOT"
    elif command -v ruff >/dev/null 2>&1; then
        ruff check "$ROOT" && ruff format --check "$ROOT"
    else
        echo "ruff not installed (pip install ruff / uvx ruff)"
        return 1
    fi
}

job_py() {
    local ver="${1:-3.13}"
    if command -v docker >/dev/null 2>&1; then
        docker run -v "$ROOT:/klipper" "$DOCKER_IMAGE" --python "$ver" py.test -n auto
    else
        echo "docker unavailable — running py.test on the local interpreter only (CI runs 3.9-3.14)"
        cd "$ROOT" && python -m pytest -n auto
    fi
}

job_sim() {
    local sel="sim_unit and not needs_hardware and not needs_renode"
    local paths="tools/sim_klippy \
        tools/test_kalico_host_io_seq_wrap.py \
        tools/test_motion_toolhead_static.py"
    if command -v docker >/dev/null 2>&1; then
        docker run --rm -v "$ROOT:/klipper" -w /klipper --entrypoint bash "$DOCKER_IMAGE" -lc \
            "make -C tools/sim_klippy/preload >/dev/null && uv run py.test -n auto $paths -m '$sel'"
    else
        echo "docker unavailable — running kalico-sim unit tests on the local interpreter"
        make -C "$ROOT/tools/sim_klippy/preload" >/dev/null 2>&1 || true
        cd "$ROOT" && python -m pytest -n auto $paths -m "$sel"
    fi
}

job_docs() { cd "$ROOT/docs/_kalico" && uv run mkdocs build --strict; }

PASS=0; FAIL=0
FAILED_JOBS=()

red()    { printf '\033[1;31m%s\033[0m\n' "$*"; }
green()  { printf '\033[1;32m%s\033[0m\n' "$*"; }

run_check() {
    local name="$1"; shift
    printf '%-20s ' "$name"
    local log rc=0
    log="$(mktemp)"
    ( set -e; "$@" ) >"$log" 2>&1 && rc=0 || rc=$?
    if [ "$rc" -eq 0 ]; then
        green "PASS"; PASS=$((PASS + 1))
    else
        red "FAIL ($rc)"; FAIL=$((FAIL + 1)); FAILED_JOBS+=("$name")
        sed 's/^/    /' "$log" | tail -50
    fi
    rm -f "$log"
}

run_all() {
    local quick="${1:-false}"
    run_check "ruff"            job_ruff
    run_check "rust-test"       job_rust_test
    run_check "rust-clippy"     job_rust_clippy
    run_check "rust-fmt"        job_rust_fmt
    run_check "watchdog-canary" job_watchdog_canary
    if [ "$quick" != "true" ]; then
        run_check "cbindgen-drift"  job_cbindgen_drift
        run_check "c-smoke"         job_c_smoke
        run_check "rust-mcu-h7"     job_rust_mcu_h7
        run_check "rust-mcu-f4"     job_rust_mcu_f4
        run_check "rust-mcu-g0"     job_rust_mcu_g0
        run_check "rust-no-stepper" job_rust_no_stepper
        run_check "rust-loom"       job_rust_loom
        run_check "miri"            job_miri
        run_check "panic-grep"      job_panic_grep
        run_check "deny"            job_deny
        run_check "docs"            job_docs
        run_check "py"              job_py
        run_check "sim"             job_sim
    fi
    echo "────────────────────────────────────────"
    printf '  %s   %s\n' "$(green "$PASS pass")" "$([ "$FAIL" -gt 0 ] && red "$FAIL fail" || echo "0 fail")"
    [ "$FAIL" -eq 0 ] || { printf '  failed: %s\n' "${FAILED_JOBS[*]}"; }
    echo "────────────────────────────────────────"
    return "$FAIL"
}

usage() {
    sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'
}

job_install_hooks() {
    cd "$ROOT"
    [ -x .githooks/pre-push ] || {
        echo "error: .githooks/pre-push missing or not executable" >&2
        return 1
    }
    git config core.hooksPath .githooks
    echo "pre-push hook enabled (core.hooksPath = .githooks)."
    echo "  runs './scripts/ci.sh quick' before every push — incl. direct pushes to sota-motion"
    echo "  bypass once:  git push --no-verify"
    echo "  disable:      git config --unset core.hooksPath"
}

case "${1:-all}" in
    rust-host)        job_rust_host ;;
    rust-build)       job_rust_build ;;
    rust-test)        job_rust_test ;;
    rust-clippy)      job_rust_clippy ;;
    rust-fmt)         job_rust_fmt ;;
    rust-loom)        job_rust_loom ;;
    rust-mcu-h7)      job_rust_mcu_h7 ;;
    rust-mcu-f4)      job_rust_mcu_f4 ;;
    rust-mcu-g0)      job_rust_mcu_g0 ;;
    rust-no-stepper)  job_rust_no_stepper ;;
    cbindgen-drift)   job_cbindgen_drift ;;
    c-smoke)          job_c_smoke ;;
    deny)             job_deny ;;
    miri)             job_miri ;;
    panic-grep)       job_panic_grep ;;
    watchdog-canary)  job_watchdog_canary ;;
    ruff)             job_ruff ;;
    py)               shift; job_py "${1:-3.13}" ;;
    docs)             job_docs ;;
    sim)              job_sim ;;
    all)              run_all false ;;
    quick|--quick)    run_all true ;;
    install-hooks|hooks) job_install_hooks ;;
    -h|--help|help)   usage ;;
    *) echo "unknown job: ${1:-}" >&2; usage >&2; exit 2 ;;
esac
