# syntax=docker/dockerfile:1
#
# Multi-stage build — ordered from least- to most-frequently-changed.
# Each stage's expensive work is cached independently; only the changed
# stage and everything after it re-executes on a rebuild.
#
#   toolchain   apt packages + Rust toolchain  (changes: never)
#   rust-deps   cargo dep graph compiled       (changes: Cargo.lock)
#   rust-build  workspace crates compiled      (changes: rust/ source)
#   firmware    C firmware ELFs + chelper      (changes: src/, klippy/)
#   runtime     final image — no build tools   (changes: klippy/, tools/)

# --------------------------------------------------------------------
# Stage 1 — toolchain
# Installs OS packages and Rust. Changes almost never.
# --------------------------------------------------------------------
FROM ubuntu:24.04 AS toolchain

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    gcc \
    g++ \
    python3 \
    python3-dev \
    python3-pip \
    python3-venv \
    python3-cffi \
    python3-greenlet \
    python3-serial \
    python3-jinja2 \
    python3-markupsafe \
    libffi-dev \
    libudev-dev \
    pkg-config \
    git \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages \
    greenlet cffi pyserial jinja2 markupsafe numpy 2>/dev/null || true

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
    sh -s -- -y --default-toolchain stable --profile minimal
ENV PATH="/root/.cargo/bin:${PATH}"

# --------------------------------------------------------------------
# Stage 2 — rust-deps
# Copies only the Cargo manifests and stubs every crate with a minimal
# source file so `cargo build` can download and compile all external
# dependencies without touching workspace source. This layer is
# invalidated only when Cargo.lock changes, not when .rs files change.
# --------------------------------------------------------------------
FROM toolchain AS rust-deps

WORKDIR /kalico/rust

COPY rust/Cargo.toml rust/Cargo.lock /kalico/rust/

# Copy each member's Cargo.toml into place, then stub its source.
# Members with [[bin]] targets need a src/main.rs; lib-only members
# need a src/lib.rs. cdylib crates (motion-bridge) also need lib.rs.
COPY rust/nurbs/Cargo.toml              /kalico/rust/nurbs/Cargo.toml
COPY rust/kalico-c-api/Cargo.toml       /kalico/rust/kalico-c-api/Cargo.toml
COPY rust/gcode/Cargo.toml              /kalico/rust/gcode/Cargo.toml
COPY rust/geometry/Cargo.toml           /kalico/rust/geometry/Cargo.toml
COPY rust/temporal/Cargo.toml           /kalico/rust/temporal/Cargo.toml
COPY rust/trajectory/Cargo.toml         /kalico/rust/trajectory/Cargo.toml
COPY rust/runtime/Cargo.toml            /kalico/rust/runtime/Cargo.toml
COPY rust/kalico-host-rt/Cargo.toml     /kalico/rust/kalico-host-rt/Cargo.toml
COPY rust/compat/Cargo.toml             /kalico/rust/compat/Cargo.toml
COPY rust/motion-bridge/Cargo.toml      /kalico/rust/motion-bridge/Cargo.toml
COPY rust/kalico-native-transport/Cargo.toml /kalico/rust/kalico-native-transport/Cargo.toml
COPY rust/kalico-protocol/Cargo.toml    /kalico/rust/kalico-protocol/Cargo.toml

# Stub every crate. lib-only crates get src/lib.rs; crates with [[bin]]
# also get src/main.rs (compat, kalico-c-api). kalico-c-api has an
# additional bin at src/bin/gen_headers.rs.
RUN set -e; \
    for crate in nurbs gcode geometry temporal trajectory runtime \
                 kalico-host-rt motion-bridge kalico-native-transport \
                 kalico-protocol; do \
        mkdir -p /kalico/rust/$crate/src; \
        echo "" > /kalico/rust/$crate/src/lib.rs; \
    done; \
    mkdir -p /kalico/rust/compat/src; \
    echo "" > /kalico/rust/compat/src/lib.rs; \
    echo "fn main() {}" > /kalico/rust/compat/src/main.rs; \
    mkdir -p /kalico/rust/kalico-c-api/src/bin; \
    echo "" > /kalico/rust/kalico-c-api/src/lib.rs; \
    echo "fn main() {}" > /kalico/rust/kalico-c-api/src/main.rs; \
    echo "fn main() {}" > /kalico/rust/kalico-c-api/src/bin/gen_headers.rs

# Compile dependencies only. The stubs make cargo resolve the full dep
# graph but produce no real code. Errors from stub compilation are
# expected and intentionally suppressed; we only care that dep crates
# are compiled and cached in /root/.cargo and the target directory.
RUN cargo build -p motion-bridge --release 2>&1 || true
# Wipe stub artifacts so the real build in rust-build starts clean for
# workspace crates, but leaves dep artifacts untouched. Both the compiled
# outputs AND the .fingerprint entries must go — stale fingerprints make
# cargo believe the stub output is still valid and skip recompilation.
RUN set -e; \
    for pat in \
        libmotion_bridge libkalico libnurbs libgcode libgeometry \
        libtemporal libtrajectory libruntime libcompat \
        libkalico_host_rt libkalico_native_transport libkalico_protocol; do \
        rm -f /kalico/rust/target/release/${pat}*; \
    done; \
    for pat in \
        motion_bridge kalico nurbs gcode geometry \
        temporal trajectory runtime compat \
        kalico_host_rt kalico_native_transport kalico_protocol; do \
        rm -f /kalico/rust/target/release/deps/${pat}* \
              /kalico/rust/target/release/deps/lib${pat}*; \
    done; \
    rm -rf /kalico/rust/target/release/.fingerprint/nurbs-* \
           /kalico/rust/target/release/.fingerprint/gcode-* \
           /kalico/rust/target/release/.fingerprint/geometry-* \
           /kalico/rust/target/release/.fingerprint/temporal-* \
           /kalico/rust/target/release/.fingerprint/trajectory-* \
           /kalico/rust/target/release/.fingerprint/runtime-* \
           /kalico/rust/target/release/.fingerprint/kalico-host-rt-* \
           /kalico/rust/target/release/.fingerprint/kalico-c-api-* \
           /kalico/rust/target/release/.fingerprint/compat-* \
           /kalico/rust/target/release/.fingerprint/motion-bridge-* \
           /kalico/rust/target/release/.fingerprint/kalico-native-transport-* \
           /kalico/rust/target/release/.fingerprint/kalico-protocol-*

# --------------------------------------------------------------------
# Stage 3 — rust-build
# Copies the real Rust source and builds motion-bridge for real.
# Only workspace-local crates are recompiled; all external deps are
# already in the cache from stage 2.
# --------------------------------------------------------------------
FROM rust-deps AS rust-build

COPY rust/ /kalico/rust/

RUN cd /kalico/rust && \
    cargo build -p motion-bridge --release 2>&1 | tail -20

# --------------------------------------------------------------------
# Stage 4 — firmware
# Builds the two MACH_LINUX ELFs (H7, F4), the libvtime shims, and
# the klippy chelper .so. Completely independent of Rust source.
# --------------------------------------------------------------------
FROM toolchain AS firmware

WORKDIR /kalico

# Copy what the C/firmware build needs.
COPY Makefile /kalico/Makefile
COPY src/ /kalico/src/
COPY lib/ /kalico/lib/
COPY scripts/ /kalico/scripts/
COPY klippy/ /kalico/klippy/
COPY tools/ /kalico/tools/

# Build shims (libvtime.so + libsim_intercept.so)
RUN make -C tools/kalico-sim/libvtime

# Copy Rust source for kalico-c-api build (make needs `cd rust`)
COPY rust/ /kalico/rust/

# Patch source tree for MACH_LINUX builds (idempotent).
RUN bash tools/kalico-sim/patches/fix_linux_build.sh /kalico

RUN mkdir -p /sim_elfs

# Build H7 MACH_LINUX ELF
RUN set -e; \
    if [ -f tools/sim_klippy/configs/h7-sim.config ]; then \
        cp tools/sim_klippy/configs/h7-sim.config .config; \
    else \
        cp tools/kalico-sim/configs/h7-sim.config .config; \
    fi; \
    make olddefconfig 2>/dev/null || true; \
    make clean && make -j$(nproc); \
    cp out/klipper.elf /sim_elfs/klipper-h7-sim.elf

# Build F4 MACH_LINUX ELF
RUN set -e; \
    if [ -f tools/sim_klippy/configs/f4-sim.config ]; then \
        cp tools/sim_klippy/configs/f4-sim.config .config; \
    else \
        cp tools/kalico-sim/configs/f4-sim.config .config; \
    fi; \
    make olddefconfig 2>/dev/null || true; \
    make clean && make -j$(nproc); \
    cp out/klipper.elf /sim_elfs/klipper-f4-sim.elf

# Build klippy chelper .so
RUN cd klippy && python3 chelper/__init__.py

# Fetch pinned third-party klippy plugins (no-op if the script is absent).
# The target directory is always created so the runtime COPY --from is safe.
RUN mkdir -p tools/sim_klippy/printer_real/third_party_repos; \
    if [ -f tools/sim_klippy/fetch_plugins.sh ]; then \
        bash tools/sim_klippy/fetch_plugins.sh 2>/dev/null || true; \
    fi

# --------------------------------------------------------------------
# Stage 5 — runtime (final image)
# No build toolchain. Copies only the artifacts and Python sources
# needed to run the simulator.
# --------------------------------------------------------------------
FROM ubuntu:24.04 AS runtime

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-cffi \
    python3-greenlet \
    python3-serial \
    python3-jinja2 \
    python3-markupsafe \
    libffi8 \
    libudev1 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages \
    greenlet cffi pyserial jinja2 markupsafe numpy 2>/dev/null || true

WORKDIR /kalico

# Python source first — artifacts copied after so they are not overwritten.
COPY klippy/ /kalico/klippy/
COPY tools/kalico-sim/ /kalico/tools/kalico-sim/
COPY tools/sim_klippy/ /kalico/tools/sim_klippy/

# Copy firmware ELFs
COPY --from=firmware /sim_elfs/ /kalico/out/

# Overlay chelper compiled .so files on top of the Python source copy.
COPY --from=firmware /kalico/klippy/chelper/ /kalico/klippy/chelper/

# Copy libvtime shims
COPY --from=firmware \
    /kalico/tools/kalico-sim/libvtime/libvtime.so \
    /kalico/tools/kalico-sim/libvtime/libsim_intercept.so \
    /kalico/tools/kalico-sim/libvtime/

# Overlay motion_bridge_native.so on top of the Python source copy.
COPY --from=rust-build \
    /kalico/rust/target/release/libmotion_bridge_native.so \
    /kalico/klippy/motion_bridge_native.so

# Copy third-party plugins fetched during the firmware stage (may not exist).
COPY --from=firmware /kalico/tools/sim_klippy/printer_real/third_party_repos/ \
    /kalico/tools/sim_klippy/printer_real/third_party_repos/

# Apply sim-only Python traces. Can't go in fix_linux_build.sh because
# the runtime stage re-copies klippy/ fresh from the build context.
COPY tools/kalico-sim/patches/patch_klippy_traces.py /tmp/patch_klippy_traces.py
RUN python3 /tmp/patch_klippy_traces.py klippy/motion_toolhead.py

RUN mkdir -p /gcode /tmp/kalico_sim_gcodes

ENTRYPOINT ["python3", "tools/kalico-sim/runner.py"]
CMD []
