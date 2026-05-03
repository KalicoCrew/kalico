# Self-contained sim build + run image. Linux klipper.elf + klippy
# share one container so /tmp/klipper_sim_socket is local. Used by
# tools/sim_klippy/run_local.sh from a macOS host.
FROM debian:bookworm-slim

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    libgpiod-dev \
    libgpiod2 \
    libudev-dev \
    procps \
    pkg-config \
    python3 \
    python3-cffi \
    python3-pip \
    python3-serial \
    python3-greenlet \
    python3-dev \
    python3-numpy \
    python3-jinja2 \
    python3-can \
    python3-markdown \
    python3-toml \
    git \
    curl \
    ca-certificates \
    libffi-dev \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# Rust via rustup (matches what the Pi uses).
ENV RUSTUP_HOME=/opt/rust CARGO_HOME=/opt/cargo PATH=/opt/cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- \
    -y --default-toolchain 1.85 --profile minimal

WORKDIR /work
