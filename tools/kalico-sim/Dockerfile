# Kalico Simulator — full-stack Klipper simulation with virtual time.
#
# Build:
#   docker build -t kalico-sim -f tools/kalico-sim/Dockerfile .
#
# Run (homing test on current branch):
#   docker run --rm kalico-sim
#
# Run (print a G-code file):
#   docker run --rm -v /path/to/file.gcode:/gcode/print.gcode \
#       kalico-sim --gcode /gcode/print.gcode
#
# Run against a specific branch:
#   docker build --build-arg BRANCH=sota-motion -t kalico-sim-sota .
#   docker run --rm kalico-sim-sota
#
# Parallel instances: each container gets its own /dev/shm.

FROM ubuntu:24.04

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
    pkg-config \
    git \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages \
    greenlet cffi pyserial jinja2 markupsafe numpy 2>/dev/null || true

WORKDIR /kalico

ARG BRANCH=HEAD

# Copy the entire repository (from the simulator branch / build context)
COPY . /kalico/

# Save simulator tools before potential branch switch — they may not
# exist on the target branch. We stash the whole tools/kalico-sim
# directory and restore it after checkout.
RUN cp -a tools/kalico-sim /tmp/_kalico_sim_tools

# Checkout target branch if specified. Use --force to discard any
# working tree changes that conflict with the target branch.
RUN if [ "$BRANCH" != "HEAD" ]; then \
        git checkout --force "$BRANCH" 2>/dev/null || echo "WARN: branch $BRANCH not found, using HEAD"; \
    fi

# Restore simulator tools on top of the target branch
RUN cp -a /tmp/_kalico_sim_tools tools/kalico-sim && rm -rf /tmp/_kalico_sim_tools

# Build shims (libvtime.so + libsim_intercept.so)
RUN make -C tools/kalico-sim/libvtime

# Build MACH_LINUX H7 firmware
# Use branch-specific config if available, else generic
# Build ELFs to a safe directory so make clean doesn't nuke them
RUN mkdir -p /sim_elfs

RUN set -e; \
    if [ -f tools/sim_klippy/configs/h7-sim.config ]; then \
        echo "=== Using branch sim_klippy H7 config ==="; \
        cp tools/sim_klippy/configs/h7-sim.config .config; \
    else \
        echo "=== Using generic kalico-sim H7 config ==="; \
        cp tools/kalico-sim/configs/h7-sim.config .config; \
    fi; \
    make olddefconfig 2>/dev/null || true; \
    make clean && make -j$(nproc); \
    cp out/klipper.elf /sim_elfs/klipper-h7-sim.elf

# Build MACH_LINUX F4 firmware
RUN set -e; \
    if [ -f tools/sim_klippy/configs/f4-sim.config ]; then \
        echo "=== Using branch sim_klippy F4 config ==="; \
        cp tools/sim_klippy/configs/f4-sim.config .config; \
    else \
        echo "=== Using generic kalico-sim F4 config ==="; \
        cp tools/kalico-sim/configs/f4-sim.config .config; \
    fi; \
    make olddefconfig 2>/dev/null || true; \
    make clean && make -j$(nproc); \
    cp out/klipper.elf /sim_elfs/klipper-f4-sim.elf

# Copy ELFs back to out/
RUN mkdir -p out && cp /sim_elfs/*.elf out/

# Build klippy C helpers
RUN cd klippy && python3 chelper/__init__.py

# Fetch third-party plugins if the script exists
RUN if [ -f tools/sim_klippy/fetch_plugins.sh ]; then \
        bash tools/sim_klippy/fetch_plugins.sh 2>/dev/null || true; \
    fi

# Create mount points and work dirs
RUN mkdir -p /gcode /tmp/kalico_sim_gcodes

ENTRYPOINT ["python3", "tools/kalico-sim/runner.py"]
CMD []
