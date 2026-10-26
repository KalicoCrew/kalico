# Kalico runtime image, a drop-in replacement for mkuf/klipper
#
# Derived from prind (https://github.com/mkuf/prind), GPL-3.0.
# Paths, uid and volumes match mkuf/klipper so existing compose setups
# only need to change the image name.

## Build venv and chelper
##
FROM python:3.13-trixie AS build

COPY --from=ghcr.io/astral-sh/uv:latest /uv /bin/
ENV UV_PROJECT_ENVIRONMENT=/opt/venv UV_PYTHON=/usr/local/bin/python3 \
    UV_PYTHON_DOWNLOADS=never

WORKDIR /opt/klipper
COPY . .

# The image runs on other CPUs than the build host, so build chelper
# without -march=native
RUN (git describe --tags --always --dirty > klippy/.version || true) \
 && rm -rf .git \
 && uv sync --frozen --no-dev --group graphs --no-install-project \
 && /opt/venv/bin/python -m compileall klippy \
 && rm -f klippy/chelper/c_helper.so \
 && /opt/venv/bin/python -c "import sys; sys.path.insert(0, 'klippy'); \
      import chelper; chelper.check_gcc_option = lambda o: False; \
      chelper.get_ffi()"
###


## Runtime
##
FROM python:3.13-slim-trixie AS run

WORKDIR /opt
RUN groupadd kalico --gid 1000 \
 && useradd kalico --create-home --uid 1000 --gid kalico \
 && usermod kalico --append --groups dialout,tty \
 && mkdir -p printer_data/run printer_data/gcodes printer_data/logs \
      printer_data/config \
 && chown -R kalico:kalico /opt/*

# Optional compose healthcheck: test: ["python3", "/opt/health.py"]
COPY --chown=kalico:kalico <<EOF health.py
import json, socket, sys
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.connect("/opt/printer_data/run/klipper.sock")
sock.sendall(json.dumps({"id": 1, "method": "info"}).encode() + b"\x03")
resp = json.loads(sock.recv(4096).decode().strip("\x03"))
sys.exit(0 if resp["result"]["state"] == "ready" else 1)
EOF

# Only what klippy needs at runtime; lib/ and src/ are for MCU builds
COPY --chown=kalico:kalico --from=build /opt/klipper/klippy ./klipper/klippy
COPY --chown=kalico:kalico --from=build /opt/klipper/config ./klipper/config
COPY --chown=kalico:kalico --from=build /opt/klipper/scripts ./klipper/scripts
COPY --chown=kalico:kalico --from=build /opt/venv ./venv

USER kalico
VOLUME ["/opt/printer_data/run", "/opt/printer_data/gcodes", "/opt/printer_data/logs", "/opt/printer_data/config"]
ENTRYPOINT ["/opt/venv/bin/python", "klipper/klippy/klippy.py"]
CMD ["-I", "printer_data/run/klipper.tty", "-a", "printer_data/run/klipper.sock", "printer_data/config/printer.cfg"]
