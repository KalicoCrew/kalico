# Playground "printer" image: our klippy + simulated MCUs (from the kalico-sim
# image) PLUS Moonraker + Mainsail + nginx, run together by supervisord — the
# MainsailOS model in one container. Socket sharing is trivial (one filesystem).
# VictoriaLogs + Vector are sidecars (see docker-compose.yml).
#
# BASE must be a kalico-sim image already built for the branch under test, e.g.
#   bash tools/kalico-sim/run.sh --branch <branch>     # -> kalico-sim-<branch>
#   docker build -t kalico-playground \
#     --build-arg BASE=kalico-sim-<branch>:latest \
#     -f tools/kalico-playground/Dockerfile tools/kalico-playground
ARG BASE=kalico-sim:latest
FROM ${BASE}

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      nginx git curl unzip ca-certificates supervisor procps \
      python3-venv python3-dev build-essential \
      libsodium-dev liblmdb-dev zlib1g-dev libjpeg-dev libopenjp2-7 libffi-dev \
    && rm -rf /var/lib/apt/lists/*

# --- Moonraker (shallow clone of master) ----------------------------------
RUN git clone --depth 1 https://github.com/Arksine/moonraker.git /opt/moonraker
RUN python3 -m venv /opt/moonraker-env \
    && /opt/moonraker-env/bin/pip install --no-cache-dir -U pip wheel \
    && /opt/moonraker-env/bin/pip install --no-cache-dir \
         -r /opt/moonraker/scripts/moonraker-requirements.txt

# --- Mainsail (latest release static bundle) ------------------------------
RUN mkdir -p /usr/share/nginx/html/mainsail \
    && curl -sSL -o /tmp/mainsail.zip \
         https://github.com/mainsail-crew/mainsail/releases/latest/download/mainsail.zip \
    && unzip -q /tmp/mainsail.zip -d /usr/share/nginx/html/mainsail \
    && rm /tmp/mainsail.zip

# --- configs --------------------------------------------------------------
RUN mkdir -p /moonraker_data/config /moonraker_data/logs /moonraker_data/gcodes
COPY moonraker.conf /moonraker_data/config/moonraker.conf
COPY nginx-mainsail.conf /etc/nginx/sites-available/default
COPY supervisord.conf /etc/supervisor/conf.d/playground.conf

EXPOSE 80 7125
ENTRYPOINT ["/usr/bin/supervisord", "-n", "-c", "/etc/supervisor/supervisord.conf"]
