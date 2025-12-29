FROM ubuntu:18.04 AS builder

RUN apt-get update -qq && \
    apt-get install -y --no-install-recommends \
      software-properties-common && \
    add-apt-repository -y ppa:team-gcc-arm-embedded/ppa && \
    apt-get update -qq && \
    apt-get install -y --no-install-recommends \
      git make cmake python3 \
      build-essential g++ \
      gcc-arm-embedded && \
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*

# Project sources volume should be mounted at /app
COPY . /opt/microbit-samples
WORKDIR /opt/microbit-samples

# Build cache-buster (set at build time to force rebuild of this stage)
ARG BUILD_TS=""

# Debug: show libraries content before build (helps diagnose empty/partial clones)
RUN ls -la libraries || true

# Ensure git operations inside the container are allowed and start from a clean libraries dir
RUN git config --global --add safe.directory /opt/microbit-samples || true
RUN rm -rf libraries/* || true

# Run the build (will clone dependencies into `libraries/` if missing)
RUN python3 build.py

# Debug: show libraries content after build and locate any target-locked.json files
RUN ls -la libraries || true
RUN find libraries -type f -name 'target-locked.json' -print || true

# Record build timestamp and update mtimes of produced firmware files
RUN echo "$BUILD_TS" > .build_ts || true
RUN find . -maxdepth 2 -type f \( -name '*.bin' -o -name '*.hex' \) -exec touch {} \; || true

FROM scratch AS export-stage
COPY --from=builder /opt/microbit-samples/MICROBIT.bin .
COPY --from=builder /opt/microbit-samples/MICROBIT.hex .

ENTRYPOINT ["/bin/bash"]
