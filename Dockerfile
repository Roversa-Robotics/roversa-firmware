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

WORKDIR /opt/microbit-samples

# Copy only dependency files first (codal.json, module.json, build script, utils)
# These rarely change, so this layer gets cached
COPY codal.json module.json build.py CMakeLists.txt ./
COPY utils ./utils

# Create build directory and run initial build to fetch and compile libraries only
# This step gets cached if dependencies haven't changed
RUN mkdir -p build && \
    python3 build.py

# Now copy all source files (this is quick because libraries are already cached)
COPY source ./source

# Final build with all source code
RUN rm -rf build && \
    python3 build.py

FROM scratch AS export-stage
COPY --from=builder /opt/microbit-samples/MICROBIT.bin .
COPY --from=builder /opt/microbit-samples/MICROBIT.hex .

ENTRYPOINT ["/bin/bash"]
