ARG OPEN_LMM_CALIBRATION_BASE=hwan0806/open-lmm@sha256:aabcc53791995ce4ddf9606f9710cbffa730bb555646f01b61844c7c6724eb6c
FROM ${OPEN_LMM_CALIBRATION_BASE}

LABEL org.opencontainers.image.title="OpenLMM Clang 15 benchmark calibration"
LABEL org.opencontainers.image.description="Clang 15 extension of the immutable owner-controlled OpenLMM calibration image"

USER root
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg && \
    curl -fsSL https://apt.llvm.org/llvm-snapshot.gpg.key \
      | gpg --dearmor -o /usr/share/keyrings/llvm-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/llvm-archive-keyring.gpg] https://apt.llvm.org/jammy/ llvm-toolchain-jammy-15 main" \
      > /etc/apt/sources.list.d/llvm15.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      clang-15=1:15.0.7-0ubuntu0.22.04.3 \
      libomp-15-dev=1:15.0.7-0ubuntu0.22.04.3 && \
    rm -rf /var/lib/apt/lists/*

ENV CC=/usr/bin/clang-15
ENV CXX=/usr/bin/clang++-15
