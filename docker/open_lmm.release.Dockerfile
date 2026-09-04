ARG UBUNTU_BASE=ubuntu@sha256:79676deb51ebb02885b0b9d33788e78a37cf1045ad79d1bb04c6a222c3556b3d
ARG OPEN_LMM_VERSION=3.0.0
FROM ${UBUNTU_BASE} AS build

ARG UBUNTU_SNAPSHOT=20250801T000000Z
ARG SOURCE_DATE_EPOCH=0
ENV DEBIAN_FRONTEND=noninteractive

RUN printf '%s\n' \
      "deb [check-valid-until=no] https://snapshot.ubuntu.com/ubuntu/${UBUNTU_SNAPSHOT} jammy main universe restricted multiverse" \
      "deb [check-valid-until=no] https://snapshot.ubuntu.com/ubuntu/${UBUNTU_SNAPSHOT} jammy-updates main universe restricted multiverse" \
      "deb [check-valid-until=no] https://snapshot.ubuntu.com/ubuntu/${UBUNTU_SNAPSHOT} jammy-security main universe restricted multiverse" \
      > /etc/apt/sources.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      build-essential ca-certificates curl g++-12 gcc-12 git ninja-build patch \
      libboost-all-dev libeigen3-dev liblzf-dev libnanoflann-dev \
      libpcl-dev libtbb-dev python3-pip python3-venv && \
    rm -rf /var/lib/apt/lists/*

RUN curl --fail --location --silent --show-error \
      https://github.com/Kitware/CMake/releases/download/v3.25.3/cmake-3.25.3-linux-x86_64.tar.gz \
      --output /tmp/cmake.tar.gz && \
    echo 'd4d2ba83301b215857d3b6590cd4434a414fa151c5807693abe587bd6c03581e  /tmp/cmake.tar.gz' | sha256sum --check && \
    tar --extract --gzip --file /tmp/cmake.tar.gz --strip-components=1 --directory /usr/local && \
    rm /tmp/cmake.tar.gz

RUN curl --fail --location --silent --show-error \
      https://github.com/borglab/gtsam/archive/c57988fe554e7213c77fe379c1d7c483de26ad33.tar.gz \
      --output /tmp/gtsam.tar.gz && \
    echo '0a0c443ea84f45c637341a657f916e0a8fcb913dc203b8f5df1c1bbf32ef1d9d  /tmp/gtsam.tar.gz' | sha256sum --check && \
    tar --extract --gzip --file /tmp/gtsam.tar.gz --directory /tmp && \
    cmake -S /tmp/gtsam-c57988fe554e7213c77fe379c1d7c483de26ad33 \
      -B /tmp/gtsam-build -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_WITH_TBB=OFF && \
    cmake --build /tmp/gtsam-build --parallel 2 && \
    cmake --install /tmp/gtsam-build && \
    rm -rf /tmp/gtsam-build /tmp/gtsam-c57988fe554e7213c77fe379c1d7c483de26ad33 /tmp/gtsam.tar.gz

WORKDIR /src
COPY . .
RUN python3 -m venv --system-site-packages /opt/open_lmm-python-build && \
    /opt/open_lmm-python-build/bin/python -m pip install \
      --require-hashes --no-deps -r bindings/python/build-constraints.txt
RUN cmake -S open_lmm -B /tmp/open-lmm-build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release -DUSE_CCACHE=OFF -DBUILD_TESTING=OFF \
      -DFETCHCONTENT_UPDATES_DISCONNECTED=ON && \
    cmake --build /tmp/open-lmm-build --parallel 2 && \
    cmake --install /tmp/open-lmm-build --prefix /tmp/open-lmm-sdk && \
    cmake -S applications/cli -B /tmp/open-lmm-cli-build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      -DCMAKE_PREFIX_PATH=/tmp/open-lmm-sdk && \
    cmake --build /tmp/open-lmm-cli-build --parallel 2 && \
    cmake --install /tmp/open-lmm-build --prefix /opt/open_lmm --component Runtime && \
    cmake --install /tmp/open-lmm-build --prefix /opt/open_lmm --component Plugins && \
    cmake --install /tmp/open-lmm-cli-build --prefix /opt/open_lmm --component Tools

FROM ${UBUNTU_BASE} AS runtime
ARG UBUNTU_SNAPSHOT=20250801T000000Z
ARG OPEN_LMM_VERSION
ENV DEBIAN_FRONTEND=noninteractive
RUN printf '%s\n' \
      "deb [check-valid-until=no] https://snapshot.ubuntu.com/ubuntu/${UBUNTU_SNAPSHOT} jammy main universe restricted multiverse" \
      "deb [check-valid-until=no] https://snapshot.ubuntu.com/ubuntu/${UBUNTU_SNAPSHOT} jammy-updates main universe restricted multiverse" \
      "deb [check-valid-until=no] https://snapshot.ubuntu.com/ubuntu/${UBUNTU_SNAPSHOT} jammy-security main universe restricted multiverse" \
      > /etc/apt/sources.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates libboost-chrono1.74.0 libboost-filesystem1.74.0 \
      libboost-iostreams1.74.0 libboost-program-options1.74.0 \
      libboost-serialization1.74.0 libboost-timer1.74.0 libfmt8 libgomp1 \
      liblzf1 liblz4-1 libopencv-imgcodecs4.5d libopencv-photo4.5d \
      libpcl-common1.12 libpcl-features1.12 libpcl-filters1.12 \
      libpcl-io1.12 libpcl-kdtree1.12 libpcl-octree1.12 \
      libpcl-registration1.12 libpcl-sample-consensus1.12 \
      libpcl-search1.12 libpcl-segmentation1.12 libspdlog1 libtbb12 && \
    rm -rf /var/lib/apt/lists/* && \
    groupadd --gid 65532 openlmm && \
    useradd --no-create-home --uid 65532 --gid 65532 --shell /usr/sbin/nologin openlmm

COPY --from=build /usr/local/lib/ /usr/local/lib/
COPY --from=build /opt/open_lmm/ /opt/open_lmm/
RUN ldconfig

LABEL org.opencontainers.image.source="https://github.com/sparolab/uni-mapper" \
      org.opencontainers.image.licenses="GPL-3.0-only" \
      org.opencontainers.image.version="${OPEN_LMM_VERSION}"
ENV PATH=/opt/open_lmm/bin:$PATH
USER 65532:65532
ENTRYPOINT ["/opt/open_lmm/bin/open_lmm_batch"]
