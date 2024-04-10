FROM debian:bookworm-slim

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ninja-build \
        cmake \
        wget \
        git \
        gfortran \
        ca-certificates \
        libeigen3-dev \
        libopencv-dev \
        libprotobuf-dev \
        libceres-dev \
        libcxxopts-dev \
        qt6-base-dev \
        qt6-multimedia-dev \
        protobuf-compiler \
        file \
        gcc \
        g++ \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir /spdlog \
    && cd /spdlog \
    && git clone https://github.com/gabime/spdlog.git \
    && cd spdlog && mkdir build && cd build \
    && cmake -GNinja -DCMAKE_BUILD_TYPE=Release ../ \
    && ninja install \
    && rm -rf /spdlog


COPY . /cater

RUN mkdir cater/build \
    && cd cater/build \
    && cmake -GNinja \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_GUI=OFF \
        -DBUILD_TOOLS=ON \
        -WITH_SUPERGLUE=OFF \
        ../super \
    && ninja -j 4 \
    && cd external/Build/cater \
    && cmake --install .
