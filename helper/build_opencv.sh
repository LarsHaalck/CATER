# /usr/bin/env sh
set -euxo pipefail

VERSION=4.6.0

apt update
apt install -y build-essential cmake unzip wget ninja-build
apt install -y libjpeg-dev libpng-dev libtiff-dev
apt install -y libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
apt install -y libxvidcore-dev libx264-dev
apt install -y libatlas-base-dev gfortran libbtbb-dev

wget https://github.com/opencv/opencv/archive/${VERSION}.tar.gz -O opencv.tar.gz

tar -xf opencv.tar.gz
rm opencv.tar.gz
cd opencv-${VERSION}

mkdir build
cd build
cmake \
    -GNinja \
    -DCMAKE_BUILD_TYPE=Release \
    -DWITH_TBB=ON \
    -DBUILD_WITH_DEBUG_INFO=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DINSTALL_C_EXAMPLES=OFF \
    -DINSTALL_PYTHON_EXAMPLES=OFF \
    -DOPENCV_ENABLE_NONFREE=OFF \
    -DOPENCV_GENERATE_SETUPVARS=OFF \
    ../
ninja -j 8 && ninja install
rm -rf opencv-${VERSION}

