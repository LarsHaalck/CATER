# /usr/bin/env sh
set -euxo pipefail

VERSION=4.6.0

sudo apt update
sudo apt install -y --no-install-recommends libjpeg-dev libpng-dev libtiff-dev
sudo apt install -y --no-install-recommends libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt install -y --no-install-recommends libxvidcore-dev libx264-dev
sudo apt install -y --no-install-recommends libatlas-base-dev gfortran libbtbb-dev
wget https://github.com/opencv/opencv/archive/${VERSION}.tar.gz -O opencv.tar.gz

tar -xf opencv.tar.gz
rm opencv.tar.gz
cd opencv-${VERSION}

mkdir build
cd build

# adapted from https://github.com/archlinux/svntogit-packages/blob/packages/opencv/trunk/PKGBUILD
cmake \
    -DCMAKE_INSTALL_PREFIX=/usr \
    -DCMAKE_BUILD_TYPE=Release \
    -DWITH_OPENCL=ON \
    -DWITH_OPENGL=ON \
    -DWITH_TBB=ON \
    -DWITH_VULKAN=ON \
    -DWITH_QT=OFF \
    -DBUILD_WITH_DEBUG_INFO=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_PROTOBUF=OFF \
    -DPROTOBUF_UPDATE_FILES=ON \
    -DINSTALL_C_EXAMPLES=OFF \
    -DINSTALL_PYTHON_EXAMPLES=OFF \
    -DCMAKE_INSTALL_PREFIX=/usr \
    -DCPU_BASELINE_DISABLE=SSE3 \
    -DCPU_BASELINE_REQUIRE=SSE2 \
    -DOPENCV_SKIP_PYTHON_LOADER=ON \
    -DOPENCV_GENERATE_PKGCONFIG=ON \
    -DOPENCV_ENABLE_NONFREE=ON \
    -DOPENCV_JNI_INSTALL_PATH=lib \
    -DOPENCV_GENERATE_SETUPVARS=OFF \
    -DEIGEN_INCLUDE_PATH="/usr/include/eigen3" \
    ../
make -j  && sudo make install
rm -rf opencv-${VERSION}
