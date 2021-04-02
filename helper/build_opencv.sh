# /usr/bin/env sh
VERSION=4.5.1

sudo apt install -y build-essential cmake unzip pkg-config
sudo apt install -y libjpeg-dev libpng-dev libtiff-dev
sudo apt install -y libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt install -y libxvidcore-dev libx264-dev
sudo apt install -y libatlas-base-dev gfortran libbtbb-dev

wget https://github.com/opencv/opencv/archive/${VERSION}.tar.gz -O opencv.tar.gz

tar -xf opencv.tar.gz
rm opencv.tar.gz
cd opencv-${VERSION}

mkdir build
cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=../ \
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
make -j && make install

