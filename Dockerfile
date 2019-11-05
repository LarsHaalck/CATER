FROM ubuntu:19.10

# timezone settings
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && apt-get install -y \
    build-essential \
    wget \
    tar \
    git \
    cmake \
    libceres-dev \
    liblapacke-dev \
    libopenblas-dev \
    libbtbb-dev \
    ffmpeg \
    libdc1394-22-dev \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    libavresample-dev \
    libx264-dev \
    && rm -rf /var/lib/apt/lists/*

# https://github.com/opencv/opencv/issues/12957
RUN cd /tmp \
    && wget -O opencv.tar.gz https://github.com/opencv/opencv/archive/4.1.2.tar.gz \
    && wget -O opencv_contrib.tar.gz https://github.com/opencv/opencv_contrib/archive/4.1.2.tar.gz \
    && tar xf opencv.tar.gz \
    && tar xf opencv_contrib.tar.gz \
    && mv opencv-4.1.2 opencv \
    && mv opencv_contrib-4.1.2 opencv_contrib \
    && cd opencv \
    && ln -s /usr/include/lapacke.h /usr/include/x86_64-linux-gnu \
    && sed -i 's/SET(Open_BLAS_INCLUDE_SEARCH_PATHS/SET(Open_BLAS_INCLUDE_SEARCH_PATHS\n  \/usr\/include\/x86_64-linux-gnu/g' cmake/OpenCVFindOpenBLAS.cmake \
    && sed -i 's/SET(Open_BLAS_LIB_SEARCH_PATHS/SET(Open_BLAS_INCLUDE_SEARCH_PATHS\n  \/usr\/lib\/x86_64-linux-gnu/g' cmake/OpenCVFindOpenBLAS.cmake \
    && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
        -D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules \
        -D OPENCV_ENABLE_NONFREE=ON ../ \
        -DWITH_OPENCL=ON \
        -DWITH_OPENGL=ON \
        -DWITH_TBB=ON \
        -DBUILD_WITH_DEBUG_INFO=OFF \
        -DBUILD_TESTS=OFF \
        -DBUILD_PERF_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DINSTALL_C_EXAMPLES=OFF \
        -DINSTALL_PYTHON_EXAMPLES=OFF \
        -DOPENCV_GENERATE_PKGCONFIG=ON \
        -DOPENCV_ENABLE_NONFREE=ON \
        -DOPENCV_JNI_INSTALL_PATH=lib \
        -DOPENCV_GENERATE_SETUPVARS=OFF \
        ../ \
    && make -j16 && make install && rm -rf /tmp/*

RUN cd /tmp \
    && git clone https://github.com/LarsHaalck/MILD \
    && cd MILD && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ../ \
    && make && make install \
    && rm -rf /tmp/*

RUN cd /tmp \
    && git clone -b onlyBinary --single-branch https://github.com/LarsHaalck/cereal \
    && cd cereal && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ../ \
    && make && make install \
    && rm -rf /tmp/*

