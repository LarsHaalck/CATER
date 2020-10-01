#!/bin/env sh
export CC=gcc-10
export CXX=g++-10
mkdir -p buildApp
cd buildApp
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_GUI=ON \
    -DBUILD_TOOLS=OFF \
    -DWITH_SUPERGLUE=OFF \
    -Dcxxopts_DIR=${HAB_DEP}/cxxopts/install/lib/cmake/cxxopts \
    -Dmild_DIR=${HAB_DEP}/MILD/install/cmake \
    -DCeres_DIR=${HAB_DEP}/Ceres/ \
    -Dcereal_DIR=${HAB_DEP}/cereal/install/share/cmake/cereal \
    -Dspdlog_DIR=${HAB_DEP}/spdlog/install/lib/cmake/spdlog \
    -DCMAKE_INSTALL_PREFIX=AppDir \
    ../

make install -j

linuxdeploy-x86_64.AppImage \
    --appdir AppDir \
    -d "AppDir/share/applications/habitrack-gui.desktop" \
    -i "AppDir/share/icons/hicolor/128x128/apps/habitrack-gui.png" \
    -e "AppDir/bin/habitrack-gui" \
    --output appimage \
    --plugin qt

