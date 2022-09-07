#!/bin/env sh
set -euxo pipefail

export CC=gcc-10
export CXX=g++-10

# build dependencies
mkdir -p buildDeps
cd buildDeps
cmake -DBUILD_DEPS_ONLY=ON ../super
make -j
cd ../

# build cater
mkdir -p buildApp
cd buildApp
../buildDeps/cmake_cater \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_GUI=ON \
    -DBUILD_TOOLS=OFF \
    -DWITH_SUPERGLUE=OFF \
    -DCMAKE_INSTALL_PREFIX=AppDir \
    ../
make install -j 2

# select correct version of qmake
if test -f ../buildDeps/stage/bin/qmake; then
    export QMAKE=../buildDeps/stage/bin/qmake
fi

# build cater appimage
cp -r AppDir cater-appdir
linuxdeploy-x86_64.AppImage \
    --appdir AppDir \
    -d "AppDir/share/applications/cater-gui.desktop" \
    -i "AppDir/share/icons/hicolor/128x128/apps/cater-gui.png" \
    -e "AppDir/bin/cater-gui" \
    --output appimage \
    --plugin qt

# build panowizard appimage
cp -r AppDir panowizard-appdir
linuxdeploy-x86_64.AppImage \
    --appdir panowizard-appdir \
    -d "AppDir/share/applications/panowizard.desktop" \
    -i "AppDir/share/icons/hicolor/128x128/apps/panowizard.png" \
    -e "AppDir/bin/panowizard" \
    --output appimage \
    --plugin qt

