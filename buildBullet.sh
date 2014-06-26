#!/bin/sh

# Configuration
TOP=`pwd`
SRC="$TOP/src/bullet-2.82-r2704"
BUILD_DIR="$SRC/BuildDir"
INSTALL_PREFIX="$TOP/install"
CPU_CORES=`grep -c ^processor /proc/cpuinfo`

# Ensure a clean built
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure and Build
cmake -DCMAKE_C_FLAGS="-m32" -DCMAKE_CXX_FLAGS="-m32" -DCMAKE_SHARED_LINKER_FLAGS="-m32" \
    -DINSTALL_LIBS=True -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
    -DBUILD_SHARED_LIBS=True -DBUILD_CPU_DEMOS=False -DBUILD_DEMOS=False -DBUILD_EXTRAS=False \
    -DUSE_GLUT=False -DUSE_GRAPHICAL_BENCHMARK=False -DCMAKE_INSTALL_RPATH="." \
    -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=True \
    .. 
make "-j$CPU_CORES"

# Install
make install

