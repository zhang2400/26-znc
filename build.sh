#!/bin/bash
set -euo pipefail

BUILD_DIR="${BUILD_DIR:-cmake-build-cross}"
TOOLCHAIN_PREFIX="${TOOLCHAIN_PREFIX:-loongarch64-linux-gnu-}"
TARGET="${1:-cv}"

if [ ! -f "CMakeLists.txt" ]; then
    echo "Error: CMakeLists.txt not found."
    exit 1
fi

echo "Configuring cross build..."
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_MAKE_PROGRAM=ninja \
    -DCMAKE_C_COMPILER="${TOOLCHAIN_PREFIX}gcc" \
    -DCMAKE_CXX_COMPILER="${TOOLCHAIN_PREFIX}g++" \
    -DUSE_LOONGARCH=ON \
    -G Ninja \
    -S . \
    -B "$BUILD_DIR"

echo "Building target: $TARGET"
cmake --build "$BUILD_DIR" --target "$TARGET"

if [ -f "$BUILD_DIR/$TARGET" ]; then
    cp "$BUILD_DIR/$TARGET" "./$TARGET"
    echo "Build finished: ./$TARGET"
    file "./$TARGET" || true
else
    echo "Build finished, but executable was not found at $BUILD_DIR/$TARGET"
fi

if [ -f "$BUILD_DIR/compile_commands.json" ]; then
    cp "$BUILD_DIR/compile_commands.json" ./compile_commands.json
    echo "Updated: ./compile_commands.json"
fi
