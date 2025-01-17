#!/bin/bash

# 检测是否存在 CMakeLists.txt 文件
if [ ! -f "CMakeLists.txt" ]; then
    echo "Error: CMakeLists.txt not found."
    exit 1
fi

GENERATOR="Ninja"
# release: r, debug: d
if [ "$1" == "d" ]; then
    BUILD_DIR="cmake-build-debug"
    CMAKE_OPTIONS="-DCMAKE_BUILD_TYPE=Debug"
else
    BUILD_DIR="cmake-build-release"
    CMAKE_OPTIONS="-DCMAKE_BUILD_TYPE=Release"
fi

# 创建构建目录（如果不存在）
if [ ! -d "$BUILD_DIR" ]; then
    mkdir "$BUILD_DIR"
fi

# 进入构建目录
cd "$BUILD_DIR" || exit

# 清理构建目录
echo "Cleaning build directory..."
rm -rf ./*

# 运行 CMake 配置
echo "Configuring project with CMake..."
cmake -G "$GENERATOR" $CMAKE_OPTIONS ..

# 运行 Ninja 构建
echo "Building project with Ninja..."
ninja

# 返回上级目录
cd ..

# 复制构建结果
build_name=$(grep "set(EXECUTABLE" < CMakeLists.txt | cut -d ")" -f 1 | cut -d " " -f 2)
cp "$BUILD_DIR/$build_name" .
