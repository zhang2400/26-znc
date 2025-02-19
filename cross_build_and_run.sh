#!/bin/bash

ip="192.168.1.130"
username="loongson"
toolchain_path="loongarch64-linux-gnu-"

# 检测是否存在 CMakeLists.txt 文件
if [ ! -f "CMakeLists.txt" ]; then
    echo "Error: CMakeLists.txt not found."
    exit 1
fi

BUILD_DIR="cmake-build-cross"

# 创建构建目录（如果不存在）
if [ ! -d "$BUILD_DIR" ]; then
    mkdir "$BUILD_DIR"
fi

# 运行 CMake 配置
echo "Configuring project with CMake..."
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_MAKE_PROGRAM=ninja -DCMAKE_C_COMPILER=$toolchain_path"gcc" -DCMAKE_CXX_COMPILER=$toolchain_path"g++" -DUSE_LOONGARCH=ON -G Ninja -S . -B cmake-build-cross

# 运行 CMake 构建
echo "Building project with Ninja..."
cmake --build "$BUILD_DIR"

# 复制构建结果
build_name=$(grep "set(EXECUTABLE" < CMakeLists.txt | cut -d ")" -f 1 | cut -d " " -f 2)
cp "$BUILD_DIR/$build_name" .

# 上传到开发板并运行
echo "Uploading :scp $build_name $username@$ip:/home/$username"
scp $build_name $username@$ip:/home/$username
echo "Running :ssh $username@$ip ./$build_name"
ssh $username@$ip "sudo ./$build_name"
