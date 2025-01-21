cmake_minimum_required(VERSION 3.10)
project(loongson_camera)

# 设置C和C++编译器
set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

set(EXECUTABLE_NAME cv)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fopenmp")

# 设置OpenCV路径
set(OpenCV_DIR "/usr/lib/loongarch64-linux-gnu")
find_package(OpenCV REQUIRED)

# 设置apriltag路径
find_package(apriltag REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${apriltag_INCLUDE_DIRS})

link_directories(${CMAKE_SOURCE_DIR}/lib)
link_directories(${apriltag_LIBRARY_DIRS})

# 递归搜索inc目录下的所有.h文件
file(GLOB_RECURSE HEADERS "${CMAKE_SOURCE_DIR}/inc/*.h")

# 递归搜索src目录下的所有.cpp文件
file(GLOB_RECURSE SOURCES "${CMAKE_SOURCE_DIR}/src/*.cpp")

# 查找 CPU 架构
execute_process(
        COMMAND uname -m
        OUTPUT_VARIABLE ARCHITECTURE
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

add_executable(${EXECUTABLE_NAME} ${SOURCES} ${HEADERS})

# 链接OpenCV和apriltag库
target_link_libraries(${EXECUTABLE_NAME} PRIVATE ${OpenCV_LIBS} apriltag::apriltag)