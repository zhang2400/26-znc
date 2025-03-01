cmake_minimum_required(VERSION 3.10)
project(loongson_camera)

execute_process(
        COMMAND uname -m
        OUTPUT_VARIABLE ARCHITECTURE
        OUTPUT_STRIP_TRAILING_WHITESPACE
)
set(EXECUTABLE_NAME cv)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_definitions(-DHTTP_STREAM)
add_definitions(-DOVERWRITE_LOG)
if(WIN32)
    set(OPENCV_MAP_IMPORTED_CONFIG "")
    set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/lib/windows-x64/opencv-mobile/x64/vc17/staticlib)
    set(OpenCV_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/lib/windows-x64/opencv-mobile/include)
    set(apriltag_DIR ${CMAKE_SOURCE_DIR}/lib/windows-x64/apriltag/lib/apriltag/cmake)
    set(apriltag_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/lib/windows-x64/apriltag/include)
elseif (ARCHITECTURE STREQUAL "loongarch64" OR USE_LOONGARCH)
    set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/lib/loongarch64/opencv-mobile/lib/cmake/opencv4)
    set(OpenCV_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/lib/loongarch64/opencv-mobile/include/opencv4)
    set(apriltag_DIR ${CMAKE_SOURCE_DIR}/lib/loongarch64/apriltag/lib/apriltag/cmake)
    set(apriltag_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/lib/loongarch64/apriltag/include)
elseif (ARCHITECTURE STREQUAL "x86_64")
    set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/lib/x86_64/opencv-mobile/lib/cmake/opencv4)
    set(OpenCV_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/lib/x86_64/opencv-mobile/include/opencv4)
    set(apriltag_DIR ${CMAKE_SOURCE_DIR}/lib/x86_64/apriltag/lib/apriltag/cmake)
    set(apriltag_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/lib/x86_64/apriltag/include)
endif()

find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${apriltag_INCLUDE_DIRS})

# 源文件列表
file(GLOB SRC_FILES "${CMAKE_SOURCE_DIR}/src/*.cpp")
file(GLOB CODE_FILES "${CMAKE_SOURCE_DIR}/usr/*.cpp")

# 合并源文件
set(SOURCES ${SRC_FILES} ${CODE_FILES}
)

# 创建可执行文件
add_executable(${EXECUTABLE_NAME} ${SOURCES})

# 链接OpenCV和apriltag库
target_link_libraries(${EXECUTABLE_NAME} PRIVATE ${OpenCV_LIBS} apriltag::apriltag)
