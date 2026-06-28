cmake_minimum_required(VERSION 3.10)
project(loongson_camera)

set(USE_OPENCV_MOBILE ON)

if(USE_OPENCV_MOBILE)
    set(CUSTOM_OPENCV_DIR "opencv-mobile")
else()
    set(CUSTOM_OPENCV_DIR "opencv")
endif()

execute_process(
        COMMAND uname -m
        OUTPUT_VARIABLE ARCHITECTURE
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(USE_LOONGARCH)
    set(ARCHITECTURE "loongarch64")
endif()

set(EXECUTABLE_NAME cv)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_definitions(-DHTTP_STREAM)
add_definitions(-DOVERWRITE_LOG)
add_definitions(-DLOG_LEVEL=3)

if(ARCHITECTURE STREQUAL "loongarch64")
    set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/lib/loongarch64/${CUSTOM_OPENCV_DIR}/lib/cmake/opencv4)
    set(OpenCV_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/lib/loongarch64/${CUSTOM_OPENCV_DIR}/include/opencv4)
elseif(ARCHITECTURE STREQUAL "x86_64")
    if(USE_OPENCV_MOBILE)
        set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/lib/x86_64/${CUSTOM_OPENCV_DIR}/lib/cmake/opencv4)
        set(OpenCV_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/lib/x86_64/${CUSTOM_OPENCV_DIR}/include/opencv4)
    endif()
    set(apriltag_DIR ${CMAKE_SOURCE_DIR}/lib/x86_64/apriltag/lib/apriltag/cmake)
    set(apriltag_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/lib/x86_64/apriltag/include)
endif()

find_package(OpenCV REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${ncnn_INCLUDE_DIRS})

file(GLOB SRC_FILES "${CMAKE_SOURCE_DIR}/src/*.cpp")
file(GLOB SRCC_FILES "${CMAKE_SOURCE_DIR}/src/*.c")
list(REMOVE_ITEM SRC_FILES "${CMAKE_SOURCE_DIR}/src/receive.cpp")
set(SOURCES ${SRC_FILES} ${CODE_FILES} ${SRCC_FILES})

add_executable(${EXECUTABLE_NAME} ${SOURCES})
target_link_libraries(${EXECUTABLE_NAME} PRIVATE ${OpenCV_LIBS} pthread)

add_executable(receive src/receive.cpp)
