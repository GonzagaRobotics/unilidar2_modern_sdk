cmake_minimum_required(VERSION 3.8)
project(unilidar2_sdk LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ZLIB REQUIRED)
find_package(PCL REQUIRED)

include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})

add_library(unilidar2_sdk STATIC src/lidar.cpp src/decoder.cpp src/out_buffer.cpp)
target_link_libraries(unilidar2_sdk PRIVATE ZLIB::ZLIB PUBLIC ${PCL_LIBRARIES})

target_include_directories(unilidar2_sdk PUBLIC include)