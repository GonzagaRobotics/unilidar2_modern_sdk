cmake_minimum_required(VERSION 3.8)
project(unilidar2_sdk LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ZLIB REQUIRED)

include_directories(include)

add_library(unilidar2_sdk STATIC src/lidar.cpp src/decoder.cpp)
target_link_libraries(unilidar2_sdk PRIVATE ZLIB::ZLIB)

target_include_directories(unilidar2_sdk PUBLIC include)