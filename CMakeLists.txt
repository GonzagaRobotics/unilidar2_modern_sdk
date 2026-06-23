cmake_minimum_required(VERSION 3.15)
project(unilidar2_sdk LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(ZLIB REQUIRED)

include_directories(include)

add_executable(unilidar2 src/main.cpp src/decoder.cpp src/lidar.cpp)
target_link_libraries(unilidar2 PRIVATE ZLIB::ZLIB)