cmake_minimum_required(VERSION 3.8)
project(unilidar2_sdk LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
find_package(ZLIB REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

pkg_check_modules(LIBSERIAL REQUIRED IMPORTED_TARGET libserial)

include_directories(include/unilidar2_sdk)
include_directories(${PCL_INCLUDE_DIRS})

add_library(unilidar2_sdk STATIC src/lidar.cpp src/enet_source.cpp src/uart_source.cpp src/decoding.cpp src/clock_sync.cpp)
target_link_libraries(unilidar2_sdk PRIVATE ZLIB::ZLIB PUBLIC ${PCL_LIBRARIES} PkgConfig::LIBSERIAL)

target_include_directories(unilidar2_sdk PUBLIC include)
