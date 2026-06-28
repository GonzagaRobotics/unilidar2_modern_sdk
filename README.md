# Unilidar2 Modern SDK

A modern, from-scratch SDK for working with the Unitree L2 LiDAR. The original SDK has numerous issues, to the point of being nearly non-functional. This iteration seeks to be simple, performant, and use few dependencies that are common on a wide range of Linux systems.

This is meant to be used with ROS2, but the core SDK can be compiled and used independently.

# Dependencies

For the core SDK, only `zlib` and `PCL` are needed. C++ 17 is used.

# Building

The simplest way is to build the ROS2 node.

```bash
colcon build --symlink-install
```

The core SDK can be built with CMake as a static library.

```bash
# From the repository root
cd unilidar2_ros2/unilidar2_sdk/ 
mkdir build && cd build
cmake ..
make -j 4
```