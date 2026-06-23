#!/bin/bash
set -euo pipefail

BUILD_DIR="build"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake ..
make -j 2
