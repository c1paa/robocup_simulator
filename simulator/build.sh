#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"

echo "Building RoboCup Simulator..."
echo "Source: $SCRIPT_DIR"
echo "Build:  $BUILD_DIR"

# -----------------------------------------------------------
# Configure
# -----------------------------------------------------------
if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    echo "[cmake] First-time configure..."
    cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
else
    echo "[cmake] Re-configuring..."
    if ! cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release; then
        # A stale cache (e.g. after `brew upgrade` changed grpc/openssl) breaks
        # reconfiguring; a fresh configure works. Downloaded deps are kept.
        echo "[cmake] Reconfigure failed -- clearing the CMake cache and retrying..."
        rm -rf "$BUILD_DIR/CMakeCache.txt" "$BUILD_DIR/CMakeFiles"
        cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
    fi
fi

# -----------------------------------------------------------
# Build
# -----------------------------------------------------------
echo "[make] Compiling..."
cmake --build "$BUILD_DIR" -- -j$(sysctl -n hw.logicalcpu 2>/dev/null || nproc 2>/dev/null || echo 4)

echo "[OK] Build successful."
echo "Binary: $BUILD_DIR/simulator"
