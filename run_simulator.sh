#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SIM_DIR="$SCRIPT_DIR/simulator"
BUILD_DIR="$SIM_DIR/build"
EXE="$BUILD_DIR/simulator"

if [ ! -f "$EXE" ]; then
    echo "Simulator not built yet. Building..."
    "$SIM_DIR/build.sh" || exit 1
fi

if [ ! -f "$EXE" ]; then
    echo "[ERROR] Build failed. Check output above."
    exit 1
fi

exec "$EXE" --config-dir "$SIM_DIR/configs" "$@"
