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

# No arguments: an empty field in "waiting" for programs to join (python main.py ID=0..3).
if [ $# -eq 0 ]; then
    set -- --match "$SIM_DIR/configs/match.json"
fi

exec "$EXE" --config-dir "$SIM_DIR/configs" "$@"
