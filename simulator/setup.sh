#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=== RoboCup Simulator Setup ==="
echo "Simulator dir: $SCRIPT_DIR"
echo "Project root:  $PROJECT_ROOT"

# -----------------------------------------------------------
# Check Homebrew
# -----------------------------------------------------------
if ! command -v brew &>/dev/null; then
    echo "[ERROR] Homebrew not found."
    echo "Install it: /bin/bash -c \"\$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)\""
    exit 1
fi

# -----------------------------------------------------------
# Install dependencies
# -----------------------------------------------------------
echo ""
echo "[1/3] Installing dependencies via Homebrew..."
brew install cmake sdl2 grpc protobuf bullet 2>/dev/null || brew upgrade cmake sdl2 grpc protobuf bullet

# -----------------------------------------------------------
# Create run script in project root
# -----------------------------------------------------------
echo ""
echo "[2/3] Creating run script at project root..."

RUN_SCRIPT="$PROJECT_ROOT/run_simulator.sh"

cat > "$RUN_SCRIPT" << 'RUNEOF'
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
RUNEOF

chmod +x "$RUN_SCRIPT"
echo "   Created: $RUN_SCRIPT"

# -----------------------------------------------------------
# Build
# -----------------------------------------------------------
echo ""
echo "[3/3] Building simulator..."
bash "$SCRIPT_DIR/build.sh"

echo ""
echo "=== Setup complete ==="
echo "Run the simulator: ./run_simulator.sh"
echo "Configs are in:     simulator/configs/"
