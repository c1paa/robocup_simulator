#!/bin/bash
# Pulls the latest simulator code and rebuilds it. Safe to call from anywhere
# (cds into this script's own directory first) -- e.g. as
# ./robocup_simulator/update.sh from a project that cloned this repo as a
# subfolder (see python/README.md).
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "[update] Pulling latest changes..."
git pull

echo "[update] Rebuilding..."
bash simulator/setup.sh

echo "[update] Done. If the simulator is currently running, restart it"
echo "         (close the window and run ./run_simulator.sh again) to pick"
echo "         up the new build -- an already-running process keeps using"
echo "         the old binary."
