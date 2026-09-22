#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROTO="$SCRIPT_DIR/../simulator/proto/simulator.proto"
OUT_DIR="$SCRIPT_DIR/generated"

mkdir -p "$OUT_DIR"

python -m grpc_tools.protoc \
    --proto_path="$(dirname "$PROTO")" \
    --python_out="$OUT_DIR" \
    --grpc_python_out="$OUT_DIR" \
    "$PROTO"

echo "Generated Python gRPC stubs in $OUT_DIR"
