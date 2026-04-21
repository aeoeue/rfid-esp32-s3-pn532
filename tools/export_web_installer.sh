#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

export PLATFORMIO_CORE_DIR="${PLATFORMIO_CORE_DIR:-.platformio}"

.venv/bin/platformio run -e esp32s3

BUILD_DIR=".pio/build/esp32s3"
OUT_DIR="web-installer/firmware"
mkdir -p "$OUT_DIR"

cp "$BUILD_DIR/firmware.bin" "$OUT_DIR/firmware.bin"
cp "$BUILD_DIR/bootloader.bin" "$OUT_DIR/bootloader.bin"
cp "$BUILD_DIR/partitions.bin" "$OUT_DIR/partitions.bin"

# Create merged image for one-click browser flashing.
.venv/bin/platformio pkg exec -p tool-esptoolpy -- esptool.py \
  --chip esp32s3 merge_bin -o "$OUT_DIR/merged.bin" \
  0x0 "$BUILD_DIR/bootloader.bin" \
  0x8000 "$BUILD_DIR/partitions.bin" \
  0x10000 "$BUILD_DIR/firmware.bin"

echo "Web installer firmware exported to $OUT_DIR"
