#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export PLATFORMIO_CORE_DIR="${PLATFORMIO_CORE_DIR:-.platformio}"
exec .venv/bin/platformio "$@"
