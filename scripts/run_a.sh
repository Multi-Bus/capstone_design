#!/usr/bin/env bash
set -euo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)/env.a.sh"
exec "$A_PYTHON" "$PROJECT_ROOT/scripts/run_a.py" "$@"
