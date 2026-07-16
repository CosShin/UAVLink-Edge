#!/usr/bin/env bash
set -euo pipefail
SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
ROOT="$(cd "$SIM_DIR/../.." && pwd)"
pattern="${1:-center}"
case "$pattern" in
  center) args=(--pattern center --duration 30) ;;
  sine) args=(--pattern sine --duration 40 --amplitude-deg 8 --noise-deg 0.5 --packet-loss 0.1) ;;
  dropout) args=(--pattern center --duration 35 --dropout-start 15 --dropout-duration 4) ;;
  *) echo "Dùng: $0 {center|sine|dropout}" >&2; exit 2 ;;
esac
exec "$ROOT/venv/bin/python" "$ROOT/tools/landing_target_sitl.py" \
  --sitl-confirm --endpoint udpin:127.0.0.1:14551 "${args[@]}"

