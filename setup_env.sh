#!/usr/bin/env bash
# Create the Python venv used by the MAVLink-side tools.
#
#   ./setup_env.sh && source .venv/bin/activate
#
# Kept separate from the flashing tools: fetch_firmware.sh and flash_apm28.sh
# are pure shell + avrdude and need none of this.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
cd "$SCRIPT_DIR"

PY="${PYTHON:-python3}"
command -v "$PY" >/dev/null || { echo "python3 not found" >&2; exit 1; }

[ -d .venv ] || "$PY" -m venv .venv
./.venv/bin/pip install --quiet --upgrade pip
./.venv/bin/pip install --quiet -r requirements.txt

echo "ok — venv ready"
echo "activate with: source $SCRIPT_DIR/.venv/bin/activate"
