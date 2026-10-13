#!/usr/bin/env bash
# Static check across the toolkit. Run before trusting any edit.
#
#   ./lint.sh
#
# Exists because a NameError shipped twice: code referencing a module that the
# file never imported. Syntax checking with ast.parse does not catch that --
# the file parses fine and dies at runtime, halfway through a procedure you are
# performing on real hardware. pyflakes catches it statically in a second.
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
cd "$SCRIPT_DIR"

PY=".venv/bin/python"
[ -x "$PY" ] || PY="python3"

if ! "$PY" -c "import pyflakes" 2>/dev/null; then
    echo "installing pyflakes into the venv ..."
    "$PY" -m pip install --quiet pyflakes || {
        echo "could not install pyflakes" >&2; exit 1; }
fi

echo "=== pyflakes ==="
# lib/ is on sys.path at runtime via the sys.path.insert in each script, so tell
# pyflakes the same thing or every 'import apmlink' reads as unresolved.
PYTHONPATH=lib "$PY" -m pyflakes *.py lib/*.py tests/*.py
status=$?

echo
echo "=== --help smoke test ==="
fail=0
for f in *.py; do
    case "$f" in motion.py) continue ;; esac   # library, no CLI
    if "$PY" "$f" --help >/dev/null 2>&1; then
        echo "  ok   $f"
    else
        echo "  FAIL $f"
        fail=1
    fi
done

echo
if [ $status -eq 0 ] && [ $fail -eq 0 ]; then
    echo "clean"
else
    echo "problems found"
    exit 1
fi
