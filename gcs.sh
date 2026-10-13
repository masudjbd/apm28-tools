#!/usr/bin/env bash
# Launch MAVProxy as the ground station for the APM 2.8.
#
#   ./gcs.sh                 # terminal GCS on the board
#   ./gcs.sh --out udp:127.0.0.1:14550   # forward to another v1-speaking tool
#   ./gcs.sh --port /dev/cu.usbmodem113301
#
# Why MAVProxy and not a GUI:
#
# MAVProxy speaks MAVLink v1 natively, which is the whole problem with modern
# QGroundControl on this board. It is pure Python and installs into the venv,
# so there is no app bundle, no Gatekeeper prompt, no Rosetta, and no
# unmaintained Intel binary to fight -- the three things that make APM Planner 2
# painful on Apple Silicon.
#
# It is also the reference implementation for ArduPilot's interactive
# calibration handshakes, which is why calibrate_accel.py in this directory
# follows its exact ack scheme.
#
# Useful commands once it is running:
#
#   param fetch                DO THIS FIRST -- downloads the parameter table.
#                              `param show` reads a local cache, not the board,
#                              so before a fetch it silently prints nothing.
#                              Expect ~60-90s for ~600 params at 115200 on a
#                              2560; wait for "Received NNN parameters".
#   param show COMPASS*        list matching parameters
#   param set COMPASS_ORIENT 8 write one
#   accelcal                   six-position accelerometer calibration
#   magcal start               compass calibration (3.3+ firmware only, so NOT
#                              on 3.2.1 -- use ./calibrate_compass.py instead)
#   status                     link and vehicle state
#   link                       per-link packet loss and error counts
#   help                       everything else
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
cd "$SCRIPT_DIR"

PORT=""
FORWARD=""
EXTRA=()
while [ $# -gt 0 ]; do
    case "$1" in
        --port) PORT="$2"; shift 2 ;;
        --out)  FORWARD="--out=$2"; shift 2 ;;
        --qgc)
            # Measured, not assumed: MAVProxy re-emits the board's frames on its
            # UDP output with the v1 magic byte 0xFE intact -- it forwards, it
            # does not translate. QGroundControl would therefore land in exactly
            # the same one-way state as a direct serial connection: vehicle
            # visible, every request timing out.
            echo "--qgc will not work, and it is better to say so than to let you" >&2
            echo "spend an hour on it." >&2
            echo >&2
            echo "MAVProxy forwards MAVLink v1 unchanged; it does not re-encode to" >&2
            echo "v2. QGroundControl needs v2 inbound, so it would fail the same way" >&2
            echo "it does on a direct serial link." >&2
            echo >&2
            echo "For QGroundControl, use the translating bridge instead:" >&2
            echo "    ./mavbridge.py" >&2
            exit 1
            ;;
        -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
        *) EXTRA+=("$1"); shift ;;
    esac
done

if [ ! -x .venv/bin/mavproxy.py ]; then
    echo "MAVProxy not installed. Run ./setup_env.sh first." >&2
    exit 1
fi

# MAVProxy ships several undeclared dependencies -- `future` everywhere,
# `gnureadline` on macOS -- so a successful pip install is not evidence that it
# will start. Import the module that pulls them in and report once, rather than
# handing over a traceback from inside site-packages.
if ! .venv/bin/python -c "from MAVProxy.modules.lib import rline" >/dev/null 2>&1; then
    echo "MAVProxy is installed but cannot start -- a dependency it does not" >&2
    echo "declare is missing. The import error was:" >&2
    echo >&2
    .venv/bin/python -c "from MAVProxy.modules.lib import rline" 2>&1 | tail -1 | sed 's/^/    /' >&2
    echo >&2
    echo "Fix:  pip install -r requirements.txt" >&2
    exit 1
fi

if [ -z "$PORT" ]; then
    PORT="$(ls /dev/cu.usbmodem* /dev/cu.usbserial* /dev/cu.wchusbserial* 2>/dev/null | head -1 || true)"
fi
if [ -z "$PORT" ]; then
    echo "No USB serial port found. Is the board plugged in?" >&2
    exit 1
fi

# One process owns a serial port. QGroundControl reopens it via AutoConnect
# even after you disconnect the link, so check rather than emitting a stack
# trace forty lines deep in pyserial.
if command -v lsof >/dev/null 2>&1; then
    HOLDER="$(lsof -t -w "$PORT" 2>/dev/null | head -1 || true)"
    if [ -n "$HOLDER" ]; then
        NAME="$(ps -p "$HOLDER" -o comm= 2>/dev/null | xargs basename 2>/dev/null || echo '?')"
        echo "$PORT is already open by $NAME (pid $HOLDER)." >&2
        echo "Quit it first. If it is QGroundControl, disconnecting the link is" >&2
        echo "not enough -- its Comm Links -> AutoConnect settings reopen the port." >&2
        exit 1
    fi
fi

echo "  board  $PORT @ 115200"
[ -n "$FORWARD" ] && echo "  forwarding: ${FORWARD#--out=}"
echo

# "${EXTRA[@]:-}" looks like the safe way to expand a possibly-empty array and
# is not: with EXTRA empty it produces one empty-string argument, which mavproxy
# rejects as a positional. ${EXTRA[@]+"${EXTRA[@]}"} expands to nothing at all
# when empty, keeps word boundaries when populated, and works on the bash 3.2
# that macOS still ships.
exec .venv/bin/mavproxy.py \
    --master="$PORT" \
    --baudrate 115200 \
    --source-system 255 \
    ${FORWARD:+$FORWARD} \
    ${EXTRA[@]+"${EXTRA[@]}"}
