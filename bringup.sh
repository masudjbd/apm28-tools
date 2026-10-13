#!/usr/bin/env bash
# Full APM 2.8 bring-up, in the order that matters.
#
#   ./bringup.sh --frame quad
#
# The ordering is not cosmetic. Accel calibration has to precede compass work
# because the compass fit is interpreted in the body frame the accels define.
# COMPASS_ORIENT has to precede COMPASS_OFS because changing the orientation
# permutes the axes the offsets were measured in. Radio comes before failsafes
# because FS_THR_VALUE is only meaningful once RC3_MIN is known.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
cd "$SCRIPT_DIR"

FRAME="quad"
SKIP_FLASH=""
while [ $# -gt 0 ]; do
    case "$1" in
        --frame) FRAME="$2"; shift 2 ;;
        --skip-flash) SKIP_FLASH=1; shift ;;
        -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 1 ;;
    esac
done

if [ ! -x .venv/bin/python ]; then
    echo "run ./setup_env.sh first" >&2
    exit 1
fi
PY="$SCRIPT_DIR/.venv/bin/python"

step() {
    echo
    echo "=============================================================="
    echo "  $1"
    echo "=============================================================="
    printf "  run this step? [Y/n/q] "
    read -r reply
    case "$reply" in
        q|Q) echo "  stopping here."; exit 0 ;;
        n|N) echo "  skipped."; return 1 ;;
        *) return 0 ;;
    esac
}

echo
echo "  ##  PROPELLERS OFF FOR ALL OF THIS.  ##"
echo "  Battery unplugged except where a step says otherwise."

if step "0/7  Identify what is already on the board (read-only)"; then
    "$PY" identify.py
    echo
    echo "  Read that checklist before continuing. If the board already runs a"
    echo "  working 3.2.1 you may want --skip-flash and only the steps it flags."
    printf "  press Enter to continue "
    read -r _
fi

if [ -z "$SKIP_FLASH" ]; then
    if step "1/7  Flash ArduCopter 3.2.1 ($FRAME)"; then
        ./fetch_firmware.sh "$FRAME"
        ./flash_apm28.sh --frame "$FRAME"
        echo "  waiting for the board to boot ..."
        sleep 10
    fi
fi

if step "2/7  Wipe EEPROM (stale params survive an avrdude flash)"; then
    "$PY" preload_params.py --factory-reset
    sleep 8
fi

if step "3/7  Back up parameters as shipped"; then
    "$PY" preload_params.py --backup
fi

if step "4/7  Accelerometer calibration"; then
    "$PY" calibrate_accel.py
fi

if step "5/7  Solve COMPASS_ORIENT (external compass)"; then
    "$PY" solve_compass_orient.py --write
fi

if step "6/7  Compass offsets"; then
    "$PY" calibrate_compass.py --write
fi

if step "7/7  Radio endpoints"; then
    "$PY" calibrate_radio.py --write
fi

FRAME_PROFILE="frame-x"
[ "$FRAME" = "quad" ] || FRAME_PROFILE="frame-x"
if step "Apply parameter profiles (base, $FRAME_PROFILE, compass, power, failsafe)"; then
    "$PY" preload_params.py base "$FRAME_PROFILE" compass-external power-module failsafe-3s
fi

echo
echo "  Bring-up finished. Still to do by hand, because they are not safely"
echo "  scriptable:"
echo "    - ESC calibration (props off, battery in, throttle high at power-up)"
echo "    - Flight mode switch assignment (FLTMODE1..6)"
echo "    - Verify FS_THR_VALUE against the RC3_MIN the radio step reported"
echo "    - A tethered arm test before anything leaves the ground"
