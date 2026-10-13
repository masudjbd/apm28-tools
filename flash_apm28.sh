#!/usr/bin/env bash
# Flash an APM 2.x (ATmega2560) over USB with avrdude.
#
#   ./flash_apm28.sh                       # auto-detect port, quad firmware
#   ./flash_apm28.sh --frame hexa
#   ./flash_apm28.sh --port /dev/cu.usbmodem113301
#   ./flash_apm28.sh --list-ports
#
# ##  TAKE THE PROPELLERS OFF FIRST.  ##
# The board can twitch its outputs while the bootloader runs and again when the
# fresh firmware boots. A powered quad with props on is how people lose
# fingers. Firmware work happens on USB power with the battery unplugged.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
FRAME="quad"
PORT=""
HEX=""
ASSUME_YES=""

# Homebrew is not on PATH in a non-login shell.
if ! command -v avrdude >/dev/null 2>&1; then
    for b in /opt/homebrew/bin /usr/local/bin; do
        [ -x "$b/avrdude" ] && PATH="$b:$PATH"
    done
fi

list_ports() {
    # /dev/cu.* rather than /dev/tty.*: on macOS the tty.* node blocks on open
    # waiting for carrier detect, which a USB CDC board never asserts, so
    # avrdude appears to hang forever. The callout node has no such wait.
    #
    # A genuine APM 2.8 uses an ATmega16U2 bridge and enumerates as usbmodem.
    # CP2102 clones show as usbserial (driverless on macOS 11+), CH340 clones
    # as wchusbserial (needs WCH's DriverKit driver on Apple Silicon).
    ls /dev/cu.usbmodem* /dev/cu.usbserial* /dev/cu.wchusbserial* \
       /dev/cu.SLAB_USBtoUART* 2>/dev/null || true
}

while [ $# -gt 0 ]; do
    case "$1" in
        --frame) FRAME="$2"; shift 2 ;;
        --port)  PORT="$2";  shift 2 ;;
        --hex)   HEX="$2";   shift 2 ;;
        -y|--yes) ASSUME_YES=1; shift ;;
        --list-ports)
            found="$(list_ports)"
            if [ -z "$found" ]; then
                echo "no USB serial ports found"
            else
                echo "$found"
            fi
            exit 0
            ;;
        -h|--help) sed -n '2,14p' "$0"; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 1 ;;
    esac
done

if ! command -v avrdude >/dev/null 2>&1; then
    echo "avrdude not found — install it with: brew install avrdude" >&2
    exit 1
fi

[ -n "$HEX" ] || HEX="$SCRIPT_DIR/firmware/ArduCopter-$FRAME-3.2.1.hex"
if [ ! -f "$HEX" ]; then
    echo "no firmware at $HEX" >&2
    echo "fetch it first: ./fetch_firmware.sh $FRAME" >&2
    exit 1
fi

if [ -z "$PORT" ]; then
    # Plain string + line count rather than mapfile/readarray: macOS still ships
    # bash 3.2, where those builtins do not exist and an empty array under
    # `set -u` is an error besides.
    FOUND="$(list_ports)"
    COUNT="$(printf '%s\n' "$FOUND" | grep -c . || true)"
    case "$COUNT" in
        0)
            echo "No USB serial port found. Things to check, in order:" >&2
            echo "  1. Is the board plugged in? Try a different cable -- many" >&2
            echo "     mini-USB cables are charge-only and carry no data." >&2
            echo "  2. Plug straight into the Mac, not through a hub or dock." >&2
            echo "  3. ls /dev/cu.* to see everything macOS has enumerated." >&2
            exit 1
            ;;
        1) PORT="$FOUND" ;;
        *)
            echo "Several serial ports present — pick one with --port:" >&2
            printf '%s\n' "$FOUND" | sed 's/^/  /' >&2
            exit 1
            ;;
    esac
fi

echo
echo "  board     APM 2.x (ATmega2560)"
echo "  port      $PORT"
echo "  firmware  $HEX"
echo "  frame     $FRAME"
echo
echo "  PROPELLERS OFF, BATTERY UNPLUGGED — USB power only."
echo

if [ -z "$ASSUME_YES" ]; then
    printf "flash now? [y/N] "
    read -r reply
    case "$reply" in
        y|Y|yes|YES) ;;
        *) echo "aborted"; exit 1 ;;
    esac
fi

# -c wiring is the Mega 2560 bootloader protocol. Older guides say stk500v2,
# but that is for burning a bootloader over ISP -- a different operation with
# different hardware. -D suppresses the full chip erase avrdude would otherwise
# do first; the Wiring bootloader erases per page as it writes and choking it
# with a chip erase is what produces "programmer is not responding".
exec avrdude \
    -p atmega2560 \
    -c wiring \
    -P "$PORT" \
    -b 115200 \
    -D \
    -U "flash:w:$HEX:i"
