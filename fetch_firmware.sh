#!/usr/bin/env bash
# Fetch an ArduCopter 3.2.1 build for the APM 2.x from ArduPilot's archive.
#
#   ./fetch_firmware.sh quad      # default
#   ./fetch_firmware.sh hexa
#
# 3.2.1 (Feb 2015) is the last ArduCopter that fits in the ATmega2560's 256 KB
# of flash. There is nothing newer to fetch -- this is the ceiling for the
# board, not just the newest thing in this directory.
#
# Builds are published per airframe, not per board, so the frame type is part
# of the URL and picking the wrong one gives you a flying brick.
set -euo pipefail

FRAME="${1:-quad}"
VALID="quad hexa octa octa-quad tri y6 heli"

case " $VALID " in
    *" $FRAME "*) ;;
    *)
        echo "unknown frame '$FRAME'" >&2
        echo "valid: $VALID" >&2
        exit 1
        ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
OUT_DIR="$SCRIPT_DIR/firmware"
OUT="$OUT_DIR/ArduCopter-$FRAME-3.2.1.hex"
URL="https://firmware.ardupilot.org/Copter/stable-3.2.1/apm2-$FRAME/ArduCopter.hex"

mkdir -p "$OUT_DIR"
echo "fetching $FRAME firmware"
echo "  $URL"
curl -fL --progress-bar -o "$OUT" "$URL"

# An HTTP error page saved to disk is still a "successful" download as far as
# the shell is concerned. Intel HEX has a rigid shape, so check it: every line
# starts with a colon and the file ends with the EOF record.
#
# Strip CR before comparing: these 2015-era builds are all CRLF, so a literal
# comparison against ":00000001FF" rejects every one of them. Note that `tail`
# in a terminal hides this -- the CR just returns the cursor -- so eyeballing
# the last line is not a check. avrdude parses CRLF fine.
if grep -qv '^:' "$OUT" || [ "$(tail -1 "$OUT" | tr -d '\r\n')" != ":00000001FF" ]; then
    echo "ERROR: $OUT is not valid Intel HEX — deleting" >&2
    rm -f "$OUT"
    exit 1
fi

echo "ok: $OUT ($(wc -c < "$OUT" | tr -d ' ') bytes)"
echo "flash it with: ./flash_apm28.sh --frame $FRAME"
