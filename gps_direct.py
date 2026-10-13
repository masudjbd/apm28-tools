#!/usr/bin/env python3
"""Read a u-blox GPS module DIRECTLY over a USB-serial adapter.

    ./gps_direct.py                 # autodetect adapter and baud
    ./gps_direct.py --port /dev/cu.usbserial-XXXX --baud 9600

Bypasses the autopilot entirely. This is the test that separates a deaf RF
front end from an acquisition problem, because NMEA GSV sentences carry a
carrier-to-noise value per satellite -- the one number that says whether the
antenna hears the constellation at all.

Wiring (module -> adapter):
    module TX  -> adapter RX
    module GND -> adapter GND
    module VCC -> adapter 5V     (powering from the adapter rather than the
                                  APM also clears the autopilot's 5V rail and
                                  the mast cable as suspects)
Module RX is not needed; this only listens.
"""
import argparse
import glob
import os
import sys
import time

_VENV_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".venv")
_VENV_PY = os.path.join(_VENV_DIR, "bin", "python3")
if (os.path.exists(_VENV_PY)
        and os.path.abspath(sys.prefix) != os.path.abspath(_VENV_DIR)
        and not os.environ.get("_APM_REEXEC")):
    try:
        import serial  # noqa: F401
    except ImportError:
        os.environ["_APM_REEXEC"] = "1"
        os.execve(_VENV_PY, [_VENV_PY] + sys.argv, os.environ)

import serial

# The APM itself enumerates as cu.usbmodem*; excluded so we never read the
# autopilot by accident and report its silence as a dead GPS.
ADAPTER_GLOBS = ("/dev/cu.usbserial*", "/dev/cu.wchusbserial*", "/dev/cu.SLAB*")
BAUDS = (9600, 38400, 4800, 19200, 57600, 115200)


def find_adapter():
    found = []
    for pat in ADAPTER_GLOBS:
        found.extend(glob.glob(pat))
    return sorted(found)


def nmea_ok(line):
    """Validate an NMEA checksum. Garbage from a baud mismatch can still
    contain a stray '$GPGSV', so trusting the prefix alone invents data."""
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, cks = line[1:].partition("*")
    try:
        want = int(cks[:2], 16)
    except ValueError:
        return False
    got = 0
    for ch in body:
        got ^= ord(ch)
    return got == want


def sniff(port, baud, seconds=4.0):
    """Return (nmea_lines, ubx_frames) seen at this baud."""
    lines, ubx, buf = [], 0, b""
    try:
        with serial.Serial(port, baud, timeout=0.5) as ser:
            deadline = time.time() + seconds
            while time.time() < deadline:
                chunk = ser.read(512)
                if not chunk:
                    continue
                buf += chunk
                ubx += buf.count(b"\xb5\x62")
                while b"\n" in buf:
                    raw, _, buf = buf.partition(b"\n")
                    text = raw.decode("ascii", "replace").strip()
                    if nmea_ok(text):
                        lines.append(text)
                if len(buf) > 8192:
                    buf = buf[-1024:]
    except (serial.SerialException, OSError) as exc:
        return None, str(exc)
    return (lines, ubx), None


def parse_gsv(lines):
    """GSV: $GPGSV,total,idx,inview,(prn,elev,azim,snr)x4*cks
    snr is C/No in dB-Hz, EMPTY when the satellite is seen but not tracked."""
    sats, inview = {}, 0
    for ln in lines:
        f = ln.split("*")[0].split(",")
        if len(f) < 4 or not f[0].endswith("GSV"):
            continue
        try:
            inview = max(inview, int(f[3] or 0))
        except ValueError:
            pass
        for i in range(4, len(f) - 3, 4):
            prn = f[i]
            snr = f[i + 3] if i + 3 < len(f) else ""
            if not prn:
                continue
            try:
                sats[int(prn)] = int(snr) if snr else 0
            except ValueError:
                continue
    return inview, sats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int)
    ap.add_argument("--seconds", type=float, default=25.0)
    args = ap.parse_args()

    port = args.port
    if not port:
        cands = find_adapter()
        if not cands:
            print("no USB-serial adapter found.")
            print("looked for:", " ".join(ADAPTER_GLOBS))
            print()
            print("Ports present right now:")
            for p in sorted(glob.glob("/dev/cu.*")):
                tag = "  <- the APM, not the GPS" if "usbmodem" in p else ""
                print("   %s%s" % (p, tag))
            print()
            print("Plug in an FTDI/CP2102/CH340 adapter wired to the GPS "
                  "module's TX and GND, then re-run.")
            return 2
        port = cands[0]
        if len(cands) > 1:
            print("multiple adapters; using %s (override with --port)" % port)

    bauds = [args.baud] if args.baud else list(BAUDS)
    print("adapter %s" % port)

    chosen, best = None, 0
    for baud in bauds:
        res, err = sniff(port, baud)
        if err:
            print("  %6d  cannot open: %s" % (baud, err))
            continue
        lines, ubx = res
        print("  %6d  %2d valid NMEA sentences, %d UBX frames"
              % (baud, len(lines), ubx))
        if len(lines) > best:
            chosen, best = baud, len(lines)
        if not chosen and ubx and not lines:
            chosen = baud  # UBX-only module: talking, just not in NMEA

    if chosen is None:
        print()
        print("NOTHING received at any baud.")
        print("The module is not transmitting. Check TX/GND wiring and that "
              "the module has power -- this is not an antenna fault.")
        return 3

    print()
    print("listening at %d for %.0fs ..." % (chosen, args.seconds))
    res, err = sniff(port, chosen, args.seconds)
    if err:
        print("read failed:", err)
        return 3
    lines, ubx = res

    if not lines:
        print()
        print("%d UBX frames but no NMEA -- the module is in UBX-only mode." % ubx)
        print("It is alive and talking. Re-run with u-center or enable NMEA to "
              "read C/No; this test cannot see satellites in binary mode.")
        return 4

    inview, sats = parse_gsv(lines)
    tracked = {p: s for p, s in sats.items() if s > 0}

    fix = [l for l in lines if l.split(",")[0].endswith(("GGA", "RMC"))]
    if fix:
        print("sample fix sentence:", fix[-1])

    print()
    print("satellites in view (GSV): %d" % inview)
    print("satellites with non-zero C/No: %d" % len(tracked))
    if sats:
        print()
        print("  PRN   C/No (dB-Hz)")
        for prn in sorted(sats):
            snr = sats[prn]
            bar = "#" * (snr // 3) if snr else ""
            print("  %3d   %3s  %s" % (prn, snr if snr else "--", bar))

    print()
    print("=" * 60)
    if tracked:
        strongest = max(tracked.values())
        print("VERDICT: the antenna HEARS satellites (strongest C/No %d dB-Hz)."
              % strongest)
        print("The RF path is intact -- antenna and LNA both work.")
        if strongest < 30:
            print("But %d dB-Hz is weak; 35-50 is healthy. Suspect a partially "
                  "damaged antenna, a lossy pigtail, or nearby interference." % strongest)
        if len(tracked) < 4:
            print("Only %d tracked; 4 are needed for a 3D fix." % len(tracked))
    elif inview:
        print("VERDICT: %d satellites in view but EVERY C/No is zero." % inview)
        print("Those are almanac predictions, not received signals -- the "
              "module knows where satellites should be and hears none of them.")
        print("The RF front end is deaf: antenna, pigtail, or LNA/bias.")
    else:
        print("VERDICT: no satellites in view and no C/No at all.")
        print("The module is talking but receiving nothing. Deaf RF front end, "
              "or a module with no almanac that also hears nothing.")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
