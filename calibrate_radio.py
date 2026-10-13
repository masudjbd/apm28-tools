#!/usr/bin/env python3
"""Radio endpoint calibration: measure RCn_MIN/MAX/TRIM from live stick input.

    ./calibrate_radio.py
    ./calibrate_radio.py --write

Transmitter on, receiver bound, PROPELLERS OFF, battery unplugged. This only
reads PWM and writes parameters -- nothing is armed and no motor is commanded.
"""

import argparse
import os
import sys
import time

# Re-exec inside the project venv if we are not already there. Forgetting
# `source .venv/bin/activate` is easy, and the resulting ModuleNotFoundError
# points at pymavlink, which reads like a broken install rather than a shell
# that lost its prompt.
#
# The check is sys.prefix, NOT realpath(sys.executable). A venv's bin/python3
# is a symlink chain back to the base interpreter, so realpath() collapses the
# venv python and the system python to the SAME path -- making a comparison
# between them always report "already in the venv" and never re-exec. Inside a
# venv sys.prefix is the venv directory; outside it is the system prefix. That
# actually distinguishes them.
#
# _APM_REEXEC guards against an exec loop if the venv itself lacks the module.
_VENV_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".venv")
_VENV_PY = os.path.join(_VENV_DIR, "bin", "python3")
if (os.path.exists(_VENV_PY)
        and os.path.abspath(sys.prefix) != os.path.abspath(_VENV_DIR)
        and not os.environ.get("_APM_REEXEC")):
    try:
        import pymavlink  # noqa: F401
    except ImportError:
        os.environ["_APM_REEXEC"] = "1"
        os.execve(_VENV_PY, [_VENV_PY] + sys.argv, os.environ)

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))

from pymavlink import mavutil

import apmlink

CHANNELS = range(1, 9)
# A receiver that is powered but unbound emits a fixed value forever. Anything
# narrower than this never saw real stick travel and must not be written --
# an RCn_MIN equal to RCn_MAX makes that axis dead on the aircraft.
MIN_TRAVEL = 200


def capture(master, seconds, rate_hz=20):
    apmlink.ensure_stream(master, mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, rate_hz)
    apmlink.drain(master, 0.5)
    if not apmlink.wait_for_message(master, "RC_CHANNELS_RAW", timeout=6.0):
        apmlink.die(apmlink.no_stream_message("RC_CHANNELS_RAW", "SR0_RC_CHAN"))

    print()
    print("  Move every stick and switch to BOTH extremes, several times.")
    print("  Include the throttle. Do not skip channels you think you don't use --")
    print("  an uncalibrated aux channel is how a flight mode ends up unreachable.")
    print()
    input("  Press Enter to start ...")

    lo = {ch: 10000 for ch in CHANNELS}
    hi = {ch: 0 for ch in CHANNELS}
    seen = 0
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = master.recv_match(type="RC_CHANNELS_RAW", blocking=True, timeout=1.0)
        if msg is None:
            continue
        seen += 1
        for ch in CHANNELS:
            value = getattr(msg, "chan%d_raw" % ch, 0)
            if not 700 < value < 2400:
                continue  # 0 or 65535 means the channel is not present
            lo[ch] = min(lo[ch], value)
            hi[ch] = max(hi[ch], value)
        print("\r  %4.1fs  %d frames" % (deadline - time.time(), seen), end="", flush=True)
    print("\r  %d frames%s" % (seen, " " * 24))
    return lo, hi, seen


def read_trims(master, seconds=3, rate_hz=20):
    print()
    print("  Now CENTRE the sticks and leave throttle at minimum.")
    input("  Press Enter when settled ...")
    samples = {ch: [] for ch in CHANNELS}
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = master.recv_match(type="RC_CHANNELS_RAW", blocking=True, timeout=1.0)
        if msg is None:
            continue
        for ch in CHANNELS:
            value = getattr(msg, "chan%d_raw" % ch, 0)
            if 700 < value < 2400:
                samples[ch].append(value)
    return {ch: (sorted(v)[len(v) // 2] if v else None) for ch, v in samples.items()}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=apmlink.DEFAULT_BAUD)
    parser.add_argument("--seconds", type=int, default=25)
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()

    try:
        master = apmlink.connect(args.port, args.baud)
    except apmlink.LinkError as exc:
        apmlink.die(exc)

    lo, hi, seen = capture(master, args.seconds)
    if seen < 20:
        apmlink.die("no RC_CHANNELS_RAW arriving -- receiver not powered or not bound")
    trims = read_trims(master)

    print()
    print("  ch   min   max  travel  trim")
    print("  " + "-" * 34)
    usable = []
    for ch in CHANNELS:
        if hi[ch] == 0:
            continue
        travel = hi[ch] - lo[ch]
        flag = "" if travel >= MIN_TRAVEL else "  <- no travel, skipping"
        print("  %2d  %4d  %4d   %4d   %s%s"
              % (ch, lo[ch], hi[ch], travel, trims[ch] or "-", flag))
        if travel >= MIN_TRAVEL:
            usable.append(ch)

    if not usable:
        apmlink.die("no channel showed real stick travel")

    if 3 in usable:
        suggested = max(900, lo[3] - 30)
        print()
        print("  Throttle failsafe: FS_THR_VALUE has to sit below RC3_MIN (%d)" % lo[3])
        print("  and above whatever the receiver outputs when the signal is lost.")
        print("  %d is a reasonable value here -- confirm it by switching the" % suggested)
        print("  transmitter off and watching what RC3 actually falls to.")

    if not args.write:
        print()
        print("  dry run -- nothing written. Re-run with --write to apply.")
        return 0

    if not apmlink.confirm("\n  write RCn_MIN/MAX/TRIM for channels %s?"
                           % ",".join(str(c) for c in usable)):
        return 1

    for ch in usable:
        apmlink.set_param(master, "RC%d_MIN" % ch, lo[ch])
        apmlink.set_param(master, "RC%d_MAX" % ch, hi[ch])
        # Throttle trim is meaningless -- ArduCopter takes throttle from the raw
        # value, and a trim written at stick-low would offset the whole range.
        if ch != 3 and trims[ch]:
            apmlink.set_param(master, "RC%d_TRIM" % ch, trims[ch])
        print("  ok RC%d" % ch)
    print("\n  done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
