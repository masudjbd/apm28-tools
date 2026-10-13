#!/usr/bin/env python3
"""Live view of every RC channel at once.

    ./rc_live.py

Transmitter on, receiver bound, PROPELLERS OFF. Read-only: nothing is armed and
no motor is commanded.

Use this when you do not know which physical control drives which channel --
which is the normal state after assigning an aux switch, because the
transmitter's channel numbering and the flight controller's do not have to
agree, and a mis-set Aux menu silently lands your switch on the wrong one.

Move one control at a time and watch which row lights up.
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

CHANNELS = list(range(1, 9))
MOVED_THRESHOLD = 100      # us of travel before a channel counts as "moving"
BAR_WIDTH = 32
PWM_LO, PWM_HI = 1000, 2000

# What ArduCopter expects on the first four, so a swapped stick is obvious.
ROLES = {1: "roll", 2: "pitch", 3: "throttle", 4: "yaw", 5: "flight mode"}


def bar(pwm):
    frac = (pwm - PWM_LO) / float(PWM_HI - PWM_LO)
    frac = max(0.0, min(1.0, frac))
    filled = int(round(frac * (BAR_WIDTH - 1)))
    cells = ["-"] * BAR_WIDTH
    cells[filled] = "#"
    return "".join(cells)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=apmlink.DEFAULT_BAUD)
    args = parser.parse_args()

    try:
        master = apmlink.connect(args.port, args.baud)
    except apmlink.LinkError as exc:
        apmlink.die(exc)

    apmlink.ensure_stream(master, mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10)
    apmlink.drain(master, 0.5)
    if not apmlink.wait_for_message(master, "RC_CHANNELS_RAW", timeout=6.0):
        apmlink.die(apmlink.no_stream_message("RC_CHANNELS_RAW", "SR0_RC_CHAN"))

    # Pull the throttle failsafe settings so the live view can catch the
    # dangerous combination directly: a throttle that rests BELOW the failsafe
    # threshold means the flight controller reads "radio lost" every time the
    # stick is at minimum -- it will refuse to arm, or failsafe in flight.
    calibrated = {}
    for name in ("RC3_MIN", "RC3_MAX", "FS_THR_VALUE", "FS_THR_ENABLE"):
        try:
            calibrated[name] = apmlink.get_param(master, name, timeout=3.0, retries=2)
        except apmlink.LinkError:
            calibrated[name] = None

    now = {ch: 0 for ch in CHANNELS}
    lo = {ch: 10000 for ch in CHANNELS}
    hi = {ch: 0 for ch in CHANNELS}
    present = set()

    print()
    print("  Move ONE control at a time. Channels that move are marked MOVING.")
    print("  Ctrl-C when done.")
    print()
    for _ in CHANNELS:
        print()
    last_draw = 0.0

    try:
        while True:
            msg = master.recv_match(type="RC_CHANNELS_RAW", blocking=True, timeout=1.0)
            if msg is None:
                continue
            for ch in CHANNELS:
                pwm = getattr(msg, "chan%d_raw" % ch, 0)
                if not 700 < pwm < 2400:
                    continue
                present.add(ch)
                now[ch] = pwm
                lo[ch] = min(lo[ch], pwm)
                hi[ch] = max(hi[ch], pwm)

            if time.time() - last_draw < 0.2:
                continue
            last_draw = time.time()

            sys.stdout.write("\033[%dA" % len(CHANNELS))
            for ch in CHANNELS:
                if ch not in present:
                    sys.stdout.write("\033[2K  ch%d  %-8s %s\n"
                                     % (ch, ROLES.get(ch, ""), "not present"))
                    continue
                span = hi[ch] - lo[ch]
                flag = "MOVING" if span > MOVED_THRESHOLD else ""
                sys.stdout.write("\033[2K  ch%d  %-11s %4d us  [%s]  %4d-%4d  %s\n"
                                 % (ch, ROLES.get(ch, ""), now[ch], bar(now[ch]),
                                    lo[ch], hi[ch], flag))
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass

    print()
    print("  === summary ===")
    if not present:
        print("  No RC channels seen at all. Transmitter off, or receiver unbound.")
        return 1

    moving = [ch for ch in sorted(present) if hi[ch] - lo[ch] > MOVED_THRESHOLD]
    static = [ch for ch in sorted(present) if hi[ch] - lo[ch] <= MOVED_THRESHOLD]

    for ch in sorted(present):
        span = hi[ch] - lo[ch]
        print("    ch%d  %-11s  %4d-%4d us  (%4d us travel)%s"
              % (ch, ROLES.get(ch, ""), lo[ch], hi[ch], span,
                 "" if span > MOVED_THRESHOLD else "   <- never moved"))

    # All channels pinned to one identical value -- including inputs the
    # receiver does not physically have -- is not "several dead channels". It is
    # the APM's PPM encoder emitting its no-signal default because it is
    # decoding nothing at all. The fault is upstream of the individual leads.
    values = {now[ch] for ch in present}
    if len(present) >= 6 and len(values) == 1 and not moving:
        stuck_at = values.pop()
        print()
        print("  === no RC input at all ===")
        print("  Every channel reads exactly %d us, including inputs 7 and 8 which" % stuck_at)
        print("  your 6-channel receiver does not have. That is the APM's PPM encoder")
        print("  emitting its no-signal default -- it is decoding nothing.")
        print()
        print("  So the fault is upstream of any single lead. In order of likelihood:")
        print()
        print("   1. The receiver has no power. Look at its LED:")
        print("        off        -> no power reaching it")
        print("        flashing   -> powered, but not bound / no transmitter")
        print("        solid      -> powered and receiving")
        print("      The receiver is fed from the APM input rail, so it needs a lead")
        print("      carrying + and - as well as signal, seated on the correct rows.")
        print()
        print("   2. You wired PPM to input 1 and the APM's encoder is not in")
        print("      pass-through mode. It cannot decode a PPM stream as PWM, so it")
        print("      reports nothing on every channel. Go back to six leads.")
        print()
        print("   3. Every lead is shifted one row, so signal is landing on the +")
        print("      or - row. Check against the s + - silkscreen.")
        print()
        print("   4. Transmitter off, or the receiver lost its binding.")
        return 1

    if len(moving) == 1 and len(static) >= 3:
        print()
        print("  === one channel live, the rest frozen ===")
        print("  ch%d responds; %s do not."
              % (moving[0], ", ".join("ch%d" % c for c in static)))
        print()
        print("  Those frozen values are not zero and not identical, which means")
        print("  they are leftovers the PPM encoder is holding from when those")
        print("  inputs last had a signal -- so those channels DID work before.")
        print("  This is a connection that has come apart, not a setting.")
        print()
        print("  Isolate it: move the ch2 lead onto input 1 and re-run. If it")
        print("  responds there, the lead is fine and the input pin is not. If it")
        print("  stays frozen, the fault is in that lead or its receiver pin.")

    print()
    if 5 not in moving:
        print("  Channel 5 did not move.")
        print("  If you were flicking a switch, it is assigned to a different")
        print("  channel -- look at which row above says MOVING. On the FS-i6 that")
        print("  is Functions setup -> Aux. channels; SwC is the 3-position one.")
        if moving:
            print("  Channels that did move: %s"
                  % ", ".join("ch%d" % c for c in moving))
    else:
        print("  Channel 5 is live with %d us of travel." % (hi[5] - lo[5]))

    if static:
        print()
        print("  Never moved: %s" % ", ".join("ch%d" % c for c in static))

    # --- throttle vs failsafe threshold ---
    problems = []
    fs_value = calibrated.get("FS_THR_VALUE")
    fs_on = calibrated.get("FS_THR_ENABLE")
    rc3_min = calibrated.get("RC3_MIN")

    # Only judge the throttle if it actually moved. A channel whose lead is not
    # connected reports a frozen leftover value, and reading a failsafe
    # conclusion out of that is worse than saying nothing -- it sends you off
    # adjusting endpoints when the real fault is a plug.
    throttle_live = 3 in moving

    if not throttle_live and 3 in present:
        problems.append(
            "throttle never moved, so its reading of %d us is a frozen leftover,\n"
            "     not a live measurement. Fix the connection before drawing any\n"
            "     conclusion from that number." % lo[3])

    if throttle_live and fs_value is not None:
        if lo[3] < fs_value:
            problems.append(
                "throttle reached %d us, BELOW the failsafe threshold "
                "FS_THR_VALUE=%g.\n     The flight controller reads that as a lost "
                "radio.%s\n     Either raise the transmitter's throttle low endpoint, or "
                "lower\n     FS_THR_VALUE below the real minimum -- but it must still sit "
                "above\n     whatever the receiver outputs on genuine signal loss."
                % (lo[3], fs_value,
                   " With FS_THR_ENABLE on, it will refuse to arm or will\n     failsafe in flight."
                   if fs_on and fs_on >= 1 else ""))

    if throttle_live and rc3_min is not None and abs(lo[3] - rc3_min) > 60:
        problems.append(
            "throttle now bottoms at %d us but RC3_MIN was calibrated at %g.\n"
            "     The transmitter's endpoints or subtrim have changed since then, so\n"
            "     the stored calibration no longer matches the radio. Re-run\n"
            "     ./calibrate_radio.py --write." % (lo[3], rc3_min))

    if problems:
        print()
        print("  === PROBLEMS ===")
        for item in problems:
            print("   - %s" % item)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
