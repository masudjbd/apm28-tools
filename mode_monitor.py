#!/usr/bin/env python3
"""Live view of the mode switch: PWM in, flight mode out.

    ./mode_monitor.py

Transmitter on, receiver bound, PROPELLERS OFF. Nothing is armed and no motor
is commanded -- this only reads.

Why this rather than guessing
-----------------------------
ArduCopter picks a flight mode from the raw PWM on channel 5, through fixed
boundaries. A three-position switch therefore does NOT reach six modes; it
reaches whichever three slots its endpoints happen to land in, and that depends
on the transmitter's actual output, its endpoint settings, and any subtrim.

Rather than assume the usual 1000/1500/2000, flick the switch and watch. What
you see is what the firmware sees.

Slot boundaries, from ArduCopter 3.2.1 readSwitch():

      < 1231  -> FLTMODE1
   1231-1360  -> FLTMODE2
   1361-1490  -> FLTMODE3
   1491-1620  -> FLTMODE4
   1621-1749  -> FLTMODE5
     >= 1750  -> FLTMODE6
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

SWITCH_BOUNDS = [(1231, 1), (1361, 2), (1491, 3), (1621, 4), (1750, 5)]

MODE_NAMES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 7: "CIRCLE", 8: "POSITION", 9: "LAND",
    10: "OF_LOITER", 11: "DRIFT", 13: "SPORT", 14: "FLIP", 15: "AUTOTUNE",
    16: "POSHOLD",
}

RECOVERY_MODES = {6, 9}


def slot_for(pwm):
    for limit, slot in SWITCH_BOUNDS:
        if pwm < limit:
            return slot
    return 6


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=apmlink.DEFAULT_BAUD)
    parser.add_argument("--channel", type=int, default=5,
                        help="mode channel, default %(default)s")
    args = parser.parse_args()

    try:
        master = apmlink.connect(args.port, args.baud)
    except apmlink.LinkError as exc:
        apmlink.die(exc)

    apmlink.ensure_stream(master, mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10)
    apmlink.ensure_stream(master, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 4)
    apmlink.drain(master, 0.5)
    if not apmlink.wait_for_message(master, "RC_CHANNELS_RAW", timeout=6.0):
        apmlink.die(apmlink.no_stream_message("RC_CHANNELS_RAW", "SR0_RC_CHAN"))

    modes = {}
    for slot in range(1, 7):
        try:
            modes[slot] = int(apmlink.get_param(master, "FLTMODE%d" % slot,
                                                timeout=3.0, retries=2))
        except apmlink.LinkError:
            modes[slot] = None

    print()
    print("  configured slots:")
    for slot in range(1, 7):
        value = modes[slot]
        print("    FLTMODE%d  %s" % (slot, MODE_NAMES.get(value, value)))

    print()
    print("  Flick the mode switch through every position. Ctrl-C when done.")
    print()

    seen_slots = {}
    lo, hi = 10000, 0
    vehicle_mode = None
    last_draw = 0.0

    try:
        while True:
            msg = master.recv_match(type=["RC_CHANNELS_RAW", "HEARTBEAT"],
                                    blocking=True, timeout=1.0)
            if msg is None:
                continue
            if msg.get_type() == "HEARTBEAT":
                vehicle_mode = msg.custom_mode
                continue

            pwm = getattr(msg, "chan%d_raw" % args.channel, 0)
            if not 700 < pwm < 2400:
                continue

            lo, hi = min(lo, pwm), max(hi, pwm)
            slot = slot_for(pwm)
            seen_slots[slot] = seen_slots.get(slot, 0) + 1

            if time.time() - last_draw > 0.1:
                last_draw = time.time()
                configured = MODE_NAMES.get(modes.get(slot), "?")
                actual = MODE_NAMES.get(vehicle_mode, vehicle_mode)
                # Live range and slot count, so a switch that is not actually
                # moving the channel is obvious immediately rather than only in
                # the summary after Ctrl-C. A span of a few microseconds is
                # noise on a stationary channel, not switch travel.
                span = hi - lo
                marker = "" if span > 100 else "   <- no travel yet"
                print("\r  ch%d %4d us -> slot %d -> %-10s | vehicle: %-10s | "
                      "range %d-%d (%d us), slots seen: %s%s"
                      % (args.channel, pwm, slot, configured, actual,
                         lo, hi, span,
                         ",".join(str(x) for x in sorted(seen_slots)) or "-",
                         marker),
                      end="", flush=True)
    except KeyboardInterrupt:
        pass

    print()
    print()
    print("  === what your switch actually reaches ===")
    if not seen_slots:
        print("  nothing seen on channel %d." % args.channel)
        return 1

    print("  PWM range observed: %d - %d us" % (lo, hi))
    print()
    reachable = sorted(seen_slots)
    for slot in reachable:
        value = modes.get(slot)
        print("    slot %d  (%5d samples)  ->  FLTMODE%d = %s"
              % (slot, seen_slots[slot], slot, MODE_NAMES.get(value, value)))

    unreachable = [s for s in range(1, 7) if s not in seen_slots]
    if unreachable:
        print()
        print("  never reached: %s"
              % ", ".join("FLTMODE%d" % s for s in unreachable))
        print("  Whatever those slots contain is irrelevant -- you cannot select them.")

    print()
    if len(reachable) < 2:
        print("  Only one slot reachable. The switch has no travel, or no switch is")
        print("  assigned to channel %d on the transmitter. On an FS-i6 that is" % args.channel)
        print("  Functions setup -> Aux. channels, and SwC is the 3-position one.")
        return 1

    selectable = {modes.get(s) for s in reachable if modes.get(s) is not None}
    if not (RECOVERY_MODES & selectable):
        print("  WARNING: none of the slots you can reach selects RTL or LAND.")
        print("  There is no way to hand control back to the aircraft in the air.")
        print("  Put RTL in one of: %s"
              % ", ".join("FLTMODE%d" % s for s in reachable))
        return 1

    print("  %d positions reachable, and one of them is a recovery mode. Good."
          % len(reachable))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
