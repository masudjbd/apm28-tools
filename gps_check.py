#!/usr/bin/env python3
"""Live GPS check: is the receiver alive, and is the fix good enough to fly on?

    ./gps_check.py                  # watch until Ctrl-C
    ./gps_check.py --seconds 600    # stop after 10 minutes
    ./gps_check.py --outdoors       # you are under open sky -- sharpens the verdict

Read-only. Nothing is armed, no motor is commanded, no parameter is written
unless you pass --enable.

Take the aircraft outside, powered, antenna facing straight up with nothing
metal above it, and run this. It prints a live view and then a verdict.

The distinction this tool exists to make:

    fix 0  NO_GPS   the DRIVER has not detected a receiver at all. This is
                    wiring, or GPS_TYPE 0, or the wrong port. Sky is irrelevant.
    fix 1  NO_FIX   the receiver IS talking and cannot see satellites. Indoors
                    this is the correct and expected reading.
    fix 2  2D       position without altitude; still acquiring.
    fix 3  3D       usable, if the satellite count and HDOP are good enough.

Confusing 0 with 1 sends you outside to wait for satellites that were never the
problem. Note also that connecting reboots the flight controller -- the GPS
module keeps its own power through that, so it does not lose what it has
already downloaded.
"""

import argparse
import math
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

FIX_NAMES = {0: "NO_GPS", 1: "NO_FIX", 2: "2D fix", 3: "3D fix", 4: "DGPS", 5: "RTK float", 6: "RTK fixed"}

FIX_MEANING = {
    0: "driver sees no receiver -- wiring or GPS_TYPE, not sky",
    1: "receiver is talking, no satellites yet",
    2: "2D only -- still acquiring, altitude not solved",
    3: "3D -- usable if sats and HDOP are good",
}

# From TARGETS.md. These are the numbers the aircraft is judged against.
SATS_GOOD, SATS_OK = 10, 8
HDOP_GOOD, HDOP_OK = 1.5, 2.0

# How long "no satellites at all" has to persist before it stops being
# "give it time" and starts being a finding. A cold u-blox with a clear view
# has the almanac and a fix well inside this.
PATIENCE = 300.0

MILESTONE_LINES = 6
EARTH_M_PER_DEG = 111320.0


def hms(seconds):
    return "%d:%02d" % (int(seconds) // 60, int(seconds) % 60)


def sat_bar(sats, width=20):
    filled = min(width, int(sats))
    return "[" + "#" * filled + "-" * (width - filled) + "]"


def scatter_metres(points):
    """Spread of a set of (lat, lon) degrees, in metres.

    A stationary receiver's reported position wanders; how far it wanders is
    the only accuracy figure available without a survey mark. Returns
    (rms, worst) or (None, None) if there is not enough to measure.
    """
    if len(points) < 10:
        return None, None
    lat0 = sum(p[0] for p in points) / len(points)
    lon0 = sum(p[1] for p in points) / len(points)
    scale_lon = EARTH_M_PER_DEG * math.cos(math.radians(lat0))
    d = [math.hypot((la - lat0) * EARTH_M_PER_DEG, (lo - lon0) * scale_lon)
         for la, lo in points]
    rms = math.sqrt(sum(x * x for x in d) / len(d))
    return rms, max(d)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=apmlink.DEFAULT_BAUD)
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="stop after this many seconds (default: run until Ctrl-C)")
    parser.add_argument("--outdoors", action="store_true",
                        help="you are under open sky; makes 'no satellites' a fault rather than expected")
    parser.add_argument("--enable", action="store_true",
                        help="if GPS_TYPE is 0, set it to 1 and reboot before watching")
    args = parser.parse_args()

    try:
        master = apmlink.connect(args.port, args.baud)
    except apmlink.LinkError as exc:
        apmlink.die(exc)

    # GPS_TYPE 0 disables the driver outright. Watching the sky for ten minutes
    # with the driver switched off is the single most wasteful way to run this
    # tool, so check the parameter before spending the time.
    gps_type = None
    try:
        gps_type = apmlink.get_param(master, "GPS_TYPE", timeout=4.0, retries=2)
    except apmlink.LinkError:
        pass

    if gps_type == 0:
        if not args.enable:
            print()
            print("  GPS_TYPE is 0 -- the GPS driver is switched OFF in software.")
            print("  The board will report 'no GPS' however clear the sky is.")
            print()
            print("  Fix it with either of:")
            print("      ./gps_check.py --enable")
            print("      ./preload_params.py --yes base")
            print()
            return 1
        print("  GPS_TYPE 0 -> 1, then rebooting so the driver probes the port ...")
        apmlink.set_param(master, "GPS_TYPE", 1)
        apmlink.reboot(master, wait=True)
        time.sleep(2)

    apmlink.ensure_stream(master, mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 3)
    apmlink.drain(master, 0.5)
    if not apmlink.wait_for_message(master, "GPS_RAW_INT", timeout=8.0):
        apmlink.die(apmlink.no_stream_message("GPS_RAW_INT", "SR0_EXT_STAT"))

    started = time.time()
    milestones = []
    fixed_points = []
    samples = 0
    best_fix = 0
    best_sats = 0
    best_hdop = None
    ttff = None
    first_sat_at = None
    last = None

    def note(text):
        milestones.append("  %6s  %s" % (hms(time.time() - started), text))

    print()
    print("  Antenna facing UP, nothing metal above it. Ctrl-C when done.")
    print()
    for _ in range(MILESTONE_LINES + 6):
        print()
    last_draw = 0.0

    try:
        while True:
            if args.seconds and time.time() - started >= args.seconds:
                break

            msg = master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2.0)
            if msg is None:
                continue

            samples += 1
            last = msg
            hdop = msg.eph / 100.0 if msg.eph and msg.eph < 9999 else None

            if msg.satellites_visible and first_sat_at is None:
                first_sat_at = time.time() - started
                note("first satellite acquired")
            if msg.fix_type > best_fix:
                best_fix = msg.fix_type
                note("fix reached %d (%s)" % (msg.fix_type, FIX_NAMES.get(msg.fix_type, "?")))
                if msg.fix_type >= 3 and ttff is None:
                    ttff = time.time() - started
            if msg.satellites_visible > best_sats:
                best_sats = msg.satellites_visible
                if best_sats in (SATS_OK, SATS_GOOD):
                    note("satellite count reached %d" % best_sats)
            if hdop is not None and (best_hdop is None or hdop < best_hdop):
                best_hdop = hdop
                if hdop <= HDOP_GOOD:
                    note("HDOP reached %.2f" % hdop)
            if msg.fix_type >= 3:
                fixed_points.append((msg.lat / 1e7, msg.lon / 1e7))

            if time.time() - last_draw < 0.5:
                continue
            last_draw = time.time()

            block = [
                "  elapsed     %-8s samples %d" % (hms(time.time() - started), samples),
                "  fix         %d (%s) -- %s" % (msg.fix_type,
                                                 FIX_NAMES.get(msg.fix_type, "?"),
                                                 FIX_MEANING.get(msg.fix_type, "")),
                "  satellites  %-2d %s  want %d+" % (msg.satellites_visible,
                                                     sat_bar(msg.satellites_visible),
                                                     SATS_GOOD),
                "  HDOP        %-6s                        want under %.1f"
                % ("--" if hdop is None else "%.2f" % hdop, HDOP_GOOD),
                "  position    %s" % ("waiting" if msg.fix_type < 2 else
                                      "%.7f, %.7f   alt %.1f m"
                                      % (msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0)),
                "",
            ]
            shown = milestones[-MILESTONE_LINES:]
            block.extend(shown)
            block.extend([""] * (MILESTONE_LINES - len(shown)))

            sys.stdout.write("\033[%dA" % len(block))
            for line in block:
                sys.stdout.write("\033[2K%s\n" % line)
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass

    elapsed = time.time() - started
    print()
    print("  === summary after %s ===" % hms(elapsed))
    if last is None or samples == 0:
        print("  No GPS_RAW_INT at all -- the board never reported GPS state.")
        return 1

    hdop_txt = "--" if best_hdop is None else "%.2f" % best_hdop
    print("    best fix        %d (%s)" % (best_fix, FIX_NAMES.get(best_fix, "?")))
    print("    best satellites %d" % best_sats)
    print("    best HDOP       %s" % hdop_txt)
    if first_sat_at is not None:
        print("    first satellite %s" % hms(first_sat_at))
    if ttff is not None:
        print("    time to 3D fix  %s" % hms(ttff))
    if last.fix_type >= 2:
        print("    final position  %.7f, %.7f  alt %.1f m"
              % (last.lat / 1e7, last.lon / 1e7, last.alt / 1000.0))

    rms, worst = scatter_metres(fixed_points)
    if rms is not None:
        print("    position wander %.1f m rms, %.1f m worst, over %d fixed samples"
              % (rms, worst, len(fixed_points)))
        print("      (the aircraft was standing still, so this is the receiver's own noise)")

    print()
    print("  === verdict ===")

    if best_fix == 0:
        print("  BLOCKER: the driver never detected a receiver (fix stayed at 0).")
        print("  This is not a sky problem -- fix 0 means the GPS was never talking.")
        print("    - GPS_TYPE is %s; it must be 1" % ("unknown" if gps_type is None else "%g" % gps_type))
        print("    - check the 6-pin lead is in the APM's GPS port, seated and the right way round")
        print("    - check the module has power: its LED should do something")
        return 1

    if best_sats == 0:
        print("  The receiver is alive and talking (fix reached 1, NO_FIX), but it")
        print("  heard no satellites at all in %s." % hms(elapsed))
        if not args.outdoors:
            print()
            print("  Indoors that is the expected result and proves nothing is broken.")
            print("  Repeat outside, under open sky, with --outdoors.")
            return 0
        if elapsed < PATIENCE:
            print()
            print("  Under open sky, give it at least %s before concluding anything --" % hms(PATIENCE))
            print("  a cold receiver has to download the almanac at 50 bits per second.")
            return 0
        print()
        print("  BLOCKER: outdoors, with a clear view, %s with zero satellites" % hms(elapsed))
        print("  is not slow acquisition. The RF side is not working:")
        print("    - the patch antenna is facing down, or shadowed by the airframe")
        print("    - the antenna's connection inside the module has failed")
        print("    - the module is counterfeit or dead")
        print("  Serial is proven good, so this is the antenna or the module itself.")
        return 1

    if best_fix < 3:
        print("  Acquiring: %d satellite(s) heard, but no 3D fix in %s."
              % (best_sats, hms(elapsed)))
        print("  Give it longer, and move away from buildings and trees -- half the")
        print("  sky blocked doubles the time and can prevent a fix entirely.")
        return 0

    problems = []
    if best_sats < SATS_OK:
        problems.append("only %d satellites (want %d+, %d minimum for position modes)"
                        % (best_sats, SATS_GOOD, SATS_OK))
    elif best_sats < SATS_GOOD:
        problems.append("%d satellites is workable but thin; %d+ is what you want"
                        % (best_sats, SATS_GOOD))
    if best_hdop is None:
        problems.append("no HDOP reported")
    elif best_hdop > HDOP_OK:
        problems.append("HDOP %.2f is poor geometry (want under %.1f)" % (best_hdop, HDOP_GOOD))
    elif best_hdop > HDOP_GOOD:
        problems.append("HDOP %.2f is acceptable but not good (want under %.1f)"
                        % (best_hdop, HDOP_GOOD))
    if rms is not None and rms > 3.0:
        problems.append("position wanders %.1f m rms while standing still" % rms)

    if not problems:
        print("  GOOD. 3D fix, %d satellites, HDOP %s%s."
              % (best_sats, hdop_txt,
                 "" if rms is None else ", wander %.1f m rms" % rms))
        print("  This is enough for LOITER, POSHOLD and RTL -- once the compass is")
        print("  calibrated, which is a separate job and the one that actually")
        print("  decides whether RTL flies home or somewhere else.")
        return 0

    print("  3D fix reached, but not yet trustworthy for position modes:")
    for item in problems:
        print("    - %s" % item)
    print()
    print("  Wait longer, or move somewhere with more open sky. STABILIZE and")
    print("  ALT_HOLD do not use GPS and are unaffected.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
