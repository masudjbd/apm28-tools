#!/usr/bin/env python3
"""Compass hard-iron calibration by sphere fit.

    ./calibrate_compass.py               # measure and report
    ./calibrate_compass.py --write

ArduCopter 3.2.1's onboard "live calibration" is a running min/max tracker: it
takes whatever extremes it happens to see and halves them. One sloppy rotation
and the offsets are wrong, with no indication that anything went wrong.

This fits a sphere to the whole sample cloud in closed form and reports how
well the data actually fits it, so a bad rotation is visible as a bad fit
rather than a bad flight.

3.2.1 stores hard-iron offsets only (COMPASS_OFS_*). The soft-iron / ellipsoid
terms (COMPASS_DIA, COMPASS_ODI) arrived in 3.4 and do not exist here, so a
sphere -- not an ellipsoid -- is the right model for this firmware.

Run this AFTER ./solve_compass_orient.py: changing COMPASS_ORIENT permutes the
axes and invalidates any offsets measured under the old value.
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

import numpy as np
from pymavlink import mavutil

import apmlink

# ArduCopter 3.2.1's pre-arm check rejects a compass whose offset vector
# exceeds this length -- it is the firmware's own "this calibration is
# nonsense" threshold, so we warn at the same number.
OFFSET_ARMING_LIMIT = 600.0


def fit_sphere(points):
    """Closed-form least-squares sphere fit.

    ||p - c||^2 = r^2  expands to  2p.c + (r^2 - |c|^2) = |p|^2, which is
    linear in (c, r^2 - |c|^2). No iteration, no initial guess, no local minima.
    """
    A = np.hstack([2.0 * points, np.ones((len(points), 1))])
    b = (points ** 2).sum(axis=1)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    center = sol[:3]
    radius = float(np.sqrt(max(sol[3] + center @ center, 0.0)))
    residual = np.linalg.norm(points - center, axis=1) - radius
    return center, radius, residual


def capture(master, seconds, rate_hz=20):
    apmlink.ensure_stream(master, mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, rate_hz)
    apmlink.drain(master, 0.5)
    if not apmlink.wait_for_message(master, "RAW_IMU", timeout=6.0):
        apmlink.die(apmlink.no_stream_message("RAW_IMU", "SR0_RAW_SENS"))

    print()
    print("  Rotate the aircraft slowly through every attitude you can reach:")
    print("  nose up, nose down, on each side, inverted, and a full turn in each.")
    print("  The goal is to paint the whole sphere, not to move fast.")
    print()
    input("  Press Enter to start ...")

    points = []
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = master.recv_match(type="RAW_IMU", blocking=True, timeout=1.0)
        if msg is None:
            continue
        points.append([msg.xmag, msg.ymag, msg.zmag])
        print("\r  %4.1fs  %d samples" % (deadline - time.time(), len(points)),
              end="", flush=True)
    print("\r  captured %d samples%s" % (len(points), " " * 20))
    return np.array(points, dtype=float)


def coverage(points, center):
    """Fraction of the sphere's octants that got sampled.

    A fit can look excellent while covering only one hemisphere -- the residual
    is small because the points really do lie on a sphere, just not on enough
    of one to pin the centre down. Octant count catches that; residual cannot.
    """
    rel = points - center
    signs = (rel > 0).astype(int)
    codes = signs[:, 0] * 4 + signs[:, 1] * 2 + signs[:, 2]
    return len(set(codes.tolist())) / 8.0


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=apmlink.DEFAULT_BAUD)
    parser.add_argument("--seconds", type=int, default=60)
    parser.add_argument("--write", action="store_true")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    try:
        master = apmlink.connect(args.port, args.baud)
    except apmlink.LinkError as exc:
        apmlink.die(exc)

    print()
    print("  PROPELLERS OFF.")
    if not apmlink.confirm("  ready?"):
        return 1

    # Offsets must be zeroed during capture: RAW_IMU reports the field with the
    # stored offsets already applied, so sampling with old offsets in place
    # measures a sphere that has already been shifted once.
    print("  zeroing existing offsets for the capture ...")
    for axis in "XYZ":
        apmlink.set_param(master, "COMPASS_OFS_%s" % axis, 0)
    apmlink.set_param(master, "COMPASS_LEARN", 0)

    points = capture(master, args.seconds)
    if len(points) < 200:
        apmlink.die("only %d samples -- board is not streaming RAW_IMU" % len(points))

    center, radius, residual = fit_sphere(points)
    offsets = -center
    spread = float(np.std(residual))
    fit_quality = spread / radius if radius > 0 else float("inf")
    octants = coverage(points, center)

    print()
    print("  field radius     %.0f  (a healthy HMC5883 reads roughly 300-600)" % radius)
    print("  fit scatter      %.1f  (%.1f%% of radius)" % (spread, fit_quality * 100))
    print("  sphere coverage  %.0f%% of octants" % (octants * 100))
    print("  offsets          X %+.0f  Y %+.0f  Z %+.0f  (|v| = %.0f)"
          % (offsets[0], offsets[1], offsets[2], np.linalg.norm(offsets)))

    problems = []
    if fit_quality > 0.08:
        problems.append("scatter is %.1f%% of radius -- there is magnetic "
                        "interference nearby, or the module moved during capture"
                        % (fit_quality * 100))
    if octants < 0.75:
        problems.append("only %.0f%% of the sphere was covered -- rotate through "
                        "more attitudes, including inverted" % (octants * 100))
    if np.linalg.norm(offsets) > OFFSET_ARMING_LIMIT:
        problems.append("offset magnitude %.0f exceeds the %.0f pre-arm limit -- "
                        "the compass is sitting too close to power wiring; raise "
                        "it on the mast" % (np.linalg.norm(offsets), OFFSET_ARMING_LIMIT))
    if not 150 < radius < 900:
        problems.append("field radius %.0f is implausible -- wrong sensor, or the "
                        "external compass is not actually the one being read" % radius)

    print()
    if problems:
        print("  PROBLEMS:")
        for problem in problems:
            print("    - %s" % problem)
    else:
        print("  calibration looks good")

    if args.write and (not problems or args.force):
        for axis, value in zip("XYZ", offsets):
            apmlink.set_param(master, "COMPASS_OFS_%s" % axis, float(value))
        print("  wrote COMPASS_OFS_X/Y/Z")
    elif args.write:
        print("  not writing -- fix the problems above, or --force")
    else:
        print("  dry run. Offsets left at zero; re-run with --write to apply.")
        print("  NOTE: offsets are currently ZERO on the board. Do not fly like this.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
