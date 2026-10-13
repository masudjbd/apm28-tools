#!/usr/bin/env python3
"""Determine COMPASS_ORIENT for an external compass by measurement.

    ./solve_compass_orient.py                 # 30s capture, then report
    ./solve_compass_orient.py --seconds 45
    ./solve_compass_orient.py --write         # write the answer if confident
    ./solve_compass_orient.py --replay logs/orient-xxx.json

Rather than guessing ROTATION_ROLL_180 and seeing whether the aircraft toilet
bowls, this samples gyro and magnetometer together while you rotate the
airframe by hand, then finds the one rotation in the table under which the
magnetometer actually obeys rigid-body kinematics. See lib/orientsolve.py.

PROPELLERS OFF. Battery unplugged, USB power only -- you will be picking the
aircraft up and turning it over.
"""

import argparse
import json
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
import orientsolve
import rotations

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(SCRIPT_DIR, "logs")

# Sampling runs with the board's own rotation set to identity so we see the
# sensor frame directly. COMPASS_LEARN must be off or the firmware quietly
# walks the offsets mid-capture and the constant-bias assumption breaks.
SAMPLING_PARAMS = {
    "COMPASS_EXTERNAL": 1,
    "COMPASS_ORIENT": 0,
    "COMPASS_LEARN": 0,
    "COMPASS_USE": 1,
}


def capture(master, seconds, rate_hz=50):
    apmlink.ensure_stream(master, mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, rate_hz)
    apmlink.drain(master, 0.5)
    if not apmlink.wait_for_message(master, "RAW_IMU", timeout=6.0):
        apmlink.die(apmlink.no_stream_message("RAW_IMU", "SR0_RAW_SENS"))

    print()
    print("  Rotate the aircraft continuously for %d seconds." % seconds)
    print("  Roll it, pitch it, yaw it, turn it upside down -- all three axes.")
    print("  A yaw-only spin is NOT enough and the tool will tell you so.")
    print()
    input("  Press Enter to start capturing ...")

    t, gyro, mag = [], [], []
    host0 = time.time()
    deadline = host0 + seconds
    last_print = 0.0

    while time.time() < deadline:
        msg = master.recv_match(type="RAW_IMU", blocking=True, timeout=1.0)
        if msg is None:
            continue
        # time_usec is the board clock; some builds send 0, so fall back to host
        # time. Never mix the two within a run -- decided once, below.
        t.append(msg.time_usec / 1e6 if msg.time_usec else time.time())
        # RAW_IMU gyro is milliradians/second on ArduPilot.
        gyro.append([msg.xgyro / 1000.0, msg.ygyro / 1000.0, msg.zgyro / 1000.0])
        mag.append([msg.xmag, msg.ymag, msg.zmag])

        now = time.time()
        if now - last_print > 1.0:
            last_print = now
            print("\r  %4.1fs  %d samples" % (deadline - now, len(t)), end="", flush=True)

    print("\r  captured %d samples in %.1fs%s" % (len(t), seconds, " " * 12))

    if len(t) < 100:
        raise SystemExit(
            "Only %d RAW_IMU messages arrived. The board is not streaming raw "
            "sensors -- check SR0_RAW_SENS." % len(t)
        )

    # If time_usec was flat/zero the board clock is unusable; rebuild from host.
    t = np.array(t, dtype=float)
    if np.ptp(t) < seconds * 0.5:
        print("  board clock unusable, falling back to host timestamps")
        t = np.linspace(0, seconds, len(t))
    return t, np.array(gyro), np.array(mag)


def report(results, diagnostics):
    print()
    print("  %-4s %-30s %-10s" % ("enum", "rotation", "residual"))
    print("  " + "-" * 48)
    for row in results[:6]:
        print("  %-4d %-30s %.4f" % (row["enum"], row["name"], row["residual"]))
    print("  ... %d more" % (len(results) - 6))

    cov = diagnostics["axis_coverage"]
    print()
    print("  samples used     %d" % diagnostics["samples"])
    print("  margin over #2   %.2fx" % diagnostics["margin"])
    print("  fitted gyro gain %.3f  (should be near +/-1.0)" % diagnostics["gyro_gain"])
    print("  axis coverage    X %.0f%%  Y %.0f%%  Z %.0f%%"
          % (cov[0] * 100, cov[1] * 100, cov[2] * 100))

    confident, problems = orientsolve.verdict(diagnostics)
    print()
    if confident:
        best = results[0]
        print("  ANSWER  COMPASS_ORIENT = %d  (%s)" % (best["enum"], best["name"]))
    else:
        print("  INCONCLUSIVE -- do not write this value:")
        for problem in problems:
            print("    - %s" % problem)
    return confident


def save_log(t, gyro, mag, results, diagnostics):
    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, "orient-%s.json" % time.strftime("%Y%m%d-%H%M%S"))
    with open(path, "w") as handle:
        json.dump(
            {
                "t": np.asarray(t).tolist(),
                "gyro": np.asarray(gyro).tolist(),
                "mag": np.asarray(mag).tolist(),
                "diagnostics": diagnostics,
                "ranking": [
                    {"enum": r["enum"], "name": r["name"], "residual": r["residual"]}
                    for r in results
                ],
            },
            handle,
        )
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=apmlink.DEFAULT_BAUD)
    parser.add_argument("--seconds", type=int, default=30)
    parser.add_argument("--write", action="store_true",
                        help="write COMPASS_ORIENT if the result is confident")
    parser.add_argument("--force", action="store_true",
                        help="write even when the result is inconclusive")
    parser.add_argument("--replay", help="re-solve a saved capture, no board needed")
    args = parser.parse_args()

    if args.replay:
        with open(args.replay) as handle:
            data = json.load(handle)
        results, diagnostics = orientsolve.solve(data["t"], data["gyro"], data["mag"])
        report(results, diagnostics)
        return 0

    try:
        master = apmlink.connect(args.port, args.baud)
    except apmlink.LinkError as exc:
        apmlink.die(exc)

    print()
    print("  PROPELLERS OFF -- you are about to pick this thing up and turn it over.")
    if not apmlink.confirm("  ready?"):
        return 1

    saved = {}
    for key in SAMPLING_PARAMS:
        try:
            saved[key] = apmlink.get_param(master, key)
        except apmlink.LinkError:
            saved[key] = None
    print("  current COMPASS_ORIENT = %s (%s)"
          % (saved.get("COMPASS_ORIENT"), rotations.name(saved.get("COMPASS_ORIENT") or 0)))

    for key, value in SAMPLING_PARAMS.items():
        apmlink.set_param(master, key, value)

    t, gyro, mag = capture(master, args.seconds)
    results, diagnostics = orientsolve.solve(t, gyro, mag)
    confident = report(results, diagnostics)
    path = save_log(t, gyro, mag, results, diagnostics)
    print("  capture saved to %s" % os.path.relpath(path, SCRIPT_DIR))

    best = results[0]["enum"]
    if args.write and (confident or args.force):
        apmlink.set_param(master, "COMPASS_ORIENT", best)
        print("  wrote COMPASS_ORIENT = %d" % best)
        print("  run ./calibrate_compass.py next -- offsets must be redone after this")
    elif args.write:
        print("  not writing: result inconclusive (use --force to override)")
    else:
        print("  dry run. Re-run with --write to apply, or set it by hand.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
