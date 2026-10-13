#!/usr/bin/env python3
"""Determine WHICH magnetometer the board is actually reading.

    ./which_compass.py

APM 2.8 carries an onboard magnetometer, and a GPS mast module carries another.
Both sit on the same I2C bus. Which one the firmware reads depends on a hardware
link on the board -- and on APM 2.8 that link is a SOLDER BRIDGE you cut with a
knife, not the removable plastic shunt on the 2-pin JP1 header. JP1 is the servo
rail power link and has nothing to do with the compass. Pulling JP1 and cutting
the compass bridge are easy to confuse, and the parameters look identical either
way, so guessing is not good enough.

This settles it physically. A magnetometer measures the Earth's field in its own
frame, so rotating the sensor changes the reading and rotating anything else does
not. Turn the GPS mast while the airframe stays still:

  reading changes a lot  -> the external compass is the one being read
  reading barely moves   -> you are still on the internal one

No parameter can fake this, which is the point.
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

# Rotating a magnetometer 180 degrees flips the horizontal component, so the
# vector should move by roughly twice that component -- a large fraction of the
# total field. Anything under a tenth is sensor noise and slop, not rotation.
EXTERNAL_THRESHOLD = 0.25
INTERNAL_THRESHOLD = 0.08


def sample(master, seconds=3.0):
    apmlink.ensure_stream(master, mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 20)
    points = []
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = master.recv_match(type="RAW_IMU", blocking=True, timeout=1.0)
        if msg is not None:
            points.append([msg.xmag, msg.ymag, msg.zmag])
    if len(points) < 10:
        apmlink.die("not enough RAW_IMU samples -- board is not streaming raw sensors")
    return np.array(points, dtype=float).mean(axis=0)


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

    external = apmlink.get_param(master, "COMPASS_EXTERNAL")
    print()
    print("  COMPASS_EXTERNAL is currently %g" % external)
    print()

    print("=== baseline ===")
    input("  Put the aircraft down and keep everything still. Press Enter ")
    base = sample(master)
    magnitude = float(np.linalg.norm(base))
    print("  field  X %+.0f  Y %+.0f  Z %+.0f   |v| = %.0f"
          % (base[0], base[1], base[2], magnitude))
    if magnitude < 50:
        apmlink.die("field magnitude %.0f is far too small -- no magnetometer is "
                    "responding at all. If you cut the internal compass link and "
                    "the external module is not wired to the I2C port, this is "
                    "exactly what you would see." % magnitude)

    print()
    print("=== test A: rotate ONLY the GPS mast ===")
    print("  Hold the airframe completely still. Twist just the GPS/compass module")
    print("  on its mast, about half a turn.")
    input("  Press Enter once it is turned and you are holding it there ")
    rotated = sample(master)
    delta_ext = float(np.linalg.norm(rotated - base)) / magnitude
    print("  field  X %+.0f  Y %+.0f  Z %+.0f   change = %.0f%% of field"
          % (rotated[0], rotated[1], rotated[2], delta_ext * 100))

    print()
    print("=== test B: rotate the WHOLE aircraft ===")
    print("  Put the mast back roughly where it was, then turn the entire aircraft")
    print("  about half a turn.")
    input("  Press Enter once turned ")
    whole = sample(master)
    delta_all = float(np.linalg.norm(whole - base)) / magnitude
    print("  field  X %+.0f  Y %+.0f  Z %+.0f   change = %.0f%% of field"
          % (whole[0], whole[1], whole[2], delta_all * 100))

    print()
    print("=== verdict ===")
    if delta_all < INTERNAL_THRESHOLD:
        print("  INCONCLUSIVE -- turning the whole aircraft barely changed the")
        print("  reading either, so the test itself did not work. Turn it further,")
        print("  and keep it away from steel furniture, laptops and speakers.")
        return 2

    if delta_ext >= EXTERNAL_THRESHOLD:
        print("  EXTERNAL compass. Rotating the mast alone moved the field by %.0f%%,"
              % (delta_ext * 100))
        print("  so the firmware is reading the module on the mast.")
        if external < 1:
            print()
            print("  But COMPASS_EXTERNAL is still 0. Set it, then redo the")
            print("  orientation and offsets:")
            print("      ./preload_params.py compass-external")
            print("      ./solve_compass_orient.py --write")
            print("      ./calibrate_compass.py --write")
        return 0

    if delta_ext <= INTERNAL_THRESHOLD:
        print("  INTERNAL compass. Rotating the mast alone changed the field by only")
        print("  %.0f%%, while rotating the airframe changed it %.0f%% -- the sensor"
              % (delta_ext * 100, delta_all * 100))
        print("  being read is bolted to the board, not sitting on the mast.")
        print()
        print("  So the compass link on the APM has NOT been cut. If you removed a")
        print("  plastic jumper, that was JP1 -- the servo rail power link, a")
        print("  different thing entirely. The compass selection on APM 2.8 is a")
        print("  solder bridge that has to be cut with a knife.")
        print()
        print("  Also check the external module's 4-pin I2C plug is actually in the")
        print("  I2C port -- the GPS plug alone does not carry the magnetometer.")
        return 1

    print("  AMBIGUOUS -- mast rotation moved the field %.0f%%, between the %.0f%%"
          % (delta_ext * 100, INTERNAL_THRESHOLD * 100))
    print("  and %.0f%% thresholds. Rotate the mast further and run it again."
          % (EXTERNAL_THRESHOLD * 100))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
