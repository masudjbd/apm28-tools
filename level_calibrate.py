#!/usr/bin/env python3
"""Set the level reference (AHRS_TRIM) by measurement, and verify it took.

    ./level_calibrate.py            # measure, apply, verify
    ./level_calibrate.py --dry-run  # measure and report only

Put the whole aircraft on a surface you have checked with a spirit level,
battery fitted, resting on its landing gear exactly as it will sit before
takeoff. Do not hold it. Do not level the board -- level the aircraft.

Why a script rather than the GUI button
---------------------------------------
APM Planner's accel calibration times out against ArduCopter 3.2.1, and
QGroundControl cannot talk to this firmware at all. Both leave you without a
level reference. This does the same job over plain parameter writes.

Why it iterates
---------------
AHRS_TRIM's sign convention is not worth trusting from documentation -- get it
backwards and you double the error instead of removing it. So this applies a
correction, measures the result, and if the error grew it flips the sign and
tries again. It converges either way, and it tells you the final measured
attitude rather than asserting success.

A flight controller that believes it is banked when it is level will fight that
imaginary bank from the moment you arm. That is what makes an aircraft lean and
tip on takeoff.
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

import numpy as np
from pymavlink import mavutil

import apmlink

TARGET_DEG = 0.5        # good enough; below this is noise
MAX_TRIM_DEG = 10.0     # ArduCopter clamps AHRS_TRIM to +/- 10 degrees
MAX_PASSES = 5
SETTLE = 1.5


def measure(master, seconds=4.0):
    """Mean roll and pitch in degrees, plus how still the aircraft was."""
    roll, pitch = [], []
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = master.recv_match(type="ATTITUDE", blocking=True, timeout=1.0)
        if msg is None:
            continue
        roll.append(math.degrees(msg.roll))
        pitch.append(math.degrees(msg.pitch))
    if len(roll) < 10:
        apmlink.die("no ATTITUDE messages -- SR0_EXTRA1 is not streaming")
    return (float(np.mean(roll)), float(np.mean(pitch)),
            float(np.std(roll)), float(np.std(pitch)))


def clamp(deg):
    return max(-MAX_TRIM_DEG, min(MAX_TRIM_DEG, deg))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=apmlink.DEFAULT_BAUD)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--monitor", action="store_true",
                        help="stream roll/pitch live and change nothing. Use this "
                             "on a soft anti-vibration mount to watch the angle "
                             "settle, and to see what cable routing does to it.")
    args = parser.parse_args()

    try:
        master = apmlink.connect(args.port, args.baud)
    except apmlink.LinkError as exc:
        apmlink.die(exc)

    apmlink.ensure_stream(master, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10)
    apmlink.drain(master, 0.5)
    if not apmlink.wait_for_message(master, "ATTITUDE", timeout=6.0):
        apmlink.die(apmlink.no_stream_message("ATTITUDE", "SR0_EXTRA1"))

    if args.monitor:
        # A rigid mount holds whatever angle it was bolted at. A gel or rubber
        # mount does not: it creeps for minutes after being disturbed, and any
        # cable with stiffness in it -- USB, GPS, ESC leads -- holds the board
        # off-level for as long as it is connected. Watching the number is the
        # only way to tell settled from still-moving.
        print()
        print("  Live attitude. Ctrl-C to stop. Nothing is being written.")
        print("  Watch for: the reading creeping as the mount settles, and the")
        print("  reading jumping when you nudge a cable.")
        print()
        window = []
        try:
            while True:
                msg = master.recv_match(type="ATTITUDE", blocking=True, timeout=1.0)
                if msg is None:
                    continue
                roll_now = math.degrees(msg.roll)
                pitch_now = math.degrees(msg.pitch)
                window.append((roll_now, pitch_now))
                window = window[-100:]
                arr = np.array(window)
                drift_r = float(arr[:, 0].max() - arr[:, 0].min())
                drift_p = float(arr[:, 1].max() - arr[:, 1].min())
                settled = "settled" if max(drift_r, drift_p) < 0.3 else "still moving"
                print("\r  roll %+7.2f  pitch %+7.2f   |  spread over last "
                      "%ds: roll %.2f pitch %.2f  -> %-12s"
                      % (roll_now, pitch_now, len(window) // 10,
                         drift_r, drift_p, settled), end="", flush=True)
        except KeyboardInterrupt:
            print()
            print()
            print("  stopped.")
        return 0

    trim_x = math.degrees(apmlink.get_param(master, "AHRS_TRIM_X"))
    trim_y = math.degrees(apmlink.get_param(master, "AHRS_TRIM_Y"))
    print()
    print("  stored trim   roll %+.2f  pitch %+.2f  degrees" % (trim_x, trim_y))

    print()
    print("  The aircraft must be on a surface you have checked with a spirit")
    print("  level, battery fitted, sitting on its landing gear. Hands off.")
    if not apmlink.confirm("  is it?"):
        return 1

    roll, pitch, roll_noise, pitch_noise = measure(master)
    print()
    print("  measured      roll %+.2f  pitch %+.2f  degrees" % (roll, pitch))
    print("  steadiness    roll %.2f   pitch %.2f   (std dev)" % (roll_noise, pitch_noise))

    # Steadiness is judged per axis, and noise is a reason to average longer
    # rather than to refuse. A wandering estimate still has a usable mean; only
    # gross movement makes the mean meaningless.
    CLEAN, USABLE = 0.5, 2.0
    worst = max(roll_noise, pitch_noise)

    if worst > USABLE:
        print()
        print("  Too much movement to measure anything (%.2f degrees of wander)." % worst)
        print("  Settle the aircraft before retrying.")
        return 1

    if worst > CLEAN:
        # Which axis is moving tells you what is moving it.
        noisy = "roll" if roll_noise > pitch_noise else "pitch"
        ratio = (max(roll_noise, pitch_noise) /
                 max(min(roll_noise, pitch_noise), 0.01))
        print()
        print("  %s is wandering %.2f degrees while %s sits at %.2f."
              % (noisy, max(roll_noise, pitch_noise),
                 "pitch" if noisy == "roll" else "roll",
                 min(roll_noise, pitch_noise)))
        if ratio > 3:
            print()
            print("  One axis moving and the other steady is not vibration -- vibration")
            print("  would shake both. Something is rocking the aircraft about its")
            print("  %s axis. The usual causes:" % noisy)
            print("    - the USB cable is stiff and slowly pulling on the board;")
            print("      leave a loose loop so it applies no force")
            print("    - the aircraft is rocking on its landing gear, resting on three")
            print("      legs rather than four; press each leg to find the short one")
            print("    - the surface has give in it -- put the whole aircraft on a")
            print("      rigid flat board instead")
        print()
        print("  This is still measurable by averaging for longer.")
        if not apmlink.confirm("  sample for 20 seconds and use the average?"):
            return 1
        roll, pitch, roll_noise, pitch_noise = measure(master, 20.0)
        print()
        print("  averaged      roll %+.2f  pitch %+.2f  degrees" % (roll, pitch))
        print("  steadiness    roll %.2f   pitch %.2f" % (roll_noise, pitch_noise))

    if abs(roll) < TARGET_DEG and abs(pitch) < TARGET_DEG:
        print()
        print("  Already level within %.1f degrees. Nothing to do." % TARGET_DEG)
        return 0

    if args.dry_run:
        print()
        print("  dry run -- would correct by roll %+.2f, pitch %+.2f" % (-roll, -pitch))
        return 0

    # Sign of the correction is discovered, not assumed. Apply, re-measure, and
    # flip if the error grew.
    sign = 1.0
    best = (abs(roll) + abs(pitch), trim_x, trim_y, roll, pitch)

    for attempt in range(1, MAX_PASSES + 1):
        want_x = trim_x + sign * roll
        want_y = trim_y + sign * pitch
        new_x, new_y = clamp(want_x), clamp(want_y)

        # AHRS_TRIM can only express +/- 10 degrees. If the correction we need is
        # outside that, no amount of iterating helps -- the fault is the
        # accelerometer calibration underneath, and trim is the wrong lever.
        if abs(want_x) > MAX_TRIM_DEG + 0.01 or abs(want_y) > MAX_TRIM_DEG + 0.01:
            print()
            print("  The correction needed (roll %+.2f, pitch %+.2f) is beyond what"
                  % (want_x, want_y))
            print("  AHRS_TRIM can express -- it clamps at +/- %.0f degrees."
                  % MAX_TRIM_DEG)
            print()
            print("  Trim is the wrong lever for an error this size. The")
            print("  accelerometer calibration itself is wrong. Run the 6-position")
            print("  calibration first:  ./gcs.sh   then  accelcal")
            print("  Then come back and run this.")
            apmlink.set_param(master, "AHRS_TRIM_X", math.radians(best[1]))
            apmlink.set_param(master, "AHRS_TRIM_Y", math.radians(best[2]))
            return 1

        if attempt > 1 and abs(new_x - trim_x) < 0.01 and abs(new_y - trim_y) < 0.01:
            print()
            print("  Correction has stopped changing anything -- converged as far as")
            print("  trim can take it, at roll %+.2f pitch %+.2f." % (roll, pitch))
            break

        print()
        print("  pass %d: setting trim roll %+.2f  pitch %+.2f" % (attempt, new_x, new_y))
        apmlink.set_param(master, "AHRS_TRIM_X", math.radians(new_x))
        apmlink.set_param(master, "AHRS_TRIM_Y", math.radians(new_y))
        time.sleep(SETTLE)
        apmlink.drain(master, 0.3)

        roll, pitch, _, _ = measure(master, 3.0)
        error = abs(roll) + abs(pitch)
        print("          now   roll %+.2f  pitch %+.2f  degrees" % (roll, pitch))

        if error < best[0]:
            best = (error, new_x, new_y, roll, pitch)

        if abs(roll) < TARGET_DEG and abs(pitch) < TARGET_DEG:
            print()
            print("  level reference set. Reads roll %+.2f, pitch %+.2f."
                  % (roll, pitch))
            return 0

        if attempt == 1 and error > best[0]:
            print("          error grew -- the sign convention is the other way, flipping")
            sign = -1.0
            trim_x, trim_y = best[1], best[2]
            roll, pitch = best[3], best[4]
            continue

        trim_x, trim_y = new_x, new_y

    # Did not converge: leave the best measured setting rather than the last one.
    print()
    print("  did not reach %.1f degrees in %d passes." % (TARGET_DEG, MAX_PASSES))
    print("  restoring the best result: trim roll %+.2f pitch %+.2f, which read"
          % (best[1], best[2]))
    print("  roll %+.2f pitch %+.2f" % (best[3], best[4]))
    apmlink.set_param(master, "AHRS_TRIM_X", math.radians(best[1]))
    apmlink.set_param(master, "AHRS_TRIM_Y", math.radians(best[2]))
    print()
    print("  A residual this large usually means the accelerometer calibration")
    print("  itself is wrong, not the trim. Run the 6-position accel calibration")
    print("  (MAVProxy: ./gcs.sh then 'accelcal') and come back to this.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
