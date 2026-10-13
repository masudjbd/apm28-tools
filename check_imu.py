#!/usr/bin/env python3
"""Verify the gyro and accelerometer work, and are oriented the way the
firmware believes they are.

    ./check_imu.py

USB power is enough -- no battery, no motors. Propellers off anyway, because
you will be picking the aircraft up and turning it.

What this catches that a static check cannot
--------------------------------------------
A board bolted down rotated from what AHRS_ORIENTATION says reads perfectly
plausible numbers. Every axis responds, the noise is healthy, nothing looks
wrong -- and the aircraft corrects the wrong way the instant it leaves the
ground. The only way to see it is to move the aircraft in a known direction and
check that the RIGHT AXIS responded, with the RIGHT SIGN.

So each motion is judged three ways:

  1. did the expected axis dominate?   (catches a board rotated in yaw, which
                                        swaps roll and pitch)
  2. was the sign correct?             (catches a board mounted upside down or
                                        backwards)
  3. did the attitude estimate agree?  (catches the accelerometer disagreeing
                                        with the gyro)

Body frame is Forward-Right-Down, so with the FORWARD arrow pointing away:

    roll RIGHT  -> gyro X positive, roll angle increases
    nose DOWN   -> gyro Y negative, pitch angle decreases
    yaw RIGHT   -> gyro Z positive

Yaw is judged on the GYRO only, never on the heading estimate. Heading comes
from the magnetometer, and on this aircraft the compass is still unresolved --
using it here would test the wrong thing and could fail a perfectly good gyro.
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
from motion import MotionRecorder

# np.trapz was renamed np.trapezoid in NumPy 2.0 and the old name now warns or
# is gone depending on version. Bind once rather than guessing which NumPy the
# venv ended up with.
_trapz = getattr(np, "trapezoid", None) or np.trapz

STILL_GYRO_LIMIT = 0.05     # rad/s at rest
LEVEL_LIMIT_DEG = 8.0
MIN_ROTATION_DEG = 20.0     # motion must be this big to judge
DOMINANCE_RATIO = 2.0       # expected axis must exceed the others by this much

AXES = "XYZ"


def stream(master, rate=30):
    for sid in (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS):
        apmlink.request_stream(master, sid, rate)
    apmlink.drain(master, 0.5)


def collect(master, seconds):
    """Gather timestamped gyro plus attitude for a fixed window (rest test only)."""
    t, gyro, att = [], [], []
    start = time.time()
    deadline = start + seconds
    while time.time() < deadline:
        msg = master.recv_match(type=["ATTITUDE", "RAW_IMU"], blocking=True, timeout=1.0)
        if msg is None:
            continue
        if msg.get_type() == "RAW_IMU":
            t.append(time.time() - start)
            gyro.append([msg.xgyro / 1000.0, msg.ygyro / 1000.0, msg.zgyro / 1000.0])
        else:
            att.append([math.degrees(msg.roll), math.degrees(msg.pitch)])
    return (np.array(t), np.array(gyro) if gyro else np.empty((0, 3)),
            np.array(att) if att else np.empty((0, 2)))


def collect_motion(master, bias):
    """Wait for the aircraft to move, record until it stops, show it happening.

    Replaces a fixed capture window. The window version required the operator to
    read a prompt, pick the aircraft up and rotate it inside a few seconds; miss
    that and it reports zero rotation, which looks exactly like a dead gyro.
    """
    recorder = MotionRecorder()
    att = []
    start = time.time()
    last_draw = 0.0
    print("      waiting for movement ... ", end="", flush=True)
    armed = False

    while True:
        msg = master.recv_match(type=["ATTITUDE", "RAW_IMU"], blocking=True, timeout=1.0)
        if msg is None:
            if time.time() - start > recorder.timeout_seconds:
                break
            continue
        if msg.get_type() == "ATTITUDE":
            att.append([math.degrees(msg.roll), math.degrees(msg.pitch)])
            continue

        now = time.time() - start
        gyro = [msg.xgyro / 1000.0, msg.ygyro / 1000.0, msg.zgyro / 1000.0]
        state = recorder.feed(now, gyro)

        if state == "recording":
            if not armed:
                armed = True
                print("moving")
            if time.time() - last_draw > 0.15:
                last_draw = time.time()
                so_far = current_rotation(recorder, bias)
                print("\r      recording   X %+6.1f  Y %+6.1f  Z %+6.1f  deg"
                      % tuple(so_far), end="", flush=True)
        elif state in ("done", "timeout"):
            break

    if recorder.state == "timeout" or not recorder.samples:
        print("\r      no movement detected in %ds%s"
              % (int(recorder.timeout_seconds), " " * 20))
        return np.empty(0), np.empty((0, 3)), np.array(att) if att else np.empty((0, 2))

    print("\r      recorded %.1fs, %d samples%s"
          % (recorder.duration(), len(recorder.samples), " " * 20))
    t = np.array([s[0] for s in recorder.samples])
    gyro = np.array([s[1] for s in recorder.samples])
    return t, gyro, np.array(att) if att else np.empty((0, 2))


def current_rotation(recorder, bias):
    if len(recorder.samples) < 3:
        return np.zeros(3)
    t = np.array([s[0] for s in recorder.samples])
    gyro = np.array([s[1] for s in recorder.samples])
    return integrate(t, gyro, bias)


def integrate(t, gyro, bias):
    """Total rotation about each body axis over the window, in degrees.

    Integrating rather than taking a peak: a peak is one noisy sample, while the
    integral is the angle you actually turned through, which is the thing being
    compared against "about 30 degrees".
    """
    if len(t) < 3:
        return np.zeros(3)
    corrected = gyro - bias
    return np.degrees(_trapz(corrected, t, axis=0))


def check(label, ok, detail=""):
    print("    %-48s %s%s" % (label, "PASS" if ok else "FAIL",
                              "  " + detail if detail else ""))
    return ok


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=apmlink.DEFAULT_BAUD)
    parser.add_argument("--rest-seconds", type=float, default=5.0,
                        help="how long to measure gyro bias at rest")
    args = parser.parse_args()

    try:
        master = apmlink.connect(args.port, args.baud)
    except apmlink.LinkError as exc:
        apmlink.die(exc)

    stream(master)
    failures = 0

    print()
    print("=== 1. at rest ===")
    input("  Set it down level and do not touch it. Press Enter ")
    t, gyro, att = collect(master, args.rest_seconds)
    if len(gyro) < 20:
        apmlink.die("no RAW_IMU arriving -- the board is not streaming raw sensors")

    bias = gyro.mean(axis=0)
    noise = gyro.std(axis=0)
    print("    gyro bias   X %+.4f  Y %+.4f  Z %+.4f  rad/s" % tuple(bias))
    print("    gyro noise  X %+.4f  Y %+.4f  Z %+.4f  rad/s" % tuple(noise))
    for i, axis in enumerate(AXES):
        if not check("gyro %s rests near zero" % axis,
                     abs(bias[i]) < STILL_GYRO_LIMIT, "%+.4f" % bias[i]):
            failures += 1
    for i, axis in enumerate(AXES):
        if not check("gyro %s is live, not stuck" % axis,
                     noise[i] > 1e-5, "sigma %.5f" % noise[i]):
            failures += 1

    if len(att):
        level = att.mean(axis=0)
        print("    attitude    roll %+.1f  pitch %+.1f  deg" % tuple(level))
        if not check("reads level on a level surface",
                     abs(level[0]) < LEVEL_LIMIT_DEG and abs(level[1]) < LEVEL_LIMIT_DEG,
                     "roll %+.1f pitch %+.1f" % tuple(level)):
            print("      -> the bench is not level, or the accel calibration was")
            print("         done on a surface that was not.")
            failures += 1
    else:
        level = np.zeros(2)

    def motion(title, instruction, axis, positive, attitude_index=None):
        nonlocal failures
        print()
        print("=== %s ===" % title)
        print("  %s" % instruction)
        print("  Take your time -- recording starts when I see the aircraft move,")
        print("  and stops when it settles. No stopwatch.")
        input("  Press Enter when you are ready to pick it up ")
        t2, gyro2, att2 = collect_motion(master, bias)
        if len(gyro2) < 20:
            check(title, False, "no movement recorded")
            print("      -> nothing moved. If you did move it, the gyro is not")
            print("         reporting rotation and that is a real fault.")
            failures += 1
            return

        rotation = integrate(t2, gyro2, bias)
        print("    rotated   X %+.0f  Y %+.0f  Z %+.0f  deg" % tuple(rotation))

        magnitude = abs(rotation[axis])
        others = [abs(rotation[i]) for i in range(3) if i != axis]

        if not check("%s: moved far enough" % AXES[axis],
                     magnitude >= MIN_ROTATION_DEG,
                     "%.0f deg, need %.0f" % (magnitude, MIN_ROTATION_DEG)):
            failures += 1
            return

        if not check("%s: the expected axis dominated" % AXES[axis],
                     magnitude > DOMINANCE_RATIO * max(others),
                     "%.0f vs %.0f" % (magnitude, max(others))):
            print("      -> another axis moved as much as the intended one. Either")
            print("         the movement was sloppy, or the board is rotated and")
            print("         its axes do not line up with the airframe.")
            failures += 1

        correct = (rotation[axis] > 0) if positive else (rotation[axis] < 0)
        if not check("%s: sign is correct" % AXES[axis], correct,
                     "%+.0f deg" % rotation[axis]):
            print("      -> inverted. The board is not mounted the way")
            print("         AHRS_ORIENTATION describes.")
            failures += 1

        if attitude_index is not None and len(att2):
            change = att2[-1][attitude_index] - level[attitude_index]
            name = ["roll", "pitch"][attitude_index]
            agrees = (change > 0) == (rotation[axis] > 0)
            if not check("%s: attitude estimate agrees with the gyro" % AXES[axis],
                         agrees or abs(change) < 5,
                         "%s changed %+.0f deg" % (name, change)):
                print("      -> the accelerometer and gyro disagree about which way")
                print("         it turned. Redo the accelerometer calibration.")
                failures += 1

    motion("2. roll", "Roll the aircraft RIGHT about 30 degrees and hold it there.",
           axis=0, positive=True, attitude_index=0)
    motion("3. pitch", "Return to level, then tilt the NOSE DOWN about 30 degrees.",
           axis=1, positive=False, attitude_index=1)
    motion("4. yaw", "Return to level, then YAW RIGHT (nose to the right) about 45 degrees.",
           axis=2, positive=True, attitude_index=None)

    print()
    if failures:
        print("  %d check(s) failed. Do not fly until each is understood." % failures)
    else:
        print("  IMU healthy, axes correctly oriented, accel and gyro agree.")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
