#!/usr/bin/env python3
"""Does the aircraft lean the way the stick asks? Measured, on the bench.

    ./check_stick_sense.py

Transmitter on, PROPELLERS OFF, battery not required. Read-only: nothing is
armed, no motor is commanded. Takes about 50 seconds and tells you which way
each stick moves its channel and which way the firmware will lean the aircraft
for it.

Why this is not obvious
-----------------------
A reversed pitch channel does not stop the aircraft flying. Self-levelling comes
from the IMU and keeps working perfectly, so it hovers, holds attitude and looks
healthy -- it just goes backward when you ask for forward. Nothing in
preflight.py can see it, because a reversed channel and a correct one produce
identical PWM ranges. Only the SIGN relative to trim tells them apart, and that
needs someone to hold a known stick position while something records it.

The chain this checks, one link at a time
-----------------------------------------
  stick position -> PWM above or below RCn_TRIM
                 -> control_in, sign flipped by RCn_REV
                 -> commanded lean angle
                 -> which way the aircraft actually goes

ArduPilot's convention, which fixes the last two links: POSITIVE pitch is NOSE
UP, and a nose-up quad flies BACKWARD. Positive roll is right wing down, which
flies RIGHT. So a forward stick must end up producing NEGATIVE pitch. If it
produces positive, the channel is reversed and the fix is RC2_REV -1.

This does NOT diagnose the other cause of "my controls are backwards", which is
far more common and costs nothing to rule out first: flying with the aircraft's
nose pointed AT you. Every control is mirrored then, and no parameter is wrong.
Fly with the nose pointed AWAY before believing any of this.
"""

import argparse
import os
import sys
import time

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

from pymavlink import mavutil  # noqa: E402
import apmlink  # noqa: E402

HOLD = 6.0
SETTLE = 2.0


def countdown(label, seconds):
    sys.stdout.write("\n  >>> %s\n      " % label)
    sys.stdout.flush()
    for i in range(int(seconds), 0, -1):
        sys.stdout.write("%d " % i)
        sys.stdout.flush()
        time.sleep(1.0)
    sys.stdout.write("\n")
    sys.stdout.flush()


def sample(master, seconds):
    """Mean of ch1 and ch2 over the window, ignoring the first moment."""
    end = time.time() + seconds
    c1, c2 = [], []
    while time.time() < end:
        m = master.recv_match(type="RC_CHANNELS_RAW", blocking=True, timeout=1.0)
        if m is None:
            continue
        c1.append(m.chan1_raw)
        c2.append(m.chan2_raw)
    if not c1:
        return None, None
    return sum(c1) / float(len(c1)), sum(c2) / float(len(c2))


def phase(master, label):
    countdown(label, SETTLE)
    print("      HOLD IT ...")
    a, b = sample(master, HOLD)
    if a is None:
        apmlink.die("no RC_CHANNELS_RAW during the hold. Is the transmitter on?")
    print("      ch1 %.0f   ch2 %.0f" % (a, b))
    return a, b


def verdict(name, chan, trim, rev, pushed, released, positive_means, stick_word):
    """Report whether `pushed` ends up commanding the direction the pilot asked."""
    delta = pushed - released
    print()
    print("  %s (%s)" % (name, chan))
    print("    trim %.0f, %s %.0f  ->  %+.0f us from centre"
          % (trim, stick_word, pushed, delta))
    if abs(delta) < 80:
        print("    NOT ENOUGH TRAVEL to judge -- did you hold the right stick?")
        return None
    print("    RC%s_REV %+g" % (chan[-1], rev))
    control_in = delta * (1.0 if rev >= 0 else -1.0)
    print("    control_in is %s" % ("POSITIVE" if control_in > 0 else "NEGATIVE"))
    print("    which ArduPilot flies as: %s"
          % (positive_means[0] if control_in > 0 else positive_means[1]))
    return control_in


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=apmlink.DEFAULT_BAUD)
    args = ap.parse_args()

    try:
        master = apmlink.connect(args.port, args.baud)
    except apmlink.LinkError as exc:
        apmlink.die(exc)

    apmlink.ensure_stream(master, mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10)
    if not apmlink.wait_for_message(master, "RC_CHANNELS_RAW", timeout=8.0):
        apmlink.die(apmlink.no_stream_message("RC_CHANNELS_RAW", "SR0_RC_CHAN"))

    p = {}
    for n in ("RC1_TRIM", "RC2_TRIM", "RC1_REV", "RC2_REV", "AHRS_ORIENTATION"):
        try:
            p[n] = apmlink.get_param(master, n, timeout=3.0, retries=2)
        except apmlink.LinkError:
            p[n] = None

    print()
    print("  PROPELLERS OFF. Nothing is armed and no motor is commanded,")
    print("  but hold the aircraft still and keep hands clear anyway.")
    print()
    print("  Stand BEHIND the aircraft, nose pointing AWAY from you --")
    print("  the same way you should be flying it.")

    countdown("Centre both sticks and let go", 5)
    c1_mid, c2_mid = sample(master, 4.0)
    print("      centred: ch1 %.0f  ch2 %.0f" % (c1_mid, c2_mid))

    _, c2_fwd = phase(master, "Push the PITCH stick FULLY FORWARD (away from you)")
    _, c2_back = phase(master, "Pull the PITCH stick FULLY BACK (toward you)")
    c1_right, _ = phase(master, "Push the ROLL stick FULLY RIGHT")
    c1_left, _ = phase(master, "Push the ROLL stick FULLY LEFT")

    t1 = p["RC1_TRIM"] if p["RC1_TRIM"] else c1_mid
    t2 = p["RC2_TRIM"] if p["RC2_TRIM"] else c2_mid
    r1 = p["RC1_REV"] if p["RC1_REV"] is not None else 1.0
    r2 = p["RC2_REV"] if p["RC2_REV"] is not None else 1.0

    print()
    print("================================================================")
    pitch_fwd = verdict("PITCH, stick forward", "RC2", t2, r2, c2_fwd, c2_mid,
                        ("NOSE UP -> flies BACKWARD", "NOSE DOWN -> flies FORWARD"),
                        "forward")
    roll_right = verdict("ROLL, stick right", "RC1", t1, r1, c1_right, c1_mid,
                         ("roll RIGHT -> flies RIGHT", "roll LEFT -> flies LEFT"),
                         "right")

    print()
    print("--- verdict ----------------------------------------------------")
    bad = []
    if pitch_fwd is not None and pitch_fwd > 0:
        bad.append(("pitch", "RC2_REV", r2))
    if roll_right is not None and roll_right < 0:
        bad.append(("roll", "RC1_REV", r1))

    if not bad:
        if pitch_fwd is None or roll_right is None:
            print("  Incomplete -- re-run and hold each stick to its stop.")
        else:
            print("  Both axes are correct. Forward stick flies forward, right")
            print("  stick flies right.")
            print()
            print("  If it still feels backwards in the air, the aircraft's NOSE")
            print("  WAS POINTED AT YOU. With the nose toward you every control")
            print("  is mirrored, and no parameter can fix that -- it is the")
            print("  single most common thing beginners hit. Put a bright marker")
            print("  on one arm, keep the nose pointed away, and fly it in a")
            print("  straight line out and back before trying to hover.")
        return 0

    if len(bad) == 2:
        print("  BOTH axes are reversed. That is not two channel faults -- it is")
        print("  the board mounted 180 degrees round. AHRS_ORIENTATION is %s."
              % p["AHRS_ORIENTATION"])
        print("  Fix the ORIENTATION, not the channels. Reversing both channels")
        print("  would leave the gyro and accel axes still pointing backwards.")
        return 1

    axis, param, cur = bad[0]
    print("  %s is REVERSED." % axis.upper())
    print("  The stick asks for one direction and the firmware commands the")
    print("  other. Self-levelling is unaffected, which is why it still hovers.")
    print()
    print("  Fix:  set %s to %+g" % (param, -cur if cur else -1))
    print()
    print("  Only one axis is wrong, so this is the CHANNEL, not the board")
    print("  orientation -- a board mounted backwards reverses both.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
