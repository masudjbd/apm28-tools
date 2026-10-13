#!/usr/bin/env python3
"""Drive ArduCopter 3.2.1's six-position accelerometer calibration.

    ./calibrate_accel.py

Unlike the compass and radio tools here, this one cannot be reimplemented on
the host: 3.2.1 computes and stores the accel scaling on the board and only
exposes it through this interactive handshake. So this is a relay -- it starts
the routine, prints the board's prompts, and acknowledges each position.

The handshake is the one MAVProxy's accelcal module uses: the vehicle emits a
STATUSTEXT asking for a position, the GCS replies with a COMMAND_ACK, and the
vehicle samples. It is specific to this firmware generation. If the prompts
stall, fall back to APM Planner 2's Calibrate Accel button rather than
retrying -- a half-finished accel calibration is worse than none.

You need a genuinely flat reference surface. A table that is visibly off level
puts a standing error into every attitude estimate the aircraft makes.
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

POSITIONS = ["LEVEL", "LEFT side", "RIGHT side", "NOSE DOWN", "NOSE UP", "BACK (inverted)"]
STALL_TIMEOUT = 45.0


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

    print()
    print("  Six positions, in this order:")
    for i, position in enumerate(POSITIONS, 1):
        print("    %d. %s" % (i, position))
    print()
    print("  Hold each one still until the next prompt. PROPELLERS OFF.")
    if not apmlink.confirm("  start?"):
        return 1

    apmlink.drain(master, 0.5)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
        0,
        0, 0, 0, 0, 1, 0, 0,   # param5 = 1 -> accelerometer calibration
    )

    step = 0
    last_msg = time.time()
    while True:
        if time.time() - last_msg > STALL_TIMEOUT:
            print()
            print("  No prompt for %ds -- the board is not running the handshake."
                  % int(STALL_TIMEOUT))
            print("  Use APM Planner 2's Calibrate Accel instead. Nothing was written.")
            return 1

        msg = master.recv_match(type=["STATUSTEXT", "COMMAND_ACK"], blocking=True, timeout=1.0)
        if msg is None:
            continue
        last_msg = time.time()

        if msg.get_type() == "COMMAND_ACK":
            continue

        text = msg.text.strip("\x00").strip()
        if not text:
            continue
        print("  board: %s" % text)

        lowered = text.lower()
        # Trigger and acknowledgement both copied from MAVProxy's
        # mavproxy_calibration module, which is the reference implementation
        # for this handshake. Two details matter and are easy to get wrong:
        # the prompt test is exactly "Place " at the start of the STATUSTEXT,
        # and the ack carries the POSITION INDEX (0-5) in the command field --
        # not MAV_CMD_PREFLIGHT_CALIBRATION, which is what it looks like it
        # should be. Sending the command id makes the board wait forever.
        if text.startswith("Place ") or "place" in lowered:
            label = POSITIONS[step] if step < len(POSITIONS) else "next position"
            input("  --> put the aircraft %s, hold still, press Enter " % label)
            master.mav.command_ack_send(step, 1)
            step += 1
        elif "success" in lowered:
            print()
            print("  calibration accepted and saved.")
            return 0
        elif "fail" in lowered:
            print()
            print("  calibration FAILED -- nothing was saved. Usually this means a")
            print("  position was held at a visible angle, or the board moved while")
            print("  sampling. Retry on a flatter surface.")
            return 1


if __name__ == "__main__":
    raise SystemExit(main())
