#!/usr/bin/env python3
"""Spin each motor individually, to verify wiring and rotation direction.

    ./motor_test.py                 # 8% throttle, 2s per motor, one at a time
    ./motor_test.py --throttle 12 --seconds 3
    ./motor_test.py --motor 3       # just one

  ##############################################################
  ##  PROPELLERS OFF. NOT "pointed away". OFF THE MOTORS.     ##
  ##############################################################

This commands real motor output with a live battery. There is no way to test a
motor without turning it, so the only protection is that nothing is attached to
the shaft. Take the props off, put them out of reach, and check the shafts by
hand before you start.

Quad X motor map in ArduCopter (looking down, FORWARD arrow pointing away):

        FRONT
    3 (CW)   1 (CCW)
        \\   /
         \\ /
         / \\
        /   \\
    2 (CCW)  4 (CW)
        REAR

  motor 1 -> APM output 1 -> front-RIGHT  -> spins COUNTER-clockwise
  motor 2 -> APM output 2 -> rear-LEFT    -> spins COUNTER-clockwise
  motor 3 -> APM output 3 -> front-LEFT   -> spins CLOCKWISE
  motor 4 -> APM output 4 -> rear-RIGHT   -> spins CLOCKWISE

Diagonal pairs share a direction. If a motor spins the wrong way, swap any two
of the three fat wires between that ESC and that motor -- do not try to fix it
in software.

The test does NOT run in motor-number order. ArduCopter walks clockwise around
the airframe, so the sequence is:

  test 1 -> motor 1 (output 1)  front-right   CCW
  test 2 -> motor 4 (output 4)  rear-right    CW
  test 3 -> motor 2 (output 2)  rear-left     CCW
  test 4 -> motor 3 (output 3)  front-left    CW

--motor N selects the TEST NUMBER, matching what the firmware expects.
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

# MAV_CMD_DO_MOTOR_TEST param1 is the TEST SEQUENCE number, not the motor
# number. AP_MotorsMatrix::_output_test_seq compares it against _test_order[i],
# and the quad-X table in AP_MotorsMatrix.cpp is:
#
#     {   45, YAW_FACTOR_CCW,  1 },   motor 1  front-right  test order 1
#     { -135, YAW_FACTOR_CCW,  3 },   motor 2  rear-left    test order 3
#     {  -45, YAW_FACTOR_CW,   4 },   motor 3  front-left   test order 4
#     {  135, YAW_FACTOR_CW,   2 },   motor 4  rear-right   test order 2
#
# (angles are degrees clockwise from forward). So the test walks CLOCKWISE
# around the airframe -- front-right, rear-right, rear-left, front-left --
# and sequence number does NOT equal motor number for 2, 3 and 4.
#
# Getting this wrong makes a correctly-wired aircraft look miswired, which is
# exactly the sort of "fix" that produces a real fault.
#
#   sequence -> (motor number, position, direction, APM output)
TEST_SEQUENCE = {
    1: (1, "front-RIGHT", "counter-clockwise", 1),
    2: (4, "rear-RIGHT", "clockwise", 4),
    3: (2, "rear-LEFT", "counter-clockwise", 2),
    4: (3, "front-LEFT", "clockwise", 3),
}

# kept for the summary text: motor number -> (position, direction)
MOTOR_LAYOUT = {
    1: ("front-RIGHT", "counter-clockwise"),
    2: ("rear-LEFT", "counter-clockwise"),
    3: ("front-LEFT", "clockwise"),
    4: ("rear-RIGHT", "clockwise"),
}

ACK_RESULTS = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
}


# ArduCopter 3.2.1 refuses a motor test by sending a STATUSTEXT that names the
# reason, then returning MAV_RESULT_FAILED. The ack alone tells you nothing --
# every refusal looks identical. These are the refusals motor_test.pde can emit.
REFUSAL_HINTS = {
    "rc not calibrated": (
        "Channels 1-4 need RC*_MIN <= 1300 and RC*_MAX >= 1700.\n"
        "       Run ./calibrate_radio.py --write."),
    "vehicle not landed": (
        "The firmware thinks it is airborne, so it will not spin a motor.\n"
        "       This is usually a stale landing-detector state: power-cycle the\n"
        "       board (unplug USB and battery, plug back in) and retry."),
    "board initialising": (
        "The board is still booting. Wait a few seconds and retry."),
    "safety switch": (
        "A safety switch is engaged. APM 2.8 has no safety switch, so if you\n"
        "       see this, BRD_SAFETYENABLE is set -- clear it."),
}


def collect_statustext(master, seconds):
    """Drain STATUSTEXT for a window. The refusal reason arrives this way."""
    lines = []
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = master.recv_match(type="STATUSTEXT", blocking=True, timeout=0.3)
        if msg is None:
            continue
        text = msg.text.strip("\x00").strip()
        if text and text not in lines:
            lines.append(text)
    return lines


def run_motor(master, motor, throttle_pct, seconds):
    apmlink.drain(master, 0.3)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,
        float(motor),                                        # motor instance
        float(mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT),  # throttle type
        float(throttle_pct),                                 # throttle value
        float(seconds),                                      # timeout, seconds
        0.0,                                                 # motor count (0 = just this one)
        0.0,                                                 # test order
        0.0,
    )
    # Listen for both. The ack says whether it was refused; the STATUSTEXT says
    # why, and only one of those is actually useful.
    result, texts = None, []
    deadline = time.time() + 3.0
    while time.time() < deadline:
        msg = master.recv_match(type=["COMMAND_ACK", "STATUSTEXT"],
                                blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            text = msg.text.strip("\x00").strip()
            if text and text not in texts:
                texts.append(text)
        elif msg.command == mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST:
            result = msg.result
            if result != 0:
                texts.extend(t for t in collect_statustext(master, 1.0) if t not in texts)
            break
    return result, texts


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=apmlink.DEFAULT_BAUD)
    parser.add_argument("--throttle", type=int, default=15,
                        help="percent, default %(default)s. Measured on this "
                             "airframe: 8%% is BELOW these ESCs' start threshold, so a "
                             "motor that fails to turn at 8%% proves nothing.")
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--motor", type=int, help="test only this motor (1-4)")
    parser.add_argument("--motors", type=int, default=4)
    args = parser.parse_args()

    if not 1 <= args.throttle <= 25:
        apmlink.die("--throttle %d refused. Above 25%% a bare motor can move the "
                    "airframe and anything left on a shaft becomes a projectile."
                    % args.throttle)

    try:
        master = apmlink.connect(args.port, args.baud)
    except apmlink.LinkError as exc:
        apmlink.die(exc)

    # Opening the port reboots the board, and ArduCopter's landing detector needs
    # time to settle afterwards. Command a motor too early and it refuses with
    # "Motor Test: vehicle not landed" -- which reads like a fault and is not one.
    print()
    print("  waiting 18s for the board to settle after the reboot ...", end="", flush=True)
    settle_end = time.time() + 18
    while time.time() < settle_end:
        master.recv_match(blocking=False)
        time.sleep(0.2)
    print(" ready")

    print()
    print("  " + "#" * 62)
    print("  ##  PROPELLERS OFF. Check every shaft by hand, right now.  ##")
    print("  " + "#" * 62)
    print()
    print("  The battery must be connected -- USB alone does not power the ESCs.")
    print("  Throttle %d%%, %.1fs per motor." % (args.throttle, args.seconds))
    print()
    if not apmlink.confirm("  Are all four propellers physically off the motors?"):
        print("  stopping. Nothing was commanded.")
        return 1
    if not apmlink.confirm("  Battery connected and the aircraft on a stable surface?"):
        print("  stopping. Nothing was commanded.")
        return 1

    motors = [args.motor] if args.motor else list(range(1, args.motors + 1))
    results = {}

    for seq in motors:
        motor, position, direction, output = TEST_SEQUENCE.get(
            seq, (seq, "?", "?", seq))
        print()
        print("  --- test %d = motor %d (APM output %d): expect the %s arm, "
              "spinning %s ---" % (seq, motor, output, position, direction))
        input("      press Enter to spin it ")
        result, texts = run_motor(master, seq, args.throttle, args.seconds)
        for text in texts:
            print("      board: %s" % text)

        if result is None:
            print("      no COMMAND_ACK -- the firmware may not support motor test")
            results[seq] = ("no ack", None)
            continue
        name = ACK_RESULTS.get(result, "result %d" % result)
        if result != 0:
            print("      command %s" % name)
            if result == 3:
                print("      MAV_CMD_DO_MOTOR_TEST is unsupported on this firmware.")
                print("      Fall back to the transmitter check described in the README.")
            matched = False
            for text in texts:
                for key, hint in REFUSAL_HINTS.items():
                    if key in text.lower():
                        print("      -> %s" % hint)
                        matched = True
            if not matched and result == 4:
                print("      The board refused but sent no reason. Most often that is")
                print("      the landing-detector state: power-cycle the board fully")
                print("      (USB and battery out), then retry before anything else.")
            results[seq] = (name, None)
            continue

        print("      command ACCEPTED")
        time.sleep(args.seconds + 0.5)
        answer = input("      Which arm turned? [%s / other / none] " % position).strip().lower()
        spun_right = answer in ("", "y", "yes", position.lower(), "correct", "ok")
        direction_ok = None
        if answer not in ("none", "n"):
            d = input("      Direction, looking down from above? [cw / ccw] ").strip().lower()
            direction_ok = (d == "cw" and direction.startswith("clock")) or \
                           (d == "ccw" and direction.startswith("counter"))
        results[seq] = ("spun" if answer != "none" else "did not spin",
                        (spun_right, direction_ok))

    print()
    print("  === summary ===")
    problems = []
    for seq in motors:
        status, detail = results.get(seq, ("not tested", None))
        motor, position, direction, output = TEST_SEQUENCE.get(
            seq, (seq, "?", "?", seq))
        print("  test %d = motor %d (%s, %s): %s"
              % (seq, motor, position, direction, status))
        if detail is None:
            continue
        spun_right, direction_ok = detail
        if status == "did not spin":
            problems.append("motor %d never turned -- check the ESC signal lead on "
                            "APM output %d, and that the ESC is powered"
                            % (motor, output))
        elif not spun_right:
            problems.append("motor %d turned the wrong ARM -- your ESC signal leads "
                            "are in the wrong output order" % motor)
        elif direction_ok is False:
            problems.append("motor %d turned the wrong WAY -- swap any two of the "
                            "three motor wires at that ESC" % motor)

    print()
    if problems:
        print("  problems:")
        for p in problems:
            print("    - %s" % p)
    else:
        print("  all tested motors correct.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
