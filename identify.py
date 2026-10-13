#!/usr/bin/env python3
"""Report what is actually on the board, and what bring-up still needs.

    ./identify.py

Read-only: connects, catches the boot banner, reads a handful of parameters,
and prints a checklist. Writes nothing. Run this first -- it answers the
questions that would otherwise have to be guessed at, including which frame
firmware is already loaded.
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
import rotations

FRAME_NAMES = {0: "PLUS", 1: "X", 2: "V", 3: "H", 4: "NEWY6", 5: "Y6"}

# ArduCopter 3.2.1 readSwitch(): the PWM on channel 5 picks one of six slots.
# Boundaries straight from the firmware.
SWITCH_BOUNDS = [(1231, 1), (1361, 2), (1491, 3), (1621, 4), (1750, 5)]


def switch_position(pwm):
    """Which FLTMODE slot (1-6) this channel-5 PWM selects."""
    for limit, slot in SWITCH_BOUNDS:
        if pwm < limit:
            return slot
    return 6


MODE_NAMES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT",
    13: "SPORT", 14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD",
}

# Modes that can recover the aircraft on their own. At least one of these has
# to be reachable from the switch, or a problem in the air has no answer.
RECOVERY_MODES = {6, 9}

# (name, description) -- read individually rather than fetching the whole list,
# which takes minutes over a 115200 link on an ATmega2560.
PROBE = [
    ("FORMAT_VERSION", "EEPROM format marker"),
    ("SYSID_SW_MREV", "parameter revision"),
    ("FRAME", "frame configuration"),
    ("COMPASS_EXTERNAL", "external compass selected"),
    ("COMPASS_ORIENT", "compass rotation"),
    ("COMPASS_USE", "compass used for heading"),
    ("COMPASS_LEARN", "in-flight offset learning"),
    ("COMPASS_OFS_X", "hard-iron X"),
    ("COMPASS_OFS_Y", "hard-iron Y"),
    ("COMPASS_OFS_Z", "hard-iron Z"),
    ("INS_PRODUCT_ID", "IMU type"),
    ("INS_ACCSCAL_X", "accel scale X"),
    ("INS_ACCSCAL_Y", "accel scale Y"),
    ("INS_ACCSCAL_Z", "accel scale Z"),
    ("INS_ACCOFFS_X", "accel offset X"),
    ("INS_ACCOFFS_Y", "accel offset Y"),
    ("INS_ACCOFFS_Z", "accel offset Z"),
    ("ARMING_CHECK", "pre-arm checks"),
    ("BATT_MONITOR", "battery monitoring"),
    ("FS_THR_ENABLE", "throttle failsafe"),
    ("FS_THR_VALUE", "throttle failsafe threshold"),
    ("FS_GCS_ENABLE", "ground station failsafe"),
    ("FLTMODE1", "flight mode switch pos 1"),
    ("FLTMODE2", "flight mode switch pos 2"),
    ("FLTMODE3", "flight mode switch pos 3"),
    ("FLTMODE4", "flight mode switch pos 4"),
    ("FLTMODE5", "flight mode switch pos 5"),
    ("FLTMODE6", "flight mode switch pos 6"),
    ("RC5_MIN", "mode channel low"),
    ("RC5_MAX", "mode channel high"),
    ("RC5_TRIM", "mode channel resting value"),
    ("RC3_MIN", "throttle low endpoint"),
    ("RC3_MAX", "throttle high endpoint"),
    ("RC1_MIN", "roll low endpoint"),
    ("RC1_MAX", "roll high endpoint"),
]


def banner(master, seconds=8):
    """ArduCopter prints its version as STATUSTEXT shortly after boot, and
    opening the port has just reset the board, so the banner is still coming."""
    lines = []
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = master.recv_match(type="STATUSTEXT", blocking=True, timeout=1.0)
        if msg is None:
            continue
        text = msg.text.strip("\x00").strip()
        if text and text not in lines:
            lines.append(text)
    return lines


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=apmlink.DEFAULT_BAUD)
    parser.add_argument("--reboot", action="store_true",
                        help="reboot the board first, to capture the version banner")
    args = parser.parse_args()

    try:
        master = apmlink.connect(args.port, args.baud)
    except apmlink.LinkError as exc:
        apmlink.die(exc)

    if args.reboot:
        # The version banner is emitted once, within the first second of boot,
        # and the heartbeat wait in connect() has already consumed it. Rebooting
        # deliberately is the only reliable way to see it on 3.2.1, which has no
        # AUTOPILOT_VERSION request.
        print("  rebooting to catch the version banner ...")
        apmlink.reboot(master, wait=False)
        time.sleep(1.5)

    print()
    print("  boot messages:")
    # Two sources, because the board uses both. STATUSTEXT is MAVLink and only
    # appears once the link is up; the plain-text banner is written to the same
    # wire before MAVLink starts and is captured during connect(). The
    # ESC-calibration notice only ever appears in the second one.
    lines = list(apmlink.boot_banner(master).splitlines())
    lines = [l.strip() for l in lines if l.strip()]
    for extra in banner(master, 12 if args.reboot else 8):
        if extra not in lines:
            lines.append(extra)
    if lines:
        for line in lines:
            print("    %s" % line)
    else:
        print("    (none caught -- board may have booted before the port opened)")

    if apmlink.esc_cal_active(master):
        print()
        print("  *** BOARD HALTED IN ESC CALIBRATION ***")
        print("      It read the throttle channel as HIGH at startup and stopped")
        print("      before its main loop, so it streams no GPS, RC, IMU or")
        print("      battery telemetry -- those all read as absent below, which")
        print("      is a consequence of this and not five separate faults.")
        print("      Clear it: transmitter ON with throttle fully DOWN (or")
        print("      reconnect the receiver), then power-cycle the board.")

    # GPS health. Detection is not reception: the boot banner says "ublox" as
    # soon as the module answers, which tells you nothing about whether the
    # antenna can see sky. Satellite count and HDOP do.
    print()
    print("  gps:")
    apmlink.ensure_stream(master, mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2)
    fix = None
    deadline = time.time() + 8
    while time.time() < deadline:
        msg = master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=1.0)
        if msg is not None:
            fix = msg
            break
    if fix is None:
        print("    no GPS_RAW_INT received")
    else:
        fix_names = {0: "no GPS", 1: "no fix", 2: "2D fix", 3: "3D fix",
                     4: "DGPS", 5: "RTK float", 6: "RTK fixed"}
        hdop = fix.eph / 100.0 if fix.eph not in (0, 65535) else float("nan")
        print("    fix type    %d (%s)" % (fix.fix_type, fix_names.get(fix.fix_type, "?")))
        print("    satellites  %d" % fix.satellites_visible)
        print("    HDOP        %.2f" % hdop)
        if fix.fix_type < 3:
            gps_problem = ("no 3D fix. Indoors that is normal. Outdoors with a "
                           "clear view it means the antenna cannot see sky -- "
                           "check it faces UP and is not under a plate.")
        elif fix.satellites_visible < 8:
            gps_problem = ("only %d satellites. Below 8 is marginal; ArduCopter "
                           "wants 8+ before Loiter or RTL are trustworthy."
                           % fix.satellites_visible)
        elif hdop == hdop and hdop > 2.0:
            gps_problem = "HDOP %.2f is poor -- 1.5 or below is healthy." % hdop
        else:
            gps_problem = None

    print()
    print("  parameters:")
    values = {}
    width = max(len(name) for name, _ in PROBE)
    for name, description in PROBE:
        try:
            value = apmlink.get_param(master, name, timeout=3.0, retries=2)
            values[name] = value
            extra = ""
            if name == "FRAME":
                extra = "  (%s)" % FRAME_NAMES.get(int(value), "?")
            elif name == "COMPASS_ORIENT":
                extra = "  (%s)" % rotations.name(value)
            elif name.startswith("FLTMODE"):
                extra = "  (%s)" % MODE_NAMES.get(int(value), "?")
            print("    %-*s %-10g%s" % (width, name, value, extra))
        except apmlink.LinkError:
            print("    %-*s --  not present" % (width, name))

    print()
    print("  what still needs doing:")
    todo = []

    try:
        if gps_problem:
            todo.append("GPS: %s" % gps_problem)
    except NameError:
        pass

    # Accelerometer calibration is the one thing here with no direct "was this
    # done" flag. Uncalibrated firmware leaves the scale factors at exactly 1.0
    # and the offsets at exactly 0.0; a real six-position calibration never
    # produces those, because no physical IMU is perfect. So exact defaults
    # mean it was never run.
    scal = [values.get("INS_ACCSCAL_%s" % a) for a in "XYZ"]
    accoffs = [values.get("INS_ACCOFFS_%s" % a) for a in "XYZ"]
    if all(v is not None for v in scal + accoffs):
        pristine = (all(abs(v - 1.0) < 1e-6 for v in scal)
                    and all(abs(v) < 1e-6 for v in accoffs))
        if pristine:
            todo.append("accelerometers have NEVER been calibrated -- scale is "
                        "exactly 1.0 and offsets exactly 0.0 on all three axes "
                        "(./calibrate_accel.py). ARMING_CHECK will refuse to arm.")

    ofs = [values.get("COMPASS_OFS_%s" % a, 0.0) for a in "XYZ"]
    if all(abs(v) < 1e-6 for v in ofs):
        todo.append("compass offsets are all zero -- never calibrated "
                    "(./calibrate_compass.py --write)")
    elif values.get("COMPASS_EXTERNAL", 0) < 1:
        todo.append("the stored compass offsets (|v| = %.0f) were measured for "
                    "the INTERNAL compass -- they are meaningless once you "
                    "switch to the external one, so recalibrate after the "
                    "jumper cut" % (sum(v * v for v in ofs) ** 0.5))
    elif (sum(v * v for v in ofs)) ** 0.5 > 600:
        todo.append("compass offsets exceed the 600 pre-arm limit -- recalibrate, "
                    "and move the compass away from power wiring")

    if values.get("COMPASS_EXTERNAL", 0) < 1:
        todo.append("COMPASS_EXTERNAL is off, but you have a GPS/compass module "
                    "(cut the onboard-compass jumper, then apply the "
                    "compass-external profile)")

    if values.get("COMPASS_LEARN", 0) >= 1:
        todo.append("COMPASS_LEARN is on -- it will corrupt a good calibration "
                    "(base profile turns it off)")

    if values.get("COMPASS_ORIENT") == 0 and values.get("COMPASS_EXTERNAL", 0) >= 1:
        todo.append("COMPASS_ORIENT is 0 with an external compass -- almost "
                    "certainly wrong (./solve_compass_orient.py --write)")

    rc3_min, rc3_max = values.get("RC3_MIN"), values.get("RC3_MAX")
    if rc3_min is not None and rc3_max is not None:
        if abs(rc3_min - 1100) < 1 and abs(rc3_max - 1900) < 1:
            todo.append("RC endpoints are still at defaults -- radio never "
                        "calibrated (./calibrate_radio.py --write)")
        fs = values.get("FS_THR_VALUE")
        if values.get("FS_THR_ENABLE", 0) < 1:
            todo.append("throttle failsafe is DISABLED -- lose the transmitter "
                        "and the aircraft keeps its last command instead of "
                        "coming home (failsafe profile)")
        if fs is not None and fs >= rc3_min:
            todo.append("FS_THR_VALUE (%g) is inside the live throttle range "
                        "(RC3_MIN %g) -- this WILL trigger a failsafe in flight"
                        % (fs, rc3_min))

    # A mode channel with no travel does not merely mean "no mode switching" --
    # it means the aircraft is permanently locked to whichever slot its resting
    # PWM happens to select. Applying a mode profile without fixing that just
    # changes WHICH mode you are stuck in, and it may not be the gentle one.
    rc5_min = values.get("RC5_MIN")
    rc5_max = values.get("RC5_MAX")
    rc5_trim = values.get("RC5_TRIM")
    stuck_slot = None
    if None not in (rc5_min, rc5_max, rc5_trim) and (rc5_max - rc5_min) < 50:
        stuck_slot = switch_position(rc5_trim)
        selected = values.get("FLTMODE%d" % stuck_slot)
        todo.append(
            "mode channel 5 has no travel (sits at %g), so the aircraft is "
            "LOCKED to slot %d = FLTMODE%d, currently %s. Assign a switch to "
            "channel 5 on the transmitter and re-run calibrate_radio.py BEFORE "
            "applying a flight-mode profile -- otherwise you simply become stuck "
            "in whatever that profile puts in slot %d."
            % (rc5_trim, stuck_slot, stuck_slot,
               MODE_NAMES.get(int(selected), "?") if selected is not None else "?",
               stuck_slot))

    modes = [values.get("FLTMODE%d" % i) for i in range(1, 7)]
    if all(m is not None for m in modes):
        if len(set(modes)) == 1:
            todo.append("every flight mode switch position is %s -- the switch "
                        "does nothing (flight-modes profile, then match it to "
                        "your transmitter)"
                        % MODE_NAMES.get(int(modes[0]), int(modes[0])))
        elif not RECOVERY_MODES & {int(m) for m in modes}:
            todo.append("no switch position selects RTL or LAND -- there is no "
                        "way to hand control back to the aircraft in the air")

    if values.get("FS_GCS_ENABLE", 0) >= 1:
        todo.append("GCS failsafe is on. That is right only if you fly with a "
                    "telemetry radio; with USB-only it can fire off a failsafe "
                    "the moment you unplug (failsafe profile turns it off)")

    if values.get("BATT_MONITOR", 0) < 1:
        todo.append("battery monitoring is off (power-module profile)")

    if values.get("ARMING_CHECK", 0) < 1:
        todo.append("pre-arm checks are disabled -- turn them back on")

    if todo:
        for item in todo:
            print("    - %s" % item)
    else:
        print("    nothing obvious. Verify accel calibration by hand.")

    print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
