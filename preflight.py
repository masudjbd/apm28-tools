#!/usr/bin/env python3
"""Full pre-flight readout for an APM 2.8 / ArduCopter 3.2.1 quad.

    ./preflight.py            # battery connected, propellers OFF
    ./preflight.py --quick    # skip the parts that need you to move things

Read-only. Writes nothing, arms nothing, commands no motor.

Everything is separated into BLOCKERS -- reasons not to fly today -- and
WARNINGS, which are things to know rather than things to stop for. A check that
cannot be performed is reported as unknown rather than quietly passing, because
a preflight that says "all good" when it merely failed to look is worse than no
preflight at all.
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
import rotations

MODE_NAMES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT",
    13: "SPORT", 14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD",
}
RECOVERY_MODES = {6, 9}
SWITCH_BOUNDS = [(1231, 1), (1361, 2), (1491, 3), (1621, 4), (1750, 5)]

blockers, warnings, unknowns = [], [], []

# Set when the board halted in ESC calibration. Every telemetry-derived check
# is meaningless in that state, so they must not report independent faults --
# one halted board would otherwise masquerade as five separate ones.
board_halted = False


def block(msg):
    blockers.append(msg)


def unknown_stream(msg):
    """An unknown that is only worth reporting if the board is actually running.

    When the board is halted the single halt blocker already explains why no
    stream arrived; repeating it per-subsystem buries the one thing that
    matters under a list of consequences.
    """
    if not board_halted:
        unknowns.append(msg)


def warn(msg):
    warnings.append(msg)


def unknown(msg):
    unknowns.append(msg)


def head(title):
    print()
    print("--- %s " % title + "-" * max(0, 58 - len(title)))


def row(label, value, note=""):
    print("  %-22s %-22s %s" % (label, value, note))


def slot_for(pwm):
    for limit, slot in SWITCH_BOUNDS:
        if pwm < limit:
            return slot
    return 6


def params(master, names):
    out = {}
    for name in names:
        try:
            out[name] = apmlink.get_param(master, name, timeout=3.0, retries=2)
        except apmlink.LinkError:
            out[name] = None
    return out


def check_gps(master):
    head("GPS")
    apmlink.ensure_stream(master, mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2)
    fix = None
    deadline = time.time() + 8
    while time.time() < deadline:
        msg = master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=1.0)
        if msg is not None:
            fix = msg
            break
    if fix is None:
        row("status", "no GPS_RAW_INT")
        unknown_stream("GPS never reported -- could not assess it")
        return
    names = {0: "no GPS", 1: "no fix", 2: "2D fix", 3: "3D fix", 4: "DGPS"}
    hdop = fix.eph / 100.0 if fix.eph not in (0, 65535) else float("nan")
    row("fix", "%d (%s)" % (fix.fix_type, names.get(fix.fix_type, "?")))
    row("satellites", fix.satellites_visible)
    row("HDOP", "%.2f" % hdop if hdop == hdop else "n/a")

    if fix.fix_type < 3:
        block("no 3D GPS fix. Outdoors with clear sky this means the antenna "
              "cannot see enough of it -- check it faces up and is clear of the "
              "airframe. Indoors this is expected and not a fault.")
    elif fix.satellites_visible < 8:
        warn("only %d satellites. ArduCopter wants 8+ before Loiter or RTL are "
             "trustworthy." % fix.satellites_visible)
    elif hdop == hdop and hdop > 2.0:
        warn("HDOP %.2f is poor; 1.5 or below is healthy." % hdop)


def check_battery(master, values):
    head("battery")
    monitor = values.get("BATT_MONITOR")
    row("BATT_MONITOR", monitor, "4 = voltage and current" if monitor == 4 else "")
    if not monitor:
        warn("battery monitoring is off -- no low-voltage protection at all")
        return

    apmlink.ensure_stream(master, mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2)
    status = None
    deadline = time.time() + 8
    while time.time() < deadline:
        msg = master.recv_match(type="SYS_STATUS", blocking=True, timeout=1.0)
        if msg is not None:
            status = msg
            break
    if status is None:
        unknown_stream("battery monitoring is enabled but no SYS_STATUS arrived")
        return

    volts = status.voltage_battery / 1000.0
    amps = status.current_battery / 100.0 if status.current_battery != -1 else float("nan")
    row("voltage", "%.2f V" % volts)
    row("current", "%.2f A" % amps if amps == amps else "not reported")

    fs_v = values.get("FS_BATT_VOLTAGE")
    if volts < 1.0:
        block("battery voltage reads %.2f V. Either the pack is not connected or "
              "the power module is not wired to the APM's PM port." % volts)
        return

    cells = int(round(volts / 3.85)) or 1
    per_cell = volts / cells
    row("implied pack", "%dS at %.2f V/cell" % (cells, per_cell))
    if not 3.0 < per_cell < 4.25:
        block("%.2f V per cell is not a plausible lithium pack. BATT_VOLT_MULT "
              "(%s) is almost certainly wrong for your power module -- measure the "
              "pack with a meter and scale it. Until then the battery failsafe is "
              "acting on a fiction." % (per_cell, values.get("BATT_VOLT_MULT")))
    elif per_cell < 3.5:
        warn("%.2f V per cell -- that pack is already low, charge before flying"
             % per_cell)
    if fs_v and volts < fs_v:
        block("voltage %.2f V is already below FS_BATT_VOLTAGE %.2f -- the battery "
              "failsafe will fire immediately." % (volts, fs_v))
    print()
    row("", "", "verify against a multimeter before trusting the failsafe")


AIRFRAME_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "airframe.local.json")


def airframe_choice(key, default=None):
    """Read a deliberate per-airframe decision, if one has been recorded."""
    try:
        import json
        with open(AIRFRAME_FILE) as handle:
            return json.load(handle).get(key, default)
    except (IOError, OSError, ValueError):
        return default


def check_compass(master, values):
    head("compass")
    external = values.get("COMPASS_EXTERNAL")
    orient = values.get("COMPASS_ORIENT")
    ofs = [values.get("COMPASS_OFS_%s" % a) for a in "XYZ"]
    row("COMPASS_EXTERNAL", external)
    row("COMPASS_ORIENT", "%s (%s)" % (orient, rotations.name(orient or 0)))
    row("COMPASS_USE", values.get("COMPASS_USE"))
    row("COMPASS_LEARN", values.get("COMPASS_LEARN"))
    if None not in ofs:
        magnitude = math.sqrt(sum(v * v for v in ofs))
        row("offsets", "X %+.0f Y %+.0f Z %+.0f" % tuple(ofs), "|v| = %.0f" % magnitude)
        if magnitude > 600:
            block("compass offset magnitude %.0f exceeds the 600 pre-arm limit -- "
                  "it will refuse to arm" % magnitude)
        elif all(abs(v) < 1e-6 for v in ofs):
            block("compass offsets are all zero -- never calibrated")

    if values.get("COMPASS_USE") and not external:
        # Which magnetometer is correct is a property of the airframe, not of
        # the firmware, and no parameter records the intent -- COMPASS_EXTERNAL
        # 0 looks identical whether it is a mistake or a decision. So the choice
        # is recorded alongside the toolkit and honoured here; without it this
        # would block forever on a board deliberately flying the onboard sensor.
        if airframe_choice("compass") == "internal":
            row("compass in use", "internal (recorded choice)")
        else:
            block("COMPASS_USE is on with COMPASS_EXTERNAL off, so the onboard "
                  "magnetometer is being used. If the external module on the "
                  "mast is meant to be the live one, run ./which_compass.py to "
                  "confirm, then solve_compass_orient.py and "
                  "calibrate_compass.py. If the onboard sensor is deliberate, "
                  "record it with: echo '{\"compass\": \"internal\"}' > "
                  "airframe.local.json")
    if values.get("COMPASS_LEARN"):
        warn("COMPASS_LEARN is on -- in-flight learning on 3.2.1 will corrupt a "
             "good calibration")


def check_rc(master, values, interactive):
    head("radio")
    apmlink.ensure_stream(master, mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10)
    if not apmlink.wait_for_message(master, "RC_CHANNELS_RAW", timeout=6.0):
        if board_halted:
            # The board produces no RC telemetry when halted, so silence here
            # says nothing about the receiver either way.
            row("status", "not assessable -- board halted")
            unknowns.append("RC could not be assessed: the board is halted in "
                            "ESC calibration and streams no RC telemetry. It "
                            "will report again once it boots normally.")
        else:
            block("no RC input at all -- transmitter off, receiver unbound, or "
                  "the input leads are wrong")
        return

    lo = {ch: 10000 for ch in range(1, 9)}
    hi = {ch: 0 for ch in range(1, 9)}
    seen_slots = set()

    if interactive:
        print("  Sweep every stick to both extremes and flick the mode switch")
        print("  through all its positions.")
        input("  Press Enter, then do it -- 12 seconds ")
        window = 12
    else:
        window = 3

    deadline = time.time() + window
    while time.time() < deadline:
        msg = master.recv_match(type="RC_CHANNELS_RAW", blocking=True, timeout=1.0)
        if msg is None:
            continue
        for ch in range(1, 9):
            pwm = getattr(msg, "chan%d_raw" % ch, 0)
            if 700 < pwm < 2400:
                lo[ch] = min(lo[ch], pwm)
                hi[ch] = max(hi[ch], pwm)
        pwm5 = getattr(msg, "chan5_raw", 0)
        if 700 < pwm5 < 2400:
            seen_slots.add(slot_for(pwm5))
        if interactive:
            print("\r  %4.1fs left " % (deadline - time.time()), end="", flush=True)
    if interactive:
        print("\r" + " " * 24)

    for ch in range(1, 6):
        if hi[ch] == 0:
            row("ch%d" % ch, "absent")
            block("channel %d is not present" % ch)
            continue
        row("ch%d" % ch, "%d - %d us" % (lo[ch], hi[ch]),
            "%d us travel" % (hi[ch] - lo[ch]))

    fs_value = values.get("FS_THR_VALUE")
    fs_on = values.get("FS_THR_ENABLE")
    rc3_min = values.get("RC3_MIN")
    row("FS_THR_ENABLE", fs_on)
    row("FS_THR_VALUE", fs_value)
    row("RC3_MIN (stored)", rc3_min)

    if hi[3] and fs_value is not None:
        if lo[3] < fs_value:
            block("throttle reached %d us, below FS_THR_VALUE %g. The flight "
                  "controller reads that as a lost radio and will refuse to arm."
                  % (lo[3], fs_value))
        elif lo[3] - fs_value < 30:
            warn("throttle minimum %d us is only %d us above FS_THR_VALUE %g -- "
                 "tight margin" % (lo[3], lo[3] - fs_value, fs_value))
    if hi[3] and rc3_min is not None and abs(lo[3] - rc3_min) > 60:
        warn("throttle bottoms at %d us but RC3_MIN is stored as %g -- re-run "
             "calibrate_radio.py --write" % (lo[3], rc3_min))
    if not fs_on:
        block("throttle failsafe is disabled")

    return seen_slots


def check_modes(values, seen_slots, exercised=True):
    head("flight modes")
    modes = {}
    for slot in range(1, 7):
        value = values.get("FLTMODE%d" % slot)
        modes[slot] = value
        mark = ""
        if seen_slots is not None:
            mark = "reachable" if slot in seen_slots else ""
        row("FLTMODE%d" % slot, MODE_NAMES.get(int(value) if value is not None else -1,
                                               value), mark)

    if not seen_slots:
        unknown_stream("mode switch was not exercised, so reachability is unknown")
        selectable = {int(v) for v in modes.values() if v is not None}
    elif not exercised:
        # --quick never asks for a sweep, so the switch was only ever seen in
        # whatever position it happened to be parked in. One observed slot is
        # then a statement about the test, not about the aircraft -- calling it
        # a stuck switch would be inventing a fault we did not look for.
        unknown("mode switch was only seen in slot %s because --quick skips the "
                "sweep; re-run without --quick to test reachability"
                % ", ".join(str(s) for s in sorted(seen_slots)))
        selectable = {int(v) for v in modes.values() if v is not None}
    else:
        selectable = {int(modes[s]) for s in seen_slots if modes.get(s) is not None}
        if len(seen_slots) < 2:
            block("the mode switch only ever reached one position -- no mode "
                  "switching, and you are locked in whatever that slot holds")

    if not (RECOVERY_MODES & selectable):
        if exercised and seen_slots:
            block("no reachable switch position selects RTL or LAND. There is no "
                  "way to hand control back to the aircraft in the air.")
        else:
            unknown("no configured switch position selects RTL or LAND -- "
                    "confirm with a full run before flying")


def check_imu(master, values):
    head("IMU")
    scal = [values.get("INS_ACCSCAL_%s" % a) for a in "XYZ"]
    accoffs = [values.get("INS_ACCOFFS_%s" % a) for a in "XYZ"]
    if None not in scal:
        row("accel scale", "%.3f %.3f %.3f" % tuple(scal))
    if None not in accoffs:
        row("accel offsets", "%.2f %.2f %.2f" % tuple(accoffs))
    if None not in scal + accoffs:
        if all(abs(v - 1.0) < 1e-6 for v in scal) and all(abs(v) < 1e-6 for v in accoffs):
            block("accelerometers have never been calibrated")

    apmlink.ensure_stream(master, mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 20)
    samples = []
    deadline = time.time() + 4
    while time.time() < deadline:
        msg = master.recv_match(type="RAW_IMU", blocking=True, timeout=1.0)
        if msg is not None:
            samples.append([msg.xgyro / 1000.0, msg.ygyro / 1000.0, msg.zgyro / 1000.0])
    if len(samples) < 20:
        unknown_stream("no RAW_IMU -- gyro not assessed")
        return
    arr = np.array(samples)
    bias, noise = arr.mean(axis=0), arr.std(axis=0)
    row("gyro bias", "%+.4f %+.4f %+.4f" % tuple(bias), "rad/s, at rest")
    row("gyro noise", "%.4f %.4f %.4f" % tuple(noise))
    if max(abs(bias)) > 0.05:
        block("gyro is not at rest (bias %.3f rad/s). Either the aircraft is "
              "moving or an axis is faulty." % max(abs(bias)))
    # RAW_IMU carries the gyro as int16 milli-rad/s, so one LSB is 0.001 rad/s
    # (~0.057 deg/s). A healthy MPU-6000 sitting still is quieter than that, so
    # every sample lands on the same integer and the measured noise is exactly
    # zero. Silence at rest therefore cannot distinguish "quiet" from "stuck"
    # at this resolution -- only a reading pinned far from zero is diagnostic.
    LSB = 0.001
    for axis, (n, b) in enumerate(zip(noise, bias)):
        if n >= 1e-5:
            continue
        if abs(b) > 2 * LSB:
            block("gyro axis %s is frozen at %+.3f rad/s -- it never changes and "
                  "is not near zero, which is a stuck sensor rather than a quiet "
                  "one" % ("XYZ"[axis], b))
        else:
            unknown("gyro axis %s reads a constant %+.3f rad/s at rest. That is "
                    "expected -- RAW_IMU quantises to %.3f rad/s and a still "
                    "board is quieter than one count -- so being stuck cannot be "
                    "ruled out from a stationary sample. ./check_imu.py rotates "
                    "each axis to test it properly."
                    % ("XYZ"[axis], b, LSB))
    row("INS_MPU6K_FILTER", values.get("INS_MPU6K_FILTER"),
        "20 Hz suits a soft-mounted APM")


def check_prearm(master):
    head("pre-arm messages")
    texts = []
    deadline = time.time() + 6
    while time.time() < deadline:
        msg = master.recv_match(type="STATUSTEXT", blocking=True, timeout=1.0)
        if msg is None:
            continue
        text = msg.text.strip("\x00").strip()
        if text and text not in texts:
            texts.append(text)
    prearm = [t for t in texts if "prearm" in t.lower() or "arm" in t.lower()]
    if prearm:
        for text in prearm:
            row("board", text)
            warn("pre-arm message from the board: %s" % text)
    else:
        row("", "none seen in 6s", "(they appear when arming is attempted)")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=apmlink.DEFAULT_BAUD)
    parser.add_argument("--quick", action="store_true",
                        help="skip the RC sweep that needs you to move the sticks")
    args = parser.parse_args()

    try:
        master = apmlink.connect(args.port, args.baud)
    except apmlink.LinkError as exc:
        apmlink.die(exc)

    print()
    print("  PROPELLERS OFF for this. Battery connected.")

    global board_halted
    # Two detectors, because neither alone is reliable. The banner is definite
    # when present but is often missed: /dev/cu.* does not assert DTR, so the
    # board frequently is not reset on connect and never reprints it. The
    # behavioural test does not care -- a board that streams nothing at all is
    # not running its main loop, whatever it did or did not print.
    apmlink.sample_banner(master, 2.0)
    esc_text = apmlink.esc_cal_active(master)
    streaming = apmlink.telemetry_alive(master, 5.0)
    board_halted = esc_text or not streaming
    values_preview = params(master, ["RC3_MAX"]) if board_halted else {}
    if board_halted:
        head("board state")
        row("startup", "HALTED in ESC calibration" if esc_text
                       else "NOT RUNNING -- no telemetry of any kind")
        row("telemetry", "none" if not streaming else "present")
        for line in apmlink.boot_banner(master).splitlines():
            line = line.strip()
            if "esc cal" in line.lower():
                row("banner", line)
        inputs = apmlink.esc_cal_inputs(master)
        detail = ""
        if inputs:
            row("RC seen at boot", "roll %d  pitch %d  THROTTLE %d  yaw %d"
                % tuple(inputs))
            rc3_max = values_preview.get("RC3_MAX")
            detail = (" The throttle channel was reading %d us at boot%s, which "
                      "is what tripped it."
                      % (inputs[2],
                         " against RC3_MAX %d" % int(rc3_max) if rc3_max else ""))
            row("ESC outputs", "%d us to all motors" % inputs[2],
                "MOTORS WOULD SPIN -- keep propellers off")
        cause = ("it entered ESC calibration and stopped there"
                 if esc_text else
                 "it produces no telemetry at all, which on 3.2.1 is nearly "
                 "always ESC calibration entered because the throttle channel "
                 "read high at startup")
        block("the board is not running its main loop: %s, so it streams no "
              "GPS, RC, IMU or battery data whatsoever -- every such check "
              "below is unassessable rather than failing.%s To clear it: turn "
              "the transmitter ON with the throttle stick fully DOWN (or "
              "reconnect the receiver if it is unplugged), then power-cycle the "
              "board and run this again. Parameters and the compass check below "
              "are still valid; nothing that needs a live sensor is."
              % (cause, detail))
        if inputs:
            block("while halted like this the board is driving every ESC at "
                  "%d us, which is full throttle. With a battery connected the "
                  "motors will run the instant they are able to -- keep the "
                  "propellers off until this is cleared." % inputs[2])

    names = ["BATT_MONITOR", "BATT_VOLT_MULT", "FS_BATT_VOLTAGE",
             "COMPASS_EXTERNAL", "COMPASS_ORIENT", "COMPASS_USE", "COMPASS_LEARN",
             "COMPASS_OFS_X", "COMPASS_OFS_Y", "COMPASS_OFS_Z",
             "FS_THR_ENABLE", "FS_THR_VALUE", "FS_GCS_ENABLE", "FS_BATT_ENABLE",
             "RC3_MIN", "ARMING_CHECK", "FRAME", "INS_MPU6K_FILTER",
             "INS_ACCSCAL_X", "INS_ACCSCAL_Y", "INS_ACCSCAL_Z",
             "INS_ACCOFFS_X", "INS_ACCOFFS_Y", "INS_ACCOFFS_Z"]
    names += ["FLTMODE%d" % i for i in range(1, 7)]
    values = params(master, names)

    head("board")
    row("FRAME", "%s" % values.get("FRAME"), "1 = quad X")
    row("ARMING_CHECK", values.get("ARMING_CHECK"))
    if not values.get("ARMING_CHECK"):
        block("pre-arm checks are disabled -- turn them back on")

    check_gps(master)
    check_battery(master, values)
    check_compass(master, values)
    seen_slots = check_rc(master, values, not args.quick)
    check_modes(values, seen_slots, exercised=not args.quick)
    check_imu(master, values)
    check_prearm(master)

    print()
    print("=" * 64)
    if blockers:
        print("  %d BLOCKER(S) -- do not fly" % len(blockers))
        for item in blockers:
            print("   x %s" % item)
    if warnings:
        print()
        print("  %d warning(s)" % len(warnings))
        for item in warnings:
            print("   ! %s" % item)
    if unknowns:
        print()
        print("  %d thing(s) could not be checked" % len(unknowns))
        for item in unknowns:
            print("   ? %s" % item)
    if not blockers:
        print()
        print("  No blockers found." if not warnings else
              "  No blockers, but read the warnings above.")
    print("=" * 64)
    return 1 if blockers else 0


if __name__ == "__main__":
    raise SystemExit(main())
