#!/usr/bin/env python3
"""Apply parameter profiles to an APM 2.x, with read-back verification.

    ./preload_params.py --list
    ./preload_params.py --backup                     # dump everything first
    ./preload_params.py --dry-run base frame-x compass-external
    ./preload_params.py base frame-x compass-external power-module failsafe-3s
    ./preload_params.py --restore logs/params-xxx.params
    ./preload_params.py --factory-reset              # wipe EEPROM to defaults

Profiles live in profiles/*.params as plain "NAME VALUE" lines, which is the
same format Mission Planner reads and writes, so anything saved here can be
loaded on a Windows box later.

Every write is confirmed by comparing the PARAM_VALUE the board broadcasts
back. ArduPilot silently clamps or ignores out-of-range and read-only writes,
so a write that is never checked is a write you cannot claim happened.
"""

import argparse
import glob
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

import apmlink

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROFILE_DIR = os.path.join(SCRIPT_DIR, "profiles")
LOG_DIR = os.path.join(SCRIPT_DIR, "logs")


def available_profiles():
    out = {}
    for path in sorted(glob.glob(os.path.join(PROFILE_DIR, "*.params"))):
        stem = os.path.splitext(os.path.basename(path))[0]
        # strip the numeric ordering prefix: "20-compass-external" -> "compass-external"
        short = stem.split("-", 1)[1] if "-" in stem and stem.split("-", 1)[0].isdigit() else stem
        out[short] = path
    return out


def parse_params_file(path):
    entries = []
    with open(path) as handle:
        for lineno, line in enumerate(handle, 1):
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.replace(",", " ").split()
            if len(parts) != 2:
                raise ValueError("%s:%d: expected 'NAME VALUE', got %r" % (path, lineno, line))
            entries.append((parts[0].upper(), float(parts[1])))
    return entries


def collect(profile_names, profiles):
    """Merge profiles in the order given; later profiles win, and say so."""
    merged = []
    seen = {}
    for name in profile_names:
        if name not in profiles:
            apmlink.die("unknown profile %r (see --list)" % name)
        for key, value in parse_params_file(profiles[name]):
            if key in seen and seen[key][1] != value:
                print("  note: %s overridden by '%s': %g -> %g"
                      % (key, name, seen[key][1], value))
                merged = [entry for entry in merged if entry[0] != key]
            seen[key] = (name, value)
            merged.append((key, value))
    # dedupe keeping last occurrence
    final, taken = [], set()
    for key, value in reversed(merged):
        if key not in taken:
            taken.add(key)
            final.append((key, value))
    return list(reversed(final))


# The distinction that matters after a crash.
#
# CALIBRATION describes the physical world. When the board shifts on its mount,
# AHRS_TRIM genuinely SHOULD change -- the aircraft really is at a new angle.
# Restoring an old value here re-imposes a lie. These get re-measured, never
# restored.
#
# CONFIGURATION describes your choices. Nothing physical alters it, so if it
# differs from your baseline something overwrote it, and pushing the baseline
# back is exactly right.
REMEASURE = {
    "AHRS_TRIM_X", "AHRS_TRIM_Y", "AHRS_TRIM_Z",
    "COMPASS_OFS_X", "COMPASS_OFS_Y", "COMPASS_OFS_Z",
    "COMPASS_DEC",
    "INS_ACCOFFS_X", "INS_ACCOFFS_Y", "INS_ACCOFFS_Z",
    "INS_ACCSCAL_X", "INS_ACCSCAL_Y", "INS_ACCSCAL_Z",
    "INS_GYROFFS_X", "INS_GYROFFS_Y", "INS_GYROFFS_Z",
}
REMEASURE_PREFIXES = ("RC1_", "RC2_", "RC3_", "RC4_", "RC5_", "RC6_", "RC7_", "RC8_")

REMEASURE_HOW = {
    "AHRS_TRIM": "./level_calibrate.py",
    "COMPASS_OFS": "./calibrate_compass.py --write",
    "INS_ACC": "./gcs.sh  then  accelcal",
    "INS_GYRO": "re-measured automatically at every boot; nothing to do",
    "RC": "./calibrate_radio.py --write",
}


def is_remeasure(name):
    return name in REMEASURE or name.startswith(REMEASURE_PREFIXES)


def how_to_remeasure(name):
    for prefix, advice in REMEASURE_HOW.items():
        if name.startswith(prefix):
            return advice
    return "re-measure it"


def diff(master, path):
    """Compare the board against a saved baseline and classify what moved."""
    entries = parse_params_file(path)
    print()
    print("  comparing %d parameters against %s" % (len(entries), os.path.basename(path)))
    print()

    same, config_drift, calib_drift, missing = 0, [], [], []
    for name, expected in entries:
        try:
            actual = apmlink.get_param(master, name, timeout=3.0, retries=2)
        except apmlink.LinkError:
            missing.append(name)
            continue
        if abs(actual - expected) < 1e-4:
            same += 1
        elif is_remeasure(name):
            calib_drift.append((name, expected, actual))
        else:
            config_drift.append((name, expected, actual))

    print("  %d unchanged" % same)

    if config_drift:
        print()
        print("  === CONFIGURATION that changed -- restore these ===")
        print("  Nothing physical alters these. If they moved, something overwrote them.")
        for name, expected, actual in config_drift:
            print("    %-20s baseline %-12g now %g" % (name, expected, actual))
        print()
        print("  restore with:  ./preload_params.py --restore %s"
              % os.path.relpath(path, SCRIPT_DIR))

    if calib_drift:
        print()
        print("  === CALIBRATION that changed -- do NOT restore ===")
        print("  These describe the physical aircraft. After a crash or a remount")
        print("  they SHOULD differ, because the hardware really did change.")
        print("  Restoring an old value here re-imposes a measurement that is no")
        print("  longer true. Re-measure instead.")
        for name, expected, actual in calib_drift:
            print("    %-20s baseline %-12g now %-12g -> %s"
                  % (name, expected, actual, how_to_remeasure(name)))

    if missing:
        print()
        print("  not present on this firmware: %s" % ", ".join(missing))

    if not config_drift and not calib_drift:
        print()
        print("  nothing has drifted.")
    return 1 if config_drift else 0


def backup(master):
    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, "params-%s.params" % time.strftime("%Y%m%d-%H%M%S"))

    # ArduPilot answers PARAM_REQUEST_LIST from queued_param_send(), which runs
    # at the SR0_PARAMS rate. At 0 the board replies with NOTHING -- no error,
    # no partial list -- and this function used to save an empty file that
    # looked like a backup. log_download.py zeroes every stream to give the log
    # the whole link, and ArduPilot PERSISTS those rates, so any backup taken
    # after a log download silently captured nothing.
    #
    # Write it blind rather than reading first: SR parameter reads time out on
    # this board often enough that a read-gated write is a write that never
    # happens, and a redundant write costs nothing.
    try:
        apmlink.set_param(master, "SR0_PARAMS", 10, verify=False)
    except apmlink.LinkError:
        pass
    apmlink.drain(master, 0.3)

    print("  fetching full parameter list (this takes a minute on a 2560) ...")
    master.mav.param_request_list_send(master.target_system, master.target_component)

    values, expected, last = {}, None, time.time()
    while time.time() - last < 8.0:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=2.0)
        if msg is None:
            continue
        last = time.time()
        expected = msg.param_count
        values[msg.param_id.strip("\x00")] = msg.param_value
        if expected and len(values) >= expected:
            break
        print("\r  %d/%s" % (len(values), expected or "?"), end="", flush=True)
    print("\r  %d parameters captured%s" % (len(values), " " * 12))

    if not values:
        print()
        print("  The board returned NO parameters, so nothing was saved -- an empty")
        print("  file here would look like a backup and is worse than no file at all.")
        print("  SR0_PARAMS gates this reply; it is set above, so if you just ran")
        print("  log_download.py, power-cycle the board and try again.")
        return None

    if expected and len(values) < expected:
        print("  WARNING: got %d of %d -- this backup is INCOMPLETE" % (len(values), expected))

    with open(path, "w") as handle:
        handle.write("# APM 2.x parameter backup %s\n" % time.strftime("%Y-%m-%d %H:%M:%S"))
        for key in sorted(values):
            handle.write("%s %g\n" % (key, values[key]))
    print("  saved %s" % os.path.relpath(path, SCRIPT_DIR))
    return path


def apply(master, entries, dry_run=False):
    width = max(len(key) for key, _ in entries)
    failures = []
    for key, value in entries:
        if dry_run:
            try:
                current = apmlink.get_param(master, key)
                mark = "=" if abs(current - value) < 1e-4 else ">"
                print("  %s %-*s %-12g (now %g)" % (mark, width, key, value, current))
            except apmlink.LinkError:
                print("  ? %-*s %-12g (not present on this firmware)" % (width, key, value))
            continue
        try:
            apmlink.set_param(master, key, value)
            print("  ok %-*s %g" % (width, key, value))
        except apmlink.LinkError as exc:
            failures.append((key, str(exc)))
            print("  -- %-*s FAILED: %s" % (width, key, exc))
    return failures


# The EEPROM format marker is exposed under different names depending on
# vehicle and vintage. ArduCopter 3.2.1 calls it SYSID_SW_MREV; FORMAT_VERSION
# is the Plane/Rover and later-generation name. Confirmed against a live APM
# 2.8 running 3.2.1, where FORMAT_VERSION does not exist at all -- writing it
# would have silently reset nothing while reporting success.
FORMAT_MARKERS = ["SYSID_SW_MREV", "FORMAT_VERSION"]

# Parameters ArduCopter only reads at startup. Everything else applies live.
REBOOT_REQUIRED = {
    "INS_MPU6K_FILTER",
    "FRAME",
    "SERIAL0_BAUD",
    "SERIAL1_BAUD",
    "GPS_TYPE",
    "BATT_MONITOR",
}


def factory_reset(master, assume_yes=False):
    """Force ArduPilot to rewrite defaults on next boot.

    Zeroing the EEPROM format marker makes the firmware treat the EEPROM as
    uninitialised, so it reformats it. This matters after flashing with
    avrdude: avrdude only writes flash, so whatever parameters shipped on the
    board survive a firmware change and the new firmware happily flies on
    stale settings.
    """
    marker = None
    for candidate in FORMAT_MARKERS:
        try:
            apmlink.get_param(master, candidate, timeout=3.0, retries=2)
            marker = candidate
            break
        except apmlink.LinkError:
            continue
    if marker is None:
        print("  no EEPROM format marker found (tried %s) -- refusing to guess."
              % ", ".join(FORMAT_MARKERS))
        return False

    print()
    print("  This erases ALL parameters: calibration, radio, failsafes, tuning.")
    print("  Format marker on this firmware: %s" % marker)
    if not apmlink.confirm("  wipe EEPROM to defaults?", assume_yes):
        return False
    apmlink.set_param(master, marker, 0, verify=False)
    print("  %s zeroed, rebooting ..." % marker)
    apmlink.reboot(master)
    time.sleep(8)
    print("  done. Reconnect and re-apply profiles.")
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("profiles", nargs="*")
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=apmlink.DEFAULT_BAUD)
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--backup", action="store_true")
    parser.add_argument("--restore", metavar="FILE")
    parser.add_argument("--diff", metavar="FILE",
                        help="compare the board against a saved baseline and show "
                             "what drifted, split into restore-these and "
                             "re-measure-these")
    parser.add_argument("--factory-reset", action="store_true")
    parser.add_argument("-y", "--yes", action="store_true")
    args = parser.parse_args()

    profiles = available_profiles()
    if args.list:
        print("profiles:")
        for name in sorted(profiles):
            print("  %-20s %s" % (name, os.path.relpath(profiles[name], SCRIPT_DIR)))
        return 0

    if not (args.profiles or args.backup or args.restore or args.factory_reset
            or args.diff):
        parser.print_help()
        return 1

    entries = []
    if args.restore:
        entries = parse_params_file(args.restore)
    elif args.profiles:
        entries = collect(args.profiles, profiles)

    try:
        master = apmlink.connect(args.port, args.baud)
    except apmlink.LinkError as exc:
        apmlink.die(exc)

    if args.diff:
        return diff(master, args.diff)

    if args.backup:
        backup(master)

    if args.factory_reset:
        factory_reset(master, args.yes)
        return 0

    if not entries:
        return 0

    print()
    print("  %d parameters to apply%s" % (len(entries), " (dry run)" if args.dry_run else ""))
    if not args.dry_run and not apmlink.confirm("  write them?", args.yes):
        return 1

    failures = apply(master, entries, args.dry_run)
    if failures:
        print()
        print("  %d parameter(s) did not take. Most likely they do not exist on"
              % len(failures))
        print("  ArduCopter 3.2.1 -- parameter names drifted a lot after 2015.")
        return 1
    if not args.dry_run:
        # Only mention a reboot when something in THIS batch actually needs one.
        # Printing it unconditionally trains you to ignore it, which is worse
        # than not printing it at all.
        applied = {key for key, _ in entries}
        needs_reboot = sorted(applied & REBOOT_REQUIRED)
        print()
        if needs_reboot:
            print("  all verified. Reboot the board for %s to take effect."
                  % ", ".join(needs_reboot))
        else:
            print("  all verified. These take effect immediately -- no reboot needed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
