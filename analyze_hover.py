#!/usr/bin/env python3
"""Answer one question from a dataflash log: was the drift you, or the aircraft?

    ./analyze_hover.py logs/dataflash/log46.bin

A pilot hovering indoors cannot tell a mistrimmed aircraft from their own stick
input -- you correct the drift without noticing you corrected it, and both feel
identical from behind the sticks. The log can tell them apart.

What it reads, and why not RCIN
-------------------------------
The obvious source is RCIN, the raw stick channels. It is the wrong one: RCIN
is gated behind LOG_BITMASK and is absent from any log taken at 830, which is
what this aircraft flew on until 2026-09-02. log45.bin has no RCIN at all.

ATT.DesRoll / ATT.DesPitch are always logged, and they are better anyway: they
are the stick input already converted to a commanded lean angle in degrees,
with RC trim, deadzone, reversal and ANGLE_MAX applied by the firmware. No
parameter maths on this side, and nothing to get wrong.

The reading that matters
------------------------
Mean DesRoll over a hover. To hold station over one spot, a pilot must cancel
whatever the aircraft does on its own -- so a steady non-zero commanded lean IS
the aircraft's bias, measured by your hand without you noticing. It works even
if you never once released the sticks, which indoors is the normal case.

ATT.Roll is reported AFTER AHRS_TRIM is applied, so a well-behaved controller
drives it to ~0 whenever the sticks are centred even on a badly trimmed
aircraft. Reading it alone would say "level, nothing wrong" while the aircraft
slid into a wall. It is reported below as tracking error only.
"""

import argparse
import os
import sys

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

from pymavlink import mavutil  # noqa: E402

# Below this, indoor wall recirculation and ordinary pilot noise swamp the
# signal and trimming chases something that is not there.
SIGNIFICANT_DEG = 1.0


def mean(xs):
    return sum(xs) / float(len(xs)) if xs else 0.0


def stdev(xs):
    if len(xs) < 2:
        return 0.0
    m = mean(xs)
    return (sum((x - m) ** 2 for x in xs) / (len(xs) - 1)) ** 0.5


def read_log(path):
    mlog = mavutil.mavlink_connection(path)
    parm, samples, modes = {}, [], []
    ctun = None
    while True:
        m = mlog.recv_match(type=["PARM", "ATT", "CTUN", "MODE"])
        if m is None:
            break
        t = m.get_type()
        if t == "PARM":
            parm[m.Name] = m.Value
        elif t == "MODE":
            modes.append(getattr(m, "Mode", "?"))
        elif t == "CTUN":
            ctun = m
        elif t == "ATT" and ctun is not None:
            samples.append((ctun, m))
    return parm, samples, modes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--min-throttle", type=float, default=200.0,
                    help="CTUN.ThrIn (0-1000) that counts as spooled up")
    ap.add_argument("--min-alt", type=float, default=0.25,
                    help="metres above the arming point that counts as airborne")
    args = ap.parse_args()

    if not os.path.exists(args.log):
        print("no such file: %s" % args.log, file=sys.stderr)
        return 2

    parm, samples, modes = read_log(args.log)
    if not samples:
        print("\n  No ATT records. Was this aircraft ever armed on this log?\n")
        return 1

    flying = [(c, a) for c, a in samples
              if c.ThrIn > args.min_throttle and c.Alt > args.min_alt]

    print()
    print("  log            %s" % args.log)
    print("  records        %d attitude samples, %d airborne"
          % (len(samples), len(flying)))
    if modes:
        seen = []
        for mo in modes:
            if not seen or seen[-1] != mo:
                seen.append(str(mo))
        print("  modes          %s" % " -> ".join(seen))
    print("  stored trim    AHRS_TRIM_X %+.4f  AHRS_TRIM_Y %+.4f rad"
          % (parm.get("AHRS_TRIM_X", 0.0), parm.get("AHRS_TRIM_Y", 0.0)))

    if len(flying) < 40:
        print()
        print("  Not enough airborne time to judge anything (need ~40 samples,")
        print("  a few seconds of steady hover above %.2f m)." % args.min_alt)
        print("  Fly a steady 15-20 second hover, land, download and re-run.")
        print()
        return 1

    des_r = [a.DesRoll for _, a in flying]
    des_p = [a.DesPitch for _, a in flying]
    err_r = [a.Roll - a.DesRoll for _, a in flying]
    err_p = [a.Pitch - a.DesPitch for _, a in flying]
    alt = [c.Alt for c, _ in flying]

    print()
    print("--- the hover ---------------------------------------------------")
    print("  altitude       mean %.2f m, range %.2f to %.2f m"
          % (mean(alt), min(alt), max(alt)))

    print()
    print("--- what you commanded (your sticks) ----------------------------")
    print("  mean roll      %+.2f deg      spread %.2f" % (mean(des_r), stdev(des_r)))
    print("  mean pitch     %+.2f deg      spread %.2f" % (mean(des_p), stdev(des_p)))

    print()
    print("--- tracking: did it do what you asked? -------------------------")
    print("  roll  error    %+.2f deg      spread %.2f" % (mean(err_r), stdev(err_r)))
    print("  pitch error    %+.2f deg      spread %.2f" % (mean(err_p), stdev(err_p)))

    print()
    print("--- verdict -----------------------------------------------------")
    found = False
    for name, held, (pos, neg) in (
            ("roll", mean(des_r), ("right", "left")),
            ("pitch", mean(des_p), ("backward", "forward"))):
        if abs(held) >= SIGNIFICANT_DEG:
            found = True
            holding = pos if held > 0 else neg
            drifting = neg if held > 0 else pos
            print("  %-5s you held %+.2f deg of %s for the whole hover."
                  % (name, held, holding))
            print("        The aircraft drifts %s on its own. Real bias, not you."
                  % drifting)

    track = max(abs(mean(err_r)), abs(mean(err_p)))
    if track >= 2.0:
        found = True
        print("  The aircraft did not reach the attitude it was asked for")
        print("  (mean error %.2f deg). That is a control problem, not trim --" % track)
        print("  suspect a weak motor, a damaged prop, or saturated output.")

    if not found:
        print("  No steady bias over %.1f degree in either axis, and it tracked" % SIGNIFICANT_DEG)
        print("  your commands. The aircraft is trimmed.")
        print()
        print("  What you are feeling is STABILIZE doing its job. It levels the")
        print("  aircraft when you centre the sticks -- it does NOT stop it.")
        print("  A level quad keeps whatever velocity it already had, and there")
        print("  is no brake. You must push the opposite way to stop it.")
        print("  Indoors, wall recirculation adds drift no trim can remove.")
        print("  Only LOITER and POSHOLD stop a quad, and they need GPS, which")
        print("  does not work in a room.")
    print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
