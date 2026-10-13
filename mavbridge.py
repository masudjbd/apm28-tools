#!/usr/bin/env python3
"""MAVLink v1 <-> v2 bridge, so QGroundControl can talk to ArduCopter 3.2.1.

    ./mavbridge.py                 # then connect QGC to UDP 127.0.0.1:14550
    ./mavbridge.py --verbose

Why this exists
---------------
Modern QGroundControl *parses* MAVLink v1 fine -- it sees the board's
heartbeat and registers the vehicle. What it will not do is *emit* v1: every
packet it sends starts with the v2 magic byte 0xFD. ArduCopter 3.2.1 predates
MAVLink v2 entirely and its parser only recognises 0xFE, so it silently
discards every one of those frames.

The result looks baffling from the UI: the vehicle appears, telemetry streams,
and then parameter download, mission download, geofence and rally point
transfers all time out with "maximum retries exceeded" -- because nothing QGC
sends has ever reached the board. It is a one-way conversation.

This process sits in between. It speaks v1 to the board over serial and v2 to
QGC over UDP, re-encoding each message as it crosses. Nothing is translated
semantically; the frame envelope is rebuilt and the payload passes through.

Messages whose ID is above 255 cannot be represented in a v1 frame at all.
Those are dropped on the way to the board and counted, rather than being
mangled into something the firmware would misparse.
"""

import argparse
import os
import sys
import time

# Must be set before pymavlink is imported: it selects the v2 wire format at
# module load. The v2 parser reads v1 frames too, so one parser handles both
# directions; only the *writing* side needs to differ.
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402

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

import apmlink  # noqa: E402

V1_MAX_MSGID = 255


class Stats:
    def __init__(self):
        self.to_gcs = 0
        self.to_board = 0
        self.dropped = 0
        self.dropped_ids = {}

    def line(self):
        text = "  board->QGC %6d   QGC->board %6d" % (self.to_gcs, self.to_board)
        if self.dropped:
            text += "   dropped %d (v2-only)" % self.dropped
        return text


def forward(src, dst, stats, to_board, verbose=False):
    """Move every pending message from src to dst, re-encoding the frame."""
    moved = 0
    while True:
        msg = src.recv_match(blocking=False)
        if msg is None:
            return moved
        msg_type = msg.get_type()
        if msg_type in ("BAD_DATA", "UNKNOWN"):
            continue

        if to_board and msg.get_msgId() > V1_MAX_MSGID:
            stats.dropped += 1
            stats.dropped_ids[msg_type] = stats.dropped_ids.get(msg_type, 0) + 1
            if verbose:
                print("  drop -> board: %s (msgid %d has no v1 encoding)"
                      % (msg_type, msg.get_msgId()))
            continue

        # Preserve the original sender's identity rather than stamping the
        # bridge's own. QGC keys its whole vehicle model off srcSystem, and the
        # board answers to the GCS sysid it was addressed by.
        dst.mav.srcSystem = msg.get_srcSystem()
        dst.mav.srcComponent = msg.get_srcComponent()

        try:
            buf = msg.pack(dst.mav, force_mavlink1=to_board)
            dst.write(buf)
        except Exception as exc:  # a malformed frame must not kill the bridge
            if verbose:
                print("  repack failed for %s: %s" % (msg_type, exc))
            continue

        if to_board:
            stats.to_board += 1
        else:
            stats.to_gcs += 1
        moved += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial port (auto-detected if omitted)")
    parser.add_argument("--baud", type=int, default=apmlink.DEFAULT_BAUD)
    parser.add_argument("--udp", default="127.0.0.1:14550",
                        help="where QGroundControl is listening (default %(default)s)")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    try:
        port = apmlink.resolve_port(args.port)
    except apmlink.LinkError as exc:
        apmlink.die(exc)

    # Fail before printing the banner, so the message the user sees is the
    # actual problem rather than instructions followed by a stack trace.
    holders = apmlink.port_holder(port)
    if holders:
        apmlink.die(apmlink.busy_port_message(port))

    print()
    print("  serial   %s @ %d   (MAVLink v1, the board)" % (port, args.baud))
    print("  udp      %s             (MAVLink v2, QGroundControl)" % args.udp)
    print()
    print("  In QGroundControl: DISCONNECT the 'APM 2.8 USB' serial link first --")
    print("  two processes cannot own one serial port. QGC picks up the UDP link")
    print("  automatically if 'Automatically open a connection over UDP' is on.")
    print()

    try:
        board = apmlink.open_serial(port, args.baud, dialect="ardupilotmega")
    except apmlink.LinkError as exc:
        apmlink.die(exc)
    gcs = mavutil.mavlink_connection("udpout:" + args.udp, dialect="ardupilotmega",
                                     source_system=255, source_component=0)

    stats = Stats()
    print("  bridging -- Ctrl-C to stop")
    last_print = 0.0
    try:
        while True:
            busy = forward(board, gcs, stats, to_board=False, verbose=args.verbose)
            busy += forward(gcs, board, stats, to_board=True, verbose=args.verbose)
            if not busy:
                time.sleep(0.002)
            now = time.time()
            if now - last_print > 1.0:
                last_print = now
                print("\r" + stats.line(), end="", flush=True)
    except KeyboardInterrupt:
        print()
        print()
        print("  stopped." )
        print(stats.line().strip())
        if stats.dropped_ids:
            print("  v2-only messages QGC tried to send (dropped, expected):")
            for name, count in sorted(stats.dropped_ids.items(), key=lambda kv: -kv[1]):
                print("    %-40s %d" % (name, count))
        if stats.to_board == 0:
            print()
            print("  QGC never sent anything -- it was not connected to the UDP link.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
