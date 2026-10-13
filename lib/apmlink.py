"""Shared MAVLink plumbing for APM 2.x (ArduCopter 3.2.1) over USB.

Everything here is written against MAVLink 1 and the 2015-era ArduPilot
parameter protocol. It deliberately avoids anything QGroundControl-era:
no COMMAND_INT, no MAV_CMD_SET_MESSAGE_INTERVAL, no param microservice
extensions. The board does not implement them.
"""

import glob
import os
import subprocess
import sys
import time

from pymavlink import mavutil

DEFAULT_BAUD = 115200

# /dev/cu.* rather than /dev/tty.*: on macOS the tty.* node blocks on open
# waiting for carrier detect, which a USB CDC board never asserts. The callout
# node has no such wait. Ordered by how likely each is on an APM 2.8 -- the
# ATmega16U2 bridge on a genuine board enumerates as usbmodem.
PORT_GLOBS = [
    "/dev/cu.usbmodem*",
    "/dev/cu.usbserial*",
    "/dev/cu.wchusbserial*",
    "/dev/cu.SLAB_USBtoUART*",
]


class LinkError(RuntimeError):
    pass


def find_ports():
    found = []
    for pattern in PORT_GLOBS:
        for path in sorted(glob.glob(pattern)):
            if path not in found:
                found.append(path)
    return found


def resolve_port(explicit=None):
    if explicit:
        if not os.path.exists(explicit):
            raise LinkError("no such port: %s" % explicit)
        return explicit
    found = find_ports()
    if not found:
        raise LinkError(
            "No USB serial port found.\n"
            "  1. Cable -- many mini-USB cables are charge-only. Most common cause.\n"
            "  2. Plug straight into the Mac, not through a hub or dock.\n"
            "  3. ls /dev/cu.* to see everything macOS enumerated."
        )
    if len(found) > 1:
        raise LinkError(
            "Several candidate ports; pick one with --port:\n  " + "\n  ".join(found)
        )
    return found[0]


def port_holder(port):
    """Which processes currently have this serial device open.

    macOS exposes the same USB device as both /dev/cu.X and /dev/tty.X, and a
    program holding either one blocks the other, so both are checked. Returns
    a list of (pid, process name); empty if nothing holds it or lsof is absent.
    """
    candidates = [port]
    base = os.path.basename(port)
    if base.startswith("cu."):
        candidates.append(os.path.join(os.path.dirname(port), "tty." + base[3:]))
    elif base.startswith("tty."):
        candidates.append(os.path.join(os.path.dirname(port), "cu." + base[4:]))

    holders, seen = [], set()
    for candidate in candidates:
        try:
            result = subprocess.run(["lsof", "-t", "-w", candidate],
                                    capture_output=True, text=True, timeout=5)
        except (FileNotFoundError, subprocess.SubprocessError):
            return []
        for pid in result.stdout.split():
            pid = pid.strip()
            if not pid or pid in seen:
                continue
            seen.add(pid)
            try:
                name = subprocess.run(["ps", "-p", pid, "-o", "comm="],
                                      capture_output=True, text=True,
                                      timeout=5).stdout.strip()
            except (FileNotFoundError, subprocess.SubprocessError):
                name = "?"
            holders.append((pid, os.path.basename(name) or "?"))
    return holders


def busy_port_message(port):
    """Turn 'Resource busy' into something actionable."""
    holders = port_holder(port)
    lines = ["%s is already open by another process." % port]
    if holders:
        lines.append("")
        for pid, name in holders:
            lines.append("  holding it:  %s  (pid %s)" % (name, pid))
    lines.append("")
    lines.append("Only one process can own a serial port. Common culprits:")
    lines.append("  - QGroundControl. Disconnecting the link is NOT enough: its")
    lines.append("    Comm Links -> AutoConnect settings reopen the port on their")
    lines.append("    own. Quit QGC, or turn those toggles off.")
    lines.append("  - another script from this toolkit still running")
    lines.append("  - a stale screen/minicom session")
    return "\n".join(lines)


def open_serial(port, baud, **kwargs):
    """mavlink_connection with a comprehensible failure mode."""
    from pymavlink import mavutil as _mavutil
    try:
        return _mavutil.mavlink_connection(port, baud=baud, **kwargs)
    except Exception as exc:
        if "busy" in str(exc).lower() or "errno 16" in str(exc).lower():
            raise LinkError(busy_port_message(port))
        raise LinkError("could not open %s: %s" % (port, exc))


def connect(port=None, baud=DEFAULT_BAUD, timeout=30, quiet=False):
    """Open the link and wait for a heartbeat.

    Opening the port toggles DTR, which resets the ATmega2560. The board then
    spends several seconds in the bootloader before ArduCopter starts talking,
    so the first heartbeat legitimately takes a while -- this is not a hang.
    """
    port = resolve_port(port)
    if not quiet:
        print("link   %s @ %d" % (port, baud))
    master = open_serial(port, baud)

    deadline = time.time() + timeout
    if not quiet:
        print("waiting for heartbeat (board reboots on connect, ~5-10s) ...", end="", flush=True)
    hb = None
    banner = bytearray()
    while time.time() < deadline:
        msg = master.recv_match(blocking=True, timeout=2)
        if msg is None:
            if not quiet:
                print(".", end="", flush=True)
            continue
        kind = msg.get_type()
        if kind == "HEARTBEAT":
            hb = msg
            break
        if kind == "BAD_DATA":
            # ArduCopter prints a plain-text boot banner on the same wire
            # before MAVLink starts, so pymavlink surfaces it as BAD_DATA.
            # Keep it rather than discarding it: it is the only place the
            # board ever reports that it halted in ESC calibration.
            banner += bytes(msg.data)
    if not quiet:
        print()
    if hb is None:
        raise LinkError(
            "No heartbeat in %ds. Either the board has no firmware (flash it "
            "with ./flash_apm28.sh) or another program is holding the port." % timeout
        )

    master.wait_heartbeat()
    master.apm_banner = banner.decode("ascii", "replace")
    if not quiet:
        print("board  system %d component %d" % (master.target_system, master.target_component))
    return master


def boot_banner(master):
    """The plain-text startup banner captured while connecting, if any."""
    return getattr(master, "apm_banner", "")


def sample_banner(master, seconds=3.0):
    """Listen for more plain-text output and fold it into the stored banner.

    connect() stops collecting at the first heartbeat, which races the boot
    text: whether the ESC-calibration notice lands before or after that
    heartbeat varies run to run, so connect() alone detects it only sometimes.
    The passthrough loop reprints its status continuously, so sampling any
    window at all turns a coin-flip into a certainty.
    """
    buf = bytearray()
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == "BAD_DATA":
            buf += bytes(msg.data)
        elif kind == "STATUSTEXT":
            buf += (msg.text.strip("\x00").strip() + "\n").encode("ascii", "replace")
    if buf:
        master.apm_banner = (getattr(master, "apm_banner", "")
                             + buf.decode("ascii", "replace"))
    return getattr(master, "apm_banner", "")


# Messages that only exist because the main loop is running and streaming.
# HEARTBEAT, STATUSTEXT and PARAM_VALUE are excluded on purpose: all three are
# serviced from the delay callback and keep flowing on a board that has halted,
# which is precisely why a halted board looks alive from the outside.
STREAMED_MSGS = (
    "SYS_STATUS", "RAW_IMU", "SCALED_IMU2", "GPS_RAW_INT", "RC_CHANNELS_RAW",
    "ATTITUDE", "VFR_HUD", "SERVO_OUTPUT_RAW", "GLOBAL_POSITION_INT",
    "SCALED_PRESSURE", "NAV_CONTROLLER_OUTPUT",
)


def telemetry_alive(master, seconds=5.0):
    """Ask for every stream, then see whether the board produces any of them.

    This is the behavioural test for "the board is not actually running", and
    unlike reading the boot banner it does not depend on catching a message
    that may have been printed before the port was opened. Opening /dev/cu.*
    deliberately does not assert DTR, so the board is often NOT reset on
    connect and never reprints its banner at all.
    """
    for stream_id in (
        mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
        mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
    ):
        request_stream(master, stream_id, 5)
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() in STREAMED_MSGS:
            return True
    return False


def esc_cal_active(master):
    """True when the board halted in ESC calibration instead of booting.

    ArduCopter 3.2.1 samples the throttle channel during startup. If it reads
    high the board prints an ESC-calibration banner and stops in a loop, so the
    main loop never runs and not one telemetry stream is ever produced: no
    GPS_RAW_INT, no SYS_STATUS, no RC_CHANNELS_RAW, no RAW_IMU.

    Heartbeats and the parameter protocol keep working throughout, because both
    are serviced from the delay callback. That is what makes this state so
    confusing from the outside -- it presents as a dozen unrelated dead sensors
    rather than as one halted board.
    """
    text = boot_banner(master).lower()
    return "esc cal" in text


def esc_cal_inputs(master):
    """The RC inputs the board sampled when it entered ESC calibration.

    The passthrough loop logs a line like

        ESC cal input: 1511 1485 1996 1497  output: 1996 1996 1996 1996

    which is roll/pitch/throttle/yaw in microseconds. The third figure is the
    reason the board halted, so quoting it turns "throttle was high" into a
    number that can be checked against RC3_MAX. Returns None if absent.
    """
    for line in boot_banner(master).splitlines():
        if "esc cal" not in line.lower() or "input" not in line.lower():
            continue
        part = line.lower().split("input:", 1)[1]
        part = part.split("output")[0]
        vals = []
        for tok in part.split():
            try:
                vals.append(int(tok))
            except ValueError:
                pass
        if len(vals) >= 4:
            return vals[:4]
    return None


# ArduPilot gates every telemetry stream behind an SR<port>_* parameter as well
# as the REQUEST_DATA_STREAM message. On a board that has only ever talked to
# Mission Planner these can all sit at 0, and then REQUEST_DATA_STREAM alone
# gets you nothing: the request is accepted and no data appears. Setting the
# parameter is the part that actually turns the tap on.
STREAM_PARAMS = {
    mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS: "SR0_RAW_SENS",
    mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS: "SR0_EXT_STAT",
    mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS: "SR0_RC_CHAN",
    mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER: "SR0_RAW_CTRL",
    mavutil.mavlink.MAV_DATA_STREAM_POSITION: "SR0_POSITION",
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1: "SR0_EXTRA1",
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA2: "SR0_EXTRA2",
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA3: "SR0_EXTRA3",
}


def request_stream(master, stream_id, rate_hz):
    master.mav.request_data_stream_send(
        master.target_system, master.target_component, stream_id, int(rate_hz), 1
    )


def ensure_stream(master, stream_id, rate_hz, quiet=True):
    """Turn a stream on properly: set its SR0_* rate, then request it.

    Belt and braces on purpose. The parameter write is what works on a board
    whose stream rates are zeroed; the request is what works if the parameter
    is named differently on some other firmware vintage.
    """
    name = STREAM_PARAMS.get(stream_id)
    if name:
        try:
            current = get_param(master, name, timeout=3.0, retries=2)
            if current < rate_hz:
                set_param(master, name, rate_hz)
                if not quiet:
                    print("  %s %g -> %g Hz" % (name, current, rate_hz))
        except LinkError:
            # The READ timed out -- SR0_RAW_SENS does this routinely on APM 2.8.
            # Falling through to request_stream alone is not enough: if the
            # parameter is sitting at 0 the stream never flows no matter how
            # many REQUEST_DATA_STREAM messages are sent, and the caller sees
            # "board not streaming raw sensors" on a perfectly healthy board.
            # Write it blind instead -- a redundant write is harmless, a
            # skipped one is not.
            try:
                set_param(master, name, rate_hz)
                if not quiet:
                    print("  %s set to %g Hz (read timed out)" % (name, rate_hz))
            except LinkError:
                pass
    request_stream(master, stream_id, rate_hz)


def wait_for_message(master, msg_type, timeout=6.0):
    """True if at least one of these arrives within the window."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if master.recv_match(type=msg_type, blocking=True, timeout=1.0) is not None:
            return True
    return False


def no_stream_message(msg_type, param_name):
    return (
        "No %s arriving.\n\n"
        "Every SR0_* stream rate on this board may be 0, which is how a board\n"
        "that has only ever run Mission Planner arrives. The tools here set\n"
        "%s automatically, so if you are seeing this the parameter write\n"
        "itself failed -- check the link, and that nothing else holds the port."
        % (msg_type, param_name)
    )


def get_param(master, name, timeout=6.0, retries=3):
    for _ in range(retries):
        master.mav.param_request_read_send(
            master.target_system, master.target_component, name.encode("ascii"), -1
        )
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
            if msg is None:
                break
            if msg.param_id.strip("\x00") == name:
                return float(msg.param_value)
    raise LinkError("timed out reading parameter %s" % name)


def set_param(master, name, value, timeout=6.0, retries=4, verify=True, tolerance=1e-4):
    """Write a parameter and read it back.

    ArduPilot acknowledges a PARAM_SET by broadcasting the resulting
    PARAM_VALUE, so a write that silently does nothing (out-of-range value,
    read-only parameter) is indistinguishable from a dropped packet unless you
    compare what came back. That comparison is the point of this function.
    """
    value = float(value)
    for _ in range(retries):
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            name.encode("ascii"),
            value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        if not verify:
            return value
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
            if msg is None:
                break
            if msg.param_id.strip("\x00") != name:
                continue
            got = float(msg.param_value)
            if abs(got - value) <= tolerance:
                return got
            raise LinkError(
                "%s rejected: asked for %g, board reports %g "
                "(out of range, or read-only on this firmware)" % (name, value, got)
            )
    raise LinkError("timed out writing parameter %s" % name)


def reboot(master, wait=True):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        1, 0, 0, 0, 0, 0, 0,
    )
    if wait:
        time.sleep(3)


def drain(master, seconds=0.5):
    deadline = time.time() + seconds
    while time.time() < deadline:
        if master.recv_match(blocking=False) is None:
            time.sleep(0.01)


def statustexts(master, seconds):
    """Yield STATUSTEXT strings for a window, for relaying board prompts."""
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = master.recv_match(type="STATUSTEXT", blocking=True, timeout=0.5)
        if msg is not None:
            yield msg.text.strip("\x00").strip()


def confirm(prompt, assume_yes=False):
    if assume_yes:
        return True
    try:
        reply = input("%s [y/N] " % prompt).strip().lower()
    except EOFError:
        return False
    return reply in ("y", "yes")


def die(msg):
    print("error: %s" % msg, file=sys.stderr)
    raise SystemExit(1)
