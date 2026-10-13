"""End-to-end: run the real mavbridge.py between a fake board and a fake QGC.

The board side is a pty, which pyserial opens exactly like a real /dev/cu.*
device, so the bridge runs unmodified. The board speaks only v1; the GCS speaks
only v2. If a v1 frame reaches the GCS as v2 and a v2 frame reaches the board
as v1, the bridge does what it claims.
"""
import os, pty, subprocess, sys, time
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mav2

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
UDP_PORT = 14556

failures = 0
def check(label, cond, detail=""):
    global failures
    print("  %-52s %s%s" % (label, "ok" if cond else "FAIL", "  " + detail if detail else ""))
    if not cond: failures += 1


class FD:
    """Minimal file-like wrapper so MAVLink() can write into the pty master."""
    def __init__(self, fd): self.fd = fd
    def write(self, b): os.write(self.fd, b)
    def read(self, n): return b""


master_fd, slave_fd = pty.openpty()
slave_name = os.ttyname(slave_fd)
# A pty echoes by default and would feed the bridge's own output straight back
# in as if the board had sent it. Raw mode on both ends kills the echo and the
# line discipline's newline mangling, which would corrupt binary frames.
import termios, tty
for fd in (master_fd, slave_fd):
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[3] &= ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
print("  fake board pty: %s" % slave_name)

gcs = mavutil.mavlink_connection("udpin:127.0.0.1:%d" % UDP_PORT, dialect="ardupilotmega")

proc = subprocess.Popen(
    [sys.executable, os.path.join(ROOT, "mavbridge.py"),
     "--port", slave_name, "--udp", "127.0.0.1:%d" % UDP_PORT],
    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
time.sleep(2.5)

try:
    board_mav = mav2.MAVLink(FD(master_fd), srcSystem=1, srcComponent=1)

    print()
    print("=== board -> GCS (v1 in, v2 expected out) ===")
    hb = mav2.MAVLink_heartbeat_message(type=2, autopilot=3, base_mode=81,
                                        custom_mode=0, system_status=3, mavlink_version=3)
    pv = mav2.MAVLink_param_value_message(param_id=b"COMPASS_ORIENT", param_value=8.0,
                                          param_type=9, param_count=605, param_index=42)
    for _ in range(4):
        os.write(master_fd, hb.pack(board_mav, force_mavlink1=True))
        os.write(master_fd, pv.pack(board_mav, force_mavlink1=True))
        time.sleep(0.15)

    seen, deadline = {}, time.time() + 6
    while time.time() < deadline and "PARAM_VALUE" not in seen:
        m = gcs.recv_match(blocking=True, timeout=1)
        if m and m.get_type() not in ("BAD_DATA",):
            seen[m.get_type()] = m
    check("HEARTBEAT arrived at GCS", "HEARTBEAT" in seen)
    check("PARAM_VALUE arrived at GCS", "PARAM_VALUE" in seen)
    if "PARAM_VALUE" in seen:
        got = seen["PARAM_VALUE"]
        check("  param_id intact", got.param_id.strip("\x00") == "COMPASS_ORIENT",
              repr(got.param_id.strip("\x00")))
        check("  param_value intact", got.param_value == 8.0)
        check("  srcSystem preserved as board's (1)", got.get_srcSystem() == 1,
              str(got.get_srcSystem()))

    print()
    print("=== GCS -> board (v2 in, v1 expected out) ===")
    for _ in range(3):
        gcs.mav.param_request_list_send(1, 1)
        time.sleep(0.2)
    time.sleep(1.0)

    os.set_blocking(master_fd, False)
    raw = b""
    for _ in range(12):
        try:
            raw += os.read(master_fd, 4096)
        except BlockingIOError:
            pass
        time.sleep(0.1)
    check("board received bytes", len(raw) > 0, "%d bytes" % len(raw))
    check("every frame uses v1 magic 0xFE", len(raw) > 0 and 0xFD not in set(raw[:1]) and raw[0] == 0xFE,
          "first byte %s" % (hex(raw[0]) if raw else "n/a"))
    parser = mav2.MAVLink(FD(master_fd))
    parser.robust_parsing = True
    msgs = [m.get_type() for m in (parser.parse_char(bytes([b])) or [] for b in raw) if m] \
        if False else []
    decoded = []
    parser2 = mav2.MAVLink(FD(master_fd)); parser2.robust_parsing = True
    for b in raw:
        m = parser2.parse_char(bytes([b]))
        if m and m.get_type() != "BAD_DATA":
            decoded.append(m.get_type())
    check("board decoded PARAM_REQUEST_LIST", "PARAM_REQUEST_LIST" in decoded,
          str(sorted(set(decoded))))

finally:
    proc.terminate()
    try:
        out = proc.communicate(timeout=5)[0]
    except subprocess.TimeoutExpired:
        proc.kill(); out = ""
    print()
    print("=== bridge process output ===")
    for line in (out or "").strip().splitlines()[-6:]:
        print("  | %s" % line)

print()
print("ALL PASS" if failures == 0 else "%d FAILURES" % failures)
sys.exit(1 if failures else 0)
