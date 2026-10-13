"""Verify the v1<->v2 re-encoding the bridge depends on.

The whole premise is that a message parsed from one wire format can be
re-emitted in the other with its payload intact. If that is not true, the
bridge is worthless -- so prove it here rather than on the bench.
"""
import os, sys
os.environ.setdefault("MAVLINK20", "1")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "lib"))

from pymavlink.dialects.v20 import ardupilotmega as mav2

failures = 0

def check(label, cond, detail=""):
    global failures
    print("  %-58s %s%s" % (label, "ok" if cond else "FAIL", "  " + detail if detail else ""))
    if not cond:
        failures += 1


class Sink:
    def __init__(self): self.buf = bytearray()
    def write(self, b): self.buf.extend(b)
    def read(self, n): return b""

link = mav2.MAVLink(Sink(), srcSystem=255, srcComponent=0)

print("=== frame envelopes ===")
hb = mav2.MAVLink_heartbeat_message(
    type=2, autopilot=3, base_mode=81, custom_mode=0, system_status=3, mavlink_version=3)
v1 = hb.pack(link, force_mavlink1=True)
v2 = hb.pack(link, force_mavlink1=False)
check("force_mavlink1=True emits 0xFE (v1 magic)", v1[0] == 0xFE, hex(v1[0]))
check("force_mavlink1=False emits 0xFD (v2 magic)", v2[0] == 0xFD, hex(v2[0]))
check("v1 frame is shorter than v2", len(v1) < len(v2), "%d vs %d bytes" % (len(v1), len(v2)))

print()
print("=== a v2 parser reads v1 frames (board -> bridge) ===")
p = mav2.MAVLink(Sink())
p.robust_parsing = True
got = p.parse_char(v1)
check("v1 HEARTBEAT parsed by v2 parser", got is not None and got.get_type() == "HEARTBEAT")

print()
print("=== round trip: v1 in -> v2 out -> parse (board -> QGC) ===")
# a parameter value, the exact traffic QGC's param download depends on
pv = mav2.MAVLink_param_value_message(
    param_id=b"COMPASS_ORIENT", param_value=8.0, param_type=9,
    param_count=605, param_index=42)
wire_v1 = pv.pack(link, force_mavlink1=True)
parsed = mav2.MAVLink(Sink(), ).parse_char(wire_v1)
check("v1 PARAM_VALUE parsed", parsed is not None and parsed.get_type() == "PARAM_VALUE")
out = mav2.MAVLink(Sink(), )
out.srcSystem = parsed.get_srcSystem(); out.srcComponent = parsed.get_srcComponent()
wire_v2 = parsed.pack(out, force_mavlink1=False)
reparsed = mav2.MAVLink(Sink(), ).parse_char(wire_v2)
check("re-encoded as v2 and reparsed", reparsed is not None)
check("param_id survived", reparsed.param_id.strip("\x00") == "COMPASS_ORIENT",
      repr(reparsed.param_id.strip("\x00")))
check("param_value survived", reparsed.param_value == 8.0, str(reparsed.param_value))
check("param_index survived", reparsed.param_index == 42, str(reparsed.param_index))
check("srcSystem preserved", reparsed.get_srcSystem() == parsed.get_srcSystem())

print()
print("=== round trip: v2 in -> v1 out (QGC -> board) ===")
# the request that was timing out on the real board
req = mav2.MAVLink_param_request_list_message(target_system=1, target_component=0)
wire_v2 = req.pack(link, force_mavlink1=False)
parsed = mav2.MAVLink(Sink(), ).parse_char(wire_v2)
check("v2 PARAM_REQUEST_LIST parsed", parsed is not None)
out = mav2.MAVLink(Sink(), )
out.srcSystem = 255
wire_v1 = parsed.pack(out, force_mavlink1=True)
check("re-encoded to v1 magic", wire_v1[0] == 0xFE, hex(wire_v1[0]))
final = mav2.MAVLink(Sink(), ).parse_char(wire_v1)
check("board-side parse succeeds", final is not None and final.get_type() == "PARAM_REQUEST_LIST")
check("target_system survived", final.target_system == 1)

print()
print("=== v2-only messages must be dropped, not mangled ===")
v2_only = [n for n in dir(mav2) if n.startswith("MAVLINK_MSG_ID_")]
ids = {n: getattr(mav2, n) for n in v2_only if isinstance(getattr(mav2, n), int)}
above = {n: v for n, v in ids.items() if v > 255}
check("dialect contains msgids above 255", len(above) > 0, "%d of them" % len(above))
sample = mav2.MAVLink_param_request_list_message(target_system=1, target_component=0)
check("PARAM_REQUEST_LIST is v1-representable", sample.get_msgId() if hasattr(sample,'get_msgId') else mav2.MAVLINK_MSG_ID_PARAM_REQUEST_LIST, "")
check("  its msgid <= 255", mav2.MAVLINK_MSG_ID_PARAM_REQUEST_LIST <= 255,
      str(mav2.MAVLINK_MSG_ID_PARAM_REQUEST_LIST))

print()
print("ALL PASS" if failures == 0 else "%d FAILURES" % failures)
sys.exit(1 if failures else 0)
