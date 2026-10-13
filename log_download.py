#!/usr/bin/env python3
"""List and download dataflash logs from the APM 2.8 over USB.

    ./log_download.py --list          # what is on the board
    ./log_download.py                 # download the newest log
    ./log_download.py --id 46         # download one by number

Why this exists
---------------
The dataflash log is the only record of what the STICKS did and what the
AIRCRAFT did at the same instant. Nothing else can separate "the pilot was
commanding a roll" from "the aircraft rolled on its own", and a pilot in the
middle of a hover genuinely cannot tell those apart by feel. `analyze_hover.py`
reads what this writes.

MAVProxy can do this from its interactive prompt; there was no scriptable way
to get a log off the board, which is why `logs/dataflash/` had exactly one file
in it and no way to make another.

Two things that make a naive implementation appear to hang
----------------------------------------------------------
- The board sends the log as LOG_DATA packets of 90 bytes each. A 600 KB log is
  ~6800 packets at 115200 baud with the telemetry streams still running and
  competing for the same link. Turn the streams OFF first -- it roughly doubles
  the throughput and, more importantly, stops the download from stalling behind
  a burst of RAW_IMU.
- Blocks go missing. ArduPilot does not retransmit on its own; the ground side
  is expected to notice a hole and re-request that offset. A loop that only
  waits for the next packet therefore waits forever on a log that is 99%
  complete. This tracks received offsets and re-requests the first gap whenever
  the link goes quiet.
"""

import argparse
import os
import sys
import time

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

BLOCK = 90
HERE = os.path.dirname(os.path.abspath(__file__))
OUTDIR = os.path.join(HERE, "logs", "dataflash")


def quiet_streams(master):
    """Stop every telemetry stream so the log gets the whole link."""
    master.mav.request_data_stream_send(
        master.target_system, master.target_component, 0, 0, 0)
    apmlink.drain(master, 0.5)


def restore_param_stream(master):
    """Undo the one stream rate that does lasting damage when left at 0.

    ArduPilot PERSISTS stream rates, and SR0_PARAMS is what feeds the reply to
    PARAM_REQUEST_LIST. Leaving it at 0 means every later parameter backup comes
    back empty -- with no error, because the board simply says nothing. Found
    2026-09-23 after two backups in a row silently captured 0 of 306 parameters.

    The other SR0_* rates heal themselves: every tool here calls ensure_stream
    for the streams it needs. Nothing ever re-requests the parameter stream.
    """
    try:
        apmlink.set_param(master, "SR0_PARAMS", 10, verify=False)
    except apmlink.LinkError:
        print("  note: could not restore SR0_PARAMS; run ./preload_params.py --backup")
        print("        and check it captures 306 parameters, not 0.")


def list_logs(master, attempts=5):
    """Return (entries, answered).

    `answered` distinguishes "the board said it has no logs" from "the board
    never replied". Collapsing those into an empty list sends you looking for
    an erased log when the real problem is that nothing was listening yet --
    opening the serial port reboots this board, and it does not answer a log
    list request during its init.
    """
    entries = {}
    answered = False
    for attempt in range(attempts):
        master.mav.log_request_list_send(
            master.target_system, master.target_component, 0, 0xFFFF)
        deadline = time.time() + 4.0
        while time.time() < deadline:
            m = master.recv_match(type="LOG_ENTRY", blocking=True, timeout=1.0)
            if m is None:
                continue
            answered = True
            if m.num_logs == 0:
                return [], True
            entries[m.id] = m
            if len(entries) >= m.num_logs:
                return [entries[k] for k in sorted(entries)], True
            deadline = time.time() + 3.0
        if entries:
            break
        if attempt + 1 < attempts:
            print("    no reply, retrying (%d/%d) ..." % (attempt + 2, attempts))
    return [entries[k] for k in sorted(entries)], answered


def download(master, entry, path):
    size = entry.size
    got = {}
    received = 0
    last_progress = time.time()
    next_report = 0

    def request_from(ofs):
        master.mav.log_request_data_send(
            master.target_system, master.target_component,
            entry.id, ofs, size - ofs)

    request_from(0)
    while received < size:
        m = master.recv_match(type="LOG_DATA", blocking=True, timeout=1.0)
        if m is not None and m.id == entry.id:
            if m.ofs not in got:
                got[m.ofs] = bytes(bytearray(m.data[:m.count]))
                received += m.count
                last_progress = time.time()
            pct = 100.0 * received / size
            if pct >= next_report:
                sys.stdout.write("\r  %6.1f%%  %d / %d bytes" % (pct, received, size))
                sys.stdout.flush()
                next_report = pct + 2
            continue

        # Quiet link. Either we are done, or there is a hole -- find the first
        # missing offset and ask again from there. Asking from 0 would restart
        # a nearly finished download.
        if time.time() - last_progress < 1.5:
            continue
        gap = None
        for ofs in range(0, size, BLOCK):
            if ofs not in got:
                gap = ofs
                break
        if gap is None:
            break
        request_from(gap)
        last_progress = time.time()

    master.mav.log_request_end_send(master.target_system, master.target_component)
    sys.stdout.write("\r  %6.1f%%  %d / %d bytes\n" % (
        100.0 * received / size, received, size))

    blob = bytearray()
    holes = 0
    for ofs in range(0, size, BLOCK):
        chunk = got.get(ofs)
        if chunk is None:
            holes += 1
            chunk = b"\x00" * min(BLOCK, size - ofs)
        blob += chunk
    with open(path, "wb") as fh:
        fh.write(bytes(blob[:size]))
    return holes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=apmlink.DEFAULT_BAUD)
    ap.add_argument("--list", action="store_true", help="list logs and exit")
    ap.add_argument("--id", type=int, help="log number to download")
    args = ap.parse_args()

    try:
        master = apmlink.connect(args.port, args.baud)
    except apmlink.LinkError as exc:
        apmlink.die(exc)

    quiet_streams(master)
    # Opening the port reboots the board. It ignores a log list request while
    # it is still initialising, exactly as it refuses a motor test for the same
    # window, so settle first rather than reading silence as "no logs".
    print()
    sys.stdout.write("  waiting 18s for the board to settle after the reboot ")
    for _ in range(18):
        time.sleep(1)
        sys.stdout.write(".")
        sys.stdout.flush()
    print(" ready")
    print()
    print("  asking the board for its log list ...")
    entries, answered = list_logs(master)
    if not entries:
        if answered:
            apmlink.die("the board replied: it has no logs. ArduCopter only "
                        "opens a log at ARMING, and LOG_BITMASK must be "
                        "non-zero at that moment.")
        apmlink.die("the board never answered the log list request. It is "
                    "alive (heartbeat is flowing), so this is the log "
                    "subsystem, not the link -- try a power-cycle.")

    print()
    print("  %-5s %12s" % ("log", "bytes"))
    for e in entries:
        print("  %-5d %12d" % (e.id, e.size))
    print()
    if args.list:
        restore_param_stream(master)
        return 0

    wanted = args.id if args.id is not None else max(e.id for e in entries)
    match = [e for e in entries if e.id == wanted]
    if not match:
        apmlink.die("no log numbered %d on the board" % wanted)
    entry = match[0]
    if entry.size == 0:
        apmlink.die("log %d is empty" % entry.id)

    if not os.path.isdir(OUTDIR):
        os.makedirs(OUTDIR)
    path = os.path.join(OUTDIR, "log%d.bin" % entry.id)

    print("  downloading log %d (%d bytes) -> %s" % (entry.id, entry.size, path))
    print("  expect a few minutes at 115200; streams are off for the duration.")
    print()
    holes = download(master, entry, path)
    print()
    if holes:
        print("  WARNING: %d block(s) never arrived and are zero-filled." % holes)
        print("  Re-run to try again; a partly zeroed log still parses but the")
        print("  gaps are not distinguishable from real data.")
    else:
        print("  complete, no missing blocks.")
    restore_param_stream(master)
    print()
    print("  next:  ./analyze_hover.py %s" % os.path.relpath(path, HERE))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
