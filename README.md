# APM 2.8 toolkit (macOS)

Bring up an APM 2.x (ATmega2560) flight controller from macOS: flash it,
configure it, and calibrate it, without Mission Planner.

**Nothing here touches the PX4/Gazebo setup in `~/claude-setup-sitl-tooling`.**
Different autopilot, different architecture — the APM runs ArduPilot and cannot
run PX4 at all.

## Propellers off

Take the props off before plugging in the board. Outputs can twitch while the
bootloader runs and again when fresh firmware boots. Do firmware work on USB
power with the flight battery unplugged. The compass and orientation steps
involve picking the aircraft up and turning it over — doubly so there.

## Quick start

```bash
./setup_env.sh              # one-time: Python venv for the MAVLink tools
source .venv/bin/activate
./identify.py               # read-only: what is on the board, what it needs
./bringup.sh --frame quad   # guided, step-by-step, every step skippable
```

Start with `./identify.py`. It writes nothing, and its checklist tells you
which of the steps below you actually need — including which frame firmware is
already loaded, so you do not have to guess.

Or run the pieces individually:

```bash
./fetch_firmware.sh quad
./flash_apm28.sh --frame quad

source .venv/bin/activate
./identify.py
./preload_params.py --backup
./preload_params.py --factory-reset
./calibrate_accel.py
./solve_compass_orient.py --write
./calibrate_compass.py --write
./calibrate_radio.py --write
./preload_params.py base frame-x compass-external power-module failsafe-3s
```

## The firmware ceiling

**ArduCopter 3.2.1, built 11 February 2015.** That is the last release that
fits in the ATmega2560's 256 KB of flash — not merely the newest in the
archive. Anything later is Pixhawk-class only. This build predates EKF2/EKF3
and uses the older DCM/InertialNav estimator.

Fetched from `firmware.ardupilot.org/Copter/stable-3.2.1/apm2-<frame>/`.
Builds are per airframe, so the frame type is part of the URL — flashing the
hexa build onto a quad gives you a very expensive paperweight.

Known good SHA-256:

    efaa7aa53a6dd493e7c07625c11fedde654cf14f0ae23af66faa98383718c2ba  quad
    2529d838c6e030beb7dceb88c11ef0154d4cb15458129db363b99addab4686b6  hexa

That ceiling drives everything else here. Every tool in this directory speaks
MAVLink 1 and the 2015 parameter protocol, and deliberately avoids anything
newer: no `COMMAND_INT`, no `MAV_CMD_SET_MESSAGE_INTERVAL`, no ellipsoid
compass model. The board does not implement them.

## What each tool does

| Tool | What it does |
|---|---|
| `identify.py` | Read-only: firmware, parameters, and what still needs doing |
| `fetch_firmware.sh` | Download and validate the 3.2.1 hex for a frame |
| `flash_apm28.sh` | avrdude flash over USB, with port auto-detect |
| `preload_params.py` | Apply/back up/restore parameter profiles, verified |
| `calibrate_accel.py` | Relay 3.2.1's six-position accel routine |
| `solve_compass_orient.py` | **Measure** COMPASS_ORIENT instead of guessing |
| `calibrate_compass.py` | Hard-iron offsets by sphere fit, with a quality score |
| `calibrate_radio.py` | RCn_MIN/MAX/TRIM from live stick travel |
| `level_calibrate.py` | Set the level reference by measurement, with verification |
| `preflight.py` | **Everything at once**: GPS, battery, compass, RC, modes, IMU. Blockers vs warnings |
| `rc_live.py` | All eight RC channels live, with travel ranges |
| `mode_monitor.py` | Live: mode-switch PWM, the slot it selects, and the mode |
| `gps_check.py` | Live GPS: fix type, satellites, HDOP, position wander, with a verdict |
| `which_compass.py` | Which magnetometer is actually being read (settles the jumper question) |
| `motor_test.py` | Spin each motor individually: wiring order and rotation |
| `check_imu.py` | Gyro health and axis-direction check, no battery needed |
| `gcs.sh` | Launch MAVProxy as the ground station (recommended GUI-free GCS) |
| `mavbridge.py` | MAVLink v1<->v2 proxy so QGroundControl can talk to the board |
| `bringup.sh` | All of the above, in the order that matters |

### Why the order matters

Accel before compass: the compass fit is interpreted in the body frame the
accelerometers define. `COMPASS_ORIENT` before `COMPASS_OFS`: changing the
orientation permutes the axes the offsets were measured in, so offsets taken
under the old value are wrong. Radio before failsafes: `FS_THR_VALUE` is only
meaningful once `RC3_MIN` is known.

## The compass, in detail

This is where APM 2.8 builds usually go wrong, so it gets its own section.

**Step 1 — the hardware mod.** The APM 2.8 carries an onboard magnetometer,
connected to the same I2C bus as the external one on a GPS mast. There is a
solder jumper next to the I2C port that must have its trace **cut** to
disconnect the internal compass. Setting `COMPASS_EXTERNAL 1` in software does
not do this. Skip the cut and two magnetometers argue on one bus, which reads
as heading that drifts, wanders, or circles — the classic "toilet bowl".

**Step 2 — orientation, measured not guessed.** Nearly every guide says to try
`ROTATION_ROLL_180` and see whether it flies straight. `solve_compass_orient.py`
determines it instead. A world-fixed vector seen from a rotating body obeys
`dm/dt = -ω × m`. That constraint holds for the correctly rotated magnetometer
and for no other rotation in the table, so the tool records gyro and mag while
you wave the airframe around and finds the rotation whose residual collapses.
Hard-iron offset is solved for in the same least-squares fit, so no prior
calibration is needed.

It reports a confidence margin and per-axis rotation coverage, and refuses to
write a value it cannot separate from the runner-up — a yaw-only wave cannot
distinguish rotations that differ in roll, and the tool says so rather than
returning a confident wrong answer.

Verified against synthetic captures: all 34 rotations recovered, hard-iron
recovered to ~1 mGauss, and yaw-only runs correctly reported as inconclusive
(`python3 tests/synth_test.py`).

**Step 3 — offsets.** `calibrate_compass.py` fits a sphere in closed form to
the whole sample cloud and reports the scatter and how much of the sphere you
actually covered. 3.2.1's own live calibration is a min/max tracker that takes
whatever extremes it happens to see; it gives no indication when the result is
junk. Offsets over 600 fail the firmware's pre-arm check, and that usually
means the compass is mounted too near power wiring.

Only hard-iron (`COMPASS_OFS_*`) is written — the soft-iron terms
(`COMPASS_DIA`, `COMPASS_ODI`) arrived in 3.4 and do not exist on this
firmware, so a sphere rather than an ellipsoid is the correct model.

## Parameter profiles

`profiles/*.params` are plain `NAME VALUE` files in Mission Planner's format,
so anything here loads on a Windows box later.

```bash
./preload_params.py --list
./preload_params.py --dry-run base frame-x    # shows current vs proposed
./preload_params.py --backup                  # full dump to logs/
```

Every write is confirmed by reading back the `PARAM_VALUE` the board
broadcasts. ArduPilot silently clamps or ignores out-of-range and read-only
writes, so an unverified write is not a write you can claim happened.

**`--factory-reset` after flashing.** avrdude writes flash only, so parameters
from whatever firmware shipped on the board survive a firmware change, and the
new firmware will happily fly on stale settings. Zeroing `FORMAT_VERSION`
makes ArduPilot treat the EEPROM as uninitialised and rewrite defaults.

## Ground station options

None of them are good on Apple Silicon, and none of them are needed for the
steps above.

- **MAVProxy** (`./gcs.sh`) — the recommended one, and the only option here
  that installs cleanly. It speaks MAVLink v1 natively, is pure Python, and
  goes into the venv via pip: no app bundle, no Gatekeeper prompt, no Rosetta,
  no unsigned Intel binary. It is also ArduPilot's own reference
  implementation for the interactive calibration handshakes.
- **Mission Planner** (Windows, or a Windows 11 ARM VM) — the most complete
  APM 2.x support: terminal, setup wizard, log download.
- **APM Planner 2** — has macOS builds and does speak v1, but is unmaintained,
  Intel-only under Rosetta, and unsigned, so Gatekeeper blocks it and clearing
  the quarantine attribute is a manual step. Only worth the trouble if you
  specifically want its GUI.
- **QGroundControl** — needs `mavbridge.py` to work at all. See below.

### QGroundControl and the MAVLink version wall

Tested against a live APM 2.8 running 3.2.1. Connected directly, QGC:

- detects the vehicle and streams telemetry
- then fails parameter download, mission download, geofence transfer and rally
  point transfer, each with "maximum retries exceeded"
- ends up showing only Summary and Firmware, with no Sensors, Radio or Flight
  Modes tabs — so no calibration UI at all

The cause is not a setting. QGC *parses* MAVLink v1 fine, which is why the
vehicle appears. What it will not do is *emit* v1 — every packet it sends
starts with the v2 magic byte `0xFD`, and ArduCopter 3.2.1 predates MAVLink v2
entirely, so its parser sees an unrecognised start byte and discards the frame.
The conversation is one-way: the board talks, QGC listens, and nothing QGC says
is ever heard.

`mavbridge.py` fixes exactly that, by re-encoding the frame envelope in each
direction:

```bash
./mavbridge.py                # leave this running
```

Then in QGC, **disconnect the serial link** (two processes cannot own one
serial port) and let it pick up the UDP link on 127.0.0.1:14550.

Messages with IDs above 255 have no v1 encoding at all; the bridge drops those
toward the board and reports which ones, rather than mangling them.

Verified by `tests/bridge_test.py` (frame envelopes and payload round-trips)
and `tests/bridge_e2e.py`, which runs the real bridge between a fake board on a
pty and a fake GCS on UDP.

Even with the bridge, expect the ArduPilot-specific screens to be uneven: QGC's
compass calibration drives the onboard `MAG_CAL` routine that arrived in
ArduPilot 3.3, which this firmware does not have. Parameter editing and radio
calibration are the parts most likely to work. The scripts here do not depend
on any of it.

## If every sensor looks dead at once

`no GPS_RAW_INT`, `no SYS_STATUS`, `no RAW_IMU`, `no RC_CHANNELS_RAW` and "no RC
input at all", all together, while parameters read back perfectly — that is one
fault, not five.

ArduCopter 3.2.1 samples the throttle channel at startup. If it reads high the
board prints `Entering ESC Cal: restart APM.` and **stops before its main loop**,
so it never produces a single telemetry stream. Heartbeats and parameters keep
working the whole time, because both are serviced from the delay callback — which
is exactly why it looks like a board full of dead sensors.

**Propellers off.** While halted it drives every ESC at the throttle value it
saw; this board was logging `output: 1996 1996 1996 1996`, full throttle on all
four. With a battery connected the motors run as soon as they are able.

To clear it: **transmitter ON with the throttle stick fully DOWN** (or reconnect
the receiver), then power-cycle the board. The `ESC` parameter — named `ESC`, not
`ESC_CALIBRATION` — reads 0 throughout and is not the trigger.

On this airframe the cause was the FS-iA6B holding 1996 us on channel 3: the
receiver holds its last output when the transmitter is off unless failsafe is
configured on ch3 below 975. See `FSI6.md` — and test the failsafe, because an
untested one is what caused this.

`identify.py` and `preflight.py` both detect and report this state directly.
Detecting it is harder than it looks: the notice is plain serial text emitted
before MAVLink starts (so it arrives as `BAD_DATA`, never `STATUSTEXT`), and
opening `/dev/cu.*` does not assert DTR, so the board often is not reset on
connect and never reprints it. Text detection alone is a coin flip, so
`apmlink.telemetry_alive()` tests the behaviour instead: request every stream,
and if nothing arrives the main loop is not running.

## Streams that never start

`SR0_RAW_SENS` reads time out on this board, essentially always. `ensure_stream`
used to set the SR parameter only when it could first read it, so the read
failure meant the parameter stayed at 0 — and with it at 0 the stream never
flows however many `REQUEST_DATA_STREAM` messages are sent. Tools reported
"board not streaming raw sensors" on a perfectly healthy board.

It now writes the parameter blind when the read fails. If you add a tool and it
cannot get RAW_IMU, check that first.

## Calibrating without a telemetry radio

Compass calibration needs the aircraft inverted, and the mini-USB jack works
loose when it is — three disconnects in one session. Before starting:

- **Tape the USB cable to the frame** with a service loop.
- **Turn one way, then back the other way.** Continuous spinning tangles the
  cable; the magnetometer only cares which directions it has pointed, so
  reversing is equivalent and unwinds the lead.
- Zeroed offsets are a liability: capture requires them at 0, so save the
  originals first and restore on every failure path. `preflight.py` will tell
  you if they are left at zero.

## If the board is not detected

`./flash_apm28.sh --list-ports` shows nothing:

1. **Cable.** Many mini-USB cables are charge-only. This is the usual cause.
2. **Plug straight into the Mac**, not through a hub or dock.
3. `ls /dev/cu.*` to see everything macOS enumerated.

A genuine APM 2.8 uses an ATmega16U2 USB bridge and appears as
`/dev/cu.usbmodem*` with no driver needed. Clones with CP2102 show as
`/dev/cu.usbserial*` (also driverless on macOS 11+); CH340 clones show as
`/dev/cu.wchusbserial*` and need WCH's DriverKit driver — the old kext will
not load on Apple Silicon.

Use the `cu.*` node, never `tty.*`: on macOS the `tty.*` node blocks on open
waiting for carrier detect, which a USB CDC board never asserts, so avrdude
appears to hang.

## Not covered here

Deliberately not automated, because scripting them is not safe:

- **ESC calibration** — props off, battery in, throttle high at power-up.
  3.2.1 has no parameter-driven path for this.
- **Flight mode assignment** (`FLTMODE1..6`) — depends on your transmitter.
- **PID tuning** — airframe-specific; the profiles here ship stock gains.
