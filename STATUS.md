# Build status — APM 2.8 / F450 quad

Updated 2026-09-23, after two flights that were hard to fly -- see "Why it is
hard to fly, 2026-09-23" below, which is the section to read first. Before that,
2026-09-07, the compass and GPS found switched OFF in software; and 2026-09-02,
the frame rebuild, ESC calibration and the re-check that followed. "Verified"
means measured on this aircraft, not assumed.

**The airframe is not the one most of this file was written against.** The
front-left arm broke 2026-08-30 and the whole frame was replaced. Anything
depending on frame geometry -- level reference, motor wiring, compass hard iron
-- had to be re-measured on the new frame, not inherited. The rows below say
which readings are post-rebuild.

## Verified

| Item | Evidence |
|---|---|
| Firmware present, boots clean | `identify.py` — SYSID_SW_MREV 120, GROUND START, "Ready to FLY" |
| GPS working | boot banner reports `ublox` |
| Frame parameter | `FRAME 1` (X) |
| Accelerometers calibrated | `INS_ACCSCAL` 0.993 / 0.991 / 1.034, offsets non-zero — not factory defaults |
| Gyro offsets sane | `INS_GYROFFS` all < 0.016 rad/s |
| Radio ch 1–4 calibrated | 1088–1092 min, 1925 max; passes the pre-arm RC check |
| Motor output order | `motor_test.py` — tests 1/2/3/4 spun front-R, rear-R, rear-L, front-L |
| Motor directions | motors 1 & 2 CCW, 3 & 4 CW, after rewiring |
| Ground station | MAVProxy via `./gcs.sh` |
| Parameter backup | `logs/params-20260818-220655.params` (306 params) |
| Gyro health | `check_imu.py` — bias 0.0002 rad/s, all three axes live and quiet |
| Base/failsafe/battery params | 32 applied and read back verified |
| Motor order and direction, NEW frame | `motor_test.py --throttle 15` 2026-09-02 — all four arms and directions matched |
| ESC calibration | done 2026-09-02, all-at-once throttle-high method; halt cleared, all streams flowing |
| Level reference, NEW frame | `level_calibrate.py` 2026-09-02 — measured roll +0.08, pitch -0.11 deg, inside the 0.5 deg threshold, nothing written |
| Stick centres | `preflight.py --quick` — ch1 1507 vs `RC1_TRIM` 1510, ch2 1481 vs `RC2_TRIM` 1484, both inside DZ 30 |
| Stick SENSE | `check_stick_sense.py` 2026-09-02 — pitch was reversed, fixed, re-measured correct |
| Aircraft is trimmed | log52: 532 s airborne, mean commanded roll -0.33, pitch -0.01 deg |
| Attitude tracking | log52 mean error +0.04 / +0.05 deg — no weak motor, no damaged prop |
| Hover throttle | CTUN.ThrOut 445-485 across two flights vs `THR_MID` 500 — close enough, left alone |
| GPS receiver alive | 2026-09-07 — with `GPS_TYPE` restored to 1 and rebooted, fix went 0 (NO_GPS) to 1 (NO_FIX) and `SYS_STATUS` reports GPS present and healthy. Indoors, so no satellites; the receiver itself answers |
| GPS fix, outdoors | 2026-09-07 — `gps_check.py --outdoors`: 3D fix, 11 satellites, HDOP 0.87. Meets the TARGETS.md gates (10+ sats, HDOP under 1.5); position modes have the position half of what they need |
| Compass alive | 2026-09-07 — with `MAG_ENABLE` restored to 1 and rebooted, `RAW_IMU` reads X -37 Y -110 Z -323, magnitude 344, steady to 3 counts; `SYS_STATUS` 3D_MAG present and healthy |

## Outstanding, in order

0. **THE PROPS, and the pack.** Logs 70 and 71 (2026-09-23) show the two CW
   motors running 129-180 us harder than the two CCW ones and touching the
   mixer ceiling at 1928 us. Compare all four props side by side before
   anything else, then fly a healthy 3S. Full evidence in "Why it is hard to
   fly, 2026-09-23". Everything below is secondary to this.

1. ~~**Propeller direction**~~ — DONE 2026-09-02. `motor_test.py` on the new
   frame confirmed all four arms and directions: motor 1 front-right CCW,
   4 rear-right CW, 2 rear-left CCW, 3 front-left CW. Motor 3 is the arm that
   broke; it came back on the front-left turning CW as it should.

2. ~~**ESC calibration**~~ — DONE 2026-09-02, all-at-once method: props off,
   USB out, transmitter on with throttle up, battery in, out, in, then throttle
   down. Power-cycled with the throttle DOWN afterwards to clear the halt;
   `preflight.py --quick` then showed GPS, battery, RC and IMU all streaming,
   which is the behavioural proof the main loop is running again.

3. **Compass** — `COMPASS_EXTERNAL 0`, `COMPASS_ORIENT 0`, and as of
   2026-09-07 `COMPASS_OFS_X/Y/Z` are **all zero**: the old frame's offsets
   (-217 / +26 / +63) were wiped some time after the 09-06 backup, so the
   aircraft is now uncalibrated rather than calibrated-for-the-old-frame. The
   hardware itself is fine (`MAG_ENABLE` restored, mag reading live). Run
   `./which_compass.py` first: it establishes whether the internal compass is
   actually disconnected. On APM 2.8 that is a SOLDER BRIDGE to cut, not the
   removable JP1 shunt (JP1 is the servo rail power link). Then
   `solve_compass_orient.py --write`, then `calibrate_compass.py --write`
   OUTDOORS, assembled, battery in. Afterwards put `ARMING_CHECK` back to 1 and
   `FS_GPS_ENABLE` back to 1.

4. ~~**Flight modes**~~ — DONE. SwC drives channel 5, the profile is applied,
   and `preflight.py --quick` on 2026-09-02 read FLTMODE1/2 STABILIZE, 3/4
   ALT_HOLD, 5/6 LAND with ch5 at 998 (slot 1). Original note: all six
   positions were STABILIZE and CH5 sat static at 1556, which readSwitch() maps
   to slot 4 -- LOCKED to FLTMODE4 rather than merely unable to switch.

5. ~~**Failsafes and battery**~~ — DONE. `FS_THR_ENABLE 1`, `FS_BATT_ENABLE 2`
   (RTL), `FS_GCS_ENABLE 0`, `BATT_MONITOR 4`, `ANGLE_MAX` 45->30 deg,
   `INS_MPU6K_FILTER` 42->20 Hz. Reboot applied.

   Two follow-ups this created:

   a. **Verify battery voltage against a multimeter.** `BATT_VOLT_MULT 10.1` is
      the generic APM2 default and clone power modules vary. This used to be
      cosmetic; with `FS_BATT_ENABLE 2` it is not. A multiplier reading low
      triggers an RTL in flight for no reason; reading high means the failsafe
      never fires and you land on a flat pack.

   b. **Configure receiver failsafe on the transmitter.** `FS_THR_ENABLE 1` on
      the flight controller does nothing by itself -- the FS-iA6B holds its
      last position on signal loss. FS-i6: RX Setup -> Failsafe, set channel 3
      below 975. Test it: transmitter off, watch RC3 actually fall in MAVProxy.

6. **IMU direction check** — gyro health PASSED. The three motion/direction
   tests still need running (`./check_imu.py`), now that it waits for movement
   instead of racing a fixed window.

7. **Frame geometry** — confirm the FORWARD arrow bisects the angle between two
   arms. `FRAME 1` is X; if the arrow points straight down an arm the airframe
   is physically a Plus and every correction is mixed 45 degrees off.

8. **Tethered arm test** before anything leaves the ground.

## The sensors that were switched off in software — 2026-09-07

Symptom: no compass and no GPS at all. `RAW_IMU` mag was exactly `(0,0,0)` over
615 samples across several boots, `GPS_RAW_INT` reported `fix_type 0`, and
`SYS_STATUS` said **3D_MAG present: False** and **GPS present: False**. Read
literally, that is "no magnetometer and no GPS receiver are attached to this
board" — and it looked like hardware, because the internal compass had measured
fine on 2026-08-22 and the frame had been rebuilt in between.

It was two parameters:

| Parameter | Was | Restored to | Effect while off |
|---|---|---|---|
| `MAG_ENABLE` | 0 | 1 | the compass driver never initialises, so mag stays 0 and 3D_MAG never appears |
| `GPS_TYPE` | 0 | 1 | the GPS driver never opens the port, so the receiver is reported as absent, not merely unfixed |

Both devices answered immediately after a reboot with the parameters restored.

Three things worth keeping:

- **`MAG_ENABLE` is the compass on/off switch on 3.2.1, not `COMPASS_USE`.**
  `MAG_ENABLE` (`g.compass_enabled`) decides whether `init_compass()` runs at
  all; `COMPASS_USE` only decides whether AHRS uses the heading. Setting
  `COMPASS_USE 1` on a board with `MAG_ENABLE 0` changes nothing and reads back
  perfectly, which is exactly how an hour gets spent on the wrong parameter.
- **`fix_type 0` and `fix_type 1` mean different things.** 0 is NO_GPS: the
  driver has not detected a receiver. 1 is NO_FIX: the receiver is talking and
  cannot see satellites. Indoors you should see 1. Seeing 0 indoors is a wiring
  or configuration fault, not "no sky", and the `SYS_STATUS` GPS present bit
  says the same thing more plainly.
- **Diff the parameters before suspecting the hardware.** Comparing the board
  against `logs/params-20260906-202736.params` found it in seconds. No tool in
  this toolkit writes `MAG_ENABLE`, `GPS_TYPE` or `ARMING_CHECK 122`, so these
  came from a GUI session (`GPS_TYPE 1->0` between 08-30 and 09-06;
  `MAG_ENABLE 1->0`, `COMPASS_OFS` zeroed and `ARMING_CHECK 1->122` after the
  09-06 20:27 backup). `./preload_params.py --backup` after every session is
  what makes that diff possible. Current state:
  `logs/params-20260907-102544.params`.

Still open from the same session: `FS_GPS_ENABLE` is 0 and `ARMING_CHECK` is
122 (the compass bit cleared), both consistent with someone configuring the
aircraft to fly without compass or GPS. `TRIM_THROTTLE` and `POSCON_THR_HOVER`
have drifted to 367 from 585, against a measured hover of 445-485 — learned in
flight, harmless in STABILIZE, slightly soft in ALT_HOLD.

## Why it is hard to fly — 2026-09-23

Two flights, logs 70 and 71, both reported as "not stable enough to easily
fly". The cause is not tuning and not vibration. Measured, from the logs:

| Checked | Reading | Verdict |
|---|---|---|
| Attitude tracking | `ErrRP` mean 0.035 (log71) / 0.054 (log70), roll+pitch error under 0.5 deg | the controller is doing its job |
| Vibration | `AccX` sd 0.73, `AccY` sd 0.66, `AccZ` sd 1.30, AccZ 0.5-99.5% within -13.1..-7.0 | well inside limits |
| PIDs | stock 3.2.1: `STB_*_P` 4.5, `RATE_RLL/PIT_P` 0.15, I 0.10, D 0.004 | untouched, and not the problem |
| Stick centring | ch1 1483 vs `RC1_TRIM` 1510, ch2 1438 vs 1484, both inside DZ 30 | fine |

### The finding: thrust asymmetry along the CW diagonal

Mean motor output while airborne:

| Motor | log 70 | log 71 |
|---|---|---|
| 1 front-right CCW | 1505 | 1420 |
| 2 rear-left CCW | 1540 | 1456 |
| 3 front-left CW | 1668 | 1550 |
| 4 rear-right CW | 1737 | 1583 |
| **CCW pair minus CW pair** | **-180 us** | **-129 us** |

Same ordering in both flights. Left vs right is balanced (-1.5 and +17 us), so
lateral CG is fine; the split follows ROTATION DIRECTION, not position, which
is what makes it a propeller question rather than a weight or wiring one.

**Why that costs stability:** motor 4 averaged 1737 us and both CW motors hit
1928 us -- the mixer ceiling -- in both logs. A saturated motor cannot answer
the next correction. The aircraft responds, then stops responding, which is
exactly what "not stable enough to fly" feels like from the sticks.

Likely causes, in order: the two CW props differ in pitch/size/brand from the
CCW pair (easy to end up with after a rebuild); a CW prop mounted upside down;
a CW prop bent or chipped -- motor 3 is the arm that broke on 08-30.

### The battery is sagging below its own failsafe

Log 70: mean ~10.6 V, dipping to **10.05 V** at 13.5 A peak, mean draw 8.8 A,
1339 mAh out of a 3300 mAh pack. `FS_BATT_VOLTAGE` is 10.5, so the pack spends
time under the threshold. Board `Vcc` also dips to 4.635 V, at the edge of
where the APM misbehaves. Less pack voltage means less thrust, which makes the
saturation above worse.

**Do not leave `FS_BATT_ENABLE` at 2 (RTL) while `COMPASS_OFS` is all zero.**
That combination flies an RTL on a heading the aircraft does not have. Set it
to 1 (LAND) until the compass is calibrated.

### AHRS_TRIM has been rewritten three times, chasing this

| When | `AHRS_TRIM_X` | `AHRS_TRIM_Y` |
|---|---|---|
| 2026-09-02, measured | +0.67 deg | -0.52 deg |
| log 70 | 0.00 | 0.00 |
| log 71 | +3.14 deg | -5.71 deg |
| board, 2026-09-23 | -0.04 deg | -3.55 deg |

Someone has been trimming between flights to chase the drift. It cannot work:
a thrust asymmetry is not a level error, and each attempt destroys the only
level reference the aircraft has. **The drift persisted unchanged across all
three values** -- both logs still show ~2 deg of held left stick and ~2.3 deg
of held back stick -- which is itself the proof that trim was never the cause.
Measure it once with `level_calibrate.py` on a surface checked with a spirit
level, then stop touching it.

### Order of work

1. Props off, compare all four: diameter, pitch marking, profile, rotation
   type, which way up, damage.
2. `./motor_test.py --throttle 15` -- all four should sound and spin alike.
3. A fresh, healthy 3S. If the sag stays, that pack is finished.
4. `./level_calibrate.py` on a verified level surface.
5. `FS_BATT_ENABLE 1` now; back to 2 only after the compass is calibrated.
6. Re-fly, then compare the new log's CW/CCW split against the table above.

## Live readings, APM Planner session

**SUPERSEDED.** Kept for history only -- this table predates the frame rebuild
and every blocker in it has since been fixed. Current readings are in the
Verified table and in "The right-side lean, 2026-09-02".

Taken with the battery connected and the board on USB.

| Reading | Value | Verdict |
|---|---|---|
| Mode (disarmed) | ALT HOLD | **blocker** -- locked in slot 4, cannot select STABILIZE |
| Battery voltage | 0.27 V | **blocker** -- no voltage sense reaching the APM |
| Battery percent | 100% | meaningless, computed from the 0.27 V |
| GPS fix | 3D | antenna works |
| Satellites | 5 | too few for RTL or Loiter (want 8+) |
| HDOP | 3.6 | poor (want <= 2.0) |
| Attitude at rest | pitch -6.5, roll -7.1 deg | check on a known-level surface |
| Compass | internal, ORIENT 0 | decision still open |

## Level reference — FIXED

The cause of the tipping. The board believed it was banked when it was level, so
it fought that imaginary bank from the moment it armed.

| | roll | pitch |
|---|---|---|
| before | -6.96 deg | -2.11 deg |
| trim applied | -4.16 deg | +0.37 deg |
| after | **-0.19 deg** | **+0.01 deg** |

Converged in one pass, verified by re-measurement. The gyro was never at fault:
`check_imu.py` measured bias 0.0002 rad/s with clean noise on all three axes.
Roll and pitch ANGLES come from the accelerometer, not the gyro -- a still
aircraft's gyro reads zero and contributes nothing to the reported attitude.

**Re-measured on the NEW frame, 2026-09-02:** roll **+0.08 deg**, pitch
**-0.11 deg**, steadiness 0.30 / 0.25 std dev. Inside the 0.5 deg threshold, so
`level_calibrate.py` wrote nothing and the stored trim (+0.67 / -0.52) still
stands. The board mount survived the rebuild -- worth recording, because the
obvious assumption after a frame swap is that the level reference must be stale,
and here it measurably was not. Measure before rewriting it.

## Blockers before any flight

1. ~~**Battery voltage sense.**~~ FIXED -- reads 11.10 V. Still worth checking
   against a multimeter once, since `BATT_VOLT_MULT` is a generic default.
   Original note: 0.27 V meant the power module's sense lead was not
   reaching the APM PM port. `BATT_MONITOR 4` and `FS_BATT_ENABLE 2` are already
   live, so the battery failsafe is currently acting on a fiction -- it will
   either fire for no reason or never fire. Fix the lead, then verify the
   reported voltage against a multimeter and scale `BATT_VOLT_MULT` if needed.

2. ~~**Mode switch.**~~ FIXED -- SwC drives channel 5 through three positions,
   board reports STABILIZE with the switch up.
   Original note: the board sat in ALT HOLD while disarmed, meaning
   channel 5 was parked in slot 4. Confirm SwC actually drives channel 5 through
   three positions (`./mode_monitor.py`). Being unable to select STABILIZE on a
   first flight is not acceptable.

3. ~~**ESC calibration.**~~ FIXED 2026-09-02. Original note: four ESCs that do
   not share a throttle range spool at different rates, and the aircraft rolls
   or pitches on takeoff.

4. ~~**Propeller direction.**~~ VERIFIED 2026-09-02 by `motor_test.py` on the
   new frame. Original note: CCW props on motors 1 and 2, CW on 3 and 4.

5. **Compass, on the new frame.** Hard iron comes from the ESCs, wiring and
   battery, so `COMPASS_OFS` (X -217 Y +26 Z +63) was fitted to the OLD frame's
   layout. Only matters for LOITER / POSHOLD / RTL / AUTO -- STABILIZE and
   ALT_HOLD do not use heading. Re-run `calibrate_compass.py` outdoors before
   any GPS mode.

## The right-side lean, 2026-09-02

Symptom: on takeoff the aircraft dropped the motor 1 / motor 4 side, i.e.
rolled right. Everything reachable over USB was eliminated in this order, and
none of it was at fault:

| Suspect | Test | Result |
|---|---|---|
| Motor wiring swapped in the rebuild | `motor_test.py --throttle 15` | correct, all four |
| Prop direction | same | correct |
| Board believing it is banked | `level_calibrate.py` | +0.08 / -0.11 deg |
| Radio commanding roll | `preflight.py --quick` ch1 vs `RC1_TRIM` | 3 us, inside DZ 30 |
| Gyro / accel fault | same | bias 0.0000 rad/s, noise 0.0002-0.0004 |
| ESCs not sharing a throttle range | never calibrated | **the open item** — now done |

ESC calibration was the only candidate left standing, was completed, and
**cured it** -- logs 52 and 53 both show no roll bias over 1 degree.

The drift that remained after that was NOT a trim fault. log52 measured the
aircraft as trimmed over 532 airborne seconds. Two things were actually going
on: the reversed pitch channel below, and the plain fact that STABILIZE has no
brake -- it levels the aircraft when the sticks centre but does not stop it,
and indoors the wall recirculation keeps adding drift no trim can remove. Only
LOITER and POSHOLD stop a quad, and they need GPS, which does not work in a
room. That part is a hand-flying job, not a configuration problem.

Original note, superseded: If the lean survives it, everything
electrical is eliminated and the cause is physical, in likelihood order:
battery or GPS mast mounted off-centre to the right; a right-side arm seated
with a twist so motors 1/4 are not vertical; a bent prop on 1 or 4. Check CG
first by balancing the aircraft on two fingers under the centre plate.

Note `motor_test.py` and `level_calibrate.py` both prompt for observations only
a human can make -- which arm turned, which way, is the surface really level.
They cannot be driven by an agent piping answers; doing so fabricates the
measurement.

## Pitch was REVERSED — fixed 2026-09-02

Symptom: forward stick flew the aircraft BACKWARD, back stick flew it forward.
Roll was correct. Reported after the ESC calibration had already cured the
right-side lean, so it was a second, independent fault.

Measured on the bench with `check_stick_sense.py`:

| | centred | full forward | from trim | `RC2_REV` | control_in | flies |
|---|---|---|---|---|---|---|
| before | 1481 | 1777 | +296 us | +1 | POSITIVE | nose up -> BACKWARD |
| after | 1480 | 1851 | +371 us | **-1** | NEGATIVE | nose down -> FORWARD |

Roll measured correct in the same runs (+313 then +380 us -> flies right).
**That is what ruled out a board mounted 180 degrees round** -- a reversed board
reverses BOTH axes and is fixed with `AHRS_ORIENTATION`, never by reversing
channels, because reversing two channels leaves the gyro and accel axes still
pointing backwards.

Fix recorded as `profiles/45-rc-pitch-reverse.params` so it survives a re-flash.

Why nothing caught this earlier:

- **A reversed channel and a correct one produce identical PWM ranges.**
  Only the SIGN relative to trim separates them, and nothing in `preflight.py`
  or `rc_live.py` looks at sign -- `rc_live.py` reports travel, not direction.
- **Self-levelling comes from the IMU and is unaffected**, so the aircraft
  hovers, holds attitude and passes every pre-arm check. log52 is 532 seconds
  of smooth hovering flown on a reversed pitch axis.
- The FS-i6's elevator output runs opposite to what ArduPilot expects. Expect
  this on any FS-i6 build; it is a transmitter pairing, not a fault.

The other cause of "my controls are backwards" is flying with the nose pointed
AT you, which mirrors everything and no parameter can fix. Ruled out here: the
nose was pointed away. Rule it out FIRST next time -- it costs nothing.

Stick travel is fine. A first run showed pitch reaching only 48% of its
calibrated span and lopsided (+296/-183); a second run with the sticks held
properly to the stops gave +371/-314 for pitch and +380/-355 for roll. That was
measurement technique, not an endpoint fault. Worth knowing that this test is
sensitive to how hard you hold the stick.

## Tools added 2026-09-02

- **`log_download.py`** — `--list`, or download by `--id`, into `logs/dataflash/`.
  There was previously no scriptable way to get a log off the board, which is
  why that directory held exactly one file. Turns the telemetry streams off
  first (they compete for the same link) and re-requests dropped blocks, since
  ArduPilot does not retransmit on its own and a loop that only waits for the
  next packet waits forever on a log that is 99% complete.
  **It needs the same ~18 s post-reboot settle as `motor_test.py`** -- without
  it the board silently ignores the log list request and the tool reports "no
  logs" on a board holding ten.
- **`analyze_hover.py`** — reads a log and says whether a drift was the pilot or
  the aircraft. Uses `ATT.DesRoll`/`DesPitch`, NOT `RCIN`: RCIN is gated behind
  `LOG_BITMASK` and is absent from anything logged at 830, and DesRoll is the
  stick already converted to a commanded lean angle with trim, deadzone,
  reversal and `ANGLE_MAX` applied by the firmware. Mean commanded lean over a
  hover IS the aircraft's bias, measured by the pilot's hand.
  Do not read `ATT.Roll` for this: it is reported AFTER `AHRS_TRIM`, so a
  working controller drives it to ~0 even on a badly trimmed aircraft.
- **`check_stick_sense.py`** — bench test, ~50 s, props off, no arming. Traces
  stick -> PWM vs trim -> `RCn_REV` -> commanded lean -> direction of flight.
  Timed countdowns rather than prompts, so it needs no stdin.
- **`profiles/60-flight-logging.params`** — `LOG_BITMASK` 894. Put it back to
  830 once trimmed; the APM 2.8's dataflash is small and 894 fills it faster.

## Tools added 2026-09-07

- **`gps_check.py`** — live GPS view plus a verdict: fix type, satellite count,
  HDOP, and how far the reported position wanders while the aircraft stands
  still (the only accuracy figure available without a survey mark).
  Checks `GPS_TYPE` BEFORE watching, because the most wasteful way to run a GPS
  test is to stand outside for ten minutes with the driver switched off; it
  refuses to start in that state and offers `--enable` to fix it.
  Separates the two failures that look identical from a distance: `fix 0` is a
  driver/wiring fault where sky is irrelevant, `fix 1` is a live receiver with
  no satellites. Pass `--outdoors` and it will call zero satellites after five
  minutes what it is -- an antenna or module fault -- instead of counselling
  patience forever.

## Gotchas learned on this build

- QGroundControl cannot configure this board. It parses MAVLink v1 but only
  emits v2, which 3.2.1 discards, so the vehicle appears and every request
  times out. `mavbridge.py` translates if you want the QGC UI.
- QGC's serial AutoConnect reopens the USB port even after you disconnect the
  link, blocking avrdude and every script here. Those toggles are now off.
- `MAV_CMD_DO_MOTOR_TEST` param1 is the TEST SEQUENCE number, not the motor
  number — the order runs clockwise around the airframe.
- Motor test refusals arrive as STATUSTEXT; the ack alone says nothing useful.
- A motor-output split that follows ROTATION DIRECTION (the diagonal pair) is a
  propeller problem. One that follows POSITION (left/right, front/rear) is a CG
  or frame problem. Reading which way the split falls names the suspect before
  you touch anything.
- Trimming to cure a drift that a saturated motor is causing writes a wrong
  level reference and leaves the drift. If changing `AHRS_TRIM` by 3 deg does
  not change the held stick by 3 deg, trim is not what you are fighting.
- A sensor that reads exactly zero forever is usually disabled, not broken.
  `MAG_ENABLE 0` and `GPS_TYPE 0` both produce a perfect imitation of dead
  hardware, including the `SYS_STATUS` present bit going false.

## Deliberate parameter changes worth remembering

- `ANGLE_MAX` 4500 -> 3000. A 30 degree lean limit for early flights; put it
  back to 45 once it is trimmed and you trust it.
- `LOG_BITMASK` 830 -> 894, restored 2026-09-02 via
  `./preload_params.py --yes flight-logging`. 830 had LOWERED the logging, and
  the missing bits mattered: logs taken at 830 have no `RCIN` at all, which is
  why `analyze_hover.py` reads `ATT.DesRoll` instead. Put it back to 830 once
  the aircraft is trimmed and flying -- the dataflash is small and 894 fills it
  faster.
- `RC2_REV` +1 -> -1, 2026-09-02. Pitch was reversed; see the section above.
- `GPS_HDOP_GOOD` 230 -> 200, a stricter gate before GPS modes engage.
- `MAG_ENABLE` 0 -> 1 and `GPS_TYPE` 0 -> 1, 2026-09-07. NOT deliberate — these
  were found switched off and restored to the `00-base` baseline. See the
  2026-09-07 section above.
- `COMPASS_USE` 0 -> 1, 2026-09-07, back to baseline. On its own it did nothing;
  `MAG_ENABLE` was the parameter that mattered.

## What ALT_HOLD actually depends on

Not the compass. Not the GPS. Altitude comes from the BAROMETER, corrected by
the accelerometers. Heading and position play no part in holding height.

| Need | State | Action |
|---|---|---|
| Barometer shielded from airflow and light | **not done** | Put a small piece of open-cell foam over the baro sensor on the APM |
| Low vibration into the accels | done | `INS_MPU6K_FILTER 20` applied |
| Accelerometer calibration | done | scale 0.993 / 0.991 / 1.034 |
| `THR_MID` near real hover throttle | 500, unverified | Hover in STABILIZE, note the stick position, set `THR_MID` to match |
| Stable in STABILIZE first | not yet flown | ALT_HOLD on an untrimmed aircraft just adds a second problem |

The foam is the one people skip and then chase for weeks. The APM 2.8's
barometer is an exposed MS5611: prop wash changes the pressure over it, and
direct sunlight heats the die. Either one makes the aircraft climb or sink on
its own in ALT_HOLD. A scrap of open-cell foam over the sensor fixes both. It
must be open-cell -- closed-cell foam seals it and is worse than nothing.

## What the compass and GPS are actually for

LOITER, POSHOLD, RTL and AUTO. You have RTL on switch slot 6, so they do matter
eventually -- just not for altitude hold.

Compass calibration must be done OUTDOORS, away from steel-framed buildings,
rebar in concrete, cars and desks. Calibrating indoors fits the sphere to the
building's magnetic distortion and produces offsets that are wrong the moment
you carry the aircraft outside. 5 satellites and HDOP 3.6 says indoors -- so
this is a job for the field, in the same session where the satellite count
should come up to 8+.

GPS is not calibrated at all. It needs sky view and time to settle; there is no
procedure.
