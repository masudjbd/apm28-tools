# APM 2.8 bring-up runbook

Everything below needs either the USB port or your hands, so it runs from your
Terminal, not from Claude's side.

## Step 0 — before you touch anything

- **Propellers off.** All of it.
- **Quit QGroundControl.** One process owns a serial port at a time; if QGC
  holds `/dev/cu.usbmodem113301`, avrdude and every script here fail with
  "cannot open port". (Its NMEA-GPS setting was doing exactly this — fixed,
  but the app still grabs the port whenever a link is connected.)

## Step 1 — find out what you actually have

```bash
cd ~/apm28-tools
./setup_env.sh
source .venv/bin/activate
./identify.py
```

Writes nothing. Prints the firmware banner, the parameters that matter, and a
checklist of what is missing. **Everything after this depends on that output** —
including whether you need to reflash at all. The board is already running
ArduPilot, so a reflash may be unnecessary.

## Step 2 — back up before changing anything

```bash
./preload_params.py --backup
```

Lands in `logs/params-<timestamp>.params`. Mission Planner can read it.

## Step 3 — the hardware modification

Cut the trace on the solder jumper beside the APM 2.8's I2C port. This
disconnects the onboard magnetometer.

Nothing downstream is trustworthy until this is done. Two magnetometers on one
I2C bus produce heading that drifts and circles regardless of how good the
calibration is, and no parameter fixes it.

## Step 4 — calibrate, in this order

```bash
./calibrate_accel.py             # 6 positions, flat reference surface
./solve_compass_orient.py --write # wave the airframe, all 3 axes, 30s
./calibrate_compass.py --write    # slow rotation through every attitude, 60s
./calibrate_radio.py --write      # transmitter on, both extremes on every channel
```

The order is load-bearing:

- accel before compass — the compass fit is read in the body frame the
  accelerometers define
- `COMPASS_ORIENT` before `COMPASS_OFS` — changing the orientation permutes the
  axes the offsets were measured in
- radio before failsafes — `FS_THR_VALUE` is only meaningful once `RC3_MIN` is
  known

`solve_compass_orient.py` will refuse to write a value it cannot separate from
the runner-up. If it says inconclusive, wave the airframe through more roll and
pitch and run it again. Do not use `--force` to get past it.

## Step 5 — parameters

```bash
./preload_params.py --dry-run base frame-x compass-external power-module failsafe-3s
./preload_params.py base frame-x compass-external power-module failsafe-3s
```

Use `frame-plus` instead of `frame-x` if `identify.py` reported FRAME 0.
Adjust `FS_BATT_VOLTAGE` in `profiles/40-failsafe-3s.params` for your cell
count (~3.5 V per cell).

## Step 6 — the parts that stay manual

- **ESC calibration** — props off, battery in, throttle high at power-up.
  3.2.1 has no parameter-driven path; scripting it would spin motors.
- **Flight modes** — `FLTMODE1..6`, transmitter-specific.
- **Failsafe check** — switch the transmitter off and confirm RC3 falls below
  `FS_THR_VALUE`. Do this on the bench, not in the air.
- **Tethered arm test** before anything leaves the ground.

## If something goes wrong

| Symptom | Cause |
|---|---|
| `cannot open port` | QGroundControl or another script holds it |
| avrdude hangs | using a `/dev/tty.*` node instead of `/dev/cu.*` |
| `stk500v2_ReceiveMessage(): timeout` | unplug, replug, rerun immediately |
| No heartbeat | board has no firmware, or wrong baud (should be 115200) |
| Heading drifts / circles | compass jumper not cut |
