# Target values, and what to do when they move

## The distinction that matters

**Calibration describes the physical aircraft.** When the board shifts on its
mount, `AHRS_TRIM` genuinely *should* change -- the aircraft really is at a new
angle. Restoring an old value here re-imposes a measurement that is no longer
true. **Re-measure. Never restore.**

**Configuration describes your choices.** Nothing physical alters it. If it
differs from your baseline, something overwrote it, and pushing the baseline
back is exactly right. **Restore.**

Confusing the two is how people "fix" an aircraft into a worse state.

## Live readings — what good looks like

Check with `./preflight.py`. These are readings, not parameters; you cannot set
them, only fix what produces them.

| Reading | Good | Acceptable | Wrong, and what it means |
|---|---|---|---|
| roll / pitch at rest, level surface | within 0.5 deg | within 1 deg | above 2 deg -> level reference is off; the aircraft will accelerate sideways in flight |
| attitude steadiness (std dev) | under 0.1 deg | under 0.3 deg | above 0.5 -> something is rocking it: cable tension, a short landing gear leg, a soft surface |
| gyro bias at rest | under 0.001 rad/s | under 0.01 | above 0.05 -> the aircraft is moving, or an axis is faulty |
| gyro noise at rest | under 0.001 rad/s | under 0.005 | rising over time -> new vibration; check props and mount |
| Alt REL sitting on the ground | under 0.3 m | under 0.5 m | 1 m+ -> barometer drifting; needs open-cell foam over the sensor. ALT_HOLD will chase this |
| battery voltage, 3S | 11.1-12.6 V | 10.5-12.6 | under 1 V -> sense lead unplugged, not a flat pack |
| GPS fix | 3 (3D) | 3 | 1 = no fix. LOITER and RTL are unusable |
| GPS satellites | 10+ | 8+ | under 8 -> position modes not trustworthy |
| GPS HDOP | under 1.5 | under 2.0 | above 2 -> poor geometry, wait or move |
| compass offsets, magnitude | under 150 | under 400 | above 600 -> fails pre-arm; the compass is too near power wiring |

## Configuration — restore these if they move

| Parameter | Value | Why |
|---|---|---|
| `FRAME` | 1 | quad X |
| `ARMING_CHECK` | 1 | pre-arm checks on |
| `ANGLE_MAX` | 3000 | 30 deg lean limit while learning; 4500 once trimmed |
| `INS_MPU6K_FILTER` | 20 | 20 Hz IMU filter suits a soft-mounted APM |
| `FS_THR_ENABLE` | 1 | throttle failsafe on |
| `FS_THR_VALUE` | 975 | must sit BELOW the real `RC3_MIN` and above the receiver's signal-loss output |
| `FS_GCS_ENABLE` | 0 | on only with a telemetry radio |
| `FS_BATT_ENABLE` | 2 | RTL on low battery |
| `FS_BATT_VOLTAGE` | 10.5 | ~3.5 V per cell on 3S |
| `BATT_MONITOR` | 4 | voltage and current |
| `COMPASS_USE` | 1 | heading used |
| `COMPASS_LEARN` | 0 | in-flight learning corrupts a good calibration on 3.2.1 |
| `LOG_BITMASK` | 894 | detailed logging; worth having while tuning |
| `FLTMODE1/4/6` | the three reachable slots | see profiles/ |

## Calibration — re-measure these, never restore

| Parameter | Changes when | Re-measure with |
|---|---|---|
| `AHRS_TRIM_X/Y` | the board moves on its mount, any crash, a new anti-vibration mount | `./level_calibrate.py` |
| `INS_ACCOFFS_*`, `INS_ACCSCAL_*` | rarely; only if the accel drifts with age | `./gcs.sh` then `accelcal` |
| `COMPASS_OFS_*` | anything magnetic moves: wiring, battery position, the compass itself | `./calibrate_compass.py --write` |
| `COMPASS_ORIENT` | the compass module is remounted | `./solve_compass_orient.py --write` |
| `RC*_MIN/MAX/TRIM` | transmitter endpoints, subtrim, or a new radio | `./calibrate_radio.py --write` |
| `INS_GYROFFS_*` | every boot, automatically | nothing to do |

## The routine

**Take a baseline when the aircraft is right:**

```bash
./preload_params.py --backup
cp logs/params-<newest>.params logs/GOLDEN.params
```

**Before every flight:**

```bash
./preflight.py
```

**After any crash, hard landing, or hardware change:**

```bash
./preload_params.py --diff logs/GOLDEN.params
```

It splits what moved into "restore these" and "re-measure these", and tells you
which command re-measures each one. Then:

```bash
./preload_params.py --restore logs/GOLDEN.params   # config only
./level_calibrate.py                               # re-measure the level reference
./preflight.py                                     # confirm
```

**Take a fresh baseline afterwards**, so GOLDEN always reflects a known-good
aircraft rather than an increasingly stale one.

## On "self-healing"

Configuration can heal itself -- that is what `--diff` and `--restore` do, and
it is safe because those values have no physical meaning.

Calibration cannot, and should not. There is no way to know from software that
the board is now sitting 3.5 degrees left; the only source of truth is a fresh
measurement on a level surface. Anything claiming to restore that
automatically would be restoring a number, not a fact.

What can be automated is the NOTICING. That is the whole point of `--diff` and
`preflight.py`: they tell you what moved and which command fixes it, so the
gap between "something changed" and "I know what to do" is one command instead
of an evening.
