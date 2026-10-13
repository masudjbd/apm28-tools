# FlySky FS-i6 setup for this aircraft

Menu paths verified against the FS-i6 manual and a published setup guide, not
from memory. Two of them are commonly got wrong, including by me earlier in
this build:

- **Failsafe is under System setup, not Functions setup.**
- **Changes are saved with a LONG press of CANCEL**, not OK. A short CANCEL
  backs out and discards.

## Entering the menu

Power on the transmitter, then **hold OK**. Two sections appear, chosen with
UP/DOWN: **System setup** and **Functions setup**.

## 1. Assign the mode switch — Functions setup -> Aux. channels

Set **Channel 5** source to **SwC**.

SwC is the only three-position switch on this radio; SwA, SwB and SwD are
two-position. Three positions is what buys you manual / assisted / recovery.

Long-press CANCEL to save.

## 2. Receiver failsafe — System setup -> Rx Setup -> Failsafe

Enable failsafe on **channel 3** and set its value **below 975**.

This is not optional decoration. `FS_THR_ENABLE 1` on the flight controller
watches for the throttle channel dropping below `FS_THR_VALUE` (975 here). The
FS-iA6B, left alone, holds its LAST output when the signal is lost -- so the
threshold is never crossed and the failsafe never fires. The transmitter has to
be told to output something low.

Long-press CANCEL to save.

**Then test it**, because a failsafe you have not seen work is a guess: run
`./rc_live.py`, switch the transmitter off, and watch channel 3 actually fall
below 975.

## 3. Check what actually happened

```bash
./rc_live.py        # move one control at a time; see which channel responds
./mode_monitor.py   # channel 5 PWM -> mode slot -> flight mode
```

`rc_live.py` is the one to reach for when a switch does not seem to do
anything. The transmitter's channel numbering and the flight controller's do
not have to agree, and a mis-set Aux menu puts your switch quietly on the wrong
channel. Move one control, see which row lights up.

## Reference numbers for this aircraft

| Channel | Role | Calibrated range |
|---|---|---|
| 1 | roll | 1092 - 1925 |
| 2 | pitch | 1091 - 1925 |
| 3 | throttle | 1088 - 1925 |
| 4 | yaw | 1092 - 1925 |
| 5 | flight mode | needs re-calibrating once SwC drives it |

Mode slot boundaries in ArduCopter 3.2.1:

      < 1231 -> FLTMODE1      1491-1620 -> FLTMODE4
    1231-1360 -> FLTMODE2     1621-1749 -> FLTMODE5
    1361-1490 -> FLTMODE3       >= 1750 -> FLTMODE6

A three-position switch at roughly 1000/1500/2000 reaches slots 1, 4 and 6.
That is what `profiles/51-fltmodes-3pos.params` assumes: STABILIZE / ALT_HOLD /
RTL. Confirm with `mode_monitor.py` rather than trusting it.
