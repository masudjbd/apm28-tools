"""ArduPilot sensor rotation table (enum values 0-33).

COMPASS_ORIENT takes one of these enum values. The rotation transforms a
vector from the *sensor* frame into the *body* frame, both right-handed
Forward-Right-Down.

Only 0-33 are defined here, deliberately. Those six blocks (yaw-only,
roll180+yaw, roll90+yaw, roll270+yaw, pitch90, pitch270) have had identical
numbering in every ArduPilot release since 2013, so a value solved here means
the same thing to ArduCopter 3.2.1 as it does to 4.x. The enum values above 33
were added and reshuffled over the years and are not safe to emit against a
2015 firmware.

Composite rotations apply roll first, then yaw:  R = Rz(yaw) @ Rx(roll)
which is what AP_Math's hardcoded Vector3f::rotate() cases work out to.
"""

import math

import numpy as np


def _rx(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def _ry(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def _rz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def _build():
    table = {}
    yaws = [0, 45, 90, 135, 180, 225, 270, 315]

    for i, yaw in enumerate(yaws):
        name_ = "ROTATION_NONE" if yaw == 0 else "ROTATION_YAW_%d" % yaw
        table[i] = (name_, _rz(yaw) @ _rx(0))

    for block_base, roll in ((8, 180), (16, 90), (24, 270)):
        for i, yaw in enumerate(yaws):
            name_ = "ROTATION_ROLL_%d" % roll
            if yaw:
                name_ += "_YAW_%d" % yaw
            table[block_base + i] = (name_, _rz(yaw) @ _rx(roll))

    table[32] = ("ROTATION_PITCH_90", _ry(90))
    table[33] = ("ROTATION_PITCH_270", _ry(270))
    return table


ROTATIONS = _build()


def name(enum_value):
    entry = ROTATIONS.get(int(enum_value))
    return entry[0] if entry else "UNKNOWN(%s)" % enum_value


def matrix(enum_value):
    return ROTATIONS[int(enum_value)][1]


def all_items():
    """Yield (enum_value, name, 3x3 matrix) in enum order."""
    for key in sorted(ROTATIONS):
        nm, mat = ROTATIONS[key]
        yield key, nm, mat
