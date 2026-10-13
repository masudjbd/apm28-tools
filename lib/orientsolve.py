"""Solve COMPASS_ORIENT from gyro/magnetometer motion, not from guesswork.

The idea
--------
The Earth's field is fixed in the world frame. Seen from a rotating body, a
world-fixed vector obeys

    dm/dt = -w x m

where w is the body angular rate. That is a hard kinematic constraint: it holds
for the *correctly* rotated magnetometer and for no other rotation in the table.
Wave the aircraft around, record gyro and raw mag together, and the true
rotation is simply the one whose residual collapses.

Hard iron
---------
The measurement carries a constant offset b fixed in the sensor frame, so after
a fixed rotation it is constant in the body frame too:

    d(m_meas)/dt = -w x (m_meas - b)

Rearranged, with a scalar gyro gain k absorbed so gyro unit errors cannot
bias the ranking, and c = k*b:

    dm/dt = -k (w x m) + (w x c)

which is *linear* in the four unknowns (k, c). So for each candidate rotation
we solve a 4-parameter least squares and score by residual. No calibration
step is required first -- the hard-iron offset falls out of the same fit.

Why this beats a static check
-----------------------------
Sampling one attitude cannot distinguish rotations that differ by a yaw, and
sign errors hide until the aircraft banks. The cross product is sensitive to
both, so a single 30-second wave resolves the whole table -- including
reflections, which a magnitude-only test can never see.
"""

import numpy as np

import rotations


def _moving_average(arr, window):
    if window <= 1:
        return arr.copy()
    kernel = np.ones(window) / float(window)
    out = np.empty_like(arr)
    for i in range(arr.shape[1]):
        out[:, i] = np.convolve(arr[:, i], kernel, mode="same")
    # convolve tapers the ends; trim them off at the caller instead of trusting
    return out


def prepare(t, gyro, mag, smooth=5, min_rate=0.15):
    """Filter, differentiate, and drop uninformative samples.

    Returns (w, m, dm) ready for scoring, all in the *sensor* frame for mag.
    Samples where the aircraft is nearly still carry no orientation information
    (w x m vanishes) but plenty of noise, so they are discarded -- keeping them
    flattens the score spread between candidates.
    """
    t = np.asarray(t, dtype=float)
    gyro = np.asarray(gyro, dtype=float)
    mag = np.asarray(mag, dtype=float)

    order = np.argsort(t)
    t, gyro, mag = t[order], gyro[order], mag[order]

    gyro_f = _moving_average(gyro, smooth)
    mag_f = _moving_average(mag, smooth)

    edge = max(smooth, 2)
    if len(t) < 2 * edge + 8:
        raise ValueError("not enough samples after filtering (%d)" % len(t))

    # central difference on real timestamps -- the link jitters, so a fixed dt
    # would smear the derivative and favour whichever rotation happens to be
    # smoothest rather than whichever is correct.
    idx = np.arange(edge, len(t) - edge)
    dt = t[idx + 1] - t[idx - 1]
    good = dt > 1e-4
    idx, dt = idx[good], dt[good]

    dm = (mag_f[idx + 1] - mag_f[idx - 1]) / dt[:, None]
    w = gyro_f[idx]
    m = mag_f[idx]

    moving = np.linalg.norm(w, axis=1) > min_rate
    return w[moving], m[moving], dm[moving]


def _skew(v):
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def score_rotation(matrix, w, m_sensor, dm_sensor):
    """Least-squares residual for one candidate rotation. Lower is better."""
    m = m_sensor @ matrix.T
    dm = dm_sensor @ matrix.T

    n = len(w)
    A = np.zeros((3 * n, 4))
    b = dm.reshape(-1)
    for i in range(n):
        row = 3 * i
        A[row:row + 3, 0] = -np.cross(w[i], m[i])
        A[row:row + 3, 1:4] = _skew(w[i])

    sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    residual = A @ sol - b
    denom = np.linalg.norm(b)
    if denom < 1e-9:
        return float("inf"), 0.0, np.zeros(3)
    k = float(sol[0])
    bias = sol[1:4] / k if abs(k) > 1e-9 else np.zeros(3)
    return float(np.linalg.norm(residual) / denom), k, bias


def solve(t, gyro, mag, smooth=5, min_rate=0.15):
    """Rank every rotation in the table. Returns (results, diagnostics)."""
    w, m, dm = prepare(t, gyro, mag, smooth=smooth, min_rate=min_rate)
    if len(w) < 40:
        raise ValueError(
            "only %d usable samples -- the aircraft was not moved enough. "
            "Rotate it about all three axes, not just yaw." % len(w)
        )

    results = []
    for enum_value, nm, matrix in rotations.all_items():
        residual, k, bias = score_rotation(matrix, w, m, dm)
        results.append(
            {"enum": enum_value, "name": nm, "residual": residual, "k": k, "bias": bias}
        )
    results.sort(key=lambda r: r["residual"])

    best, runner_up = results[0], results[1]
    margin = runner_up["residual"] / best["residual"] if best["residual"] > 0 else float("inf")
    diagnostics = {
        "samples": int(len(w)),
        "margin": float(margin),
        "gyro_gain": best["k"],
        "axis_coverage": _axis_coverage(w),
    }
    return results, diagnostics


def _axis_coverage(w):
    """How much rotation happened about each body axis, normalised.

    A run that only spun in yaw leaves roll/pitch unconstrained and will happily
    report a confident wrong answer, so the caller warns on this.
    """
    energy = np.sqrt((w ** 2).mean(axis=0))
    total = energy.sum()
    if total < 1e-9:
        return [0.0, 0.0, 0.0]
    return (energy / total).tolist()


def verdict(diagnostics, margin_threshold=1.35, coverage_threshold=0.12):
    """Turn diagnostics into a plain confidence call."""
    problems = []
    if diagnostics["margin"] < margin_threshold:
        problems.append(
            "top two candidates are only %.2fx apart -- too close to trust"
            % diagnostics["margin"]
        )
    weak = [
        axis
        for axis, share in zip("XYZ", diagnostics["axis_coverage"])
        if share < coverage_threshold
    ]
    if weak:
        problems.append(
            "little rotation about %s -- rotations differing in that axis are "
            "not separated" % "/".join(weak)
        )
    if not 0.3 < abs(diagnostics["gyro_gain"]) < 3.0:
        problems.append(
            "fitted gyro gain %.2f is implausible -- units or sample alignment "
            "are off" % diagnostics["gyro_gain"]
        )
    return (len(problems) == 0), problems
