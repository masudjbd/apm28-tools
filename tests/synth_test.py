"""Synthetic end-to-end check of the orientation solver.

Simulates a board being waved around: known true rotation, realistic hard-iron
offset, sensor noise and link jitter. If the solver cannot recover the planted
answer here it will not recover it on the bench either.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "lib"))

import numpy as np
import rotations
import orientsolve


def expm_so3(w, dt):
    theta = np.linalg.norm(w) * dt
    if theta < 1e-12:
        return np.eye(3)
    axis = w / np.linalg.norm(w)
    K = orientsolve._skew(axis)
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)


def simulate(true_enum, seed=0, n=1500, rate=50.0, noise=3.0, bias=(45.0, -30.0, 60.0),
             jitter=0.002, axes=(1.0, 1.0, 1.0)):
    rng = np.random.default_rng(seed)
    R_true = rotations.matrix(true_enum)

    # Houston-ish field: ~500 mGauss total, 60 deg inclination, NED.
    inc = np.radians(60.0)
    m_world = 500.0 * np.array([np.cos(inc), 0.0, np.sin(inc)])

    dt = 1.0 / rate
    # smooth random angular velocity, scaled per axis so we can starve an axis
    raw = rng.normal(size=(n, 3))
    for i in range(3):
        raw[:, i] = np.convolve(raw[:, i], np.ones(25) / 25, mode="same")
    w = raw / (np.abs(raw).max() + 1e-9) * 2.5
    w *= np.array(axes)

    R_wb = np.eye(3)
    t, gyro, mag = [], [], []
    now = 0.0
    for i in range(n):
        m_body = R_wb @ m_world
        m_sensor = R_true.T @ m_body
        meas = m_sensor + np.array(bias) + rng.normal(scale=noise, size=3)
        now += dt + rng.normal(scale=jitter)
        t.append(now)
        gyro.append(w[i] + rng.normal(scale=0.004, size=3))
        mag.append(meas)
        # R_wb maps world->body, so it propagates by the NEGATIVE body rate
        R_wb = expm_so3(-w[i], dt) @ R_wb
    return np.array(t), np.array(gyro), np.array(mag)


def run():
    failures = 0
    print("=== recovery across the whole table ===")
    for enum_value, nm, _ in rotations.all_items():
        t, gyro, mag = simulate(enum_value, seed=enum_value)
        results, diag = orientsolve.solve(t, gyro, mag)
        ok = results[0]["enum"] == enum_value
        if not ok:
            failures += 1
            print("  FAIL planted %-2d %-28s -> got %-2d %s (margin %.2f)"
                  % (enum_value, nm, results[0]["enum"], results[0]["name"], diag["margin"]))
    print("  %d/%d recovered" % (34 - failures, 34))

    print("\n=== confidence reporting ===")
    t, gyro, mag = simulate(9, seed=7)
    results, diag = orientsolve.solve(t, gyro, mag)
    good, problems = orientsolve.verdict(diag)
    print("  clean run     -> %-2d %-24s margin %.2f gain %.3f confident=%s"
          % (results[0]["enum"], results[0]["name"], diag["margin"], diag["gyro_gain"], good))
    planted_body = rotations.matrix(9) @ np.array([45.0, -30.0, 60.0])
    print("     hard-iron (body frame) %s  expected %s"
          % (np.round(results[0]["bias"], 1), np.round(planted_body, 1)))

    # yaw-only run: must NOT claim confidence
    t, gyro, mag = simulate(9, seed=7, axes=(0.02, 0.02, 1.0))
    results, diag = orientsolve.solve(t, gyro, mag)
    good, problems = orientsolve.verdict(diag)
    print("  yaw-only run  -> confident=%s" % good)
    for p in problems:
        print("     flagged: %s" % p)
    if good:
        failures += 1
        print("     FAIL: should not be confident on a yaw-only run")

    # heavy noise
    t, gyro, mag = simulate(20, seed=3, noise=25.0)
    results, diag = orientsolve.solve(t, gyro, mag)
    print("  noisy run     -> %-2d %-24s (planted 20) margin %.2f"
          % (results[0]["enum"], results[0]["name"], diag["margin"]))
    if results[0]["enum"] != 20:
        failures += 1
        print("     FAIL under noise")

    print("\n%s" % ("ALL PASS" if failures == 0 else "%d FAILURES" % failures))
    return failures


if __name__ == "__main__":
    raise SystemExit(1 if run() else 0)
