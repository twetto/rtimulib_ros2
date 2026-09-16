#!/usr/bin/env python3
"""Solve per-axis accelerometer scale and bias from static poses.

Model:  a_corrected[i] = (a_measured[i] - bias[i]) / scale[i]
Fit so that |a_corrected| = g in every pose.

Why this matters more than the raw 3.7% error: a VIO estimator models
accelerometer BIAS online and will absorb it, but it does not estimate SCALE.
Scale error propagates more or less directly into metric trajectory scale.
Separating the two tells you whether there is anything to fix at all.
"""
import json, math, sys
import numpy as np

G = 9.80665


def residuals(p, A):
    b, s = p[:3], p[3:]
    c = (A - b) / s
    return np.linalg.norm(c, axis=1) - G


def jac(p, A, eps=1e-7):
    J = np.zeros((len(A), 6))
    for k in range(6):
        q = p.copy(); q[k] += eps
        J[:, k] = (residuals(q, A) - residuals(p, A)) / eps
    return J


def solve(A):
    p = np.array([0.0, 0.0, 0.0, 1.0, 1.0, 1.0])
    for _ in range(200):
        r = residuals(p, A)
        J = jac(p, A)
        try:
            dp = np.linalg.lstsq(J, -r, rcond=None)[0]
        except np.linalg.LinAlgError:
            break
        p = p + dp
        if np.linalg.norm(dp) < 1e-12:
            break
    return p


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "accel_poses.json"
    P = json.load(open(path))["poses"]
    A = np.array([p["mean"] for p in P])
    n = len(A)

    p = solve(A)
    bias, scale = p[:3], p[3:]
    r = residuals(p, A)

    print(f"poses: {n}   unknowns: 6")
    if n <= 6:
        print("NOTE: exactly determined, so the fit is forced through every point.")
        print("      The residuals below are therefore ~0 by construction and say")
        print("      NOTHING about accuracy. Capture 10-12 poses for redundancy.\n")

    print("per-axis result:")
    for i, ax in enumerate("xyz"):
        print(f"  {ax}:  scale {scale[i]:.4f}  ({100*(scale[i]-1):+.2f} %)"
              f"   bias {bias[i]:+.4f} m/s2")

    # Independent cross-check from opposed pose pairs, valid only where a pose is
    # nearly axis-aligned. Agreement with the fit is real evidence; the residual
    # is not.
    print("\ncross-check from opposed poses (only where well aligned):")
    for i, ax in enumerate("xyz"):
        pos = max(A, key=lambda v: v[i])
        neg = min(A, key=lambda v: v[i])
        align = min(abs(pos[i])/np.linalg.norm(pos), abs(neg[i])/np.linalg.norm(neg))
        s = (pos[i] - neg[i]) / 2 / G
        b = (pos[i] + neg[i]) / 2
        flag = "" if align > 0.97 else f"  (tilted, alignment {align:.3f} - approximate)"
        print(f"  {ax}:  scale {s:.4f}   bias {b:+.4f}{flag}")

    print("\nwhat this means:")
    worst_scale = max(abs(s - 1) for s in scale) * 100
    worst_bias = max(abs(b) for b in bias)
    print(f"  largest scale error: {worst_scale:.2f} %   -> VIO CANNOT absorb this;"
          f" it becomes trajectory scale error")
    print(f"  largest bias:        {worst_bias:.3f} m/s2 -> VIO estimates bias online;"
          f" it will absorb this")
    spread = max(abs(s - 1) for s in scale) - min(abs(s - 1) for s in scale)
    print(f"  per-axis scale spread: {100*spread:.2f} %   "
          f"-> {'non-uniform, so gravity direction is tilted and T_cam_imu is slightly biased' if spread > 0.005 else 'near uniform, so gravity direction is unaffected'}")

    print("\ncorrection to apply:  a_corrected[i] = (a[i] - bias[i]) / scale[i]")
    print(f"  bias:  [{bias[0]:+.6f}, {bias[1]:+.6f}, {bias[2]:+.6f}]")
    print(f"  scale: [{scale[0]:.6f}, {scale[1]:.6f}, {scale[2]:.6f}]")

    print("\nverification, corrected magnitude per pose:")
    for i, v in enumerate(A):
        c = (v - bias) / scale
        print(f"  pose {i}: |a| {np.linalg.norm(v):7.3f} -> {np.linalg.norm(c):7.4f}"
              f"   (g = {G})")


if __name__ == "__main__":
    main()
