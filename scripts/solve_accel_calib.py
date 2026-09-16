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
    doc = json.load(open(path))
    P = doc["poses"]
    A = np.array([p["mean"] for p in P])
    n = len(A)

    # If a correction was live during capture, what we solve here is a residual
    # on top of it. Compose so the printed result is always absolute.
    applied_s = np.array(doc.get("applied_scale", [1.0, 1.0, 1.0]))
    applied_b = np.array(doc.get("applied_bias", [0.0, 0.0, 0.0]))
    composed = any(abs(v - 1) > 1e-9 for v in applied_s) or any(abs(v) > 1e-9 for v in applied_b)

    p = solve(A)
    bias, scale = p[:3], p[3:]
    r = residuals(p, A)

    rss = float(np.sum(r**2))
    dof = n - 6
    print(f"poses: {n}   unknowns: 6   degrees of freedom: {dof}")
    if dof > 0:
        sigma = math.sqrt(rss/dof)
        J = jac(p, A)
        try:
            cov = sigma**2 * np.linalg.inv(J.T @ J)
            se = np.sqrt(np.diag(cov))
        except np.linalg.LinAlgError:
            se = np.full(6, float("nan"))
        print(f"  residual RMS: {sigma*1000:.2f} mm/s2   "
              f"max |residual|: {np.abs(r).max()*1000:.2f} mm/s2")
        print(f"  (this IS meaningful now: the fit is over-determined)\n")
    else:
        se = np.full(6, float("nan"))
    if n <= 6:
        print("NOTE: exactly determined, so the fit is forced through every point.")
        print("      The residuals below are therefore ~0 by construction and say")
        print("      NOTHING about accuracy. Capture 10-12 poses for redundancy.\n")

    print("per-axis result:")
    for i, ax in enumerate("xyz"):
        ss, sb = se[3+i], se[i]
        print(f"  {ax}:  scale {scale[i]:.4f} +/- {ss:.4f}  ({100*(scale[i]-1):+.2f} %)"
              f"   bias {bias[i]:+.4f} +/- {sb:.4f} m/s2")

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

    if composed:
        abs_scale = applied_s * scale
        abs_bias = applied_b + bias * applied_s
        print(f"\nposes were captured with a correction already active:")
        print(f"  applied scale {list(np.round(applied_s, 6))}")
        print(f"  residual now  {list(np.round(scale, 6))}")
        print("\nABSOLUTE correction to put in the launch file:")
        print(f"  scale: [{abs_scale[0]:.6f}, {abs_scale[1]:.6f}, {abs_scale[2]:.6f}]")
        print(f"  bias:  [{abs_bias[0]:+.6f}, {abs_bias[1]:+.6f}, {abs_bias[2]:+.6f}]")
        print("  (replace the existing values; do not multiply them again)")
    else:
        print("\ncorrection to apply:  a_corrected[i] = (a[i] - bias[i]) / scale[i]")
        print(f"  bias:  [{bias[0]:+.6f}, {bias[1]:+.6f}, {bias[2]:+.6f}]")
        print(f"  scale: [{scale[0]:.6f}, {scale[1]:.6f}, {scale[2]:.6f}]")

    print("\nper-pose check (corrected magnitude should sit on g):")
    worst = sorted(range(n), key=lambda i: -abs(r[i]))[:5]
    for i in range(n):
        c = (A[i] - bias) / scale
        mark = "  <-- worst" if i in worst[:2] else ""
        print(f"  pose {i:>2}: |a| {np.linalg.norm(A[i]):7.3f} -> {np.linalg.norm(c):7.4f}"
              f"   resid {r[i]*1000:+7.2f} mm/s2{mark}")


if __name__ == "__main__":
    main()
