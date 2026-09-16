#!/usr/bin/env python3
"""Capture static accelerometer poses for scale/bias calibration.

Rather than demanding six exact axis-aligned faces, this records any number of
arbitrary stationary orientations. The solver then fits per-axis scale and bias
so the corrected magnitude equals g in every pose. That tolerates sloppy
alignment, which matters for a rig that has no flat faces.

What matters is not precision but SPREAD: the poses must sample directions all
over the sphere, and each axis must see gravity both positive and negative.
Six poses clustered near one orientation will fit badly however carefully they
are held.

Usage:
    ros2 run rtimulib_ros2 capture_accel_poses.py        # or run directly
    press ENTER to capture a pose, q then ENTER to finish
"""

import json
import math
import os
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

SETTLE_S = 2.0          # must be still this long before recording starts
RECORD_S = 15.0         # averaging window per pose
# Gyro STANDARD DEVIATION, not magnitude. The node publishes raw gyro with bias
# left in (up to 0.045 rad/s on this unit), so an absolute |w| test can never
# pass on a stationary rig. Variation is what distinguishes still from moving.
GYRO_STILL_SD = 0.01    # rad/s
ACCEL_STILL = 0.05      # m/s^2, max per-axis sd to count as stationary


class Collector(Node):
    def __init__(self):
        super().__init__("accel_pose_capture")
        self.buf = []
        self.lock = threading.Lock()
        self.create_subscription(Imu, "/imu", self.cb, qos_profile_sensor_data)

    def cb(self, m):
        with self.lock:
            self.buf.append((
                time.time(),
                (m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z),
                (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z),
            ))
            # keep the last ~30 s only
            cutoff = time.time() - 30.0
            while self.buf and self.buf[0][0] < cutoff:
                self.buf.pop(0)

    def window(self, secs):
        now = time.time()
        with self.lock:
            return [r for r in self.buf if r[0] >= now - secs]


def stats(rows):
    n = len(rows)
    if n < 10:
        return None
    a = [r[1] for r in rows]
    g = [r[2] for r in rows]
    mean = [sum(x[i] for x in a)/n for i in range(3)]
    sd = [math.sqrt(sum((x[i]-mean[i])**2 for x in a)/n) for i in range(3)]
    gmean = [sum(x[i] for x in g)/n for i in range(3)]
    gsd = max(math.sqrt(sum((x[i]-gmean[i])**2 for x in g)/n) for i in range(3))
    return mean, sd, gsd, n


def is_still(rows):
    s = stats(rows)
    if not s:
        return False, "no data"
    _, sd, gsd, _ = s
    if gsd > GYRO_STILL_SD:
        return False, f"rotating (gyro sd {gsd:.4f} rad/s)"
    if max(sd) > ACCEL_STILL:
        return False, f"vibrating (accel sd up to {max(sd):.3f} m/s2)"
    return True, "still"


def coverage(poses):
    """How well do the captured directions span the sphere?

    Reports the smallest eigenvalue of the direction scatter matrix: near zero
    means the directions are coplanar or clustered and scale along some axis
    will be poorly determined."""
    if len(poses) < 3:
        return 0.0, [0, 0, 0]
    dirs = []
    for p in poses:
        m = p["mean"]
        n = math.sqrt(sum(c*c for c in m))
        dirs.append([c/n for c in m])
    M = [[sum(d[i]*d[j] for d in dirs)/len(dirs) for j in range(3)] for i in range(3)]
    # power iteration is overkill; use the per-axis extremes as a readable proxy
    ext = []
    for i in range(3):
        col = [d[i] for d in dirs]
        ext.append((min(col), max(col)))
    worst = min(hi - lo for lo, hi in ext)
    return worst, ext


def active_correction():
    """Read the correction the node is already applying.

    Poses captured while a correction is live are already corrected, so solving
    them yields a RESIDUAL, not an absolute calibration. Recording what was
    active lets the solver compose the two instead of double-correcting."""
    got = {}
    for name in ("accel_scale", "accel_bias"):
        try:
            r = subprocess.run(["ros2", "param", "get", "/rtimulib_node", name],
                               capture_output=True, text=True, timeout=10)
            txt = r.stdout.strip()
            nums = [float(x) for x in
                    txt.replace("[", " ").replace("]", " ").replace(",", " ").split()
                    if x.replace("-", "").replace(".", "").replace("e", "").isdigit()
                    or (x.startswith("-") and x[1:].replace(".", "").isdigit())]
            if len(nums) >= 3:
                got[name] = nums[-3:]
        except Exception:
            pass
    return got


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "accel_poses.json"
    rclpy.init()
    node = Collector()
    stop = threading.Event()

    def spin_loop():
        # Not rclpy.spin(): it cannot be interrupted cleanly from another
        # thread, and tearing it down at exit aborts the process.
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)

    spin = threading.Thread(target=spin_loop, daemon=True)
    spin.start()

    # Wait for the topic before prompting, so a dead IMU reports as a dead IMU
    # rather than surfacing later as a confusing "not stationary: no data".
    print("waiting for /imu ...", end="", flush=True)
    deadline = time.time() + 15.0
    while time.time() < deadline and len(node.window(5.0)) < 20:
        time.sleep(0.2)
    if len(node.window(5.0)) < 20:
        print("\n/imu is not publishing. Is sensor-rig-imu.service running?")
        stop.set(); spin.join(timeout=2.0); node.destroy_node(); rclpy.shutdown()
        sys.exit(1)
    print(" ok\n")

    active = active_correction()
    sc = active.get("accel_scale", [1.0, 1.0, 1.0])
    bs = active.get("accel_bias", [0.0, 0.0, 0.0])
    if any(abs(v - 1.0) > 1e-9 for v in sc) or any(abs(v) > 1e-9 for v in bs):
        print(f"\nNOTE: the node is already applying a correction:")
        print(f"  scale {sc}")
        print(f"  bias  {bs}")
        print("  These poses will therefore be ALREADY CORRECTED. The values are")
        print("  recorded in the output and the solver composes them, so the")
        print("  result it prints is the absolute calibration, not a residual.\n")

    print(__doc__)
    print(f"writing to {os.path.abspath(out)}\n")
    print("Aim for at least 6 poses. Each axis needs gravity both ways, so")
    print("include upside-down and on-edge orientations, not just tilts.\n")

    poses = []
    while True:
        try:
            cmd = input(f"[{len(poses)} captured] position the rig, ENTER to capture (q to finish): ")
        except EOFError:
            break
        if cmd.strip().lower().startswith("q"):
            break

        ok, why = is_still(node.window(SETTLE_S))
        if not ok:
            print(f"  not stationary: {why} - hold it still and try again")
            continue

        print(f"  settling {SETTLE_S:.0f}s...", end="", flush=True)
        time.sleep(SETTLE_S)
        ok, why = is_still(node.window(SETTLE_S))
        if not ok:
            print(f"\r  moved during settle: {why} - discarded")
            continue

        print(f"\r  recording {RECORD_S:.0f}s... ", end="", flush=True)
        time.sleep(RECORD_S)
        rows = node.window(RECORD_S)
        ok, why = is_still(rows)
        s = stats(rows)
        if not ok or not s:
            print(f"\r  moved while recording: {why} - discarded          ")
            continue

        mean, sd, gsd, n = s
        mag = math.sqrt(sum(c*c for c in mean))
        dom = max(range(3), key=lambda i: abs(mean[i]))
        print(f"\r  captured: [{mean[0]:+7.3f} {mean[1]:+7.3f} {mean[2]:+7.3f}]  "
              f"|a|={mag:.3f}  dominant {'xyz'[dom]}{'+' if mean[dom] > 0 else '-'}  "
              f"n={n}")
        poses.append({"mean": mean, "sd": sd, "samples": n, "magnitude": mag})

        worst, ext = coverage(poses)
        if len(poses) >= 3:
            print(f"     direction spread (want >1.5 on every axis): "
                  f"x {ext[0][1]-ext[0][0]:.2f}  y {ext[1][1]-ext[1][0]:.2f}  "
                  f"z {ext[2][1]-ext[2][0]:.2f}")

        with open(out, "w") as f:
            json.dump({"poses": poses, "applied_scale": sc, "applied_bias": bs},
                      f, indent=2)

    stop.set()
    spin.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()

    if len(poses) < 6:
        print(f"\nonly {len(poses)} poses - the solve needs at least 6 well spread")
    else:
        worst, ext = coverage(poses)
        print(f"\n{len(poses)} poses written to {out}")
        if worst < 1.5:
            print(f"WARNING: weakest axis spread is {worst:.2f}; add poses that put")
            print("gravity along that axis in both directions before solving.")
        else:
            print("spread looks adequate for a scale+bias solve.")


if __name__ == "__main__":
    main()
