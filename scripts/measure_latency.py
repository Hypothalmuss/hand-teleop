#!/usr/bin/env python3
"""End-to-end latency and tracking error from /metrics/latency during live teleop.

    ros2 launch hand_teleop_bringup teleop.launch.py &      # then teleoperate
    python3 scripts/measure_latency.py --seconds 60

Hops (all stamps are sim time, 10 ms clock resolution): capture->hand (tracker),
hand->target (mapper), target->command (IK), capture->command (end to end). Tracking error is
|target - TCP| when the command is sent, reported for samples with target speed <= 0.2 m/s.
Writes results/latency.csv and results/latency.md.
"""

import argparse
import csv
import datetime
import os
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy

from hand_teleop_msgs.msg import LatencySample, TeleopTarget

HOPS = [("capture_to_hand", "capture_stamp", "hand_stamp"),
        ("hand_to_target", "hand_stamp", "target_stamp"),
        ("target_to_command", "target_stamp", "command_stamp"),
        ("capture_to_command", "capture_stamp", "command_stamp")]
SPEED_LIMIT = 0.2  # m/s, tracking error is reported at or below this target speed


def ns(t) -> int:
    return t.sec * 1_000_000_000 + t.nanosec


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--out-dir", default="results")
    ap.add_argument("--label", default="live webcam teleop")
    args = ap.parse_args()
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true", "-r", "/clock:=/sim/clock"])
    node = rclpy.create_node("measure_latency")
    samples, targets = [], {}
    node.create_subscription(LatencySample, "/metrics/latency", samples.append,
                             QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE))

    last = {}

    def on_target(m: TeleopTarget) -> None:
        p = np.array([m.ee_target.position.x, m.ee_target.position.y, m.ee_target.position.z])
        t = ns(m.header.stamp) * 1e-9
        speed = np.nan
        if "t" in last and t > last["t"]:
            speed = float(np.linalg.norm(p - last["p"]) / (t - last["t"]))
        last.update(t=t, p=p)
        targets[ns(m.capture_stamp)] = (speed, m.engaged)

    node.create_subscription(TeleopTarget, "/teleop/target", on_target, 100)
    print(f"recording /metrics/latency for {args.seconds:.0f} s ...")
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    if not samples:
        raise SystemExit("no LatencySample received (is diff_ik_controller running?)")

    os.makedirs(args.out_dir, exist_ok=True)
    rows = []
    for s in samples:
        row = {h: (ns(getattr(s, b)) - ns(getattr(s, a))) * 1e-6 if ns(getattr(s, a)) else np.nan
               for h, a, b in HOPS}
        speed, engaged = targets.get(ns(s.capture_stamp), (np.nan, False))
        row.update(tracking_error_mm=s.tracking_error_m * 1e3, target_speed=speed,
                   engaged=engaged)
        rows.append(row)
    with open(os.path.join(args.out_dir, "latency.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    lines = [f"# Teleop latency ({args.label})", "",
             f"{len(rows)} samples over {args.seconds:.0f} s, "
             f"{datetime.date.today().isoformat()}. Stamps are sim time (10 ms resolution).", "",
             "| Hop | p50 [ms] | p95 [ms] | p99 [ms] |", "| --- | --- | --- | --- |"]
    for h, _, _ in HOPS:
        v = np.array([r[h] for r in rows], float)
        v = v[~np.isnan(v)]
        if len(v):
            lines.append(f"| {h} | {np.percentile(v, 50):.1f} | {np.percentile(v, 95):.1f} | "
                         f"{np.percentile(v, 99):.1f} |")
        else:
            lines.append(f"| {h} | n/a | n/a | n/a |")
    trk = np.array([r["tracking_error_mm"] for r in rows
                    if r["engaged"] and r["target_speed"] <= SPEED_LIMIT], float)
    if len(trk):
        lines += ["", f"Tracking error RMS while engaged at target speed <= {SPEED_LIMIT} m/s: "
                      f"{np.sqrt(np.mean(trk ** 2)):.1f} mm ({len(trk)} samples)."]
    text = "\n".join(lines) + "\n"
    with open(os.path.join(args.out_dir, "latency.md"), "w") as f:
        f.write(text)
    print(text)


if __name__ == "__main__":
    main()
