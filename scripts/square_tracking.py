#!/usr/bin/env python3
"""Trace a 10 cm square at 0.1 m/s through the running IK + sim and plot target vs TCP.

    ros2 launch hand_teleop_bringup ik_sim.launch.py headless:=true &
    python3 scripts/square_tracking.py --out results/phase5_square_tracking.png
"""

import argparse
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402

from diff_ik_controller.square import SquarePath  # noqa: E402
from hand_teleop_msgs.msg import TeleopTarget  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="results/phase5_square_tracking.png")
    ap.add_argument("--speed", type=float, default=0.10)
    args = ap.parse_args()
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true", "-r", "/clock:=/sim/clock"])
    node = rclpy.create_node("square_tracking")
    ee, sent = [], []
    node.create_subscription(PoseStamped, "/sim/ee_pose", lambda m: ee.append(
        (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
         np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]),
         m.pose.orientation)), 100)
    pub = node.create_publisher(TeleopTarget, "/teleop/target", 10)
    while len(ee) < 20:
        rclpy.spin_once(node, timeout_sec=0.05)
    start, quat = ee[-1][1], ee[-1][2]
    path = SquarePath(start, center=[0.45, 0.0, 0.20], speed=args.speed)
    t0 = time.monotonic()
    next_pub = t0
    while time.monotonic() - t0 < path.duration + 1.0:
        if time.monotonic() >= next_pub:
            p = path(time.monotonic() - t0)
            msg = TeleopTarget()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.capture_stamp = msg.header.stamp
            msg.ee_target.position.x, msg.ee_target.position.y, msg.ee_target.position.z = p
            msg.ee_target.orientation = quat
            msg.engaged, msg.gripper = True, 1.0
            pub.publish(msg)
            sent.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, p))
            next_pub += 1 / 30
        rclpy.spin_once(node, timeout_sec=0.002)
    node.destroy_node()
    rclpy.shutdown()

    ts = np.array([s[0] for s in sent])
    ps = np.array([s[1] for s in sent])
    lap = [(t, p) for t, p, _ in ee if ts[0] + path.lead_in + 0.5 <= t <= ts[0] + path.duration]
    errs = np.array([np.linalg.norm(p - [np.interp(t, ts, ps[:, i]) for i in range(3)])
                     for t, p in lap])
    rms = float(np.sqrt(np.mean(errs ** 2)))
    tcp = np.array([p for t, p, _ in ee if t >= ts[0]])
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(ps[:, 1], ps[:, 0], "-", color="#2a7ab9", lw=2, label="target (30 Hz)")
    ax.plot(tcp[:, 1], tcp[:, 0], "-", color="#d95f02", lw=1, label="TCP (sim, 100 Hz)")
    ax.set_xlabel("y [m]")
    ax.set_ylabel("x [m]")
    ax.set_aspect("equal")
    ax.set_title(f"10 cm square at {args.speed:.2f} m/s: RMS {rms * 1000:.2f} mm, "
                 f"max {errs.max() * 1000:.2f} mm")
    ax.legend(loc="upper right")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print(f"RMS {rms * 1000:.2f} mm, max {errs.max() * 1000:.2f} mm over {len(errs)} samples; "
          f"wrote {args.out}")


if __name__ == "__main__":
    main()
