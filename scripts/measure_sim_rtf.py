#!/usr/bin/env python3
"""Measure the simulator's real-time factor from /sim/clock (sim must be running).

    ros2 launch hand_teleop_bringup sim.launch.py headless:=true &
    python3 scripts/measure_sim_rtf.py --seconds 30 --out results/sim_rtf.txt
"""

import argparse
import platform
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image


def cpu_model() -> str:
    try:
        out = subprocess.run(["lscpu"], capture_output=True, text=True).stdout
        return next(line.split(":", 1)[1].strip() for line in out.splitlines()
                    if line.startswith("Model name"))
    except Exception:
        return platform.processor()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node("measure_sim_rtf")
    clock, images = [], [0]
    node.create_subscription(
        Clock, "/sim/clock",
        lambda m: clock.append((time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9)),
        QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(Image, "/sim/front/image",
                             lambda m: images.__setitem__(0, images[0] + 1), 10)
    while not clock:
        rclpy.spin_once(node, timeout_sec=0.1)
    clock.clear()
    images[0] = 0
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)
    wall = clock[-1][0] - clock[0][0]
    rtf = (clock[-1][1] - clock[0][1]) / wall
    text = (f"real-time factor: {rtf:.4f} over {wall:.1f} s wall\n"
            f"rendering: {'on' if images[0] else 'off'} ({images[0] / wall:.1f} Hz front camera)\n"
            f"cpu: {cpu_model()}\n")
    print(text, end="")
    if args.out:
        with open(args.out, "w") as f:
            f.write(text)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
