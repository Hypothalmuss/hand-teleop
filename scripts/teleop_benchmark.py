#!/usr/bin/env python3
"""Teleop pick-and-place benchmark: 20 seeded attempts, success/time/re-grasps.

    ros2 launch hand_teleop_bringup teleop.launch.py &
    python3 scripts/teleop_benchmark.py [--attempts 20] [--timeout 60]

For each attempt the script resets the sim to the next seed and watches for success (cube
within 3 cm of the target centre, cube on the table, gripper open, held 1 s: the shared
SuccessDetector). Press Enter to give up on an attempt. Writes results/teleop_benchmark.csv
and results/teleop_benchmark.md.
"""

import argparse
import csv
import os
import select
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import JointState

from hand_teleop_msgs.msg import JointCommand, ObjectPoses, TeleopTarget
from hand_teleop_msgs.srv import ResetEpisode
from ur5e_2f85_mujoco import load_model_names
from ur5e_2f85_mujoco.config import find_config_dir, load_config
from ur5e_2f85_mujoco.task import SuccessDetector, load_seeds


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--attempts", type=int, default=20)
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--seeds-file", default="seeds_train.txt")
    ap.add_argument("--out-dir", default="results")
    args = ap.parse_args()
    cfg = find_config_dir()
    task = load_config("task.yaml", cfg)
    names = load_model_names()
    lo, hi = names["gripper_joint_range"]
    seeds = load_seeds(cfg / args.seeds_file)[: args.attempts]

    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true", "-r", "/clock:=/sim/clock"])
    node = rclpy.create_node("teleop_benchmark")
    st = {"obj": None, "aperture": 1.0, "engaged": False, "grip_cmd": 1.0}
    node.create_subscription(ObjectPoses, "/sim/object_poses",
                             lambda m: st.__setitem__("obj", m), 10)
    grip_name = names["ros_gripper_joint"]

    def on_js(m: JointState) -> None:
        if grip_name in m.name:
            q = m.position[m.name.index(grip_name)]
            st["aperture"] = float(np.clip(1.0 - (q - lo) / (hi - lo), 0.0, 1.0))

    node.create_subscription(JointState, "/joint_states", on_js, 10)
    node.create_subscription(TeleopTarget, "/teleop/target",
                             lambda m: st.__setitem__("engaged", m.engaged), 10)
    node.create_subscription(JointCommand, "/arm/joint_command",
                             lambda m: st.__setitem__("grip_cmd", m.gripper), 10)
    reset = node.create_client(ResetEpisode, "/sim/reset")
    if not reset.wait_for_service(timeout_sec=10.0):
        raise SystemExit("/sim/reset not available: start teleop.launch.py first")
    now = lambda: node.get_clock().now().nanoseconds * 1e-9  # noqa: E731

    rows = []
    stdin_open = True
    for i, seed in enumerate(seeds):
        fut = reset.call_async(ResetEpisode.Request(seed=seed, randomize_target=False))
        rclpy.spin_until_future_complete(node, fut)
        det = SuccessDetector(task)
        t_reset, t_engage, closes, closed = now(), None, 0, False
        print(f"\nattempt {i + 1}/{len(seeds)} seed {seed}: go (Enter = give up)")
        outcome = "timeout"
        while now() - t_reset < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.02)
            if stdin_open and select.select([sys.stdin], [], [], 0)[0]:
                if sys.stdin.readline() == "":  # EOF (no terminal): stop polling stdin
                    stdin_open = False
                else:
                    outcome = "gave_up"
                    break
            if st["engaged"] and t_engage is None:
                t_engage = now()
            if st["grip_cmd"] < 0.5 and not closed:
                closes, closed = closes + 1, True
            elif st["grip_cmd"] > 0.5:
                closed = False
            o = st["obj"]
            if o is None:
                continue
            cube = [o.cube.position.x, o.cube.position.y, o.cube.position.z]
            tgt = [o.target.position.x, o.target.position.y, o.target.position.z]
            if det.update(now(), cube, tgt, st["aperture"]):
                outcome = "success"
                break
        t_done = now()
        row = {"attempt": i + 1, "seed": seed, "success": outcome == "success",
               "outcome": outcome, "time_from_reset_s": round(t_done - t_reset, 2),
               "time_from_engage_s": round(t_done - t_engage, 2) if t_engage else "",
               "regrasps": max(0, closes - 1)}
        rows.append(row)
        print(f"  -> {outcome} in {row['time_from_reset_s']} s, re-grasps {row['regrasps']}")
    node.destroy_node()
    rclpy.shutdown()

    os.makedirs(args.out_dir, exist_ok=True)
    with open(os.path.join(args.out_dir, "teleop_benchmark.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    ok = [r for r in rows if r["success"]]
    med = float(np.median([r["time_from_reset_s"] for r in ok])) if ok else float("nan")
    text = ("# Teleop pick-and-place benchmark\n\n"
            "| Metric | Value | Target |\n| --- | --- | --- |\n"
            f"| Success | {len(ok)} / {len(rows)} | >= 17 / 20 |\n"
            f"| Median completion time (successes, from reset) | {med:.1f} s | < 25 s |\n"
            f"| Mean re-grasps | {np.mean([r['regrasps'] for r in rows]):.2f} | |\n")
    with open(os.path.join(args.out_dir, "teleop_benchmark.md"), "w") as f:
        f.write(text)
    print("\n" + text)


if __name__ == "__main__":
    main()
