#!/usr/bin/env python3
"""Guided live teleop test session: on-screen prompts + full data logging.

    ros2 launch hand_teleop_bringup teleop.launch.py demo_view:=true &
    python3 scripts/live_session.py --attempts 20 --out recordings/live_session

A prompt window tells the operator what to do; every topic of interest is logged with the
current protocol step. Keys in the prompt window: n = skip pick-and-place attempt, q = end.
Steps: still hold, three axis sweeps, clutch re-anchoring, gripper, e-stop (triggered by this
script), then seeded pick-and-place attempts with the shared SuccessDetector.
Analyse with scripts/analyze_live_session.py.
"""

import argparse
import json
import os
import pickle
import signal
import threading
import time

import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from hand_teleop_msgs.msg import (
    HandState,
    JointCommand,
    LatencySample,
    ObjectPoses,
    TeleopTarget,
)
from hand_teleop_msgs.srv import ResetScene
from ur5e_2f85_mujoco import load_model_names
from ur5e_2f85_mujoco.config import find_config_dir, load_config
from ur5e_2f85_mujoco.task import SuccessDetector, load_seeds

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
PREP_S = 4.0
STEPS = [
    ("still", "RIGHT hand OPEN palm, centre of the view. Hold it as STILL as you can.", 12.0),
    ("sweep_lr", "RIGHT open palm: sweep slowly LEFT <-> RIGHT across your range.", 14.0),
    ("sweep_ud", "RIGHT open palm: sweep slowly UP <-> DOWN across your range.", 14.0),
    ("sweep_fb", "RIGHT open palm: move slowly TOWARDS <-> AWAY from the camera.", 14.0),
    ("clutch", "Clutch x3: FIST, move hand elsewhere, OPEN again, move a bit.", 18.0),
    ("gripper", "RIGHT hand FIST (arm frozen). LEFT hand: pinch CLOSE / OPEN 5 times.", 14.0),
    ("estop", "RIGHT open palm, keep moving slowly. The test will E-STOP the arm, then clear it.",
     16.0),
]
ESTOP_AT, CLEAR_AT = 5.0, 10.0


def stamp(t) -> float:
    return t.sec + t.nanosec * 1e-9


class Session:
    def __init__(self, args):
        cfg = find_config_dir()
        self.task = load_config("task.yaml", cfg)
        self.names = load_model_names()
        self.lo, self.hi = self.names["gripper_joint_range"]
        self.seeds = load_seeds(cfg / "seeds.txt")[: args.attempts]
        self.args = args
        rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true", "-r", "/clock:=/sim/clock"])
        self.node = n = rclpy.create_node("live_session")
        self.log = {k: [] for k in ("hand", "hand_raw", "target", "ee", "latency", "cmd", "js",
                                    "obj", "diag", "events", "attempts")}
        self.step = "idle"
        self.latest = {"aperture": 1.0, "engaged": False, "fps": 0.0, "obj": None,
                       "grip_cmd": 1.0}
        n.create_subscription(HandState, "/hand/state", self._on_hand, 100)
        n.create_subscription(HandState, "/hand/state_raw", self._on_hand_raw, 100)
        n.create_subscription(TeleopTarget, "/teleop/target", self._on_target, 100)
        n.create_subscription(PoseStamped, "/sim/ee_pose", self._on_ee, 200)
        n.create_subscription(LatencySample, "/metrics/latency", self._on_lat, 100)
        n.create_subscription(JointCommand, "/arm/joint_command", self._on_cmd, 500)
        n.create_subscription(JointState, "/joint_states", self._on_js, 200)
        n.create_subscription(ObjectPoses, "/sim/object_poses", self._on_obj, 100)
        n.create_subscription(DiagnosticArray, "/diagnostics", self._on_diag, 10)
        self.pub_estop = n.create_publisher(Bool, "/estop", LATCHED)
        self.reset_cli = n.create_client(ResetScene, "/sim/reset")
        self.clear_cli = n.create_client(Trigger, "/estop/clear")
        # ROS callbacks run in their own thread: the GUI loop must never throttle logging
        # (one spin_once per GUI frame fell seconds behind ~560 msg/s and dropped messages).
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(n)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def wait(self, fut, timeout):
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.005)
        return fut.result() if fut.done() else None

    # ---- logging ------------------------------------------------------------------------
    def _w(self):
        return time.monotonic()

    def _on_hand(self, m):
        self.latest["fps"] = m.fps
        self.log["hand"].append((self._w(), stamp(m.capture_stamp), self.step, m.right.present,
                                 np.array(m.right.landmarks, np.float32), m.right.palm_scale,
                                 m.right.is_open, m.right.is_fist, m.left.present, m.left.pinch,
                                 m.fps))

    def _on_hand_raw(self, m):
        self.log["hand_raw"].append((self._w(), stamp(m.capture_stamp), self.step,
                                     m.right.present, np.array(m.right.landmarks, np.float32),
                                     m.right.palm_scale, m.left.pinch))

    def _on_target(self, m):
        self.latest["engaged"] = m.engaged
        p = m.ee_target.position
        self.log["target"].append((self._w(), stamp(m.header.stamp), stamp(m.capture_stamp),
                                   self.step, p.x, p.y, p.z, m.gripper, m.engaged))

    def _on_ee(self, m):
        p = m.pose.position
        self.log["ee"].append((self._w(), stamp(m.header.stamp), self.step, p.x, p.y, p.z))

    def _on_lat(self, m):
        self.log["latency"].append((self._w(), self.step, stamp(m.capture_stamp),
                                    stamp(m.hand_stamp), stamp(m.target_stamp),
                                    stamp(m.command_stamp), m.tracking_error_m))

    def _on_cmd(self, m):
        self.latest["grip_cmd"] = m.gripper
        self.log["cmd"].append((self._w(), stamp(m.header.stamp), self.step,
                                np.array(m.positions), m.gripper))

    def _on_js(self, m):
        name = self.names["ros_gripper_joint"]
        if name in m.name:
            q = m.position[m.name.index(name)]
            self.latest["aperture"] = float(np.clip(1 - (q - self.lo) / (self.hi - self.lo), 0, 1))
        self.log["js"].append((self._w(), stamp(m.header.stamp), self.step,
                               np.array(m.position[:6]), np.array(m.velocity[:6]),
                               self.latest["aperture"]))

    def _on_obj(self, m):
        self.latest["obj"] = m
        c, t = m.cube.position, m.target.position
        self.log["obj"].append((self._w(), stamp(m.header.stamp), self.step, c.x, c.y, c.z,
                                t.x, t.y, t.z))

    def _on_diag(self, m):
        for st in m.status:
            self.log["diag"].append((self._w(), st.name, st.message,
                                     {kv.key: kv.value for kv in st.values}))

    def event(self, name, **kw):
        self.log["events"].append(dict(wall=self._w(), sim=self.now(), step=self.step,
                                       name=name, **kw))

    def now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    # ---- prompt window ------------------------------------------------------------------
    def show(self, title, text, remaining=None, color=(255, 255, 255), extra=""):
        img = np.zeros((260, 1000, 3), np.uint8)
        cv2.putText(img, title, (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 200, 255), 2)
        words, lines, line = text.split(), [], ""
        for w in words:
            if len(line) + len(w) > 52:
                lines.append(line)
                line = ""
            line += w + " "
        lines.append(line)
        for i, ln in enumerate(lines):
            cv2.putText(img, ln, (20, 95 + 38 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        status = (f"{'ENGAGED' if self.latest['engaged'] else 'clutch off'}   "
                  f"hand {self.latest['fps']:.0f} fps   {extra}")
        if remaining is not None:
            status = f"{remaining:4.1f} s   " + status
        cv2.putText(img, status, (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("live_session", img)
        return cv2.waitKey(1) & 0xFF

    def spin_for(self, seconds, title, text, on_tick=None, color=(255, 255, 255), extra=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            time.sleep(0.005)
            if on_tick is not None and on_tick(seconds - (end - time.monotonic())):
                return "done"
            key = self.show(title, text, end - time.monotonic(), color,
                            extra() if extra else "")
            if key == ord("q"):
                return "quit"
            if key == ord("n"):
                return "skip"
        return "timeout"

    # ---- protocol -----------------------------------------------------------------------
    def run_step(self, name, text, dur):
        self.step = f"prep_{name}"
        if self.spin_for(PREP_S, f"NEXT: {name}", text, color=(0, 255, 255)) == "quit":
            return False
        self.step = name
        self.event("step_start")
        state = {"stopped": False, "cleared": False}

        def estop_tick(t):
            if name != "estop":
                return False
            if t >= ESTOP_AT and not state["stopped"]:
                self.pub_estop.publish(Bool(data=True))
                self.event("estop_on")
                state["stopped"] = True
            if t >= CLEAR_AT and not state["cleared"]:
                res = self.wait(self.clear_cli.call_async(Trigger.Request()), 2.0)
                self.pub_estop.publish(Bool(data=False))
                self.event("estop_clear", ok=bool(res and res.success))
                state["cleared"] = True
            return False

        def extra():
            return "E-STOP ACTIVE" if state["stopped"] and not state["cleared"] else ""

        res = self.spin_for(dur, f"NOW: {name}", text, on_tick=estop_tick, extra=extra)
        self.event("step_end")
        return res != "quit"

    def reset(self, seed):
        return self.wait(self.reset_cli.call_async(
            ResetScene.Request(seed=seed, randomize_target=False)), 5.0)

    def pick_place(self):
        for i, seed in enumerate(self.seeds):
            self.step = "pp_reset"
            if self.spin_for(2.0, f"Pick & place {i + 1}/{len(self.seeds)}",
                             "Make a FIST while the scene resets...",
                             color=(0, 255, 255)) == "quit":
                return
            res = self.reset(seed)
            cq = res.cube.orientation
            yaw = float(2 * np.arctan2(cq.z, cq.w))
            det = SuccessDetector(self.task)
            self.step = f"pp_{i + 1}"
            t0, w0 = self.now(), time.monotonic()
            info = {"engage_sim": None, "closes": 0, "closed": False}
            self.event("attempt_start", seed=seed, cube_yaw=yaw,
                       cube=[res.cube.position.x, res.cube.position.y])

            def tick(_):
                if self.latest["engaged"] and info["engage_sim"] is None:
                    info["engage_sim"] = self.now()
                g = self.latest["grip_cmd"]
                if g < 0.5 and not info["closed"]:
                    info["closes"] += 1
                    info["closed"] = True
                elif g > 0.5:
                    info["closed"] = False
                o = self.latest["obj"]
                if o is None:
                    return False
                cube = [o.cube.position.x, o.cube.position.y, o.cube.position.z]
                tgt = [o.target.position.x, o.target.position.y, o.target.position.z]
                return det.update(self.now(), cube, tgt, self.latest["aperture"])

            text = ("Pick the RED cube and place it on the GREEN disc, then OPEN the gripper. "
                    "(n = give up, q = end session)")
            out = self.spin_for(self.args.timeout, f"Pick & place {i + 1}/{len(self.seeds)} "
                                f"(seed {seed}, yaw {np.degrees(yaw):.0f} deg)", text,
                                on_tick=tick)
            outcome = {"done": "success", "skip": "gave_up", "quit": "quit"}.get(out, "timeout")
            t1 = self.now()
            row = dict(attempt=i + 1, seed=seed, cube_yaw_deg=round(float(np.degrees(yaw)), 1),
                       outcome=outcome, success=outcome == "success",
                       time_from_reset_s=round(t1 - t0, 2),
                       time_from_engage_s=round(t1 - info["engage_sim"], 2)
                       if info["engage_sim"] else None,
                       regrasps=max(0, info["closes"] - 1), closes=info["closes"],
                       wall_s=round(time.monotonic() - w0, 2))
            self.log["attempts"].append(row)
            self.event("attempt_end", **row)
            msg = "SUCCESS!" if row["success"] else outcome.upper()
            self.spin_for(1.5, msg, f"{row['time_from_reset_s']} s", color=(0, 255, 0))
            if outcome == "quit":
                return

    def save(self):
        os.makedirs(self.args.out, exist_ok=True)
        with open(os.path.join(self.args.out, "session.pkl"), "wb") as f:
            pickle.dump(self.log, f)
        with open(os.path.join(self.args.out, "attempts.json"), "w") as f:
            json.dump(self.log["attempts"], f, indent=1)
        print(f"saved {sum(len(v) for v in self.log.values())} records to {self.args.out}")


def _interrupt(signum, frame):
    raise KeyboardInterrupt  # so the finally block saves the data on SIGTERM too


def main() -> None:
    signal.signal(signal.SIGTERM, _interrupt)
    ap = argparse.ArgumentParser()
    ap.add_argument("--attempts", type=int, default=20)
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--out", default="recordings/live_session")
    ap.add_argument("--skip-protocol", action="store_true", help="pick-and-place only")
    ap.add_argument("--step-scale", type=float, default=1.0, help="<1 shortens steps (dry run)")
    args = ap.parse_args()
    s = Session(args)
    cv2.namedWindow("live_session", cv2.WINDOW_NORMAL)
    try:
        s.run = True
        if not s.reset_cli.wait_for_service(timeout_sec=15.0):
            raise SystemExit("/sim/reset not available: start teleop.launch.py first")
        s.event("session_start")
        s.spin_for(3.0, "Live test session", "Starting. Watch this window for instructions.",
                   color=(0, 255, 255))
        ok = True
        if not args.skip_protocol:
            s.reset(s.seeds[0])
            for name, text, dur in STEPS:
                if not s.run_step(name, text, max(dur * args.step_scale, CLEAR_AT + 1.0)
                                  if name == "estop" else dur * args.step_scale):
                    ok = False
                    break
        if ok:
            s.pick_place()
        s.event("session_end")
        s.show("DONE", "Session finished. Thank you!")
        cv2.waitKey(1500)
    except KeyboardInterrupt:
        s.event("interrupted")
    finally:
        s.save()
        cv2.destroyAllWindows()
        s.executor.shutdown()
        s.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
