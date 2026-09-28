"""Scripted joint commands for the simulator: a joint-space sine sweep or scripted
pick-and-place runs (Python IK oracle). Runs on sim time.

    ros2 run mujoco_sim_ros scripted_trajectory --ros-args -p mode:=pick_place -p runs:=3

Each finished pick-and-place run is reported on ``scripted/result`` as JSON.
"""

from __future__ import annotations

import json

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from hand_teleop_msgs.msg import JointCommand, ObjectPoses
from hand_teleop_msgs.srv import ResetScene
from ur5e_2f85_mujoco import load_model, load_model_names
from ur5e_2f85_mujoco.config import find_config_dir, load_config
from ur5e_2f85_mujoco.kinematics import Kinematics
from ur5e_2f85_mujoco.scripted import ScriptedController

SINE_PHASES = np.array([0.0, 1.0, 2.0, 3.0, 4.0, 5.0])


class ScriptedTrajectory(Node):
    def __init__(self):
        super().__init__("scripted_trajectory")
        self.declare_parameter("config_dir", "")
        self.declare_parameter("mode", "pick_place")      # sine | pick_place
        self.declare_parameter("seed", 0)
        self.declare_parameter("runs", 1)
        self.declare_parameter("sine_amplitude", 0.3)     # rad
        self.declare_parameter("sine_frequency", 0.2)     # Hz
        cfg_dir = find_config_dir(self.get_parameter("config_dir").value or None)
        self.rates = load_config("rates.yaml", cfg_dir)
        self.task = load_config("task.yaml", cfg_dir)
        self.mode = self.get_parameter("mode").value
        self.names = load_model_names()
        self.kin = Kinematics(load_model(), self.names)

        self.pub = self.create_publisher(JointCommand, "arm/joint_command", 10)
        self.pub_result = self.create_publisher(String, "scripted/result", 10)
        self.create_subscription(JointState, "joint_states", self._on_js, 10)
        self.create_subscription(ObjectPoses, "sim/object_poses", self._on_obj, 10)
        self.reset_cli = self.create_client(ResetScene, "sim/reset")

        self.q_meas = None
        self.objects = None
        self.t0 = None
        self.ctl = None
        self.target_pos = None
        self.run_index = 0
        self.seed = int(self.get_parameter("seed").value)
        self.waiting_reset = False
        self.timer = self.create_timer(1.0 / self.rates["control"], self._on_timer)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_js(self, msg: JointState) -> None:
        self.q_meas = np.array(msg.position[:6])

    def _on_obj(self, msg: ObjectPoses) -> None:
        self.objects = msg

    def _publish(self, q, grip) -> None:
        msg = JointCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.capture_stamp = msg.header.stamp
        msg.positions = [float(v) for v in q]
        msg.gripper = float(grip)
        self.pub.publish(msg)

    # ---- sine sweep -----------------------------------------------------------------------
    def _sine(self) -> None:
        if self.t0 is None:
            if self.q_meas is None or self._now() == 0.0:
                return
            self.t0 = self._now()
            self.q0 = self.kin.q_home.copy()
        t = self._now() - self.t0
        amp = float(self.get_parameter("sine_amplitude").value) * min(1.0, t / 1.0)
        f = float(self.get_parameter("sine_frequency").value)
        q = self.q0 + amp * np.sin(2 * np.pi * f * t + SINE_PHASES)
        self._publish(q, 0.5 + 0.5 * np.cos(2 * np.pi * f * t))

    # ---- pick and place -------------------------------------------------------------------
    def _start_run(self) -> None:
        if not self.reset_cli.service_is_ready():
            return
        self.waiting_reset = True
        req = ResetScene.Request(seed=self.seed + self.run_index, randomize_target=False)
        self.reset_cli.call_async(req).add_done_callback(self._on_reset_done)

    def _on_reset_done(self, fut) -> None:
        res = fut.result()
        cube = np.array([res.cube.position.x, res.cube.position.y, res.cube.position.z])
        qw, qz = res.cube.orientation.w, res.cube.orientation.z
        self.target_pos = np.array([res.target.position.x, res.target.position.y,
                                    res.target.position.z])
        self.ctl = ScriptedController(self.kin)
        start = self.kin.fk(self.kin.q_home)[0]
        self.ctl.plan_pick_place(start, cube, 2 * np.arctan2(qz, qw), self.target_pos)
        self.t0 = self._now()
        self.waiting_reset = False

    def _pick_place(self) -> None:
        if self.waiting_reset:
            return
        if self.ctl is None:
            if self.run_index < int(self.get_parameter("runs").value):
                self._start_run()
            return
        t = self._now() - self.t0
        q, grip, _ = self.ctl.command(t)
        self._publish(q, grip)
        if t > self.ctl.duration + 0.5:
            self._report()
            self.ctl = None
            self.run_index += 1

    def _report(self) -> None:
        cube = self.objects.cube.position if self.objects else None
        dist = float("nan")
        if cube is not None:
            dist = float(np.hypot(cube.x - self.target_pos[0], cube.y - self.target_pos[1]))
        ok = bool(dist < self.task["success"]["xy_tol"])
        result = {"run": self.run_index, "seed": self.seed + self.run_index,
                  "dist_xy": dist, "success": ok}
        self.pub_result.publish(String(data=json.dumps(result)))
        self.get_logger().info(f"pick_place result {result}")

    def _on_timer(self) -> None:
        if self.mode == "sine":
            self._sine()
        else:
            self._pick_place()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScriptedTrajectory()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
