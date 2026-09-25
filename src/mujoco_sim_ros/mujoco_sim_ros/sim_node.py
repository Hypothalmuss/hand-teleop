"""``mujoco_sim`` node: real-time MuJoCo physics, cameras, reset service, tf.

Physics thread: every control tick (1 / rates.control) applies the latest joint command
(limit + max-step clamp), runs ``physics / control`` mj_steps and publishes the state topics.
Render thread: renders both cameras from a copied state snapshot at ``rates.sim_images`` so
rendering never stalls physics. Every stamp is sim time; the node publishes ``/sim/clock``.
"""

from __future__ import annotations

import array
import os
import queue
import threading
import time

import mujoco
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from hand_teleop_msgs.msg import JointCommand, ObjectPoses
from hand_teleop_msgs.srv import ResetEpisode
from ur5e_2f85_mujoco import load_model, load_model_names
from ur5e_2f85_mujoco.config import find_config_dir, load_config
from ur5e_2f85_mujoco.kinematics import mat_to_quat
from ur5e_2f85_mujoco.task import (
    SceneIndex,
    make_servo,
    reset_episode,
)

STALE_COMMAND_S = 0.5


def to_time_msg(t: float) -> Time:
    ns = int(round(t * 1e9))
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def to_pose(pos, quat) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in pos)
    p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z = (float(v) for v in quat)
    return p


class Periodic:
    """Fires once per ``1 / rate`` of sim time."""

    def __init__(self, rate: float):
        self.period = 1.0 / rate
        self.next = 0.0

    def due(self, t: float) -> bool:
        if t + 1e-9 >= self.next:
            self.next = max(self.next + self.period, t - self.period)
            return True
        return False


class MujocoSim(Node):
    def __init__(self):
        super().__init__("mujoco_sim")
        self.declare_parameter("config_dir", "")
        self.declare_parameter("viewer", False)
        self.declare_parameter("render", True)
        self.declare_parameter("seed", 0)
        self.declare_parameter("rt_factor", 1.0)  # >1 runs faster than real time (tests)
        cfg_dir = find_config_dir(self.get_parameter("config_dir").value or None)
        self.rates = load_config("rates.yaml", cfg_dir)
        self.task = load_config("task.yaml", cfg_dir)
        self.ik_cfg = load_config("ik.yaml", cfg_dir)
        self.rt_factor = float(self.get_parameter("rt_factor").value)

        self.model = load_model()
        self.names = load_model_names()
        self.data = mujoco.MjData(self.model)
        self.idx = SceneIndex.build(self.model, self.names)
        self.steps_per_tick = int(round(self.rates["physics"] / self.rates["control"]))
        self.dt_tick = self.steps_per_tick * self.model.opt.timestep
        self.servo = make_servo(self.model, self.idx, self.rates, self.ik_cfg)

        self.lock = threading.Lock()
        self.sim_time = 0.0
        self.generation = 0          # bumps on every reset; stale renders are dropped
        self._stale_warned = False
        self.render_dropped = 0
        self.render_ms = []
        self._reset_scene(int(self.get_parameter("seed").value))

        # --- ROS interfaces (relative names; remapped in hand_teleop_bringup) ---
        self.pub_js = self.create_publisher(JointState, "joint_states", 10)
        self.pub_ee = self.create_publisher(PoseStamped, "sim/ee_pose", 10)
        self.pub_obj = self.create_publisher(ObjectPoses, "sim/object_poses", 10)
        self.pub_clock = self.create_publisher(
            Clock, "sim/clock", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.pub_img = {c: self.create_publisher(Image, f"sim/{c}/image", 2)
                        for c in self.names["cameras"]}
        self.create_subscription(JointCommand, "arm/joint_command", self._on_command, 10)
        self.create_service(ResetEpisode, "sim/reset", self._on_reset)
        self.tf = TransformBroadcaster(self)
        static = StaticTransformBroadcaster(self)
        st = TransformStamped()
        st.header.frame_id, st.child_frame_id = "world", "base_link"
        st.transform.rotation.w = 1.0
        static.sendTransform(st)

        self.pub_timers = {k: Periodic(self.rates[k])
                       for k in ("joint_states", "ee_pose", "clock", "object_poses",
                                 "sim_images")}
        self.render_queue: queue.Queue = queue.Queue(maxsize=1)
        self.running = True
        self.viewer = None
        self.physics_thread = threading.Thread(target=self._physics_loop, daemon=True)
        self.render_thread = threading.Thread(target=self._render_loop, daemon=True)
        if self.get_parameter("render").value:
            self.render_thread.start()
        self.physics_thread.start()
        self.get_logger().info(
            f"mujoco_sim up: {self.rates['physics']:.0f} Hz physics, "
            f"{self.rates['control']:.0f} Hz control, config {cfg_dir}")

    # ---- command / reset ------------------------------------------------------------------
    def _reset_scene(self, seed: int, randomize_target: bool = False) -> dict:
        ep = reset_episode(self.model, self.data, self.idx, self.task, seed, randomize_target)
        self.servo.reset(self.data.ctrl[self.idx.arm_act].copy(), 1.0)
        self.last_cmd_wall = None
        self.sim_time += self.task["settle_steps"] * self.model.opt.timestep
        self.generation += 1
        return ep

    def _on_command(self, msg: JointCommand) -> None:
        with self.lock:
            self.servo.set_command(msg.positions, msg.gripper)
            self.last_cmd_wall = time.monotonic()
            self._stale_warned = False

    def _on_reset(self, req: ResetEpisode.Request, res: ResetEpisode.Response):
        with self.lock:
            ep = self._reset_scene(int(req.seed), bool(req.randomize_target))
            try:
                self.render_queue.get_nowait()
            except queue.Empty:
                pass
        res.cube = to_pose(ep["cube_pos"], ep["cube_quat"])
        res.target = to_pose(ep["target_pos"], ep["target_quat"])
        self.get_logger().info(
            f"reset seed={req.seed} cube=({ep['cube_pos'][0]:.3f}, {ep['cube_pos'][1]:.3f})")
        return res

    # ---- physics --------------------------------------------------------------------------
    def _tick(self) -> list:
        """One control tick. Returns (publisher, msg) pairs to publish outside the lock."""
        m, d, idx = self.model, self.data, self.idx
        if (self.last_cmd_wall is not None and not self._stale_warned
                and time.monotonic() - self.last_cmd_wall > STALE_COMMAND_S):
            self.get_logger().warn("no joint command for 0.5 s: holding the last command")
            self._stale_warned = True
        if self.servo.tick(d):
            n = self.servo.clamp_count
            if n in (1, 10, 100) or n % 1000 == 0:
                self.get_logger().warn(f"joint command clamped ({n} ticks)")
        for _ in range(self.steps_per_tick):
            mujoco.mj_step(m, d)
        self.sim_time += self.dt_tick
        t = self.sim_time
        stamp = to_time_msg(t)
        out = []
        if self.pub_timers["clock"].due(t):
            out.append((self.pub_clock, Clock(clock=stamp)))
        tcp_pos = d.site_xpos[idx.tcp_site].copy()
        tcp_quat = mat_to_quat(d.site_xmat[idx.tcp_site])
        if self.pub_timers["joint_states"].due(t):
            js = JointState()
            js.header.stamp = stamp
            js.name = list(self.names["arm_joints"]) + [self.names["ros_gripper_joint"]]
            js.position = [float(v) for v in d.qpos[idx.arm_qpos]] + [
                float(d.qpos[idx.grip_qpos])]
            js.velocity = [float(v) for v in d.qvel[idx.arm_dof]] + [
                float(d.qvel[self.model.jnt_dofadr[self.model.joint(
                    self.names["gripper_joint"]).id]])]
            out.append((self.pub_js, js))
        if self.pub_timers["ee_pose"].due(t):
            ps = PoseStamped()
            ps.header.stamp, ps.header.frame_id = stamp, "base_link"
            ps.pose = to_pose(tcp_pos, tcp_quat)
            out.append((self.pub_ee, ps))
            tfm = TransformStamped()
            tfm.header = ps.header
            tfm.child_frame_id = "tcp"
            tfm.transform.translation.x, tfm.transform.translation.y, \
                tfm.transform.translation.z = (float(v) for v in tcp_pos)
            tfm.transform.rotation = ps.pose.orientation
            out.append((self.tf, tfm))
        if self.pub_timers["object_poses"].due(t):
            op = ObjectPoses()
            op.header.stamp, op.header.frame_id = stamp, "base_link"
            op.cube = to_pose(d.xpos[idx.cube_body], d.xquat[idx.cube_body])
            op.target = to_pose(d.mocap_pos[idx.target_mocap], d.mocap_quat[idx.target_mocap])
            out.append((self.pub_obj, op))
        if self.pub_timers["sim_images"].due(t) and self.render_thread.is_alive():
            snap = (t, self.generation, d.qpos.copy(), d.mocap_pos.copy(), d.mocap_quat.copy())
            try:
                self.render_queue.get_nowait()  # drop a stale snapshot
                self.render_dropped += 1
            except queue.Empty:
                pass
            self.render_queue.put_nowait(snap)
        return out

    def _physics_loop(self) -> None:
        if self.get_parameter("viewer").value:
            import mujoco.viewer

            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        viewer_timer = Periodic(30.0)
        period = self.dt_tick / self.rt_factor
        next_wall = time.perf_counter()
        while self.running and rclpy.ok():
            with self.lock:
                out = self._tick()
                if self.viewer is not None and viewer_timer.due(self.sim_time):
                    if self.viewer.is_running():
                        self.viewer.sync()
            for pub, msg in out:
                if pub is self.tf:
                    self.tf.sendTransform(msg)
                else:
                    pub.publish(msg)
            next_wall += period
            delay = next_wall - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            elif delay < -0.1:  # fell far behind: resync instead of bursting
                next_wall = time.perf_counter()

    # ---- rendering ------------------------------------------------------------------------
    def _render_loop(self) -> None:
        os.environ.setdefault("MUJOCO_GL", "egl")
        size = 256
        renderer = mujoco.Renderer(self.model, size, size)
        rdata = mujoco.MjData(self.model)
        cams = list(self.names["cameras"])
        while self.running and rclpy.ok():
            try:
                t, gen, qpos, mpos, mquat = self.render_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            rdata.qpos[:] = qpos
            rdata.mocap_pos[:] = mpos
            rdata.mocap_quat[:] = mquat
            t_start = time.perf_counter()
            mujoco.mj_forward(self.model, rdata)
            images = {}
            for cam in cams:
                renderer.update_scene(rdata, cam)
                images[cam] = renderer.render()
            self.render_ms.append((time.perf_counter() - t_start) * 1e3)
            if len(self.render_ms) >= 300:
                ms = np.array(self.render_ms)
                self.get_logger().info(
                    f"render {np.median(ms):.1f} ms median, {ms.max():.1f} ms max; "
                    f"{self.render_dropped} snapshots dropped")
                self.render_ms.clear()
            if gen != self.generation:
                continue  # a reset happened while rendering
            stamp = to_time_msg(t)
            for cam, img in images.items():
                msg = Image()
                msg.header.stamp, msg.header.frame_id = stamp, f"{cam}_camera"
                msg.height, msg.width = img.shape[:2]
                msg.encoding, msg.step = "rgb8", img.shape[1] * 3
                # array.array skips rclpy's per-element check on uint8[] (tens of ms).
                msg.data = array.array("B", img.tobytes())
                self.pub_img[cam].publish(msg)
        renderer.close()

    def destroy_node(self):
        self.running = False
        self.physics_thread.join(timeout=1.0)
        if self.render_thread.is_alive():
            self.render_thread.join(timeout=1.0)
        if self.viewer is not None:
            self.viewer.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MujocoSim()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

