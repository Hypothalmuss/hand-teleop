"""Shared task logic: episode reset/randomization, gripper mapping, command application.

The ROS simulator node and the headless learning env both import these, so a reset with a
given seed and a given command stream produce identical physics in both.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np


@dataclass
class SceneIndex:
    """Cached indices into qpos/ctrl for the named scene elements."""

    arm_qpos: np.ndarray
    arm_dof: np.ndarray
    arm_limits: np.ndarray
    arm_act: np.ndarray
    grip_act: int
    grip_ctrl_max: float
    grip_qpos: int
    grip_range: tuple[float, float]
    cube_qpos: int
    cube_dof: int
    cube_body: int
    target_mocap: int
    tcp_site: int
    home_key: int

    @classmethod
    def build(cls, model: mujoco.MjModel, names: dict) -> "SceneIndex":
        joints = [model.joint(n) for n in names["arm_joints"]]
        grip_j = model.joint(names["gripper_joint"])
        cube_j = model.joint(names["cube_joint"])
        grip_act = model.actuator(names["gripper_actuator"]).id
        return cls(
            arm_qpos=np.array([j.qposadr[0] for j in joints]),
            arm_dof=np.array([j.dofadr[0] for j in joints]),
            arm_limits=np.array([model.jnt_range[j.id] for j in joints]),
            arm_act=np.array([model.actuator(n).id for n in names["arm_actuators"]]),
            grip_act=grip_act,
            grip_ctrl_max=float(model.actuator_ctrlrange[grip_act, 1]),
            grip_qpos=int(grip_j.qposadr[0]),
            grip_range=tuple(float(v) for v in model.jnt_range[grip_j.id]),
            cube_qpos=int(cube_j.qposadr[0]),
            cube_dof=int(cube_j.dofadr[0]),
            cube_body=model.body(names["cube_body"]).id,
            target_mocap=int(model.body_mocapid[model.body(names["target_body"]).id]),
            tcp_site=model.site(names["tcp_site"]).id,
            home_key=model.key(names["home_keyframe"]).id,
        )


def gripper_cmd_to_ctrl(cmd: float, ctrl_max: float) -> float:
    """Gripper command 0 (closed) .. 1 (open) -> actuator ctrl (0 open .. ctrl_max closed)."""
    return (1.0 - float(np.clip(cmd, 0.0, 1.0))) * ctrl_max


def gripper_aperture(data: mujoco.MjData, idx: SceneIndex) -> float:
    """Measured aperture 0 (closed) .. 1 (open) from the driver joint."""
    lo, hi = idx.grip_range
    return float(np.clip(1.0 - (data.qpos[idx.grip_qpos] - lo) / (hi - lo), 0.0, 1.0))


def sample_episode(seed: int, task: dict, randomize_target: bool) -> dict:
    """Deterministic cube (and optionally target) placement for a seed."""
    rng = np.random.default_rng(seed)
    sp = task["spawn"]
    cube_xy = np.array([rng.uniform(*sp["cube_x"]), rng.uniform(*sp["cube_y"])])
    cube_yaw = float(rng.uniform(*sp["cube_yaw"]))
    target = np.array(task["target"]["default_pos"], float)
    if randomize_target:
        target[:2] = [rng.uniform(*sp["target_x"]), rng.uniform(*sp["target_y"])]
    return {"cube_xy": cube_xy, "cube_yaw": cube_yaw, "target_pos": target}


def reset_episode(model: mujoco.MjModel, data: mujoco.MjData, idx: SceneIndex, task: dict,
                  seed: int, randomize_target: bool = False) -> dict:
    """Reset to the home keyframe, place cube/target for ``seed``, settle. Returns poses."""
    ep = sample_episode(seed, task, randomize_target)
    mujoco.mj_resetDataKeyframe(model, data, idx.home_key)
    hs = task["cube"]["half_size"]
    yaw = ep["cube_yaw"]
    data.qpos[idx.cube_qpos:idx.cube_qpos + 7] = [
        ep["cube_xy"][0], ep["cube_xy"][1], hs, np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
    data.qvel[idx.cube_dof:idx.cube_dof + 6] = 0.0
    data.mocap_pos[idx.target_mocap] = ep["target_pos"]
    data.mocap_quat[idx.target_mocap] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    for _ in range(int(task["settle_steps"])):
        mujoco.mj_step(model, data)
    return {"cube_pos": data.qpos[idx.cube_qpos:idx.cube_qpos + 3].copy(),
            "cube_quat": data.qpos[idx.cube_qpos + 3:idx.cube_qpos + 7].copy(),
            "target_pos": data.mocap_pos[idx.target_mocap].copy(),
            "target_quat": data.mocap_quat[idx.target_mocap].copy(),
            "seed": seed}


class JointServo:
    """Applies joint commands once per control tick, like a UR ``servoj`` driver.

    1. clamp the command to the joint limits and to ``max_step`` per tick,
    2. derive a reference velocity from the command stream (new command vs the previous new
       command, divided by the ticks between them; zeroed when commands stop),
    3. write ``ctrl = q_cmd + (kv / kp) * v_ref``. The Menagerie position servos have
       ``tau = kp (ctrl - q) - kv qdot``; without the feedforward they lag by
       ``kv / kp * qdot`` (0.2 s x velocity).

    The ROS simulator and the headless env both use this class, so a command stream produces
    identical physics in both.
    """

    def __init__(self, model: mujoco.MjModel, idx: SceneIndex, dt_tick: float,
                 max_step: float | None, ff_v_max: float = 3.0, velocity_ff: bool = True,
                 timeout_factor: float = 1.5):
        self.idx = idx
        self.dt = dt_tick
        self.max_step = max_step
        self.ff_v_max = ff_v_max
        bias = model.actuator_biasprm[idx.arm_act]
        self.ff_gain = bias[:, 2] / bias[:, 1] if velocity_ff else np.zeros(len(idx.arm_act))
        self.timeout_factor = timeout_factor
        self.clamp_count = 0
        self.reset(np.zeros(len(idx.arm_act)))

    def reset(self, q: np.ndarray, gripper: float = 1.0) -> None:
        self.target = np.array(q, float)
        self.gripper = float(gripper)
        self.applied = np.array(q, float)
        self.v_ref = np.zeros_like(self.applied)
        self.t = 0.0
        self._new = False
        self._last_cmd = None
        self._last_t = 0.0
        self._interval = self.dt

    def set_command(self, positions, gripper: float) -> None:
        self.target = np.array(positions, float)
        self.gripper = float(gripper)
        self._new = True

    def tick(self, data: mujoco.MjData) -> bool:
        """Write ctrl for this tick. Returns True if the command was clamped."""
        idx = self.idx
        self.t += self.dt
        cmd = np.clip(self.target, idx.arm_limits[:, 0], idx.arm_limits[:, 1])
        if self.max_step is not None:
            cmd = np.clip(cmd, self.applied - self.max_step, self.applied + self.max_step)
        clamped = bool(np.any(np.abs(cmd - self.target) > 1e-12))
        self.clamp_count += int(clamped)
        if self._new:
            if self._last_cmd is not None:
                self._interval = self.t - self._last_t
                self.v_ref = np.clip((cmd - self._last_cmd) / self._interval,
                                     -self.ff_v_max, self.ff_v_max)
            self._last_cmd, self._last_t, self._new = cmd, self.t, False
        elif self.t - self._last_t > self.timeout_factor * self._interval + 1e-9:
            self.v_ref[:] = 0.0
        self.applied = cmd
        data.ctrl[idx.arm_act] = cmd + self.ff_gain * self.v_ref
        data.ctrl[idx.grip_act] = gripper_cmd_to_ctrl(self.gripper, idx.grip_ctrl_max)
        return clamped


def make_servo(model: mujoco.MjModel, idx: SceneIndex, rates: dict, ik_cfg: dict,
               clamp_steps: bool = True) -> JointServo:
    """JointServo configured from ``rates.yaml`` and ``ik.yaml`` (control tick, clamps, ff)."""
    steps = int(round(rates["physics"] / rates["control"]))
    return JointServo(model, idx, steps * model.opt.timestep,
                      float(ik_cfg["max_joint_step_rad"]) if clamp_steps else None,
                      float(ik_cfg["ff_v_max"]), bool(ik_cfg["velocity_feedforward"]))


def cube_pose(data: mujoco.MjData, idx: SceneIndex) -> tuple[np.ndarray, np.ndarray]:
    return (data.xpos[idx.cube_body].copy(), data.xquat[idx.cube_body].copy())


def cube_speed(data: mujoco.MjData, idx: SceneIndex) -> float:
    return float(np.linalg.norm(data.qvel[idx.cube_dof:idx.cube_dof + 3]))


def load_seeds(path) -> list[int]:
    with open(path) as f:
        return [int(line) for line in f if line.strip() and not line.startswith("#")]


class SuccessDetector:
    """Task success: cube centre within ``xy_tol`` of the target centre, cube centre below
    ``z_max``, gripper aperture above ``gripper_min``, all held for ``hold_s`` seconds.

    Shared by the recorder, the teleop benchmark and the headless env (import, never copy).
    """

    def __init__(self, task: dict):
        s = task["success"]
        self.xy_tol, self.z_max = float(s["xy_tol"]), float(s["z_max"])
        self.gripper_min, self.hold_s = float(s["gripper_min"]), float(s["hold_s"])
        self.reset()

    def reset(self) -> None:
        self._since = None
        self.success = False

    @staticmethod
    def conditions(cube_pos, target_pos, aperture, xy_tol, z_max, gripper_min) -> dict:
        d = float(np.hypot(cube_pos[0] - target_pos[0], cube_pos[1] - target_pos[1]))
        return {"xy": d < xy_tol, "z": float(cube_pos[2]) < z_max,
                "gripper": float(aperture) > gripper_min}

    def update(self, t: float, cube_pos, target_pos, aperture: float) -> bool:
        ok = all(self.conditions(cube_pos, target_pos, aperture, self.xy_tol, self.z_max,
                                 self.gripper_min).values())
        if not ok:
            self._since = None
        elif self._since is None:
            self._since = t
        self.success = ok and t - self._since >= self.hold_s - 1e-9
        return self.success
