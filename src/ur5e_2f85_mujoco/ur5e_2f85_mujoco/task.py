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


def apply_joint_command(data: mujoco.MjData, idx: SceneIndex, positions: np.ndarray,
                        gripper: float, prev_cmd: np.ndarray | None,
                        max_step: float | None) -> tuple[np.ndarray, bool]:
    """Clamp to joint limits and a max per-tick step, then write ctrl.

    Returns the applied arm command and whether any clamping happened.
    """
    q = np.asarray(positions, float)
    cmd = np.clip(q, idx.arm_limits[:, 0], idx.arm_limits[:, 1])
    if prev_cmd is not None and max_step is not None:
        cmd = np.clip(cmd, prev_cmd - max_step, prev_cmd + max_step)
    clamped = bool(np.any(np.abs(cmd - q) > 1e-12))
    data.ctrl[idx.arm_act] = cmd
    data.ctrl[idx.grip_act] = gripper_cmd_to_ctrl(gripper, idx.grip_ctrl_max)
    return cmd, clamped


def cube_pose(data: mujoco.MjData, idx: SceneIndex) -> tuple[np.ndarray, np.ndarray]:
    return (data.xpos[idx.cube_body].copy(), data.xquat[idx.cube_body].copy())


def cube_speed(data: mujoco.MjData, idx: SceneIndex) -> float:
    return float(np.linalg.norm(data.qvel[idx.cube_dof:idx.cube_dof + 3]))
