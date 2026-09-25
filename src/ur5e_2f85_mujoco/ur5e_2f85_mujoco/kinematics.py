"""Reference kinematics (pure MuJoCo): FK, 6x6 Jacobian and a simple DLS IK.

This is the oracle the C++ ``diff_ik_controller`` is tested against. It keeps its own
``MjData`` so it never disturbs a simulation.
"""

from __future__ import annotations

import mujoco
import numpy as np

from . import load_model, load_model_names


def quat_to_mat(quat: np.ndarray) -> np.ndarray:
    mat = np.zeros(9)
    mujoco.mju_quat2Mat(mat, np.asarray(quat, float))
    return mat.reshape(3, 3)


def mat_to_quat(mat: np.ndarray) -> np.ndarray:
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.asarray(mat, float).flatten())
    return quat


def rot_error(r_target: np.ndarray, r_current: np.ndarray) -> np.ndarray:
    """Rotation vector of ``R_target @ R_current.T`` (world frame)."""
    quat = mat_to_quat(r_target @ r_current.T)
    vel = np.zeros(3)
    mujoco.mju_quat2Vel(vel, quat, 1.0)
    return vel


def yaw_quat(yaw: float) -> np.ndarray:
    return np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])


class Kinematics:
    def __init__(self, model: mujoco.MjModel | None = None, names: dict | None = None):
        self.model = model if model is not None else load_model()
        self.names = names if names is not None else load_model_names()
        self.data = mujoco.MjData(self.model)
        joints = [self.model.joint(n) for n in self.names["arm_joints"]]
        self.qpos_idx = np.array([j.qposadr[0] for j in joints])
        self.dof_idx = np.array([j.dofadr[0] for j in joints])
        self.limits = np.array([self.model.jnt_range[j.id] for j in joints])
        self.site_id = self.model.site(self.names["tcp_site"]).id
        self.q_home = np.array(self.names["q_home"])
        self._jacp = np.zeros((3, self.model.nv))
        self._jacr = np.zeros((3, self.model.nv))

    def _set(self, q: np.ndarray) -> None:
        self.data.qpos[self.qpos_idx] = q
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)

    def fk(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """TCP position (3,) and rotation matrix (3, 3) in base_link."""
        self._set(q)
        return (self.data.site_xpos[self.site_id].copy(),
                self.data.site_xmat[self.site_id].reshape(3, 3).copy())

    def fk_quat(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        pos, mat = self.fk(q)
        return pos, mat_to_quat(mat)

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """6x6 geometric Jacobian [linear; angular] of the TCP site w.r.t. the arm joints."""
        self._set(q)
        mujoco.mj_jacSite(self.model, self.data, self._jacp, self._jacr, self.site_id)
        return np.vstack([self._jacp[:, self.dof_idx], self._jacr[:, self.dof_idx]])

    def ik(self, pos: np.ndarray, mat: np.ndarray, q0: np.ndarray, iters: int = 100,
           damping: float = 1e-3, tol: float = 1e-5,
           max_step: float = 0.3) -> tuple[np.ndarray, bool]:
        """Damped least-squares position+orientation IK, warm-started at ``q0``."""
        q = np.array(q0, float)
        for _ in range(iters):
            p, r = self.fk(q)
            err = np.concatenate([pos - p, rot_error(mat, r)])
            if np.linalg.norm(err) < tol:
                return q, True
            jac = self.jacobian(q)
            dq = jac.T @ np.linalg.solve(jac @ jac.T + damping**2 * np.eye(6), err)
            n = np.linalg.norm(dq)
            if n > max_step:
                dq *= max_step / n
            q = np.clip(q + dq, self.limits[:, 0], self.limits[:, 1])
        p, r = self.fk(q)
        err = np.concatenate([pos - p, rot_error(mat, r)])
        return q, bool(np.linalg.norm(err) < 1e-3)
