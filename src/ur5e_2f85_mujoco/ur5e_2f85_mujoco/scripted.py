"""Scripted Cartesian grasp / pick-and-place using the Python IK oracle.

Used by the grasp tests, the scene clip and the scripted ROS trajectory.
A plan is a list of straight-line Cartesian segments with a gripper command each; the
controller interpolates them in time and solves IK warm-started from the last solution.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .kinematics import Kinematics, quat_to_mat


@dataclass
class ScriptParams:
    approach_height: float = 0.10     # above the cube centre
    grasp_z_offset: float = 0.004     # TCP above the cube centre when grasping
    lift_height: float = 0.10
    place_height: float = 0.03        # cube bottom above the table when released
    speed: float = 0.20               # m/s for free moves
    slow_speed: float = 0.08          # m/s for descend / lift
    close_time: float = 0.8
    open_time: float = 0.6
    hold_time: float = 2.0
    min_segment: float = 0.5


@dataclass
class Segment:
    duration: float
    p0: np.ndarray
    p1: np.ndarray
    yaw0: float
    yaw1: float
    gripper: float
    label: str = ""


@dataclass
class ScriptedController:
    kin: Kinematics
    params: ScriptParams = field(default_factory=ScriptParams)

    def __post_init__(self):
        p, r = self.kin.fk(self.kin.q_home)
        self.r_down = r                      # home orientation: gripper pointing -z
        self.home_pos = p
        # World angle of the finger closing axis (TCP site y) at home.
        self.finger_axis_angle = float(np.arctan2(r[1, 1], r[0, 1]))
        self.segments: list[Segment] = []
        self.q_last = self.kin.q_home.copy()

    # ---- planning -------------------------------------------------------------------------
    def grasp_yaw(self, cube_yaw: float) -> float:
        """Smallest gripper yaw that aligns the finger axis with a cube face normal."""
        delta = cube_yaw - self.finger_axis_angle
        return float((delta + np.pi / 4) % (np.pi / 2) - np.pi / 4)

    def _move(self, p0, p1, yaw0, yaw1, grip, speed, label):
        dist = float(np.linalg.norm(np.asarray(p1) - np.asarray(p0)))
        dur = max(self.params.min_segment, dist / speed, abs(yaw1 - yaw0) / 1.0)
        return Segment(dur, np.asarray(p0, float), np.asarray(p1, float), yaw0, yaw1, grip, label)

    def _wait(self, p, yaw, grip, dur, label):
        return Segment(dur, np.asarray(p, float), np.asarray(p, float), yaw, yaw, grip, label)

    def plan_grasp_lift(self, start_pos, cube_pos, cube_yaw) -> list[Segment]:
        pr = self.params
        yaw = self.grasp_yaw(cube_yaw)
        above = np.array([cube_pos[0], cube_pos[1], cube_pos[2] + pr.approach_height])
        grasp = np.array([cube_pos[0], cube_pos[1], cube_pos[2] + pr.grasp_z_offset])
        lifted = grasp + [0, 0, pr.lift_height]
        self.segments = [
            self._move(start_pos, above, 0.0, yaw, 1.0, pr.speed, "approach"),
            self._move(above, grasp, yaw, yaw, 1.0, pr.slow_speed, "descend"),
            self._wait(grasp, yaw, 0.0, pr.close_time, "close"),
            self._move(grasp, lifted, yaw, yaw, 0.0, pr.slow_speed, "lift"),
            self._wait(lifted, yaw, 0.0, pr.hold_time, "hold"),
        ]
        return self.segments

    def plan_pick_place(self, start_pos, cube_pos, cube_yaw, target_pos) -> list[Segment]:
        pr = self.params
        self.plan_grasp_lift(start_pos, cube_pos, cube_yaw)
        segs = self.segments[:-1]  # drop the long hold
        yaw = segs[-1].yaw1
        lifted = segs[-1].p1
        half = cube_pos[2]  # cube centre height when resting = half size
        over = np.array([target_pos[0], target_pos[1], lifted[2]])
        lower = np.array([target_pos[0], target_pos[1],
                          half + pr.place_height + pr.grasp_z_offset])
        retreat = lower + [0, 0, pr.approach_height]
        segs += [
            self._wait(lifted, yaw, 0.0, 0.3, "settle"),
            self._move(lifted, over, yaw, yaw, 0.0, pr.speed, "translate"),
            self._move(over, lower, yaw, yaw, 0.0, pr.slow_speed, "lower"),
            self._wait(lower, yaw, 1.0, pr.open_time, "open"),
            self._move(lower, retreat, yaw, yaw, 1.0, pr.slow_speed, "retreat"),
        ]
        self.segments = segs
        return segs

    @property
    def duration(self) -> float:
        return float(sum(s.duration for s in self.segments))

    # ---- execution ------------------------------------------------------------------------
    def target_at(self, t: float) -> tuple[np.ndarray, np.ndarray, float, str]:
        elapsed = 0.0
        seg = self.segments[-1]
        s = 1.0
        for cand in self.segments:
            if t < elapsed + cand.duration:
                seg = cand
                s = (t - elapsed) / cand.duration
                break
            elapsed += cand.duration
        s = 0.5 - 0.5 * np.cos(np.pi * np.clip(s, 0.0, 1.0))  # smooth start/stop
        pos = seg.p0 + s * (seg.p1 - seg.p0)
        yaw = seg.yaw0 + s * (seg.yaw1 - seg.yaw0)
        rz = quat_to_mat([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
        return pos, rz @ self.r_down, seg.gripper, seg.label

    def command(self, t: float) -> tuple[np.ndarray, float, str]:
        pos, mat, grip, label = self.target_at(t)
        q, _ = self.kin.ik(pos, mat, self.q_last, iters=20)
        self.q_last = q
        return q, grip, label
