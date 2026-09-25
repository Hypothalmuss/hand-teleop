"""Hand state -> end-effector target, gripper and clutch. Pure functions + state, no ROS."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

AXES = ("x", "y", "z")
MIDDLE_MCP = 9


def hand_axes(landmarks: np.ndarray, palm_scale: float) -> np.ndarray:
    """(u, v, d): mirrored image x of landmark 9, 1 - image y, 1 / palm_scale."""
    lm = np.asarray(landmarks, float).reshape(21, 3)
    return np.array([lm[MIDDLE_MCP, 0], 1.0 - lm[MIDDLE_MCP, 1], 1.0 / max(palm_scale, 1e-6)])


def hand_to_workspace(uvd: np.ndarray, ws: dict, clamp: bool = True) -> np.ndarray:
    """Map (u, v, d) linearly from the calibrated hand ranges onto the workspace box.

    ``ws`` is workspace.yaml. ``axis_map`` says which robot axis each hand axis drives and
    ``sign`` = -1 maps the low end of the hand range to the high end of the box.
    """
    out = np.zeros(3)
    for i, hand_axis in enumerate(("u", "v", "d")):
        robot_axis = ws["axis_map"][hand_axis]
        j = AXES.index(robot_axis)
        lo, hi = ws["hand_range"][hand_axis]
        s = (uvd[i] - lo) / (hi - lo)
        if ws["sign"][robot_axis] < 0:
            s = 1.0 - s
        if clamp:
            s = float(np.clip(s, 0.0, 1.0))
        blo, bhi = ws["box"][robot_axis]
        out[j] = blo + s * (bhi - blo)
    return out


def clamp_box(p: np.ndarray, ws: dict) -> np.ndarray:
    lo = np.array([ws["box"][a][0] for a in AXES])
    hi = np.array([ws["box"][a][1] for a in AXES])
    return np.clip(p, lo, hi)


def reanchor(mapped: np.ndarray, anchor_hand: np.ndarray, anchor_ee: np.ndarray,
             gain: np.ndarray, ws: dict) -> np.ndarray:
    """target = clamp(anchor_ee + gain * (mapped - anchor_hand), box)."""
    return clamp_box(anchor_ee + gain * (mapped - anchor_hand), ws)


def rate_limit(prev: np.ndarray, new: np.ndarray, max_rate: float, dt: float) -> np.ndarray:
    """Move from ``prev`` towards ``new`` by at most ``max_rate * dt`` (vector norm)."""
    prev, new = np.asarray(prev, float), np.asarray(new, float)
    step = new - prev
    n = float(np.linalg.norm(step))
    lim = max_rate * dt
    return new if n <= lim else prev + step * (lim / n)


def gripper_from_pinch(pinch: float, pinch_closed: float, pinch_open: float) -> float:
    return float(np.clip((pinch - pinch_closed) / (pinch_open - pinch_closed), 0.0, 1.0))


@dataclass
class HandInput:
    present: bool
    landmarks: np.ndarray
    palm_scale: float
    pinch: float
    is_open: bool
    is_fist: bool


@dataclass
class MapperOutput:
    target: np.ndarray | None
    gripper: float
    engaged: bool


class TeleopMapperCore:
    """Clutch with relative re-anchoring, speed limit and gripper mapping.

    ``update`` is called on every hand state with the latest measured EE position (or None
    before the first one), the e-stop flag and the frame time in seconds.
    """

    def __init__(self, ws: dict, rate_hz: float):
        self.ws = ws
        self.gain = np.array([ws["gain"][a] for a in AXES], float)
        self.max_speed = float(ws["max_target_speed"])
        self.absent_hold = float(ws["absent_hold_s"])
        g = ws["gripper"]
        self.pinch_closed, self.pinch_open = float(g["pinch_closed"]), float(g["pinch_open"])
        self.grip_rate = float(g["rate_limit"])
        self.default_dt = 1.0 / rate_hz
        self.engaged = False
        self.target: np.ndarray | None = None
        self.gripper = 1.0
        self.anchor_hand = None
        self.anchor_ee = None
        self.last_t = None
        self.right_seen_t = None

    def _dt(self, t: float) -> float:
        dt = self.default_dt if self.last_t is None else t - self.last_t
        return dt if dt > 0 else self.default_dt

    def update(self, right: HandInput, left: HandInput, ee_pos, estop: bool,
               t: float) -> MapperOutput:
        dt = self._dt(t)
        self.last_t = t
        if self.target is None and ee_pos is not None:
            self.target = np.array(ee_pos, float)

        # --- clutch ---
        if right.present:
            self.right_seen_t = t
        absent_too_long = (not right.present) and (
            self.right_seen_t is None or t - self.right_seen_t >= self.absent_hold)
        want = self.engaged
        if estop or absent_too_long or (right.present and right.is_fist):
            want = False
        elif right.present and right.is_open and ee_pos is not None:
            want = True
        mapped = None
        if right.present:
            mapped = hand_to_workspace(hand_axes(right.landmarks, right.palm_scale), self.ws,
                                       clamp=False)
        if want and not self.engaged:  # engage edge: re-anchor, no jump
            self.anchor_hand = mapped
            self.anchor_ee = np.array(ee_pos, float)
            self.target = self.anchor_ee.copy()
        elif want and mapped is not None:
            desired = reanchor(mapped, self.anchor_hand, self.anchor_ee, self.gain, self.ws)
            self.target = rate_limit(self.target, desired, self.max_speed, dt)
        self.engaged = want

        # --- gripper (left hand); absent -> hold ---
        if left.present:
            desired = gripper_from_pinch(left.pinch, self.pinch_closed, self.pinch_open)
            self.gripper = float(rate_limit([self.gripper], [desired], self.grip_rate, dt)[0])
        return MapperOutput(None if self.target is None else self.target.copy(), self.gripper,
                            self.engaged)
