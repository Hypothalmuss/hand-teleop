"""Square test trajectory for the IK controller (launch test and results plot). No ROS."""

from __future__ import annotations

import numpy as np


class SquarePath:
    """Lead-in from ``start`` to the first corner, then laps of an axis-aligned square in the
    horizontal plane, all at constant ``speed``."""

    def __init__(self, start, center, side: float = 0.10, speed: float = 0.10, laps: int = 2):
        c = np.asarray(center, float)
        h = side / 2
        corners = [c + [-h, -h, 0], c + [h, -h, 0], c + [h, h, 0], c + [-h, h, 0]]
        pts = [np.asarray(start, float)] + corners * laps + [corners[0]]
        self.points = np.array(pts)
        seg = np.linalg.norm(np.diff(self.points, axis=0), axis=1)
        self.t_knots = np.concatenate([[0.0], np.cumsum(seg / speed)])
        self.lead_in = float(self.t_knots[1])
        self.duration = float(self.t_knots[-1])

    def __call__(self, t: float) -> np.ndarray:
        t = float(np.clip(t, 0.0, self.duration))
        return np.array([np.interp(t, self.t_knots, self.points[:, i]) for i in range(3)])
