"""Vectorized One Euro filter (Casiez et al., CHI 2012). No ROS.

``value_scale`` converts the signal into the units the parameters were tuned in. The
standard parameters (min_cutoff=1.0, beta=0.007) assume pixel units, so landmarks in
normalized image coordinates use ``value_scale`` = image width in pixels.
"""

from __future__ import annotations

import numpy as np


def _alpha(cutoff, dt: float):
    tau = 1.0 / (2.0 * np.pi * cutoff)
    return 1.0 / (1.0 + tau / dt)


class OneEuroFilter:
    def __init__(self, min_cutoff: float = 1.0, beta: float = 0.007, d_cutoff: float = 1.0,
                 value_scale: float = 1.0):
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self.value_scale = float(value_scale)
        self.reset()

    def reset(self) -> None:
        self._x = None
        self._dx = None
        self._t = None

    @property
    def initialized(self) -> bool:
        return self._x is not None

    def __call__(self, x, t: float) -> np.ndarray:
        """Filter sample ``x`` (any shape) taken at time ``t`` (seconds)."""
        x = np.asarray(x, dtype=float)
        if self._x is None:
            self._x, self._dx, self._t = x.copy(), np.zeros_like(x), t
            return x.copy()
        dt = t - self._t
        if dt <= 0.0:  # duplicate or out-of-order timestamp: return the current estimate
            return self._x.copy()
        dx = (x - self._x) / dt
        a_d = _alpha(self.d_cutoff, dt)
        self._dx = self._dx + a_d * (dx - self._dx)
        # Speed in the tuning units (e.g. px/s) sets the adaptive cutoff.
        cutoff = self.min_cutoff + self.beta * np.abs(self._dx) * self.value_scale
        a = _alpha(cutoff, dt)
        self._x = self._x + a * (x - self._x)  # exact when x is constant
        self._t = t
        return self._x.copy()
