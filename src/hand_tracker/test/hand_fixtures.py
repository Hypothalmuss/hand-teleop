"""Synthetic (21, 3) landmark sets in normalized image coordinates (640x480 frame)."""

import numpy as np

FINGER_X = [0.46, 0.50, 0.54, 0.58]  # index, middle, ring, pinky MCP x


def _hand(curl: bool, pinch_closed: bool = False, dx: float = 0.0) -> np.ndarray:
    lm = np.zeros((21, 3))
    lm[0] = [0.52, 0.80, 0]
    # thumb
    lm[1:5] = [[0.47, 0.76, 0], [0.43, 0.72, 0], [0.40, 0.68, 0], [0.38, 0.64, 0]]
    for k, x in enumerate(FINGER_X):
        base = 5 + 4 * k
        if curl:
            chain = [[x, 0.60], [x, 0.54], [x + 0.005, 0.58], [x + 0.005, 0.63]]
        else:
            length = 0.20 if k < 3 else 0.15
            chain = [[x, 0.60 - length * s] for s in (0.0, 0.4, 0.7, 1.0)]
        lm[base:base + 4, :2] = chain
    if pinch_closed:
        lm[4, :2] = lm[8, :2] + [0.005, 0.005]
    lm[:, 0] += dx
    return lm


def open_hand(dx=0.0):
    return _hand(curl=False, dx=dx)


def fist(dx=0.0):
    return _hand(curl=True, dx=dx)


def pinched_hand(dx=0.0):
    return _hand(curl=False, pinch_closed=True, dx=dx)
