"""Hand features and gesture classification on (21, 3) landmark arrays. No ROS.

Landmark indices (MediaPipe): 0 wrist, 4 thumb tip, 5/9/13/17 finger MCPs,
8/12/16/20 finger tips.

Distances are image-plane (x, y). MediaPipe normalizes x by the width and y by the height,
so callers pass landmarks through ``isotropic`` first (y scaled by height / width); all
features are then in units of the image width.
"""

from __future__ import annotations

import numpy as np

WRIST, THUMB_TIP, INDEX_TIP, MIDDLE_MCP = 0, 4, 8, 9
FINGER_MCPS = np.array([5, 9, 13, 17])
FINGER_TIPS = np.array([8, 12, 16, 20])
# MCP, PIP, DIP, TIP chains for index, middle, ring, pinky.
FINGER_CHAINS = np.array([[5, 6, 7, 8], [9, 10, 11, 12], [13, 14, 15, 16], [17, 18, 19, 20]])
EPS = 1e-6

# Skeleton edges for drawing.
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10),
    (10, 11), (11, 12), (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (0, 17), (17, 18),
    (18, 19), (19, 20),
]


def isotropic(lm: np.ndarray, width: int, height: int) -> np.ndarray:
    out = np.array(lm, dtype=float, copy=True)
    out[:, 1] *= height / width
    return out


def palm_scale(lm: np.ndarray) -> float:
    """Wrist to middle-finger MCP distance (rigid under finger motion: the depth proxy).

    Image-plane (x, y) distance; MediaPipe z is a noisy relative depth and is not used.
    """
    return float(np.linalg.norm(lm[MIDDLE_MCP, :2] - lm[WRIST, :2]))


def pinch(lm: np.ndarray, scale: float) -> float:
    return float(np.linalg.norm(lm[THUMB_TIP, :2] - lm[INDEX_TIP, :2]) / max(scale, EPS))


def extensions(lm: np.ndarray, scale: float) -> np.ndarray:
    """Tip-to-MCP distance / palm_scale for index, middle, ring, pinky."""
    d = np.linalg.norm(lm[FINGER_TIPS, :2] - lm[FINGER_MCPS, :2], axis=1)
    return d / max(scale, EPS)


def straightness(lm: np.ndarray) -> np.ndarray:
    """Chord / arc length of each finger (MCP..TIP): ~1 straight, ~0.3-0.5 curled.

    Independent of finger length and of palm_scale, so one threshold fits all four fingers.
    """
    p = lm[FINGER_CHAINS][..., :2]                     # (4, 4, 2)
    arc = np.linalg.norm(np.diff(p, axis=1), axis=2).sum(axis=1)
    chord = np.linalg.norm(p[:, 3] - p[:, 0], axis=1)
    return chord / np.maximum(arc, EPS)


def finger_features(lm: np.ndarray, scale: float, feature: str) -> np.ndarray:
    """Per-finger openness feature used by ``classify``: 'straightness' or 'extension'."""
    if feature == "straightness":
        return straightness(lm)
    if feature == "extension":
        return extensions(lm, scale)
    raise ValueError(f"unknown gesture feature {feature!r}")


def classify(ext: np.ndarray, open_threshold: float, fist_threshold: float) -> tuple[bool, bool]:
    """(is_open, is_fist) from the raw extensions; in between is neither."""
    return bool(np.all(ext > open_threshold)), bool(np.all(ext < fist_threshold))


class Debouncer:
    """A boolean that only changes after ``frames`` consecutive frames of the new value."""

    def __init__(self, frames: int, initial: bool = False):
        self.frames = int(frames)
        self.value = initial
        self._count = 0

    def reset(self, value: bool = False) -> None:
        self.value = value
        self._count = 0

    def update(self, raw: bool) -> bool:
        if raw == self.value:
            self._count = 0
        else:
            self._count += 1
            if self._count >= self.frames:
                self.value = raw
                self._count = 0
        return self.value
