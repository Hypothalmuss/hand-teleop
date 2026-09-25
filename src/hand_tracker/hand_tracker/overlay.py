"""Debug overlay: skeleton, handedness, pinch bar, OPEN/FIST tag, fps, latency, clocks."""

from __future__ import annotations

import datetime

import cv2
import numpy as np

from .gestures import HAND_CONNECTIONS

COLORS = {"left": (255, 160, 0), "right": (0, 200, 255)}  # BGR


def draw(frame: np.ndarray, hands: dict, fps: float, latency_ms: float,
         sim_time: float | None = None, pinch_range=(0.25, 1.0)) -> np.ndarray:
    h, w = frame.shape[:2]
    img = frame.copy()
    for side, hand in hands.items():
        if not hand.present:
            continue
        color = COLORS[side]
        pts = (hand.landmarks[:, :2] * [w, h]).astype(int)
        for a, b in HAND_CONNECTIONS:
            cv2.line(img, tuple(pts[a]), tuple(pts[b]), color, 2, cv2.LINE_AA)
        for p in pts:
            cv2.circle(img, tuple(p), 3, (255, 255, 255), -1, cv2.LINE_AA)
        tag = "OPEN" if hand.is_open else "FIST" if hand.is_fist else "--"
        x, y = pts[0]
        cv2.putText(img, f"{side[0].upper()} {tag}", (x - 30, y + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
        # pinch bar
        lo, hi = pinch_range
        frac = float(np.clip((hand.pinch - lo) / (hi - lo), 0.0, 1.0))
        bx = 10 if side == "left" else w - 30
        cv2.rectangle(img, (bx, h - 130), (bx + 20, h - 30), (80, 80, 80), 1)
        cv2.rectangle(img, (bx, h - 30 - int(100 * frac)), (bx + 20, h - 30), color, -1)
        cv2.putText(img, f"{hand.pinch:.2f}", (bx - 5, h - 135), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, color, 1, cv2.LINE_AA)
    wall = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    lines = [f"{fps:4.1f} fps  {latency_ms:4.1f} ms", f"wall {wall}"]
    if sim_time is not None:
        lines.append(f"sim  {sim_time:9.3f} s")
    for i, text in enumerate(lines):
        cv2.putText(img, text, (10, 25 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0), 2, cv2.LINE_AA)
    return img
