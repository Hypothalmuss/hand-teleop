"""MediaPipe HandLandmarker + One Euro filtering + gestures for both hands. No ROS.

Input frames are already mirrored (selfie view). The tracker keeps per-hand state: filters,
debouncers and a missing-frame counter (a hand missing for ``missing_frames`` consecutive
frames becomes ``present=False``; its filters reset when it reappears).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from .gestures import Debouncer, classify, finger_features, isotropic, palm_scale, pinch
from .one_euro import OneEuroFilter

MODEL_PATH = Path(__file__).resolve().parent / "models" / "hand_landmarker.task"
SIDES = ("left", "right")


@dataclass
class HandOutput:
    present: bool = False
    landmarks: np.ndarray = field(default_factory=lambda: np.zeros((21, 3)))
    palm_scale: float = 0.0
    pinch: float = 0.0
    is_open: bool = False
    is_fist: bool = False
    extensions: np.ndarray = field(default_factory=lambda: np.zeros(4))
    raw_landmarks: np.ndarray = field(default_factory=lambda: np.zeros((21, 3)))
    raw_palm_scale: float = 0.0
    raw_pinch: float = 0.0
    score: float = 0.0


class _HandState:
    def __init__(self, cfg: dict, width: int):
        lm, ps, g = cfg["landmarks"], cfg["palm_scale"], cfg["gestures"]
        self.lm_filter = OneEuroFilter(lm["min_cutoff"], lm["beta"], lm["d_cutoff"],
                                       lm.get("value_scale", width))
        self.ps_filter = OneEuroFilter(ps["min_cutoff"], ps["beta"], ps["d_cutoff"],
                                       ps.get("value_scale", width))
        self.open_db = Debouncer(g["debounce_frames"])
        self.fist_db = Debouncer(g["debounce_frames"])
        self.missing = 0
        self.out = HandOutput()

    def reset(self) -> None:
        self.lm_filter.reset()
        self.ps_filter.reset()
        self.open_db.reset()
        self.fist_db.reset()


class HandFeatureState:
    """Per-hand filtering, features and presence logic, fed with raw landmark arrays."""

    def __init__(self, cfg: dict, width: int, height: int):
        self.cfg = cfg
        self.width, self.height = width, height
        self.missing_frames = int(cfg["missing_frames"])
        self.g = cfg["gestures"]
        self.feature = self.g.get("feature", "extension")
        self.thr = self.g["thresholds"][self.feature]
        self.state = {s: _HandState(cfg, width) for s in SIDES}

    def update(self, detections: dict, t: float) -> dict:
        """``detections``: side -> (raw (21, 3) landmarks, score). Returns side -> HandOutput."""
        out = {}
        for side in SIDES:
            st = self.state[side]
            if side not in detections:
                st.missing += 1
                if st.missing >= self.missing_frames:
                    st.out = HandOutput()
                out[side] = st.out
                continue
            raw, score = detections[side]
            if not st.out.present:
                st.reset()
            st.missing = 0
            lm = st.lm_filter(raw, t)
            raw_iso, lm_iso = isotropic(raw, self.width, self.height), isotropic(
                lm, self.width, self.height)
            raw_ps = palm_scale(raw_iso)
            ps = float(st.ps_filter(raw_ps, t))
            ext = finger_features(lm_iso, ps, self.feature)
            raw_open, raw_fist = classify(ext, self.thr["open"], self.thr["fist"])
            st.out = HandOutput(
                present=True, landmarks=lm, palm_scale=ps, pinch=pinch(lm_iso, ps),
                is_open=st.open_db.update(raw_open), is_fist=st.fist_db.update(raw_fist),
                extensions=ext, raw_landmarks=np.asarray(raw, float), raw_palm_scale=raw_ps,
                raw_pinch=pinch(raw_iso, raw_ps), score=score)
            out[side] = st.out
        return out


def assign_sides(labels_scores: list[tuple[str, float]], swap: bool) -> list[str]:
    """Map MediaPipe handedness labels to our sides; resolve duplicate labels by score."""
    sides = [None] * len(labels_scores)
    order = sorted(range(len(labels_scores)), key=lambda i: -labels_scores[i][1])
    taken = set()
    for i in order:
        side = labels_scores[i][0].lower()
        if swap:
            side = "left" if side == "right" else "right"
        if side in taken:
            side = "left" if side == "right" else "right"
        if side in taken:
            continue
        sides[i] = side
        taken.add(side)
    return sides


class HandTracker:
    def __init__(self, cfg: dict, width: int, height: int, model_path: Path | str = MODEL_PATH):
        from mediapipe.tasks import python as mp_tasks
        from mediapipe.tasks.python import vision

        opts = vision.HandLandmarkerOptions(
            base_options=mp_tasks.BaseOptions(model_asset_path=str(model_path)),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=2,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.landmarker = vision.HandLandmarker.create_from_options(opts)
        self.swap = bool(cfg.get("swap_handedness", False))
        self.features = HandFeatureState(cfg, width, height)
        self._last_ms = -1

    def detect(self, frame_bgr: np.ndarray, t: float) -> dict:
        """Raw detections side -> (landmarks (21, 3), score) for a mirrored BGR frame."""
        import mediapipe as mp

        ts_ms = max(int(round(t * 1000.0)), self._last_ms + 1)  # strictly increasing
        self._last_ms = ts_ms
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        res = self.landmarker.detect_for_video(image, ts_ms)
        labels = [(h[0].category_name, h[0].score) for h in res.handedness]
        sides = assign_sides(labels, self.swap)
        det = {}
        for side, lms, (_, score) in zip(sides, res.hand_landmarks, labels):
            if side is not None:
                det[side] = (np.array([[p.x, p.y, p.z] for p in lms]), score)
        return det

    def process(self, frame_bgr: np.ndarray, t: float) -> dict:
        return self.features.update(self.detect(frame_bgr, t), t)

    def close(self) -> None:
        self.landmarker.close()
