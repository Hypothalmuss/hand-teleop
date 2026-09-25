"""Run the tracker offline over a video clip (tests, latency measurement). No ROS.

    python3 -m hand_tracker.clip_eval test/data/right_open_sweep.mp4
"""

from __future__ import annotations

import argparse
import time

import cv2
import numpy as np

from .tracker import HandTracker


def run_clip(path: str, cfg: dict) -> list[dict]:
    """Per-frame outputs: time, side -> HandOutput, raw detections, processing ms."""
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise FileNotFoundError(path)
    w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    tracker = HandTracker(cfg, w, h)
    frames = []
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            t0 = time.perf_counter()
            t = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
            frame = cv2.flip(frame, 1)
            det = tracker.detect(frame, t)
            hands = tracker.features.update(det, t)
            frames.append({"t": t, "hands": hands, "detected": set(det),
                           "ms": (time.perf_counter() - t0) * 1e3})
    finally:
        cap.release()
        tracker.close()
    return frames


def crossings(values, level: float) -> int:
    v = np.asarray(values) > level
    return int(np.count_nonzero(v[1:] != v[:-1]))


def toggles(flags) -> int:
    f = np.asarray(flags, bool)
    return int(np.count_nonzero(f[1:] != f[:-1]))


def main() -> None:
    import yaml

    from ur5e_2f85_mujoco.config import find_config_dir

    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    args = ap.parse_args()
    with open(find_config_dir() / "filters.yaml") as f:
        cfg = yaml.safe_load(f)
    frames = run_clip(args.video, cfg)
    ms = np.array([f["ms"] for f in frames])
    print(f"{len(frames)} frames, processing p50 {np.percentile(ms, 50):.1f} ms "
          f"p95 {np.percentile(ms, 95):.1f} ms")
    for side in ("left", "right"):
        present = [f["hands"][side].present for f in frames]
        pinches = [f["hands"][side].pinch for f in frames if f["hands"][side].present]
        opens = [f["hands"][side].is_open for f in frames if f["hands"][side].present]
        print(f"{side}: present {np.mean(present):.0%}, pinch crossings(0.5) "
              f"{crossings(pinches, 0.5)}, open toggles {toggles(opens)}")


if __name__ == "__main__":
    main()
