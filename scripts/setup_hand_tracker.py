#!/usr/bin/env python3
"""Handedness setup: raise your RIGHT hand; writes swap_handedness to config/filters.yaml.

    python3 scripts/setup_hand_tracker.py [--camera 0]
"""

import argparse
import re
import sys
import time
from collections import Counter
from pathlib import Path

import cv2
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "hand_tracker"))
sys.path.insert(0, str(ROOT / "src" / "ur5e_2f85_mujoco"))

from hand_tracker import overlay  # noqa: E402
from hand_tracker.camera import open_camera  # noqa: E402
from hand_tracker.tracker import HandTracker  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    args = ap.parse_args()
    cfg_path = ROOT / "config" / "filters.yaml"
    cfg = yaml.safe_load(cfg_path.read_text())
    cfg["swap_handedness"] = False  # measure the raw MediaPipe labels
    cam = dict(cfg["camera"], index=args.camera)
    rate = yaml.safe_load((ROOT / "config" / "rates.yaml").read_text())["webcam"]
    cap, width, height = open_camera(cam, rate)  # same mode as live teleop
    view_w = cfg["display"]["view_width"]
    tracker = HandTracker(cfg, width, height)
    votes = Counter()
    t_start = time.monotonic()
    while time.monotonic() - t_start < 8.0:
        ok, frame = cap.read()
        if not ok:
            continue
        frame = cv2.flip(frame, 1)
        t = time.monotonic()
        det = tracker.detect(frame, t)
        hands = tracker.features.update(det, t)
        if time.monotonic() - t_start > 3.0 and len(det) == 1:
            votes[next(iter(det))] += 1
        small = cv2.resize(frame, (view_w, round(frame.shape[0] * view_w / frame.shape[1])))
        view = overlay.draw(small, hands, 0.0, 0.0)
        cv2.putText(view, "Raise ONLY your RIGHT hand, open palm", (10, view.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("setup_hand_tracker", view)
        cv2.waitKey(1)
    cap.release()
    cv2.destroyAllWindows()
    tracker.close()
    if not votes:
        sys.exit("no single hand seen; try again")
    label, n = votes.most_common(1)[0]
    swap = label != "right"
    print(f"MediaPipe labelled your right hand '{label}' in {n}/{sum(votes.values())} frames "
          f"-> swap_handedness: {str(swap).lower()}")
    text = cfg_path.read_text()
    text = re.sub(r"^swap_handedness:.*$", f"swap_handedness: {str(swap).lower()}", text,
                  flags=re.M)
    cfg_path.write_text(text)
    print("updated", cfg_path)


if __name__ == "__main__":
    main()
