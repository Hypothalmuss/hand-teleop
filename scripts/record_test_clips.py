#!/usr/bin/env python3
"""Record the three hand-tracker test clips from the webcam (~1 minute total).

    python3 scripts/record_test_clips.py [--camera 0]

Writes src/hand_tracker/test/data/<clip>.mp4 (raw, unmirrored, live camera mode downscaled
to display.view_width, 30 fps) plus a JSON
ground truth next to each. Follow the on-screen prompt; each clip is 10 s after a 3 s countdown.
"""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import yaml

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "src" / "hand_tracker" / "test" / "data"
sys.path.insert(0, str(ROOT / "src" / "hand_tracker"))

from hand_tracker.camera import open_camera  # noqa: E402

CLIPS = [
    ("right_open_sweep", "RIGHT hand only, OPEN palm: sweep slowly left/right/up/down",
     {"handedness": "right"}),
    ("left_pinch", "LEFT hand only: pinch thumb+index CLOSED then OPEN, 5 times",
     {"handedness": "left", "cycles": 5}),
    ("right_fist_toggle", "RIGHT hand only: OPEN palm -> FIST -> OPEN, 5 times (hold ~1 s each)",
     {"handedness": "right", "cycles": 5}),
]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()
    DATA.mkdir(parents=True, exist_ok=True)
    filters = yaml.safe_load((ROOT / "config" / "filters.yaml").read_text())
    rate = yaml.safe_load((ROOT / "config" / "rates.yaml").read_text())["webcam"]
    cap, width, height = open_camera(dict(filters["camera"], index=args.camera), rate)
    out_w = min(width, filters["display"]["view_width"])
    out_h = round(height * out_w / width)
    for name, prompt, meta in CLIPS:
        t_end = time.monotonic() + 3.0
        while time.monotonic() < t_end:  # countdown with preview
            ok, frame = cap.read()
            view = cv2.flip(cv2.resize(frame, (out_w, out_h)), 1)
            cv2.putText(view, prompt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(view, f"starts in {t_end - time.monotonic():.1f}s", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imshow("record", view)
            cv2.waitKey(1)
        out = cv2.VideoWriter(str(DATA / f"{name}.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 30.0,
                              (out_w, out_h))
        t_end = time.monotonic() + args.seconds
        while time.monotonic() < t_end:
            ok, frame = cap.read()
            if not ok:
                continue
            frame = cv2.resize(frame, (out_w, out_h), interpolation=cv2.INTER_AREA)
            out.write(frame)
            view = cv2.flip(frame, 1)
            cv2.putText(view, f"REC {name}  {t_end - time.monotonic():.1f}s", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow("record", view)
            cv2.waitKey(1)
        out.release()
        (DATA / f"{name}.json").write_text(json.dumps(meta, indent=2) + "\n")
        print("wrote", DATA / f"{name}.mp4")
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
