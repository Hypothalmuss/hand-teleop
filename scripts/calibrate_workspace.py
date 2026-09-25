#!/usr/bin/env python3
"""Hand-to-workspace calibration wizard (< 60 s). Writes config/workspace.yaml ranges.

    python3 scripts/calibrate_workspace.py [--camera 0]

Hold an OPEN right hand at centre, left, right, top, bottom, near, far (2 s each), then show
the LEFT hand pinched closed and wide open (2 s each). The wizard records u, v, d ranges for
the right hand and the left-hand pinch range.
"""

import argparse
import re
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
for pkg in ("hand_tracker", "teleop_mapper", "ur5e_2f85_mujoco"):
    sys.path.insert(0, str(ROOT / "src" / pkg))

from hand_tracker import overlay  # noqa: E402
from hand_tracker.camera import open_camera  # noqa: E402
from hand_tracker.tracker import HandTracker  # noqa: E402
from teleop_mapper.mapping import hand_axes  # noqa: E402

STEPS = [  # (key, prompt, hand)
    ("center", "RIGHT hand open: CENTRE, comfortable distance", "right"),
    ("left", "RIGHT hand open: far LEFT", "right"),
    ("right", "RIGHT hand open: far RIGHT", "right"),
    ("top", "RIGHT hand open: TOP", "right"),
    ("bottom", "RIGHT hand open: BOTTOM", "right"),
    ("near", "RIGHT hand open: NEAR the webcam (centre)", "right"),
    ("far", "RIGHT hand open: FAR from the webcam (centre)", "right"),
    ("pinch_closed", "LEFT hand: thumb and index PINCHED", "left"),
    ("pinch_open", "LEFT hand: thumb and index WIDE OPEN", "left"),
]
HOLD_S, MOVE_S = 2.0, 1.2


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--dry-run", action="store_true", help="print, do not write the config")
    args = ap.parse_args()
    cfg_dir = ROOT / "config"
    filters = yaml.safe_load((cfg_dir / "filters.yaml").read_text())
    cam = dict(filters["camera"], index=args.camera)
    rate = yaml.safe_load((ROOT / "config" / "rates.yaml").read_text())["webcam"]
    cap, width, height = open_camera(cam, rate)  # same mode as live teleop
    view_w = filters["display"]["view_width"]
    tracker = HandTracker(filters, width, height)
    samples = {}
    for key, prompt, side in STEPS:
        vals = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < MOVE_S + HOLD_S:
            ok, frame = cap.read()
            if not ok:
                continue
            frame = cv2.flip(frame, 1)
            now = time.monotonic()
            hands = tracker.process(frame, now)
            h = hands[side]
            recording = now - t0 > MOVE_S
            if recording and h.present:
                vals.append(h.pinch if key.startswith("pinch") else
                            hand_axes(h.landmarks, h.palm_scale))
            small = cv2.resize(frame, (view_w, round(frame.shape[0] * view_w / frame.shape[1])))
            view = overlay.draw(small, hands, 0.0, 0.0)
            color = (0, 0, 255) if recording else (0, 255, 255)
            cv2.putText(view, prompt, (10, view.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        color, 2)
            cv2.imshow("calibrate_workspace", view)
            cv2.waitKey(1)
        if not vals:
            sys.exit(f"no {side} hand seen during '{key}'; run again")
        samples[key] = np.median(np.array(vals), axis=0)
    cap.release()
    cv2.destroyAllWindows()
    tracker.close()

    u = sorted([samples["left"][0], samples["right"][0]])
    v = sorted([samples["bottom"][1], samples["top"][1]])
    d = sorted([samples["near"][2], samples["far"][2]])
    pc, po = float(samples["pinch_closed"]), float(samples["pinch_open"])
    print(f"u {u}\nv {v}\nd {d}\npinch closed {pc:.3f} open {po:.3f}")
    if min(u[1] - u[0], v[1] - v[0], d[1] - d[0]) <= 1e-3 or po <= pc:
        sys.exit("degenerate range; run again with larger motions")
    if args.dry_run:
        return
    path = cfg_dir / "workspace.yaml"
    text = path.read_text()
    for name, rng in (("u", u), ("v", v), ("d", d)):
        text = re.sub(rf"^(  {name}: )\[[^\]]*\]", rf"\g<1>[{rng[0]:.4f}, {rng[1]:.4f}]", text,
                      flags=re.M)
    text = re.sub(r"^(  pinch_closed: )[0-9.]+", rf"\g<1>{pc:.3f}", text, flags=re.M)
    text = re.sub(r"^(  pinch_open: )[0-9.]+", rf"\g<1>{po:.3f}", text, flags=re.M)
    path.write_text(text)
    print("updated", path)


if __name__ == "__main__":
    main()
