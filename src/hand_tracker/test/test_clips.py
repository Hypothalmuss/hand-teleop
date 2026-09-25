"""Tracker accuracy on the recorded test clips (skipped until they are recorded).

Record them with ``python3 scripts/record_test_clips.py`` (webcam, about 1 minute).
"""

import json
from pathlib import Path

import numpy as np
import pytest

from hand_tracker.clip_eval import crossings, run_clip, toggles
from ur5e_2f85_mujoco.config import load_config

DATA = Path(__file__).resolve().parent / "data"
CLIPS = {
    "right_open_sweep": "handedness",
    "left_pinch": "pinch",
    "right_fist_toggle": "toggle",
}


def load(name):
    video, meta = DATA / f"{name}.mp4", DATA / f"{name}.json"
    if not (video.exists() and meta.exists()):
        pytest.skip(f"test clip {video.name} not recorded yet (scripts/record_test_clips.py)")
    return run_clip(video, load_config("filters.yaml")), json.loads(meta.read_text())


@pytest.mark.parametrize("name", list(CLIPS))
def test_handedness(name):
    frames, meta = load(name)
    side = meta["handedness"]
    with_hand = [f for f in frames if f["detected"]]
    assert len(with_hand) > 0.5 * len(frames)
    correct = np.mean([f["detected"] == {side} for f in with_hand])
    assert correct >= 0.95, f"handedness correct in {correct:.1%} of frames"


def test_pinch_crossings():
    frames, meta = load("left_pinch")
    pinch = [f["hands"]["left"].pinch for f in frames if f["hands"]["left"].present]
    expected = 2 * meta["cycles"]
    assert abs(crossings(pinch, 0.5) - expected) <= 1


def test_fist_open_toggles():
    frames, meta = load("right_fist_toggle")
    opens = [f["hands"]["right"].is_open for f in frames if f["hands"]["right"].present]
    expected = 2 * meta["cycles"]
    assert abs(toggles(opens) - expected) <= 1
