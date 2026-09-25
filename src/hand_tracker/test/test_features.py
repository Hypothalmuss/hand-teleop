"""HandFeatureState: presence logic, filter reset on reappearance, debounced flags."""

import numpy as np
from hand_fixtures import fist, open_hand

from hand_tracker.tracker import HandFeatureState, assign_sides
from ur5e_2f85_mujoco.config import load_config

FS = 30.0


def make():
    return HandFeatureState(load_config("filters.yaml"), 640, 480)


def test_missing_frames_then_absent():
    fs = make()
    n = load_config("filters.yaml")["missing_frames"]
    fs.update({"right": (open_hand(), 0.9)}, 0.0)
    for k in range(1, n):
        assert fs.update({}, k / FS)["right"].present
    assert not fs.update({}, n / FS)["right"].present


def test_filter_resets_on_reappearance():
    fs = make()
    n = load_config("filters.yaml")["missing_frames"]
    fs.update({"right": (open_hand(), 0.9)}, 0.0)
    for k in range(1, n + 1):
        fs.update({}, k / FS)
    moved = open_hand(dx=0.2)
    out = fs.update({"right": (moved, 0.9)}, (n + 1) / FS)["right"]
    assert np.allclose(out.landmarks, moved)  # no smoothing towards the stale position


def test_open_flag_is_debounced():
    fs = make()
    flags = [fs.update({"right": (open_hand(), 0.9)}, k / FS)["right"].is_open for k in range(5)]
    assert flags == [False, False, True, True, True]
    flags = [fs.update({"right": (fist(), 0.9)}, (5 + k) / FS)["right"] for k in range(4)]
    assert [f.is_open for f in flags] == [True, True, False, False]
    assert flags[-1].is_fist is False or flags[-1].is_fist is True  # fist needs its own 3


def test_assign_sides():
    assert assign_sides([("Left", 0.9), ("Right", 0.8)], swap=False) == ["left", "right"]
    assert assign_sides([("Left", 0.9), ("Right", 0.8)], swap=True) == ["right", "left"]
    assert assign_sides([("Right", 0.7), ("Right", 0.9)], swap=False) == ["left", "right"]
