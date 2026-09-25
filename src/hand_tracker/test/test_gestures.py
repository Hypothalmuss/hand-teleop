import numpy as np
import pytest
from hand_fixtures import fist, open_hand, pinched_hand

from hand_tracker.gestures import (
    Debouncer,
    classify,
    extensions,
    finger_features,
    isotropic,
    palm_scale,
    pinch,
    straightness,
)
from ur5e_2f85_mujoco.config import load_config

W, H = 640, 480


@pytest.fixture(scope="module")
def thr():
    g = load_config("filters.yaml")["gestures"]
    return g["thresholds"][g["feature"]], g["feature"]


def features(lm, feature):
    iso = isotropic(lm, W, H)
    return finger_features(iso, palm_scale(iso), feature)


def test_open_and_fist_classify(thr):
    t, feature = thr
    assert classify(features(open_hand(), feature), t["open"], t["fist"]) == (True, False)
    assert classify(features(fist(), feature), t["open"], t["fist"]) == (False, True)


def test_straightness_values():
    assert np.all(straightness(isotropic(open_hand(), W, H)) > 0.95)
    assert np.all(straightness(isotropic(fist(), W, H)) < 0.5)


def test_palm_scale_is_rigid_under_finger_motion():
    a, b = isotropic(open_hand(), W, H), isotropic(fist(), W, H)
    assert palm_scale(a) == pytest.approx(palm_scale(b))


def test_pinch_ratio():
    open_iso, pinch_iso = isotropic(open_hand(), W, H), isotropic(pinched_hand(), W, H)
    assert pinch(open_iso, palm_scale(open_iso)) > 0.5
    assert pinch(pinch_iso, palm_scale(pinch_iso)) < 0.1


def test_extension_formula():
    iso = isotropic(open_hand(), W, H)
    ext = extensions(iso, palm_scale(iso))
    tips, mcps = iso[[8, 12, 16, 20], :2], iso[[5, 9, 13, 17], :2]
    assert np.allclose(ext, np.linalg.norm(tips - mcps, axis=1) / palm_scale(iso))


def test_debounce_blocks_single_frame_flips():
    db = Debouncer(3)
    seq = [True, False, True, True, False, True, True, True, False, True]
    out = [db.update(v) for v in seq]
    # becomes True only after three consecutive Trues (index 7), single False frames ignored
    assert out == [False] * 7 + [True] * 3


def test_debounce_changes_after_n_frames():
    db = Debouncer(3, initial=True)
    assert [db.update(False) for _ in range(3)] == [True, True, False]
