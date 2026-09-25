import itertools

import numpy as np
import pytest

from teleop_mapper.mapping import (
    AXES,
    HandInput,
    TeleopMapperCore,
    gripper_from_pinch,
    hand_axes,
    hand_to_workspace,
)
from ur5e_2f85_mujoco.config import load_config

FS = 30.0


@pytest.fixture
def ws():
    return load_config("workspace.yaml")


def hand_at(u, v, d, is_open=True, is_fist=False, present=True, pinch=1.0):
    """A HandInput whose landmark 9 / palm_scale give exactly (u, v, d)."""
    lm = np.zeros((21, 3))
    lm[9] = [u, 1.0 - v, 0.0]
    return HandInput(present, lm, 1.0 / d, pinch, is_open, is_fist)


ABSENT = HandInput(False, np.zeros((21, 3)), 0.0, 0.0, False, False)


def test_hand_axes():
    h = hand_at(0.3, 0.7, 8.0)
    assert np.allclose(hand_axes(h.landmarks, h.palm_scale), [0.3, 0.7, 8.0])


def test_corners_map_to_box_corners(ws):
    rng = ws["hand_range"]
    for cu, cv, cd in itertools.product((0, 1), repeat=3):
        uvd = np.array([rng["u"][cu], rng["v"][cv], rng["d"][cd]])
        p = hand_to_workspace(uvd, ws)
        for i, hand_axis in enumerate(("u", "v", "d")):
            robot = ws["axis_map"][hand_axis]
            corner = (cu, cv, cd)[i]
            if ws["sign"][robot] < 0:
                corner = 1 - corner
            assert p[AXES.index(robot)] == pytest.approx(ws["box"][robot][corner])


def test_outside_range_clamps(ws):
    rng = ws["hand_range"]
    far = np.array([rng["u"][1] + 1, rng["v"][0] - 1, rng["d"][1] + 50])
    p = hand_to_workspace(far, ws)
    lo = np.array([ws["box"][a][0] for a in AXES])
    hi = np.array([ws["box"][a][1] for a in AXES])
    assert np.all(p >= lo - 1e-12) and np.all(p <= hi + 1e-12)
    assert np.allclose(p, hand_to_workspace(np.clip(far, [r[0] for r in rng.values()],
                                                    [r[1] for r in rng.values()]), ws))


def test_reanchor_no_jump(ws):
    core = TeleopMapperCore(ws, 30.0)
    ee = np.array([0.45, 0.10, 0.20])
    t = 0.0
    # engaged at u = 0.5, then fist (disengage), jump 0.3 in u, then open (engage)
    for _ in range(5):
        core.update(hand_at(0.5, 0.5, 7.0), ABSENT, ee, False, t)
        t += 1 / FS
    core.update(hand_at(0.5, 0.5, 7.0, is_open=False, is_fist=True), ABSENT, ee, False, t)
    t += 1 / FS
    ee_now = np.array([0.40, -0.05, 0.15])  # the arm moved meanwhile
    core.update(hand_at(0.8, 0.5, 7.0, is_open=False, is_fist=True), ABSENT, ee_now, False, t)
    t += 1 / FS
    out = core.update(hand_at(0.8, 0.5, 7.0), ABSENT, ee_now, False, t)
    assert out.engaged
    assert np.linalg.norm(out.target - ee_now) < 1e-3


def test_speed_limit(ws):
    core = TeleopMapperCore(ws, 30.0)
    ee = np.array([0.45, 0.0, 0.20])
    t = 0.0
    core.update(hand_at(0.5, 0.5, 7.0), ABSENT, ee, False, t)
    prev = core.target.copy()
    rng = ws["hand_range"]
    du = 1.0 / (ws["box"]["y"][1] - ws["box"]["y"][0]) * (rng["u"][1] - rng["u"][0])
    speeds = []
    for _ in range(60):  # hand jumped by the equivalent of 1 m in y
        t += 1 / FS
        out = core.update(hand_at(0.5 + du, 0.5, 7.0), ABSENT, ee, False, t)
        speeds.append(np.linalg.norm(out.target - prev) * FS)
        prev = out.target
    assert max(speeds) <= ws["max_target_speed"] + 1e-9
    assert max(speeds) > 0.9 * ws["max_target_speed"]


def test_gripper_ramp_rate_limited_and_hold(ws):
    core = TeleopMapperCore(ws, 30.0)
    ee = np.array([0.45, 0.0, 0.20])
    t = 0.0
    core.update(ABSENT, hand_at(0.5, 0.5, 7.0, pinch=0.2), ee, False, t)
    # the first frame is itself rate limited from the initial open value
    grips = []
    for k in range(1, 4):  # pinch 0.2 -> 1.1 over 0.1 s
        t += 1 / FS
        pinch = 0.2 + 0.9 * min(1.0, k / 3)
        grips.append(core.update(ABSENT, hand_at(0.5, 0.5, 7.0, pinch=pinch), ee, False,
                                 t).gripper)
    for _ in range(40):
        t += 1 / FS
        grips.append(core.update(ABSENT, hand_at(0.5, 0.5, 7.0, pinch=1.1), ee, False,
                                 t).gripper)
    rates = np.abs(np.diff(grips)) * FS
    assert rates.max() <= ws["gripper"]["rate_limit"] + 1e-9
    assert grips[-1] == pytest.approx(1.0)
    held = core.update(ABSENT, ABSENT, ee, False, t + 1 / FS).gripper
    assert held == grips[-1]


def test_gripper_from_pinch(ws):
    g = ws["gripper"]
    assert gripper_from_pinch(g["pinch_closed"], g["pinch_closed"], g["pinch_open"]) == 0.0
    assert gripper_from_pinch(g["pinch_open"] + 1, g["pinch_closed"], g["pinch_open"]) == 1.0


def test_estop_disengages_within_one_message(ws):
    core = TeleopMapperCore(ws, 30.0)
    ee = np.array([0.45, 0.0, 0.20])
    assert core.update(hand_at(0.5, 0.5, 7.0), ABSENT, ee, False, 0.0).engaged
    assert not core.update(hand_at(0.5, 0.5, 7.0), ABSENT, ee, True, 1 / FS).engaged


def test_absent_hold_then_disengage(ws):
    core = TeleopMapperCore(ws, 30.0)
    ee = np.array([0.45, 0.0, 0.20])
    core.update(hand_at(0.5, 0.5, 7.0), ABSENT, ee, False, 0.0)
    assert core.update(ABSENT, ABSENT, ee, False, 0.2).engaged
    assert not core.update(ABSENT, ABSENT, ee, False, 0.31).engaged


def test_disengaged_holds_target(ws):
    core = TeleopMapperCore(ws, 30.0)
    ee = np.array([0.45, 0.0, 0.20])
    core.update(hand_at(0.5, 0.5, 7.0), ABSENT, ee, False, 0.0)
    out = core.update(hand_at(0.5, 0.5, 7.0, is_open=False, is_fist=True), ABSENT, ee, False,
                      0.1)
    moved = core.update(hand_at(0.9, 0.9, 5.0, is_open=False, is_fist=True), ABSENT, ee,
                        False, 0.2)
    assert not moved.engaged and np.array_equal(moved.target, out.target)
