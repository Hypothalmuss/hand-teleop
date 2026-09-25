"""Phase-1 scene tests (headless, no ROS)."""

import os

os.environ.setdefault("MUJOCO_GL", "egl")

import tempfile  # noqa: E402
import time  # noqa: E402
from concurrent.futures import ProcessPoolExecutor  # noqa: E402
from pathlib import Path  # noqa: E402

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402

from ur5e_2f85_mujoco import load_model, load_model_names, scene_path  # noqa: E402
from ur5e_2f85_mujoco.build_scene import (  # noqa: E402
    add_home_keyframe,
    build_spec,
    load_task_config,
)
from ur5e_2f85_mujoco.kinematics import Kinematics  # noqa: E402
from ur5e_2f85_mujoco.task import SceneIndex, reset_episode  # noqa: E402
from ur5e_2f85_mujoco.trials import load_config, run_grasp_trial  # noqa: E402


@pytest.fixture(scope="module")
def model():
    return load_model()


@pytest.fixture(scope="module")
def names():
    return load_model_names()


def test_load_time_and_names(names):
    t0 = time.perf_counter()
    m = load_model()
    assert time.perf_counter() - t0 < 1.0
    for j in names["arm_joints"] + [names["gripper_joint"], names["cube_joint"]]:
        assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) >= 0, j
    for a in names["arm_actuators"] + [names["gripper_actuator"]]:
        assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) >= 0, a
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, names["tcp_site"]) >= 0
    for c in names["cameras"]:
        assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, c) >= 0, c
    assert m.opt.timestep == pytest.approx(0.002)
    assert m.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    assert m.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC


def test_scene_xml_is_up_to_date():
    """The committed scene.xml equals a fresh build from the assets and config."""
    task = load_task_config()
    q_home = load_config("ik.yaml")["q_home"]
    out_dir = scene_path().parent
    spec = build_spec(task, out_dir)
    add_home_keyframe(spec, q_home, task)
    spec.compile()
    assert spec.to_xml() == scene_path().read_text()


def test_home_hold(model, names):
    data = mujoco.MjData(model)
    idx = SceneIndex.build(model, names)
    mujoco.mj_resetDataKeyframe(model, data, idx.home_key)
    q0 = data.qpos[idx.arm_qpos].copy()
    drift = 0.0
    for _ in range(int(2.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
        drift = max(drift, float(np.abs(data.qpos[idx.arm_qpos] - q0).max()))
    assert drift < 1e-3
    tcp = data.site_xpos[idx.tcp_site]
    assert np.linalg.norm(tcp - np.array(names["tcp_home_pos"])) < 0.01
    # TCP points down: site z axis is world -z.
    zaxis = data.site_xmat[idx.tcp_site].reshape(3, 3)[:, 2]
    assert zaxis[2] < -0.999


def _trial_ok(seed):
    return run_grasp_trial(seed)["success"]


def test_grasp_lift_200():
    with ProcessPoolExecutor(max(1, min(8, os.cpu_count() or 1))) as ex:
        results = list(ex.map(_trial_ok, range(200)))
    assert sum(results) >= 199


def test_render_cameras(model, names):
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key(names["home_keyframe"]).id)
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, 256, 256)
    try:
        for cam in names["cameras"]:
            renderer.update_scene(data, cam)
            img = renderer.render()
            assert img.shape == (256, 256, 3)
            assert img.dtype == np.uint8
            assert img.var() > 0
    finally:
        renderer.close()


def test_determinism(model, names):
    task = load_config("task.yaml")
    idx = SceneIndex.build(model, names)

    def run(seed):
        data = mujoco.MjData(model)
        reset_episode(model, data, idx, task, seed)
        data.ctrl[idx.arm_act] += 0.2  # move the arm so contacts/dynamics are exercised
        for _ in range(1000):
            mujoco.mj_step(model, data)
        return data.qpos.copy()

    a, b = run(7), run(7)
    assert np.array_equal(a, b)
    assert not np.array_equal(a, run(8))


def test_python_fk_matches_mujoco(model, names):
    kin = Kinematics(model, names)
    data = mujoco.MjData(model)
    idx = SceneIndex.build(model, names)
    rng = np.random.default_rng(0)
    for _ in range(100):
        q = rng.uniform(kin.limits[:, 0], kin.limits[:, 1])
        data.qpos[idx.arm_qpos] = q
        mujoco.mj_forward(model, data)
        pos, mat = kin.fk(q)
        assert np.allclose(pos, data.site_xpos[idx.tcp_site], atol=1e-6)
        assert np.allclose(mat.flatten(), data.site_xmat[idx.tcp_site], atol=1e-6)


def test_jacobian_matches_finite_differences(model, names):
    kin = Kinematics(model, names)
    rng = np.random.default_rng(1)
    eps = 1e-6
    for _ in range(20):
        q = rng.uniform(-np.pi, np.pi, 6)
        jac = kin.jacobian(q)
        p0, r0 = kin.fk(q)
        num = np.zeros((6, 6))
        for i in range(6):
            dq = np.zeros(6)
            dq[i] = eps
            pp, rp = kin.fk(q + dq)
            pm, rm = kin.fk(q - dq)
            num[:3, i] = (pp - pm) / (2 * eps)
            # angular: skew part of dR R^T
            w = (rp - rm) / (2 * eps) @ r0.T
            num[3:, i] = [w[2, 1], w[0, 2], w[1, 0]]
        assert np.allclose(jac, num, atol=1e-5)


def test_python_ik_reaches_targets(model, names):
    kin = Kinematics(model, names)
    p_home, r_home = kin.fk(kin.q_home)
    rng = np.random.default_rng(2)
    for _ in range(20):
        target = p_home + rng.uniform([-0.15, -0.3, -0.25], [0.1, 0.1, 0.0])
        q, ok = kin.ik(target, r_home, kin.q_home)
        assert ok
        assert np.linalg.norm(kin.fk(q)[0] - target) < 1e-4


def test_scene_builds_to_temp_dir():
    """The builder works for any output directory (relative mesh paths)."""
    task = load_task_config()
    with tempfile.TemporaryDirectory() as tmp:
        spec = build_spec(task, Path(tmp))
        path = Path(tmp) / "scene.xml"
        path.write_text(spec.to_xml())
        m = mujoco.MjModel.from_xml_path(str(path))
        assert m.nu == 7
