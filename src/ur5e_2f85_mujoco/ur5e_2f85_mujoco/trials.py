"""Scripted grasp-lift trials over random cube poses (contact tuning and tests).

    python3 -m ur5e_2f85_mujoco.trials --n 200 [--workers 8]
"""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import mujoco
import numpy as np
import yaml

from . import load_model, load_model_names
from .kinematics import Kinematics
from .scripted import ScriptedController, ScriptParams
from .task import SceneIndex, apply_joint_command, cube_pose, cube_speed, reset_episode

REPO_ROOT = Path(__file__).resolve().parents[3]


def load_config(name: str) -> dict:
    with open(REPO_ROOT / "config" / name) as f:
        return yaml.safe_load(f)


def run_grasp_trial(seed: int, task: dict | None = None, rates: dict | None = None,
                    params: ScriptParams | None = None, model: mujoco.MjModel | None = None,
                    frame_cb=None) -> dict:
    task = task or load_config("task.yaml")
    rates = rates or load_config("rates.yaml")
    model = model if model is not None else load_model()
    names = load_model_names()
    data = mujoco.MjData(model)
    idx = SceneIndex.build(model, names)
    kin = Kinematics(model, names)
    ctl = ScriptedController(kin, params or ScriptParams())

    ep = reset_episode(model, data, idx, task, seed)
    cube_yaw = 2 * np.arctan2(ep["cube_quat"][3], ep["cube_quat"][0])
    ctl.plan_grasp_lift(data.site_xpos[idx.tcp_site].copy(), ep["cube_pos"], cube_yaw)

    steps_per_tick = int(round(rates["physics"] / rates["control"]))
    dt_tick = steps_per_tick * model.opt.timestep
    n_ticks = int(np.ceil(ctl.duration / dt_tick))
    max_pen = 0.0
    rel_hold = []  # cube position relative to the TCP during the hold (slip)
    for k in range(n_ticks):
        q, grip, label = ctl.command(k * dt_tick)
        apply_joint_command(data, idx, q, grip, None, None)
        for _ in range(steps_per_tick):
            mujoco.mj_step(model, data)
        if label == "hold":
            rel_hold.append(data.xpos[idx.cube_body] - data.site_xpos[idx.tcp_site])
        if data.ncon:
            max_pen = max(max_pen, float(-data.contact.dist[: data.ncon].min()))
        if frame_cb is not None:
            frame_cb(model, data, k * dt_tick)
    pos, _ = cube_pose(data, idx)
    speed = cube_speed(data, idx)
    ok = bool(pos[2] > 0.08 and speed < 0.02)
    slip = float(np.linalg.norm(rel_hold[-1] - rel_hold[0])) if rel_hold else float("nan")
    return {"seed": seed, "success": ok, "cube_z": float(pos[2]), "cube_speed": speed,
            "hold_slip": slip,
            "max_penetration": max_pen, "sim_time": n_ticks * dt_tick}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--n", type=int, default=200)
    parser.add_argument("--seed0", type=int, default=0)
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args()
    seeds = list(range(args.seed0, args.seed0 + args.n))
    with ProcessPoolExecutor(args.workers) as ex:
        results = list(ex.map(run_grasp_trial, seeds))
    fails = [r for r in results if not r["success"]]
    print(f"success {len(results) - len(fails)}/{len(results)}")
    print(f"max hold slip {max(r['hold_slip'] for r in results) * 1000:.3f} mm")
    print(f"max penetration {max(r['max_penetration'] for r in results) * 1000:.2f} mm")
    for r in fails:
        print("FAIL", r)


if __name__ == "__main__":
    main()
