"""Render a clip of scripted pick-and-place runs (offscreen, EGL) to an mp4.

    MUJOCO_GL=egl python3 -m ur5e_2f85_mujoco.record_clip --out results/scene_grasp_clip.mp4
"""

from __future__ import annotations

import argparse
import os
import subprocess
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from . import load_model, load_model_names  # noqa: E402
from .kinematics import Kinematics  # noqa: E402
from .scripted import ScriptedController  # noqa: E402
from .task import SceneIndex, make_servo, reset_scene  # noqa: E402
from .trials import load_config  # noqa: E402

FPS = 30
W, H = 960, 540


def run_pick_place(model, data, idx, kin, task, rates, seed, on_tick=None) -> dict:
    ep = reset_scene(model, data, idx, task, seed)
    servo = make_servo(model, idx, rates, load_config("ik.yaml"))
    servo.reset(data.ctrl[idx.arm_act].copy())
    ctl = ScriptedController(kin)
    cube_yaw = 2 * np.arctan2(ep["cube_quat"][3], ep["cube_quat"][0])
    ctl.plan_pick_place(data.site_xpos[idx.tcp_site].copy(), ep["cube_pos"], cube_yaw,
                        ep["target_pos"])
    steps = int(round(rates["physics"] / rates["control"]))
    dt = steps * model.opt.timestep
    for k in range(int(np.ceil((ctl.duration + 0.5) / dt))):
        q, grip, label = ctl.command(k * dt)
        servo.set_command(q, grip)
        servo.tick(data)
        for _ in range(steps):
            mujoco.mj_step(model, data)
        if on_tick:
            on_tick(k * dt, label)
    cube = data.xpos[idx.cube_body]
    dist = float(np.linalg.norm(cube[:2] - ep["target_pos"][:2]))
    return {"seed": seed, "dist_xy": dist, "success": dist < task["success"]["xy_tol"]}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path("results/scene_grasp_clip.mp4"))
    ap.add_argument("--seeds", type=int, nargs="+", default=[3, 11])
    args = ap.parse_args()
    task, rates = load_config("task.yaml"), load_config("rates.yaml")
    model = load_model()
    names = load_model_names()
    data = mujoco.MjData(model)
    idx = SceneIndex.build(model, names)
    kin = Kinematics(model, names)
    renderer = mujoco.Renderer(model, H, W)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.3, 0.0, 0.25]
    cam.distance, cam.azimuth, cam.elevation = 1.8, 150.0, -25.0

    args.out.parent.mkdir(parents=True, exist_ok=True)
    ff = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-pix_fmt",
         "yuv420p", "-crf", "23", str(args.out)], stdin=subprocess.PIPE)
    next_frame = [0.0]

    def on_tick(t, label):
        if t + 1e-9 >= next_frame[0]:
            renderer.update_scene(data, cam)
            ff.stdin.write(renderer.render().tobytes())
            next_frame[0] += 1.0 / FPS

    for seed in args.seeds:
        next_frame[0] = 0.0
        print(run_pick_place(model, data, idx, kin, task, rates, seed, on_tick))
    ff.stdin.close()
    ff.wait()
    renderer.close()
    print("wrote", args.out)


if __name__ == "__main__":
    main()
