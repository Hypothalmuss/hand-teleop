"""Compose UR5e + Robotiq 2F-85 + table + cube + target into ``scene.xml`` with MjSpec.

Run from anywhere:  python3 -m ur5e_2f85_mujoco.build_scene [--task-config config/task.yaml]

The generated ``scene.xml`` references meshes by paths relative to its own directory, so it
works both in the source tree and in the installed share directory (same relative layout).
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import mujoco
import numpy as np
import yaml

PKG_DIR = Path(__file__).resolve().parent
ASSETS_DIR = PKG_DIR.parent / "assets"
REPO_ROOT = PKG_DIR.parents[2]
GRIPPER_PREFIX = "2f85/"

# Scene-construction constants that are not task parameters (camera/light placement).
FRONT_CAM_POS = np.array([1.1, 0.0, 0.7])
FRONT_CAM_LOOKAT = np.array([0.45, 0.0, 0.05])
FRONT_CAM_FOVY = 45.0
WRIST_CAM_FOVY = 60.0
# Wrist camera in the gripper base frame: offset off the finger plane (fingers move along y),
# looking along +z at a point just past the fingertips.
WRIST_CAM_POS = np.array([-0.07, 0.0, 0.035])
WRIST_CAM_LOOKAT = np.array([0.0, 0.0, 0.20])
WRIST_CAM_UP = (-1.0, 0.0, 0.0)
# TCP offset along the gripper base +z: midpoint of the finger pads when closed (measured).
TCP_OFFSET = 0.1439


def _look_at_quat(pos: np.ndarray, target: np.ndarray, up=(0.0, 0.0, 1.0)) -> np.ndarray:
    """Quaternion (w, x, y, z) for a MuJoCo camera at ``pos`` looking at ``target``.

    MuJoCo cameras look along their local -z with +y up.
    """
    fwd = target - pos
    fwd /= np.linalg.norm(fwd)
    z = -fwd
    x = np.cross(np.asarray(up, float), z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    mat = np.stack([x, y, z], axis=1)
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, mat.flatten())
    return quat


def _relativize_meshes(spec: mujoco.MjSpec, model_dir: Path, out_dir: Path) -> None:
    """Point every mesh ``file`` at a path relative to ``out_dir`` and clear ``meshdir``."""
    mesh_dir = model_dir / spec.meshdir
    for mesh in spec.meshes:
        abs_path = (mesh_dir / mesh.file).resolve()
        mesh.file = os.path.relpath(abs_path, out_dir)
    spec.meshdir = ""


def load_task_config(path: Path | None = None) -> dict:
    path = Path(path) if path else REPO_ROOT / "config" / "task.yaml"
    with open(path) as f:
        return yaml.safe_load(f)


def build_spec(task: dict, out_dir: Path = PKG_DIR) -> mujoco.MjSpec:
    arm_dir = ASSETS_DIR / "universal_robots_ur5e"
    grip_dir = ASSETS_DIR / "robotiq_2f85"
    arm = mujoco.MjSpec.from_file(str(arm_dir / "ur5e.xml"))
    grip = mujoco.MjSpec.from_file(str(grip_dir / "2f85.xml"))
    _relativize_meshes(arm, arm_dir, out_dir)
    _relativize_meshes(grip, grip_dir, out_dir)

    arm.modelname = "ur5e_2f85_pickplace"
    # Keyframes are rebuilt below once every joint exists; the stock light is replaced.
    for key in list(arm.keys):
        arm.delete(key)
    for light in list(arm.lights):
        arm.delete(light)

    # Relative mesh paths resolve against the directory the scene is written to.
    arm.modelfiledir = str(out_dir)
    grip.modelfiledir = str(out_dir)

    # --- options (Menagerie 2F-85 scene values), set before attach so they agree ---
    arm.option.timestep = 0.002
    arm.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    arm.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    arm.option.impratio = 10.0

    # --- attach gripper to the flange ---
    arm.site("attachment_site").attach_body(grip.body("base_mount"), GRIPPER_PREFIX, "")

    # --- gravity compensation: the real UR controller compensates gravity internally ---
    arm_joint_names = [j.name for j in arm.joints if not j.name.startswith(GRIPPER_PREFIX)]
    for body in arm.bodies:
        if body.name not in ("world", "base"):
            body.gravcomp = 1.0
    for jname in arm_joint_names:
        arm.joint(jname).actgravcomp = True

    # --- TCP site on the gripper base, on the flange +z axis ---
    gbase = arm.body(GRIPPER_PREFIX + "base")
    gbase.add_site(name="tcp", pos=[0, 0, TCP_OFFSET], size=[0.004, 0, 0], rgba=[1, 1, 0, 0.6],
                   group=4)

    # --- wrist camera: child of the gripper base, looking along +z between the fingers ---
    gbase.add_camera(name="wrist", pos=WRIST_CAM_POS,
                     quat=_look_at_quat(WRIST_CAM_POS, WRIST_CAM_LOOKAT, WRIST_CAM_UP),
                     fovy=WRIST_CAM_FOVY)

    world = arm.worldbody

    # --- visuals ---
    arm.visual.headlight.diffuse = [0.4, 0.4, 0.4]
    arm.visual.headlight.ambient = [0.3, 0.3, 0.3]
    arm.visual.headlight.specular = [0, 0, 0]
    arm.visual.global_.offwidth = 1280
    arm.visual.global_.offheight = 960
    arm.visual.quality.shadowsize = 2048
    arm.add_texture(name="skybox", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                    builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT, rgb1=[0.3, 0.5, 0.7],
                    rgb2=[0, 0, 0], width=512, height=3072)
    arm.add_texture(name="floor", type=mujoco.mjtTexture.mjTEXTURE_2D,
                    builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER, mark=mujoco.mjtMark.mjMARK_EDGE,
                    rgb1=[0.2, 0.3, 0.4], rgb2=[0.1, 0.2, 0.3], markrgb=[0.8, 0.8, 0.8],
                    width=300, height=300)
    floor_mat = arm.add_material(name="floor", texrepeat=[5, 5], texuniform=True, reflectance=0.1)
    floor_mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "floor"
    arm.add_material(name="table", rgba=[0.62, 0.52, 0.42, 1.0])

    world.add_light(name="key", pos=[0.8, -0.6, 1.8], dir=[-0.4, 0.3, -1.0], castshadow=True,
                    diffuse=[0.6, 0.6, 0.6])
    world.add_light(name="fill", pos=[-0.4, 0.6, 1.6], dir=[0.4, -0.3, -1.0], castshadow=True,
                    diffuse=[0.4, 0.4, 0.4])

    # --- table and floor ---
    tab = task["table"]
    tx, ty = tab["center_xy"]
    hx, hy = tab["size"][0] / 2, tab["size"][1] / 2
    ht = tab["thickness"] / 2
    world.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0, 0, 0.05],
                   pos=[0, 0, -0.75], material="floor")
    world.add_geom(name="table", type=mujoco.mjtGeom.mjGEOM_BOX, size=[hx, hy, ht],
                   pos=[tx, ty, -ht], material="table")

    # --- cube ---
    cube = task["cube"]
    cx, cy = [np.mean(task["spawn"]["cube_x"]), np.mean(task["spawn"]["cube_y"])]
    hs = cube["half_size"]
    cube_body = world.add_body(name="cube", pos=[cx, cy, hs])
    cube_body.add_freejoint(name="cube_joint")
    cube_body.add_geom(name="cube", type=mujoco.mjtGeom.mjGEOM_BOX, size=[hs, hs, hs],
                       mass=cube["mass"], friction=cube["friction"], priority=1, condim=4,
                       rgba=cube["rgba"], solref=cube.get("solref", [0.01, 1.0]),
                       solimp=cube.get("solimp", [0.95, 0.99, 0.001, 0.5, 2.0]))

    # --- target disc (visual only, mocap so reset can move it) ---
    tgt = task["target"]
    target_body = world.add_body(name="target", mocap=True, pos=tgt["default_pos"])
    target_body.add_geom(name="target", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                         size=[tgt["radius"], tgt["half_height"], 0], contype=0, conaffinity=0,
                         rgba=tgt["rgba"])

    # --- front camera ---
    world.add_camera(name="front", pos=FRONT_CAM_POS,
                     quat=_look_at_quat(FRONT_CAM_POS, FRONT_CAM_LOOKAT), fovy=FRONT_CAM_FOVY)
    return arm


def add_home_keyframe(spec: mujoco.MjSpec, q_home, task: dict) -> None:
    model = spec.compile()
    data = mujoco.MjData(model)
    arm_joints = [model.joint(i).name for i in range(model.njnt)
                  if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE
                  and not model.joint(i).name.startswith(GRIPPER_PREFIX)]
    for name, q in zip(arm_joints, q_home):
        data.joint(name).qpos[0] = q
    cube_q = data.joint("cube_joint").qpos
    cube_q[:3] = [np.mean(task["spawn"]["cube_x"]), np.mean(task["spawn"]["cube_y"]),
                  task["cube"]["half_size"]]
    cube_q[3:] = [1, 0, 0, 0]
    ctrl = np.zeros(model.nu)
    ctrl[: len(q_home)] = q_home  # gripper ctrl 0 = open
    spec.add_key(name="home", qpos=data.qpos.copy(), ctrl=ctrl,
                 mpos=np.asarray(task["target"]["default_pos"], float))


def write_model_names(model: mujoco.MjModel, path: Path, q_home) -> dict:
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    tcp = data.site("tcp")
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, tcp.xmat)
    arm_joints = [model.actuator(i).name for i in range(6)]
    arm_joint_names = [model.joint(model.actuator_trnid[i, 0]).name for i in range(6)]
    grip_act = model.actuator(6).name
    drv = model.joint(GRIPPER_PREFIX + "right_driver_joint")
    names = {
        "arm_joints": arm_joint_names,
        "arm_actuators": arm_joints,
        "gripper_actuator": grip_act,
        "gripper_joint": drv.name,
        "ros_gripper_joint": "finger_joint",  # name used in /joint_states (Robotiq URDF)
        "gripper_joint_range": [float(v) for v in model.jnt_range[drv.id]],
        "gripper_ctrl_range": [float(v) for v in model.actuator_ctrlrange[6]],
        "gripper_cmd_to_ctrl": "ctrl = (1 - cmd) * ctrl_max   # cmd 1 = open, 0 = closed",
        "tcp_site": "tcp",
        "cameras": ["front", "wrist"],
        "cube_body": "cube",
        "cube_joint": "cube_joint",
        "target_body": "target",
        "base_frame": "base_link (= MuJoCo world)",
        "home_keyframe": "home",
        "q_home": [float(q) for q in q_home],
        "tcp_home_pos": [round(float(v), 6) for v in tcp.xpos],
        "tcp_home_quat_wxyz": [round(float(v), 6) for v in quat],
    }
    with open(path, "w") as f:
        f.write("# Generated by build_scene.py from the Menagerie assets. Never hard-code these.\n")
        yaml.safe_dump(names, f, sort_keys=False)
    return names


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task-config", type=Path, default=REPO_ROOT / "config" / "task.yaml")
    parser.add_argument("--ik-config", type=Path, default=REPO_ROOT / "config" / "ik.yaml")
    parser.add_argument("--out", type=Path, default=PKG_DIR / "scene.xml")
    args = parser.parse_args()

    task = load_task_config(args.task_config)
    with open(args.ik_config) as f:
        q_home = yaml.safe_load(f)["q_home"]
    out_dir = args.out.resolve().parent
    spec = build_spec(task, out_dir)
    add_home_keyframe(spec, q_home, task)
    spec.compile()
    xml = spec.to_xml()
    args.out.write_text(xml)
    model = mujoco.MjModel.from_xml_path(str(args.out))  # round-trip check
    names = write_model_names(model, out_dir / "model_names.yaml", q_home)
    print(f"wrote {args.out} (nq={model.nq}, nu={model.nu}); tcp home {names['tcp_home_pos']}")


if __name__ == "__main__":
    main()
