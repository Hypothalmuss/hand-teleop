# Hand Teleop → Imitation Learning (UR5e + Robotiq 2F-85, MuJoCo, ROS 2)

Webcam hand teleoperation of a simulated UR5e + Robotiq 2F-85, used to record pick-and-place
demonstrations and train imitation-learning policies (ACT, Diffusion Policy). Simulation only,
live camera, real human input. The full plan is in [`docs/project_plan.md`](docs/project_plan.md).

> Status: work in progress. Phases 0–2 done (repo, messages, MuJoCo scene, simulator node); see `CHANGELOG.md`.

## Environment (recorded at phase 0)

| Item | Version |
| --- | --- |
| OS | Ubuntu 22.04 |
| ROS 2 | **Humble** (the plan's fallback distro; the dev machine runs 22.04) |
| Python | 3.10 |
| MuJoCo | 3.12.0 (pip wheel; also provides `libmujoco.so` + headers for the C++ IK node) |
| MediaPipe | 1.0.1 (Tasks API) |
| numpy | 1.26.4 (< 2 for Humble's `cv_bridge`) |
| Dev CPU / GPU | AMD Ryzen 5 5600 / NVIDIA RTX 3050 |

`torch` and `lerobot` are pinned when the learning phases start.

## Quick start (native)

```bash
source /opt/ros/humble/setup.bash
python3 -m pip install -r requirements.txt
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
ros2 interface show hand_teleop_msgs/msg/TeleopTarget
```

Rebuild the MuJoCo scene after changing `config/task.yaml` or the assets:

```bash
python3 -m ur5e_2f85_mujoco.build_scene       # writes scene.xml + model_names.yaml (committed)
python3 -m ur5e_2f85_mujoco.trials --n 200    # scripted grasp-lift stability check
```

Run the simulator (MuJoCo viewer + scripted pick-and-place):

```bash
ros2 launch hand_teleop_bringup sim.launch.py viewer:=true scripted:=pick_place episodes:=3
```

RViz (`rviz:=true`) shows the UR5e model when `ros-humble-ur-description` is installed.
Docker is planned after the first working version.

## Layout

```
config/                 every rate, gain, range and threshold (single source of truth)
src/hand_teleop_msgs/   messages and services
src/ur5e_2f85_mujoco/   MJCF assets (Menagerie), scene builder, kinematics oracle, task logic (no ROS)
src/mujoco_sim_ros/     simulator node, scripted trajectories
src/hand_teleop_bringup/ launch files, RViz config; installs config/ for every node
docs/                   project plan
results/                committed evidence (clips, metrics) referenced by this README
```

## Scene notes

- `base_link` is the MuJoCo world frame: x toward the table, y left, z up.
- The TCP site sits on the gripper base +z axis at the midpoint of the closed finger pads
  (0.1547 m from the flange).
- The UR5e joints use `gravcomp` + `actuatorgravcomp`, which models the gravity compensation
  the real UR controller does internally. Without it the P-controlled arm sags about 0.01 rad
  under gravity.
- The home keyframe is `[π, −π/2, π/2, −π/2, −π/2, 0]`. The plan's `pan = 0` puts the TCP at
  x = −0.49, behind the base; see `results/tuning_notes.md`.

## Licenses

Our code is MIT (`LICENSE`). Third-party models keep their licenses: UR5e MJCF (BSD-3) and
Robotiq 2F-85 MJCF (BSD-2) from MuJoCo Menagerie (Apache-2.0 changes). See
`THIRD_PARTY_LICENSES.md`.
