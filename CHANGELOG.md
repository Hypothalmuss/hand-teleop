# Changelog

## Phase 2: Simulation ROS 2 node
- `mujoco_sim` node: 500 Hz physics, 100 Hz control, threaded 30 Hz rendering, `/sim/clock`, reset service, tf; `scripted_trajectory` (sine / pick-and-place); `sim.launch.py` + RViz config.
- `JointServo` (limit and step clamp plus velocity feedforward) shared by all command paths; config locator shared by every node.
- Results: 99.8 Hz joint states, 29.8 Hz images, RTF 1.00, sine RMS 0.0015 rad, ROS pick-and-place 10/10 (`results/phase2_*.txt`).

## Phase 1: MuJoCo scene
- MjSpec-composed `scene.xml` (UR5e + 2F-85 + table + cube + target, `front`/`wrist` cameras, `home` keyframe) and a generated `model_names.yaml`.
- Python kinematics oracle (FK, Jacobian, DLS IK), shared reset/command logic, scripted grasp and pick-and-place.
- Results: scripted grasp-lift 200/200 (max hold slip 0.19 mm), 10 pytest tests green, `results/phase1_grasp.mp4`.

## Phase 0: Repo and environment
- Repo layout, MIT license, third-party licenses, pre-commit (ruff/black/clang-format), CI on ubuntu-22.04/Humble.
- `config/*.yaml` created as the single source of truth; `hand_teleop_msgs` messages and services build.
- ROS 2 Humble on Ubuntu 22.04 recorded; Docker and `lerobot` deferred (see README).
