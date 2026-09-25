# Changelog

## Phase 1: MuJoCo scene
- MjSpec-composed `scene.xml` (UR5e + 2F-85 + table + cube + target, `front`/`wrist` cameras, `home` keyframe) and a generated `model_names.yaml`.
- Python kinematics oracle (FK, Jacobian, DLS IK), shared reset/command logic, scripted grasp and pick-and-place.
- Results: scripted grasp-lift 200/200 (max hold slip 0.19 mm), 10 pytest tests green, `results/phase1_grasp.mp4`.

## Phase 0: Repo and environment
- Repo layout, MIT license, third-party licenses, pre-commit (ruff/black/clang-format), CI on ubuntu-22.04/Humble.
- `config/*.yaml` created as the single source of truth; `hand_teleop_msgs` messages and services build.
- ROS 2 Humble on Ubuntu 22.04 recorded; Docker and `lerobot` deferred (see README).
