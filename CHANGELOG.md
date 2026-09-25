# Changelog

## Post-phase-6: live webcam fixes
- Camera window with the hand overlay during teleop (`window:=false` to disable); camera at the widest mode that holds 30 fps (1920x1080 here); fixed 15 fps caused by a single V4L2 buffer.
- Steadier motion: heavier stationary smoothing, 0.3 m/s target speed limit, smoother IK feedforward; capture stamps include the camera pipeline (~41 ms).
- Operator scripts use the same camera mode as live teleop; all 62 tests green.

## Phase 6: Integration (software complete; operator runs pending)
- `teleop.launch.py` (sim, hand input webcam/video/fake, mapper, IK, optional console window and RViz), `keyboard_console`, `fake_hand`, sim `demo_view`, shared `SuccessDetector` and seed lists.
- Scripts: `measure_latency.py`, `teleop_benchmark.py`, `record_demo_video.py/.sh`; full-chain integration launch test.
- Results: 62 tests green (5 clip tests skipped); synthetic-input chain: startup 0.8 s, capture→command p50 30 ms, tracking 1.2–1.6 mm. Live latency, benchmark and demo video wait for webcam sessions.

## Phase 5: Differential IK controller (C++)
- `diff_ik_controller`: `Kinematics` (MuJoCo), `DiffIk` (DLS + truncated-pinv nullspace posture, singularity damping, joint-speed/limit handling, workspace guard, target-velocity feedforward), `EstopLatch`, 200 Hz node with latency samples and diagnostics.
- gtests (Jacobian vs FD, 50/50 targets, wrist singularity, guard, latch) and launch tests (square, e-stop); `ik_sim.launch.py`, `scripts/square_tracking.py`.
- Results: square RMS 3.8 mm at 0.1 m/s, LatencySample 30 Hz, hold drift 1.9e-4 rad (`results/phase5_*`).

## Phase 4: Teleop mapping node
- `teleop_mapper` node: open-palm/fist clutch with relative re-anchoring, 0.3 s absent hold, e-stop, 0.5 m/s target speed limit, pinch→gripper with a 2/s rate limit, RViz marker.
- Pure `mapping.py` plus `scripts/calibrate_workspace.py` (9 poses, about 30 s).
- Results: 10 unit tests green; smoke test sim + tracker (video) + mapper publishes `/teleop/target` at 30 Hz.

## Phase 3: Hand tracking node
- `hand_tracker` node: MediaPipe HandLandmarker (VIDEO mode, bundled model), One Euro filters, straightness-based open/fist with 3-frame debounce, presence logic, debug overlay (fps, latency, wall and sim clocks), `--video` offline mode.
- Pure `one_euro.py`, `gestures.py`, `tracker.py`, `clip_eval.py`; setup and clip-recording scripts for the operator.
- Results: 16 unit tests green, clip tests waiting for recordings; provisional frame→publish p95 30 ms (`results/phase3_latency.txt`).

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
