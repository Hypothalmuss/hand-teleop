# Changelog

## 1.1.0 (2026-09-28)

- Docker: `Dockerfile`, `docker-compose.yml` (+ NVIDIA and `/dev/dri` overrides) and
  `docker/run.sh` (build, setup-hands, calibrate, teleop, record, console, fake, test, shell).
  Webcam and X11 passthrough; calibration and videos stay on the host. All 63 tests pass in the
  container; tracking and rendering performance match native.
- `console:=xterm` launch option (the console window inherits the environment, for containers).

## 1.0.0 (2026-09-28)

First standalone release: webcam hand teleoperation of a simulated UR5e + Robotiq 2F-85.

- Hand tracking (MediaPipe) with One Euro filtering, straightness-based open/fist, pinch,
  1080p/30 fps camera selection and a live camera window.
- Teleop mapping with an open-palm/fist clutch, relative re-anchoring, speed limits and pinch → gripper.
- C++17 differential IK controller: damped least squares, singularity damping, nullspace posture,
  limits, workspace guard, latching e-stop, reset detection, latency metrics.
- MuJoCo simulator node with a UR5e-servo-like joint command path, cameras, scene reset and sim clock.
- Keyboard console, calibration and handedness wizards, guided live test session and analysis,
  latency measurement, pick-and-place benchmark, demo video recorder.
- Measured live: 20/20 pick-and-place, 62/76 ms p50/p95 capture-to-command latency, 5.1 mm
  tracking RMS, sub-millimetre still jitter (`results/RESULTS.md`).
