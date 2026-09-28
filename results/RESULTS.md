# Results

Webcam hand teleoperation of a simulated UR5e + Robotiq 2F-85. Development machine: AMD Ryzen 5
5600, NVIDIA RTX 3050, 1080p USB webcam (MJPG, 30 fps), ROS 2 Humble.

## Live teleop (one operator, one 14-minute session, 2026-09-28)

| Metric | Result | Target |
| --- | --- | --- |
| Pick-and-place success (20 seeded attempts) | **20 / 20** | ≥ 17 / 20 |
| Median completion time | **25.9 s** (attempts 11–20: 23.0 s) | < 25 s |
| Latency, camera capture → joint command (p50 / p95 / p99) | **62 / 76 / 93 ms** | < 120 / < 200 ms |
| Of which camera pipeline (frame age at read) | 41 ms | |
| Hand tracker processing, frame → published state (p50 / typical p95) | 20 / 23 ms | p95 < 40 ms |
| Hand tracking rate / right hand detected | 30.2 fps / 99.7 % | 30 fps |
| Tracking error RMS (target ≤ 0.2 m/s) | **5.1 mm** | < 15 mm |
| Jitter with the hand held still (x / y / z, high-frequency) | **0.15 / 0.55 / 0.48 mm** | |
| TCP lag behind target in sweeps | 35–45 ms | |
| Clutch re-engage jump (8 events) | 0.0005 mm | no jump |
| E-stop: command change / hold drift | 2.7e-14 rad / 1.2e-4 rad | 0 / < 1e-3 rad |
| Re-grasps per attempt | 2.5 | |
| Sim real-time factor | 1.00 | ≥ 0.95 |

Details: [`live_session/report.md`](live_session/report.md), raw metrics in
[`live_session/metrics.json`](live_session/metrics.json), per-attempt data in
[`teleop_benchmark.csv`](teleop_benchmark.csv).

## Controller and simulator (automated tests)

| Metric | Result | Target |
| --- | --- | --- |
| IK: random targets reached (< 2 mm, < 1°, within 3 s) | **50 / 50** | 50 / 50 |
| IK: 10 cm square at 0.1 m/s, TCP RMS error | **4.2–4.3 mm** (3 runs) | < 5 mm |
| IK: Jacobian vs finite differences (100 configs) | < 1e-5 | < 1e-5 |
| IK: wrist singularity | bounded (≤ 1 rad/s), error non-increasing, no NaN | |
| Sim: joint states / camera rates | 99.8 Hz / 29.8 Hz | 100 ± 5 / 30 ± 3 Hz |
| Sim: joint servo sine tracking | 0.0015 rad RMS | < 0.02 rad |
| Scripted grasp-lift stability | 200 / 200 (max slip 0.19 mm) | ≥ 199 / 200 |
| Scripted pick-and-place through ROS | 10 / 10 | 10 / 10 |
| Full chain startup (launch → first command) | 0.8–0.9 s | < 10 s |

Evidence: `scene_tests.txt`, `sim_tests.txt`, `sim_rtf.txt`, `ik_tests.txt`,
`ik_square_tracking.png`, `integration_tests.txt`.

## Known limitations (measured)

- ~1 cm TCP overshoot when the hand stops abruptly (Cartesian velocity feedforward).
- Depth comes from apparent palm size, so tilting the palm reads as forward/back motion.
- Gripper orientation is fixed (position-only control).
- Single operator, single session: no between-session statistics yet.
