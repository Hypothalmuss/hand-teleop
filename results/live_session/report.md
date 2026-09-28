# Live teleop session: 2026-09-28

One operator, live webcam (1920x1080 MJPG, 30 fps), guided protocol (`scripts/live_session.py`)
plus 20 seeded pick-and-place attempts. Analysis: `scripts/analyze_live_session.py` →
`metrics.json`, `attempts.csv`, `latency.csv` and the plots in this folder. Raw data (412k
records) and the full session video stay in `recordings/live_session/` (not committed).

Setup: ROS 2 Humble, `ROS_DOMAIN_ID=77`, `ROS_LOCALHOST_ONLY=1`. A first attempt on the default
domain 0 was aborted and discarded: another robot system on the network (a Gazebo quadruped)
publishes `/joint_states` on that domain, and our IK read its joints (repeated false reset
detections and a tracking fault). Config: One Euro landmarks min_cutoff 0.5, palm_scale 0.2,
beta 0.007; target speed limit 0.3 m/s; IK feedforward alpha 0.4. Calibration was run right
before the session.

## Headline results vs targets

| Metric | Result | Target | |
| --- | --- | --- | --- |
| Teleop pick-and-place success | **20 / 20** | ≥ 17 / 20 | met |
| Median completion time (from reset) | **25.9 s** (last 10 attempts: 23.0 s) | < 25 s | missed overall, met after practice |
| End-to-end latency capture→command p50 / p95 / p99 | **62 / 76 / 93 ms** | < 120 / < 200 ms | met |
| Tracking error RMS (engaged, target ≤ 0.2 m/s) | **5.1 mm** | < 15 mm | met |
| Post-e-stop hold drift (IK-reported) | **1.2e-4 rad**; command change during stop 2.7e-14 rad | < 1e-3 rad | met |
| Clutch re-engage jump (max of 8 engage edges) | **0.0005 mm** | no jump | met |
| Hand tracking | 30.2 fps median, right hand present 99.7 % | 30 Hz | met |
| Tracker processing, frame read→publish (95 × 10 s windows) | p50 20 ms; p95 23 ms typical, 3 windows > 40 ms (max 53) | p95 < 40 ms | met in 92/95 windows |
| Sim real-time factor over the session | 1.0001 | ≥ 0.95 | met |

Latency includes the camera pipeline (frame age at read ≈ 41 ms: exposure, USB, MJPG decode,
queue). The tracker stage (capture→hand) is 59 ms p50; mapper and IK stages are below the
10 ms sim-clock resolution.

## Stability and accuracy

- **Still hold, high-frequency jitter** (residual after a 0.3 s moving average): target
  0.15 / 0.55 / 0.48 mm (x depth / y / z); TCP 0.21 / 0.66 / 0.46 mm. That's sub-millimetre on every axis.
  The plain standard deviation during "still" (56 mm on y) is the operator's hand drifting
  (landmark 9 moved 405 px) and says nothing about sensor noise.
- **Filtering effect** (raw vs filtered, still hold): palm-scale noise 0.42 % → 0.10 % (4×),
  landmark-9 noise 1.28 → 0.94 px (x) and 2.41 → 2.11 px (y). The depth proxy benefits most, which
  was the goal of the heavier palm-scale filter.
- **Sweeps:** the TCP follows the target with 35–45 ms lag and 10–11 mm RMS tracking error at the
  sweep speeds (target speed p95 ≈ 0.36–0.41 m/s as estimated from 10 ms-quantized stamps).
  The TCP **overshoots ~1 cm whenever the target stops abruptly** (every plateau in
  `sweeps.png`). This comes from the Cartesian velocity feedforward, and it is the main
  remaining "wobble".
- **Cross-axis coupling:** moving towards/away from the camera also moved the target
  sideways (y std 93 mm during the depth sweep). Part is the hand drifting, part is perspective:
  image x of an off-centre hand changes with distance, and the mapping does not correct it.
- **Workspace saturation:** all three sweeps hit the workspace-box limits (flat tops), so the
  operator's comfortable hand motion covers more than the box at gain 1.0.

## Pick and place

- 20/20 successes, times 13.8–54.3 s (p25/p50/p75 = 21.9 / 25.9 / 31.9 s).
- **Operators improve quickly:** median 30.9 s for attempts 1–10 vs 23.0 s for 11–20
  (corr(attempt #, time) = −0.54).
- **Grasping is the bottleneck:** 81 gripper closes over 20 attempts, of which 11 were
  sub-0.3 s flicker, so there were about 2.5 real re-grasps per attempt (median close held 0.9 s). Most time
  goes into lining up the grasp.
- **Cube yaw did not matter** (the gripper orientation is fixed, cube yaw random): 0–15°
  misalignment had 4.4 closes on average, 15–30° had 3.4, and 30–45° had 3.3. Correlation of misalignment with
  time is 0.07. An earlier concern that fixed gripper yaw would hurt grasps is not supported (n = 20, one operator).
- Gripper: full command range 0.02–1.0; aperture lags the command by ~90 ms (2/s rate limit
  plus the finger dynamics).

## E-stop (triggered by the script during motion)

The command froze exactly (2.7e-14 rad change), 149 targets that arrived during the stop were
ignored, and the clear succeeded. The IK reported 1.2e-4 rad braking and 1.2e-4 rad hold drift. Joint travel
over the 2 s after the stop instant was 1.9e-3 rad, which includes the arm decelerating from
teleop speed (see `docs/engineering_notes.md` on how drift is defined).

## Conclusions

1. The teleop chain meets every target except median time (25.9 s vs 25 s), and it met that one
   once the operator had practised (23.0 s over the last 10).
2. Sensor noise is no longer the limiting factor: still jitter is sub-millimetre after filtering.
   What remains is **stopping overshoot (~1 cm)** from the IK feedforward and **grasp alignment
   effort** (~2.5 re-grasps per attempt).
3. The shared ROS domain was a real failure mode. Earlier sessions on domain 0 may have been
   affected by the other robot's `/joint_states`. The launch files now warn about it.
4. Limitations: one operator, one session, n = 20; no repeat sessions yet, so there are no
   between-session confidence intervals.

## Recommended next changes

- Reduce stopping overshoot: disable the velocity feedforward when the target stops (or taper
  it), or lower alpha, then re-run the square test.
- Precision mode for grasping: a lower gain (e.g. 0.5) near the table or a key to toggle it.
  The re-grasp count is the metric to watch.
- Perspective-correct the lateral mapping (scale u, v by palm scale) to reduce depth/lateral coupling.
- Increase the workspace box slightly, or lower the gain, since the operator saturated all axes.
- Add a gripper-command hysteresis to remove the 11 flicker closes.
