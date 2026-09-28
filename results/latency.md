# Teleop latency (live webcam session 2026-09-28)

Source: `results/live_session/` (22k LatencySample messages over ~14 min of live teleop). Stamps are sim time
(10 ms resolution); capture_stamp is backdated by the V4L2 frame age, so the camera pipeline is included.

| Hop | p50 [ms] | p95 [ms] | p99 [ms] |
| --- | --- | --- | --- |
| capture_to_hand | 59 | 73 | 88 |
| hand_to_target | 0 | 10 | 10 |
| target_to_command | 0 | 10 | 10 |
| capture_to_command | 62 | 76 | 93 |

Tracking error RMS while engaged at target speed <= 0.2 m/s: 5.1 mm.
