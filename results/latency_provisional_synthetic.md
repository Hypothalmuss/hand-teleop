PROVISIONAL: offline input (hand_tracker --video on a 640x480 clip synthesized from a stock
two-hand photo), not live webcam teleop. Replace with scripts/measure_latency.py during live teleop.
Camera exposure/USB latency is not included in either case (capture_stamp = frame read).

# Teleop latency (PROVISIONAL synthetic video)

628 samples over 20 s, 2026-09-25. Stamps are sim time (10 ms resolution).

| Hop | p50 [ms] | p95 [ms] | p99 [ms] |
| --- | --- | --- | --- |
| capture_to_hand | 30.0 | 40.0 | 40.0 |
| hand_to_target | 0.0 | 10.0 | 10.0 |
| target_to_command | 0.0 | 10.0 | 10.0 |
| capture_to_command | 30.0 | 40.0 | 40.0 |

Tracking error RMS while engaged at target speed <= 0.2 m/s: 1.6 mm (619 samples).
