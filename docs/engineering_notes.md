# Engineering notes

Design decisions that aren't obvious from the code, each with the measurement behind it.
Read these before changing a default.

## Scene (MuJoCo)

- **Home pose `[π, −π/2, π/2, −π/2, −π/2, 0]`.** The Menagerie UR5e base is rotated π about z (as in
  the UR `base_link`), so `pan = 0` would put the TCP behind the robot at (−0.49, −0.13, 0.33).
  `pan = π` gives the same pose over the table.
- **TCP site** sits 0.1439 m above the gripper base body (0.1547 m from the flange): the measured
  midpoint of the closed finger pads.
- **Gravity compensation** (`gravcomp` + `actuatorgravcomp`) models what the real UR controller does,
  and lets the kp = 2000 position servos hold home within 1e-3 rad.
- **Contacts:** cube solref [0.01, 1] and solimp [0.95, 0.99, 0.001, 0.5, 2] with the stock pad
  friction gave 200/200 scripted grasp-lifts, 0.19 mm max hold slip and 0.8 mm max penetration.
- **Wrist camera** sits 7 cm off the finger plane, aimed at a point 20 cm along the gripper axis,
  so both fingertips and the workspace are in view.

## Simulator node

- **Joint servo with velocity feedforward** (`JointServo`). The Menagerie servos
  (τ = kp(ctrl − q) − kv·q̇) lag by kv/kp = 0.2 s × joint speed. A sine sweep tracked at
  0.052 rad RMS, and teleop speeds would trip the IK tracking-fault check.
  `ctrl = q_cmd + (kv/kp)·v_ref`, with v_ref derived from the command stream, gives 0.0015 rad RMS.
  This mimics a UR `servoj`. A real-robot driver needs its own equivalent.
- **Rendering at 30 Hz with commands flowing:** the shadow map is 2048, and images are published
  as `array.array('B')`, because rclpy checks every byte of a `bytes` value on uint8[] fields.
  This took the camera rate from 26 to 30 Hz.
- **Reset detection:** a `/sim/reset` moves the arm instantly. The IK resyncs when measured
  joints jump more than `reset_jump_rad` between samples, and the mapper drops the clutch for one
  update when the TCP jumps more than `reset_jump_m`. Otherwise the held command drives the arm
  straight back to the pre-reset pose.

## Hand tracking

- **Camera:** `width: max` picks the widest mode that measurably holds 30 fps (1920×1080 MJPG on
  the development webcam). `buffer_size: 2` is required: with a single V4L2 buffer the driver
  drops every other frame while one is being decoded, so a 30 fps camera delivers 15 fps.
- **Latency accounting:** `capture_stamp` is backdated by the V4L2 buffer age (≈ 41 ms: exposure,
  USB, decode, queue), so end-to-end latency includes the camera.
- **One Euro filters in pixel units** (`value_scale` = 640 px reference width). Standard One Euro
  parameters assume pixels; on normalized coordinates, beta does almost nothing. Defaults:
  landmarks min_cutoff 0.5, palm scale 0.2 (the depth proxy is the noisiest signal), beta 0.007.
- **Open/fist from finger straightness** (chord/arc from MCP to tip; open > 0.85, fist < 0.6),
  debounced over 3 frames. The more common ratio |tip − mcp| / palm length measures only 0.5–0.9 on
  open hands (a pinky is ~0.7 palm lengths). It is still selectable as `feature: extension`.
- **Aspect-corrected distances** (y scaled by H/W): MediaPipe normalizes x and y by different lengths.
- **Offline `--video` mode** derives time from a frame counter. `CAP_PROP_POS_MSEC` misbehaves
  after a loop seek.

## Mapping

- **Depth sign:** d = 1/palm_scale grows as the hand moves away. `sign.x = −1` means "hand closer to the
  webcam = TCP farther from the base (towards the viewer)".
- **Relative motion** uses the unclamped mapping, and only the final target is clamped to the box, so
  re-anchoring near the edge of the hand range still moves freely.
- **Speed limit** 0.3 m/s on the target, and a 2/s rate limit on the gripper.

## Differential IK (C++)

- **Nullspace projector** uses the undamped SVD pseudo-inverse truncated at σ_thresh. With the damped
  J#, the posture term leaked into the task and left 2–3 mm steady-state error near the base
  (37/50 random targets reached vs 50/50).
- **Linearizes at q_cmd,** not at the 100 Hz joint states (which are 10 ms stale at 200 Hz). Measured
  joints drive resync, the tracking-fault check and the reported tracking error.
- **Target-velocity feedforward** (EMA alpha 0.4 of the finite-differenced 30 Hz targets). Pure P
  control at Kp = 5/s lags 20 mm at 0.1 m/s. Alpha sweep on a 10 cm square at 0.1 m/s:
  0.3 → 5.04 mm, 0.4 → 4.33 mm, 0.5 → 3.82 mm RMS. It costs ~1 cm of overshoot when the target
  stops abruptly (see `results/live_session/sweeps.png`).
- **Singularity test** checks the 6-D task error ‖[e_p; e_r]‖, which DLS makes non-increasing.
  Position alone can rise briefly when the commanded rotation is about the degenerate axis.
- **E-stop:** freezes at the measured joints, extrapolated over the joint-state age (≤ 10 ms), and
  drops `engaged`, so after a clear the arm moves only on a fresh engage edge. Drift is measured from
  the rest pose after braking. Stopping from teleop speed physically takes 0.2–2 mrad of travel with
  these servo gains, and this is reported separately as braking.

## Networking

- **Use an isolated ROS domain.** On a shared network the default domain 0 can carry another
  robot's `/joint_states`, `/tf` and `/estop`. In a live session this made the IK read foreign
  joints (false reset detections, a tracking fault). Run with
  `ROS_DOMAIN_ID=<unused> ROS_LOCALHOST_ONLY=1`; the launch files warn on domain 0.
