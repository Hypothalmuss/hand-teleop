# Tuning notes (one line per change)

- Phase 1: home keyframe shoulder_pan changed from 0 to π. In the Menagerie UR5e (base body rotated π about z, like the UR `base_link`), pan = 0 puts the TCP at (−0.49, −0.13, 0.33), behind the base and away from the table; pan = π gives the same pose mirrored to (0.49, 0.13, 0.33), pointing down over the table.
- Phase 1: TCP site at 0.1439 m above the gripper base body (0.1547 m from the flange): measured midpoint of the closed finger pads (plan start value 0.13 m).
- Phase 1: arm bodies use gravcomp with actuatorgravcomp so the kp = 2000 position servos hold home within 1e-3 rad (the real UR controller compensates gravity too).
- Phase 1: cube solref [0.01, 1], solimp [0.95, 0.99, 0.001, 0.5, 2] with the stock Menagerie pad friction gave 200/200 grasp-lifts, 0.19 mm max hold slip and 0.8 mm max penetration. No further tuning needed.
- Phase 1: wrist camera sits 7 cm off the finger plane on the gripper base, aimed at a point 20 cm along +z; it sees both fingertips and the workspace below.
- Phase 2: added velocity feedforward to the sim's joint servo (`JointServo`, shared with the future headless env). The Menagerie servos (tau = kp(ctrl − q) − kv·qdot) lag by kv/kp = 0.2 s × joint speed, which gave 0.052 rad RMS on the sine sweep and would trip the 0.1 rad IK tracking-fault check at 0.5 rad/s. With ctrl = q_cmd + (kv/kp)·v_ref, where v_ref comes from the command stream, the sweep tracks at 0.0015 rad RMS.
- Phase 2: shadow map 4096 → 2048, and image data passed as `array.array('B')` (rclpy checks every byte of a `bytes` value on uint8[] fields). The front camera went from 26 Hz to 30.0 Hz with 100 Hz commands flowing.
