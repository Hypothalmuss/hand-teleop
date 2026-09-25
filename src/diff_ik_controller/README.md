# diff_ik_controller

C++17 differential IK for the UR5e TCP (Eigen + MuJoCo `mj_jacSite` on the simulator's own
`scene.xml`; no MoveIt). Runs at 200 Hz; the math (`Kinematics`, `DiffIk`, `EstopLatch`) has no ROS.

**Control law**

1. `v = clamp(v_ff + Kp_pos (p_target − p_tcp), v_max)`, `w = clamp(Kp_rot · rotvec(R_target R_tcpᵀ), w_max)`; `v_ff` is the finite-difference target velocity.
2. `dq = Jᵀ (J Jᵀ + λ² I)⁻¹ [v; w] + (I − J⁺J) k_null (q_home − q)`, with `J⁺` the SVD pseudo-inverse truncated at `σ_thresh`.
3. Singularity damping: `λ² = λ₀² + (1 − σ_min/σ_thresh)² λ_max²` when `σ_min < σ_thresh`, else `λ₀²`.
4. Limits: scale `dq` uniformly to `|dq_i| ≤ qd_max`; zero any component that pushes into a joint limit within `limit_margin`; drop `v` if the next TCP leaves the workspace box grown by `workspace_guard_margin`.
5. `q_cmd += dq·dt` on the command, resynced to the measured joints at start, on each engage edge and after an e-stop clear; `|q_cmd − q_meas| > tracking_fault_rad` freezes until re-engage.

Topics: `teleop/target`, `joint_states`, `hand/state` (stamps), `estop` (latched) →
`arm/joint_command`, `metrics/latency`, `/diagnostics`; service `estop/clear`. Parameters come from
`config/ik.yaml`, `config/workspace.yaml` and `model_names.yaml` via `diff_ik_controller.node_parameters()`.

MuJoCo is linked from an installed CMake package, `MUJOCO_ROOT` / `third_party/mujoco`, or the
`mujoco` pip wheel (same version as the simulator).
