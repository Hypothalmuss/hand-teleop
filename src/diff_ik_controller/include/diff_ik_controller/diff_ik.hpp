// Damped least-squares differential IK with nullspace posture task (no ROS).
#pragma once

#include <Eigen/Dense>

#include "diff_ik_controller/kinematics.hpp"

namespace diff_ik {

struct DiffIkParams {
  double kp_pos = 5.0;        // 1/s
  double v_max = 0.5;         // m/s
  double kp_rot = 5.0;        // 1/s
  double w_max = 2.0;         // rad/s
  double lambda0 = 0.05;
  double sigma_thresh = 0.05;
  double lambda_max = 0.3;
  double k_null = 1.0;
  double qd_max = 1.0;        // rad/s
  double limit_margin = 0.02; // rad
  double guard_margin = 0.02; // m, workspace box growth for the guard
  Vector6d q_home = Vector6d::Zero();
  Eigen::Vector3d box_lo = Eigen::Vector3d::Constant(-1e9);
  Eigen::Vector3d box_hi = Eigen::Vector3d::Constant(1e9);
};

struct Target {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d velocity_ff = Eigen::Vector3d::Zero();  // optional feedforward (m/s)
};

struct StepResult {
  Vector6d q_cmd;
  Vector6d dq;
  double sigma_min = 0.0;
  double lambda = 0.0;
  bool velocity_scaled = false;
  bool limit_clamped = false;
  bool damping_active = false;
  bool guard_active = false;
};

// Rotation vector of R_target * R_current^T.
Eigen::Vector3d rotation_error(const Eigen::Matrix3d& r_target, const Eigen::Matrix3d& r_current);

class DiffIk {
 public:
  DiffIk(Kinematics& kin, const DiffIkParams& params) : kin_(kin), p_(params) {}

  // One control step: linearize at q (the integrated command), return q + dq * dt.
  StepResult step(const Target& target, const Vector6d& q, double dt);

  const DiffIkParams& params() const { return p_; }

 private:
  Vector6d solve(const Matrix6d& jac, const Vector6d& twist, const Vector6d& q, double lambda2);

  Kinematics& kin_;
  DiffIkParams p_;
};

}  // namespace diff_ik
