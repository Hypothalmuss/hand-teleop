#include "diff_ik_controller/diff_ik.hpp"

#include <algorithm>
#include <cmath>

namespace diff_ik {

namespace {
Eigen::Vector3d clamp_norm(const Eigen::Vector3d& v, double max_norm) {
  const double n = v.norm();
  return n > max_norm ? Eigen::Vector3d(v * (max_norm / n)) : v;
}
}  // namespace

Eigen::Vector3d rotation_error(const Eigen::Matrix3d& r_target, const Eigen::Matrix3d& r_current) {
  Eigen::Quaterniond q(r_target * r_current.transpose());
  q.normalize();
  if (q.w() < 0) q.coeffs() *= -1.0;  // shortest rotation
  const Eigen::Vector3d v = q.vec();
  const double s = v.norm();
  if (s < 1e-12) return 2.0 * v;
  return v * (2.0 * std::atan2(s, q.w()) / s);
}

Vector6d DiffIk::solve(const Matrix6d& jac, const Vector6d& twist, const Vector6d& q,
                       double lambda2) {
  // Task: damped least squares.
  const Matrix6d damped = jac * jac.transpose() + lambda2 * Matrix6d::Identity();
  const Vector6d dq_task = jac.transpose() * damped.ldlt().solve(twist);
  // Posture: projected with the (undamped, truncated) pseudo-inverse, so it is exactly zero
  // while J is well conditioned and only acts along near-singular directions.
  Eigen::JacobiSVD<Matrix6d> svd(jac, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Vector6d s = svd.singularValues();
  Vector6d s_inv = Vector6d::Zero();
  for (int i = 0; i < 6; ++i) {
    if (s[i] > p_.sigma_thresh) s_inv[i] = 1.0 / s[i];
  }
  const Matrix6d j_pinv = svd.matrixV() * s_inv.asDiagonal() * svd.matrixU().transpose();
  const Matrix6d null = Matrix6d::Identity() - j_pinv * jac;
  return dq_task + null * (p_.k_null * (p_.q_home - q));
}

StepResult DiffIk::step(const Target& target, const Vector6d& q, double dt) {
  StepResult res;
  const TcpPose tcp = kin_.fk(q);
  const Matrix6d jac = kin_.jacobian(q);

  // Cartesian servo (+ optional target-velocity feedforward).
  Eigen::Vector3d v =
      clamp_norm(target.velocity_ff + p_.kp_pos * (target.position - tcp.position), p_.v_max);
  const Eigen::Vector3d w =
      clamp_norm(p_.kp_rot * rotation_error(target.orientation.toRotationMatrix(), tcp.rotation),
                 p_.w_max);

  // Singularity-aware damping.
  Eigen::JacobiSVD<Matrix6d> svd(jac);
  res.sigma_min = svd.singularValues().minCoeff();
  double lambda2 = p_.lambda0 * p_.lambda0;
  if (res.sigma_min < p_.sigma_thresh) {
    const double f = 1.0 - res.sigma_min / p_.sigma_thresh;
    lambda2 += f * f * p_.lambda_max * p_.lambda_max;
    res.damping_active = true;
  }
  res.lambda = std::sqrt(lambda2);

  Vector6d twist;
  twist << v, w;
  Vector6d dq = solve(jac, twist, q, lambda2);

  // Workspace guard: if this step would leave the grown box, drop the linear velocity.
  const Eigen::Vector3d p_next = tcp.position + jac.topRows<3>() * dq * dt;
  const Eigen::Vector3d lo = p_.box_lo.array() - p_.guard_margin;
  const Eigen::Vector3d hi = p_.box_hi.array() + p_.guard_margin;
  auto outside_by = [&](const Eigen::Vector3d& p) {
    return ((lo - p).cwiseMax(0.0) + (p - hi).cwiseMax(0.0)).norm();
  };
  const double out_next = outside_by(p_next);
  if (out_next > 0.0 && out_next > outside_by(tcp.position)) {  // leaving, not returning
    v.setZero();
    twist << v, w;
    dq = solve(jac, twist, q, lambda2);
    res.guard_active = true;
  }

  // Joint speed limit: uniform scaling keeps the Cartesian direction.
  const double peak = dq.cwiseAbs().maxCoeff();
  if (peak > p_.qd_max) {
    dq *= p_.qd_max / peak;
    res.velocity_scaled = true;
  }
  // Never push into a joint limit within the margin.
  const Vector6d& lower = kin_.lower();
  const Vector6d& upper = kin_.upper();
  for (int i = 0; i < 6; ++i) {
    if ((q[i] >= upper[i] - p_.limit_margin && dq[i] > 0.0) ||
        (q[i] <= lower[i] + p_.limit_margin && dq[i] < 0.0)) {
      dq[i] = 0.0;
      res.limit_clamped = true;
    }
  }
  res.dq = dq;
  res.q_cmd = (q + dq * dt).cwiseMax(lower).cwiseMin(upper);
  return res;
}

}  // namespace diff_ik
