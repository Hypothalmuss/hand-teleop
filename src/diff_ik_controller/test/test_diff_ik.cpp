#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "diff_ik_controller/diff_ik.hpp"
#include "test_config.hpp"

using diff_ik::DiffIk;
using diff_ik::Kinematics;
using diff_ik::Matrix6d;
using diff_ik::Target;
using diff_ik::Vector6d;

class DiffIkTest : public ::testing::Test {
 protected:
  void SetUp() override {
    setup_ = test_config::load();
    kin_ = std::make_unique<Kinematics>(setup_.scene, setup_.joints, setup_.site);
  }
  test_config::Setup setup_;
  std::unique_ptr<Kinematics> kin_;
};

TEST_F(DiffIkTest, JacobianMatchesFiniteDifferences) {
  std::mt19937 rng(0);
  std::uniform_real_distribution<double> u(-M_PI, M_PI);
  const double eps = 1e-6;
  for (int n = 0; n < 100; ++n) {
    Vector6d q;
    for (int i = 0; i < 6; ++i) q[i] = u(rng);
    const Matrix6d jac = kin_->jacobian(q);
    Matrix6d num;
    for (int i = 0; i < 6; ++i) {
      Vector6d dq = Vector6d::Zero();
      dq[i] = eps;
      const auto p = kin_->fk(q + dq);
      const auto m = kin_->fk(q - dq);
      num.block<3, 1>(0, i) = (p.position - m.position) / (2 * eps);
      num.block<3, 1>(3, i) = diff_ik::rotation_error(p.rotation, m.rotation) / (2 * eps);
    }
    ASSERT_LT((jac - num).cwiseAbs().maxCoeff(), 1e-5) << "config " << n;
  }
}

TEST_F(DiffIkTest, ReachesRandomTargetsFromHome) {
  DiffIk ik(*kin_, setup_.params);
  const auto& p = setup_.params;
  const Eigen::Quaterniond down(kin_->fk(p.q_home).rotation);
  std::mt19937 rng(1);
  const double dt = 1.0 / 200.0;
  int reached = 0;
  for (int n = 0; n < 50; ++n) {
    Target tgt{Eigen::Vector3d::Zero(), down};
    for (int a = 0; a < 3; ++a) {
      tgt.position[a] = std::uniform_real_distribution<double>(p.box_lo[a], p.box_hi[a])(rng);
    }
    Vector6d q = p.q_home;
    bool ok = false;
    for (int k = 0; k < static_cast<int>(3.0 / dt); ++k) {
      const auto res = ik.step(tgt, q, dt);
      ASSERT_FALSE(res.q_cmd.hasNaN());
      ASSERT_LE(res.dq.cwiseAbs().maxCoeff(), p.qd_max + 1e-9);
      q = res.q_cmd;
      ASSERT_TRUE(((q - kin_->lower()).array() >= 0).all() &&
                  ((kin_->upper() - q).array() >= 0).all());
      const auto tcp = kin_->fk(q);
      const double pos_err = (tcp.position - tgt.position).norm();
      const double rot_err =
          diff_ik::rotation_error(tgt.orientation.toRotationMatrix(), tcp.rotation).norm();
      if (pos_err < 2e-3 && rot_err < M_PI / 180.0) {
        ok = true;
        break;
      }
    }
    EXPECT_TRUE(ok) << "target " << n << " " << tgt.position.transpose();
    reached += ok;
  }
  EXPECT_EQ(reached, 50);
}

TEST_F(DiffIkTest, BoundedNearWristSingularity) {
  DiffIk ik(*kin_, setup_.params);
  // UR wrist singularity: wrist_2 = 0 aligns the wrist_1 and wrist_3 axes.
  Vector6d q = setup_.params.q_home;
  q[4] = 0.0;
  const auto start = kin_->fk(q);
  const Matrix6d jac = kin_->jacobian(q);
  Eigen::JacobiSVD<Matrix6d> svd(jac);
  ASSERT_LT(svd.singularValues().minCoeff(), setup_.params.sigma_thresh);
  // Ask for a rotation about the degenerate direction plus a small translation.
  Target tgt{start.position + Eigen::Vector3d(0.0, 0.03, -0.02),
             Eigen::Quaterniond(Eigen::AngleAxisd(0.3, start.rotation.col(0)) *
                                start.rotation)};
  const double dt = 1.0 / 200.0;
  // 6-D task error [e_p; e_r]: the quantity DLS decreases (position alone need not, when the
  // commanded rotation is about the degenerate axis).
  auto task_err = [&](const Vector6d& qq) {
    const auto tcp = kin_->fk(qq);
    Vector6d e;
    e << tgt.position - tcp.position,
        diff_ik::rotation_error(tgt.orientation.toRotationMatrix(), tcp.rotation);
    return e.norm();
  };
  const double err0 = task_err(q);
  double prev_err = err0;
  bool damped = false;
  for (int k = 0; k < 600; ++k) {
    const auto res = ik.step(tgt, q, dt);
    ASSERT_FALSE(res.q_cmd.hasNaN());
    ASSERT_LE(res.dq.cwiseAbs().maxCoeff(), setup_.params.qd_max + 1e-9);
    damped |= res.damping_active;
    q = res.q_cmd;
    const double err = task_err(q);
    ASSERT_LE(err, prev_err + 1e-6) << "step " << k;
    prev_err = err;
  }
  EXPECT_TRUE(damped);
  EXPECT_LT(prev_err, 0.5 * err0);
}

TEST_F(DiffIkTest, WorkspaceGuardKeepsTcpInGrownBox) {
  auto params = setup_.params;
  DiffIk ik(*kin_, params);
  const Eigen::Quaterniond down(kin_->fk(params.q_home).rotation);
  Target tgt{Eigen::Vector3d(params.box_hi.x() + 0.3, 0.0, params.box_hi.z() + 0.3), down};
  Vector6d q = params.q_home;
  bool guarded = false;
  for (int k = 0; k < 800; ++k) {
    const auto res = ik.step(tgt, q, 1.0 / 200.0);
    guarded |= res.guard_active;
    q = res.q_cmd;
    const auto p = kin_->fk(q).position;
    const double tol = params.guard_margin + 0.005;  // one step of overshoot at most
    ASSERT_TRUE(((p - params.box_hi).array() <= tol).all()) << p.transpose();
  }
  EXPECT_TRUE(guarded);
}

TEST(RotationError, ZeroForIdentityAndMatchesAxisAngle) {
  const Eigen::Matrix3d r = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_LT(diff_ik::rotation_error(r, r).norm(), 1e-12);
  const Eigen::Vector3d e = diff_ik::rotation_error(r, Eigen::Matrix3d::Identity());
  EXPECT_NEAR(e.z(), 0.7, 1e-12);
}
