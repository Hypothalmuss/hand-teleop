// MuJoCo-based kinematics of the UR5e TCP site (no ROS).
#pragma once

#include <mujoco/mujoco.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

namespace diff_ik {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

struct TcpPose {
  Eigen::Vector3d position;
  Eigen::Matrix3d rotation;
};

// Loads the same scene.xml as the simulator and keeps its own mjModel/mjData.
class Kinematics {
 public:
  Kinematics(const std::string& scene_xml, const std::vector<std::string>& arm_joints,
             const std::string& tcp_site);
  ~Kinematics();
  Kinematics(const Kinematics&) = delete;
  Kinematics& operator=(const Kinematics&) = delete;

  TcpPose fk(const Vector6d& q);
  // 6x6 geometric Jacobian [linear; angular] of the TCP site w.r.t. the arm joints.
  Matrix6d jacobian(const Vector6d& q);

  const Vector6d& lower() const { return lower_; }
  const Vector6d& upper() const { return upper_; }

 private:
  void set(const Vector6d& q);

  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  int site_id_ = -1;
  std::vector<int> qpos_idx_, dof_idx_;
  Vector6d lower_, upper_;
  std::vector<mjtNum> jacp_, jacr_;
};

}  // namespace diff_ik
