#include "diff_ik_controller/kinematics.hpp"

#include <stdexcept>

namespace diff_ik {

Kinematics::Kinematics(const std::string& scene_xml, const std::vector<std::string>& arm_joints,
                       const std::string& tcp_site) {
  if (arm_joints.size() != 6) throw std::invalid_argument("expected 6 arm joints");
  char err[1000] = "";
  model_ = mj_loadXML(scene_xml.c_str(), nullptr, err, sizeof(err));
  if (!model_) throw std::runtime_error("mj_loadXML(" + scene_xml + "): " + err);
  data_ = mj_makeData(model_);
  site_id_ = mj_name2id(model_, mjOBJ_SITE, tcp_site.c_str());
  if (site_id_ < 0) throw std::runtime_error("site not found: " + tcp_site);
  for (size_t i = 0; i < arm_joints.size(); ++i) {
    const int j = mj_name2id(model_, mjOBJ_JOINT, arm_joints[i].c_str());
    if (j < 0) throw std::runtime_error("joint not found: " + arm_joints[i]);
    qpos_idx_.push_back(model_->jnt_qposadr[j]);
    dof_idx_.push_back(model_->jnt_dofadr[j]);
    lower_[i] = model_->jnt_range[2 * j];
    upper_[i] = model_->jnt_range[2 * j + 1];
  }
  jacp_.assign(3 * model_->nv, 0.0);
  jacr_.assign(3 * model_->nv, 0.0);
  mj_resetDataKeyframe(model_, data_, 0);
}

Kinematics::~Kinematics() {
  if (data_) mj_deleteData(data_);
  if (model_) mj_deleteModel(model_);
}

void Kinematics::set(const Vector6d& q) {
  for (int i = 0; i < 6; ++i) data_->qpos[qpos_idx_[i]] = q[i];
  mj_kinematics(model_, data_);
  mj_comPos(model_, data_);
}

TcpPose Kinematics::fk(const Vector6d& q) {
  set(q);
  TcpPose pose;
  const mjtNum* p = data_->site_xpos + 3 * site_id_;
  const mjtNum* r = data_->site_xmat + 9 * site_id_;
  pose.position = Eigen::Vector3d(p[0], p[1], p[2]);
  pose.rotation = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(r);
  return pose;
}

Matrix6d Kinematics::jacobian(const Vector6d& q) {
  set(q);
  mj_jacSite(model_, data_, jacp_.data(), jacr_.data(), site_id_);
  Matrix6d jac;
  for (int c = 0; c < 6; ++c) {
    for (int r = 0; r < 3; ++r) {
      jac(r, c) = jacp_[r * model_->nv + dof_idx_[c]];
      jac(r + 3, c) = jacr_[r * model_->nv + dof_idx_[c]];
    }
  }
  return jac;
}

}  // namespace diff_ik
