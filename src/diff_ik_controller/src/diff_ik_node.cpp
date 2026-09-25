// diff_ik_controller node: /teleop/target -> /arm/joint_command at 200 Hz.
//
// q_cmd is integrated on the command (not the measurement) and resynced to the measured
// joints at startup, on every engage edge and after an e-stop clear. A latched e-stop freezes
// q_cmd at the measured joints. Every new target yields one LatencySample.

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <deque>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <utility>
#include <vector>

#include "diff_ik_controller/diff_ik.hpp"
#include "diff_ik_controller/estop_latch.hpp"
#include "diff_ik_controller/kinematics.hpp"
#include "hand_teleop_msgs/msg/hand_state.hpp"
#include "hand_teleop_msgs/msg/joint_command.hpp"
#include "hand_teleop_msgs/msg/latency_sample.hpp"
#include "hand_teleop_msgs/msg/teleop_target.hpp"

using diff_ik::Vector6d;
using namespace std::chrono_literals;

namespace {

Vector6d to_vec6(const std::vector<double>& v, const std::string& name) {
  if (v.size() != 6) throw std::invalid_argument(name + " must have 6 elements");
  return Eigen::Map<const Vector6d>(v.data());
}

Eigen::Vector3d to_vec3(const std::vector<double>& v, const std::string& name) {
  if (v.size() != 3) throw std::invalid_argument(name + " must have 3 elements");
  return Eigen::Vector3d(v[0], v[1], v[2]);
}

int64_t stamp_ns(const builtin_interfaces::msg::Time& t) {
  return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
}

}  // namespace

class DiffIkNode : public rclcpp::Node {
 public:
  DiffIkNode() : Node("diff_ik_controller") {
    std::string scene = declare_parameter<std::string>("scene_path", "");
    if (scene.empty()) {
      scene = ament_index_cpp::get_package_share_directory("ur5e_2f85_mujoco") +
              "/ur5e_2f85_mujoco/scene.xml";
    }
    joint_names_ = declare_parameter<std::vector<std::string>>("arm_joints");
    const auto site = declare_parameter<std::string>("tcp_site", "tcp");
    rate_ = declare_parameter<double>("rate", 200.0);
    fault_rad_ = declare_parameter<double>("tracking_fault_rad", 0.1);
    velocity_ff_ = declare_parameter<bool>("target_velocity_ff", true);

    diff_ik::DiffIkParams p;
    p.kp_pos = declare_parameter<double>("kp_pos", p.kp_pos);
    p.v_max = declare_parameter<double>("v_max", p.v_max);
    p.kp_rot = declare_parameter<double>("kp_rot", p.kp_rot);
    p.w_max = declare_parameter<double>("w_max", p.w_max);
    p.lambda0 = declare_parameter<double>("lambda0", p.lambda0);
    p.sigma_thresh = declare_parameter<double>("sigma_thresh", p.sigma_thresh);
    p.lambda_max = declare_parameter<double>("lambda_max", p.lambda_max);
    p.k_null = declare_parameter<double>("k_null", p.k_null);
    p.qd_max = declare_parameter<double>("qd_max", p.qd_max);
    p.limit_margin = declare_parameter<double>("limit_margin", p.limit_margin);
    p.guard_margin = declare_parameter<double>("workspace_guard_margin", p.guard_margin);
    p.q_home = to_vec6(declare_parameter<std::vector<double>>("q_home"), "q_home");
    p.box_lo = to_vec3(declare_parameter<std::vector<double>>("box_lo"), "box_lo");
    p.box_hi = to_vec3(declare_parameter<std::vector<double>>("box_hi"), "box_hi");

    kin_ = std::make_unique<diff_ik::Kinematics>(scene, joint_names_, site);
    ik_ = std::make_unique<diff_ik::DiffIk>(*kin_, p);

    cmd_pub_ = create_publisher<hand_teleop_msgs::msg::JointCommand>("arm/joint_command", 10);
    lat_pub_ = create_publisher<hand_teleop_msgs::msg::LatencySample>("metrics/latency", 10);
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, [this](sensor_msgs::msg::JointState::SharedPtr m) { on_js(*m); });
    tgt_sub_ = create_subscription<hand_teleop_msgs::msg::TeleopTarget>(
        "teleop/target", 10,
        [this](hand_teleop_msgs::msg::TeleopTarget::SharedPtr m) { on_target(*m); });
    hand_sub_ = create_subscription<hand_teleop_msgs::msg::HandState>(
        "hand/state", 10, [this](hand_teleop_msgs::msg::HandState::SharedPtr m) {
          hand_stamps_.emplace_back(stamp_ns(m->capture_stamp), m->header.stamp);
          if (hand_stamps_.size() > 64) hand_stamps_.pop_front();
        });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "estop", rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::Bool::SharedPtr m) {
          if (m->data) on_estop();
        });
    clear_srv_ = create_service<std_srvs::srv::Trigger>(
        "estop/clear", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          on_clear(*res);
        });
    // Wall timer: the sim clock only ticks at 100 Hz, a sim-time timer could not run at 200.
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_), [this] { on_timer(); });
    diag_timer_ = create_wall_timer(1s, [this] { publish_diagnostics(); });
    RCLCPP_INFO(get_logger(), "diff_ik_controller: %.0f Hz, scene %s", rate_, scene.c_str());
  }

 private:
  void on_js(const sensor_msgs::msg::JointState& msg) {
    if (js_index_.empty()) {
      for (const auto& name : joint_names_) {
        auto it = std::find(msg.name.begin(), msg.name.end(), name);
        if (it == msg.name.end()) return;
        js_index_.push_back(static_cast<int>(it - msg.name.begin()));
      }
    }
    for (int i = 0; i < 6; ++i) {
      q_meas_[i] = msg.position[js_index_[i]];
      qd_meas_[i] = msg.velocity.size() == msg.position.size() ? msg.velocity[js_index_[i]] : 0.0;
    }
    meas_wall_ = now_wall();
    have_meas_ = true;
    if (drift_active_) {
      // Braking first (joint speeds fall below rest_speed), then hold drift from the rest pose.
      rest_count_ = qd_meas_.cwiseAbs().maxCoeff() < kRestSpeed ? rest_count_ + 1 : 0;
      if (!at_rest_ && rest_count_ >= kRestSamples) {
        at_rest_ = true;
        q_rest_ = q_meas_;
        braking_ = (q_rest_ - q_stop_).cwiseAbs().maxCoeff();
      }
      if (at_rest_) drift_max_ = std::max(drift_max_, (q_meas_ - q_rest_).cwiseAbs().maxCoeff());
      if (now_wall() - t_stop_ > 2.0) {
        drift_active_ = false;
        last_drift_ = drift_max_;
        RCLCPP_INFO(get_logger(),
                    "post-e-stop: braking distance %.2e rad, hold drift over 2 s %.2e rad",
                    braking_, drift_max_);
      }
    }
  }

  void on_target(const hand_teleop_msgs::msg::TeleopTarget& msg) {
    const auto& p = msg.ee_target.position;
    const auto& o = msg.ee_target.orientation;
    const Eigen::Vector3d pos(p.x, p.y, p.z);
    const double t = now_wall();
    if (latch_.stopped()) {  // targets are ignored until /estop/clear succeeds
      return;
    }
    if (msg.engaged && !engaged_ && have_meas_) {
      q_cmd_ = q_measured_now();  // resync on the engage edge: no jump
      fault_ = false;
      v_ff_.setZero();
      last_target_t_ = -1.0;
    }
    if (velocity_ff_ && msg.engaged && last_target_t_ > 0.0 && t > last_target_t_) {
      const Eigen::Vector3d v = (pos - target_.position) / (t - last_target_t_);
      v_ff_ = 0.5 * v_ff_ + 0.5 * v;  // light smoothing of the 30 Hz finite difference
      const double n = v_ff_.norm();
      if (n > ik_->params().v_max) v_ff_ *= ik_->params().v_max / n;
    }
    last_target_t_ = t;
    target_.position = pos;
    target_.orientation = Eigen::Quaterniond(o.w, o.x, o.y, o.z).normalized();
    engaged_ = msg.engaged;
    have_target_ = true;
    gripper_ = msg.gripper;
    capture_stamp_ = msg.capture_stamp;
    target_stamp_ = msg.header.stamp;
    pending_latency_ = true;
  }

  // Measured joints brought forward to now: joint states arrive at 100 Hz, so the last one can
  // be ~10 ms old, which at speed would freeze the arm behind where it already is.
  Vector6d q_measured_now() const {
    const double age = std::min(std::max(now_wall() - meas_wall_, 0.0), 0.05);
    return q_meas_ + qd_meas_ * age;
  }

  void on_estop() {
    if (latch_.trigger()) {
      if (have_meas_) q_cmd_ = q_measured_now();
      engaged_ = false;  // after a clear, motion resumes only on a fresh engage edge
      q_stop_ = q_cmd_;
      t_stop_ = now_wall();
      drift_max_ = 0.0;
      at_rest_ = false;
      rest_count_ = 0;
      drift_active_ = true;
      RCLCPP_WARN(get_logger(), "E-STOP latched: holding measured joints");
    }
  }

  void on_clear(std_srvs::srv::Trigger::Response& res) {
    if (latch_.clear()) {
      q_cmd_ = q_measured_now();
      engaged_ = false;
      fault_ = false;
      res.success = true;
      res.message = "e-stop cleared, resynced to measured joints";
      RCLCPP_INFO(get_logger(), "%s", res.message.c_str());
    } else {
      res.success = false;
      res.message = "not stopped";
    }
  }

  void on_timer() {
    if (!have_meas_) return;
    if (!initialized_) {
      q_cmd_ = q_meas_;
      initialized_ = true;
    }
    const double dt = 1.0 / rate_;
    if (!latch_.stopped() && engaged_ && have_target_ && !fault_) {
      diff_ik::Target tgt = target_;
      if (velocity_ff_ && now_wall() - last_target_t_ < 3.0 / 30.0) tgt.velocity_ff = v_ff_;
      const auto res = ik_->step(tgt, q_cmd_, dt);
      q_cmd_ = res.q_cmd;
      n_scaled_ += res.velocity_scaled;
      n_limit_ += res.limit_clamped;
      n_damped_ += res.damping_active;
      n_guard_ += res.guard_active;
      if ((q_cmd_ - q_meas_).cwiseAbs().maxCoeff() > fault_rad_) {
        fault_ = true;
        ++n_faults_;
        RCLCPP_ERROR(get_logger(), "tracking fault: |q_cmd - q_meas| > %.2f rad, frozen until "
                     "re-engage", fault_rad_);
      }
    }
    hand_teleop_msgs::msg::JointCommand cmd;
    cmd.header.stamp = now();
    cmd.capture_stamp = capture_stamp_;
    for (int i = 0; i < 6; ++i) cmd.positions[i] = q_cmd_[i];
    cmd.gripper = gripper_;
    cmd_pub_->publish(cmd);

    if (pending_latency_) {
      pending_latency_ = false;
      hand_teleop_msgs::msg::LatencySample s;
      s.capture_stamp = capture_stamp_;
      s.target_stamp = target_stamp_;
      s.command_stamp = cmd.header.stamp;
      const int64_t key = stamp_ns(capture_stamp_);
      for (const auto& [cap, hand] : hand_stamps_) {
        if (cap == key) s.hand_stamp = hand;
      }
      s.tracking_error_m = (target_.position - kin_->fk(q_meas_).position).norm();
      lat_pub_->publish(s);
    }
  }

  void publish_diagnostics() {
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "diff_ik_controller";
    st.hardware_id = "ur5e_sim";
    st.level = latch_.stopped() || fault_ ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                                          : diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = latch_.stopped() ? "e-stop" : fault_ ? "tracking fault"
                                             : engaged_ ? "engaged" : "holding";
    auto kv = [](const std::string& k, const std::string& v) {
      diagnostic_msgs::msg::KeyValue x;
      x.key = k;
      x.value = v;
      return x;
    };
    st.values = {kv("velocity_scaled_steps", std::to_string(n_scaled_)),
                 kv("limit_clamp_steps", std::to_string(n_limit_)),
                 kv("damping_steps", std::to_string(n_damped_)),
                 kv("workspace_guard_steps", std::to_string(n_guard_)),
                 kv("tracking_faults", std::to_string(n_faults_)),
                 kv("last_post_estop_hold_drift_rad", std::to_string(last_drift_)),
                 kv("last_post_estop_braking_rad", std::to_string(braking_))};
    arr.status.push_back(st);
    diag_pub_->publish(arr);
  }

  double now_wall() const {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }

  std::vector<std::string> joint_names_;
  std::vector<int> js_index_;
  double rate_, fault_rad_;
  bool velocity_ff_;
  std::unique_ptr<diff_ik::Kinematics> kin_;
  std::unique_ptr<diff_ik::DiffIk> ik_;
  diff_ik::EstopLatch latch_;

  static constexpr double kRestSpeed = 1e-3;  // rad/s: arm considered at rest after braking
  static constexpr int kRestSamples = 10;     // consecutive joint-state samples (100 ms)
  int rest_count_ = 0;
  bool at_rest_ = false;
  double braking_ = 0.0;
  Vector6d q_rest_ = Vector6d::Zero();
  Vector6d qd_meas_ = Vector6d::Zero();
  double meas_wall_ = 0.0;
  Vector6d q_meas_ = Vector6d::Zero(), q_cmd_ = Vector6d::Zero(), q_stop_ = Vector6d::Zero();
  bool have_meas_ = false, initialized_ = false, have_target_ = false, engaged_ = false;
  bool fault_ = false, pending_latency_ = false, drift_active_ = false;
  diff_ik::Target target_{Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()};
  Eigen::Vector3d v_ff_ = Eigen::Vector3d::Zero();
  double last_target_t_ = -1.0, gripper_ = 1.0, t_stop_ = 0.0, drift_max_ = 0.0,
         last_drift_ = 0.0;
  builtin_interfaces::msg::Time capture_stamp_, target_stamp_;
  std::deque<std::pair<int64_t, builtin_interfaces::msg::Time>> hand_stamps_;
  uint64_t n_scaled_ = 0, n_limit_ = 0, n_damped_ = 0, n_guard_ = 0, n_faults_ = 0;

  rclcpp::Publisher<hand_teleop_msgs::msg::JointCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<hand_teleop_msgs::msg::LatencySample>::SharedPtr lat_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<hand_teleop_msgs::msg::TeleopTarget>::SharedPtr tgt_sub_;
  rclcpp::Subscription<hand_teleop_msgs::msg::HandState>::SharedPtr hand_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv_;
  rclcpp::TimerBase::SharedPtr timer_, diag_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffIkNode>());
  rclcpp::shutdown();
  return 0;
}
