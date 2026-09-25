// Test helpers: the scene from the ament index and a tiny YAML-subset reader for config files.
#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "diff_ik_controller/diff_ik.hpp"

namespace test_config {

inline std::string trim(const std::string& s) {
  const auto a = s.find_first_not_of(" \t");
  const auto b = s.find_last_not_of(" \t\r");
  return a == std::string::npos ? "" : s.substr(a, b - a + 1);
}

// Reads "key: scalar", "key: [a, b]" and "key:\n- item" entries (top level and one nested
// level, flattened as "parent.key"). Comments are stripped.
inline std::map<std::string, std::vector<std::string>> read_yaml(const std::string& path) {
  std::map<std::string, std::vector<std::string>> out;
  std::ifstream f(path);
  std::string line, parent, last_key;
  while (std::getline(f, line)) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    if (trim(line).empty()) continue;
    const bool indented = line[0] == ' ';
    const std::string t = trim(line);
    if (t.rfind("- ", 0) == 0) {
      out[last_key].push_back(trim(t.substr(2)));
      continue;
    }
    const auto colon = t.find(':');
    if (colon == std::string::npos) continue;
    const std::string key = trim(t.substr(0, colon));
    std::string val = trim(t.substr(colon + 1));
    if (!indented) parent = key;
    last_key = indented ? parent + "." + key : key;
    if (val.empty()) continue;
    if (val.front() == '[') {
      val = val.substr(1, val.find(']') - 1);
      std::stringstream ss(val);
      std::string item;
      while (std::getline(ss, item, ',')) out[last_key].push_back(trim(item));
    } else {
      out[last_key].push_back(val);
    }
  }
  return out;
}

inline std::vector<double> nums(const std::vector<std::string>& v) {
  std::vector<double> out;
  for (const auto& s : v) out.push_back(std::stod(s));
  return out;
}

struct Setup {
  std::string scene;
  std::vector<std::string> joints;
  std::string site;
  diff_ik::DiffIkParams params;
};

inline Setup load() {
  Setup s;
  const std::string share = ament_index_cpp::get_package_share_directory("ur5e_2f85_mujoco");
  s.scene = share + "/ur5e_2f85_mujoco/scene.xml";
  auto names = read_yaml(share + "/ur5e_2f85_mujoco/model_names.yaml");
  s.joints = names["arm_joints"];
  s.site = names["tcp_site"].at(0);
  const std::string cfg = REPO_CONFIG_DIR;
  auto ik = read_yaml(cfg + "/ik.yaml");
  auto ws = read_yaml(cfg + "/workspace.yaml");
  auto& p = s.params;
  p.kp_pos = std::stod(ik["kp_pos"][0]);
  p.v_max = std::stod(ik["v_max"][0]);
  p.kp_rot = std::stod(ik["kp_rot"][0]);
  p.w_max = std::stod(ik["w_max"][0]);
  p.lambda0 = std::stod(ik["lambda0"][0]);
  p.sigma_thresh = std::stod(ik["sigma_thresh"][0]);
  p.lambda_max = std::stod(ik["lambda_max"][0]);
  p.k_null = std::stod(ik["k_null"][0]);
  p.qd_max = std::stod(ik["qd_max"][0]);
  p.limit_margin = std::stod(ik["limit_margin"][0]);
  p.guard_margin = std::stod(ik["workspace_guard_margin"][0]);
  const auto qh = nums(ik["q_home"]);
  for (int i = 0; i < 6; ++i) p.q_home[i] = qh[i];
  const char* axes[3] = {"box.x", "box.y", "box.z"};
  for (int i = 0; i < 3; ++i) {
    const auto r = nums(ws[axes[i]]);
    p.box_lo[i] = r[0];
    p.box_hi[i] = r[1];
  }
  return s;
}

}  // namespace test_config
