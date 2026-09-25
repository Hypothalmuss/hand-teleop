"""Launch helper: ROS parameters for diff_ik_node from config/*.yaml and model_names.yaml."""

from ur5e_2f85_mujoco import load_model_names, scene_path
from ur5e_2f85_mujoco.config import load_config

IK_KEYS = ("kp_pos", "v_max", "kp_rot", "w_max", "lambda0", "sigma_thresh", "lambda_max",
           "k_null", "qd_max", "limit_margin", "tracking_fault_rad", "workspace_guard_margin",
           "target_velocity_ff", "target_velocity_ff_alpha")


def node_parameters(config_dir=None) -> dict:
    ik = load_config("ik.yaml", config_dir)
    ws = load_config("workspace.yaml", config_dir)
    rates = load_config("rates.yaml", config_dir)
    names = load_model_names()
    params = {k: ik[k] for k in IK_KEYS}
    params.update({
        "scene_path": str(scene_path()),
        "arm_joints": list(names["arm_joints"]),
        "tcp_site": names["tcp_site"],
        "rate": float(rates["ik"]),
        "q_home": [float(v) for v in ik["q_home"]],
        "box_lo": [float(ws["box"][a][0]) for a in ("x", "y", "z")],
        "box_hi": [float(ws["box"][a][1]) for a in ("x", "y", "z")],
    })
    return params
