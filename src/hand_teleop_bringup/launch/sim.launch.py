"""Simulator (+ optional scripted trajectory and RViz).

    ros2 launch hand_teleop_bringup sim.launch.py viewer:=true scripted:=pick_place
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hand_teleop_bringup import SIM_TIME_REMAPS, config_dir, domain_warning, rviz_nodes


def _nodes(context):
    cfg = config_dir()
    headless = LaunchConfiguration("headless").perform(context) == "true"
    viewer = LaunchConfiguration("viewer").perform(context) == "true" and not headless
    scripted = LaunchConfiguration("scripted").perform(context)
    seed = int(LaunchConfiguration("seed").perform(context))
    nodes = [
        Node(package="mujoco_sim_ros", executable="mujoco_sim", name="mujoco_sim",
             output="screen",
             parameters=[{"config_dir": cfg, "viewer": viewer, "seed": seed,
                          "rt_factor": float(LaunchConfiguration("rt_factor").perform(context))}],
             additional_env={"MUJOCO_GL": "egl"} if headless else None),
    ]
    if scripted in ("sine", "pick_place"):
        nodes.append(Node(
            package="mujoco_sim_ros", executable="scripted_trajectory", output="screen",
            parameters=[{"config_dir": cfg, "mode": scripted, "seed": seed,
                         "runs": int(LaunchConfiguration("runs").perform(context)),
                         "use_sim_time": True}],
            remappings=SIM_TIME_REMAPS))
    if LaunchConfiguration("rviz").perform(context) == "true" and not headless:
        nodes += rviz_nodes()
    return nodes


def generate_launch_description():
    return LaunchDescription(domain_warning() + [
        DeclareLaunchArgument("viewer", default_value="false"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("seed", default_value="0"),
        DeclareLaunchArgument("scripted", default_value="none",
                              description="none | sine | pick_place"),
        DeclareLaunchArgument("runs", default_value="1"),
        DeclareLaunchArgument("rt_factor", default_value="1.0"),
        OpaqueFunction(function=_nodes),
    ])
