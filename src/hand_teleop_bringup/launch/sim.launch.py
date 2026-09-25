"""Simulator (+ optional scripted trajectory and RViz).

    ros2 launch hand_teleop_bringup sim.launch.py viewer:=true scripted:=pick_place
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hand_teleop_bringup import SIM_TIME_REMAPS, config_dir


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
                         "episodes": int(LaunchConfiguration("episodes").perform(context)),
                         "use_sim_time": True}],
            remappings=SIM_TIME_REMAPS))
    if LaunchConfiguration("rviz").perform(context) == "true" and not headless:
        nodes += rviz_nodes()
    return nodes


def rviz_nodes():
    share = get_package_share_directory("hand_teleop_bringup")
    out = [Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(
        share, "rviz", "sim.rviz")], parameters=[{"use_sim_time": True}],
        remappings=SIM_TIME_REMAPS)]
    try:
        ur = get_package_share_directory("ur_description")
    except PackageNotFoundError:
        return out  # RobotModel needs ur_description (apt: ros-humble-ur-description)
    import xacro

    urdf = xacro.process_file(os.path.join(ur, "urdf", "ur.urdf.xacro"),
                              mappings={"ur_type": "ur5e", "name": "ur5e"}).toxml()
    out.append(Node(package="robot_state_publisher", executable="robot_state_publisher",
                    parameters=[{"robot_description": urdf, "use_sim_time": True}],
                    remappings=SIM_TIME_REMAPS))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="false"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("seed", default_value="0"),
        DeclareLaunchArgument("scripted", default_value="none",
                              description="none | sine | pick_place"),
        DeclareLaunchArgument("episodes", default_value="1"),
        DeclareLaunchArgument("rt_factor", default_value="1.0"),
        OpaqueFunction(function=_nodes),
    ])
