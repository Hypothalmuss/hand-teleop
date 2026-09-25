"""Simulator + differential IK controller (no hand input): for scripted targets and tests.

    ros2 launch hand_teleop_bringup ik_sim.launch.py viewer:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from diff_ik_controller import node_parameters
from hand_teleop_bringup import SIM_TIME_REMAPS, config_dir


def _nodes(context):
    cfg = config_dir()
    headless = LaunchConfiguration("headless").perform(context) == "true"
    viewer = LaunchConfiguration("viewer").perform(context) == "true" and not headless
    return [
        Node(package="mujoco_sim_ros", executable="mujoco_sim", output="screen",
             parameters=[{"config_dir": cfg, "viewer": viewer,
                          "seed": int(LaunchConfiguration("seed").perform(context))}]),
        Node(package="diff_ik_controller", executable="diff_ik_node", output="screen",
             parameters=[node_parameters(cfg), {"use_sim_time": True}],
             remappings=SIM_TIME_REMAPS),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="false"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("seed", default_value="0"),
        OpaqueFunction(function=_nodes),
    ])
