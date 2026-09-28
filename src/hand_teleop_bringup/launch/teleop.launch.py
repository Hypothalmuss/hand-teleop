"""Full teleop chain: sim + hand input + teleop_mapper + diff_ik_controller (+ console, RViz).

    ros2 launch hand_teleop_bringup teleop.launch.py                   # webcam, viewer on
    ros2 launch hand_teleop_bringup teleop.launch.py video:=clip.mp4    # offline hand input
    ros2 launch hand_teleop_bringup teleop.launch.py hand:=fake viewer:=false
    ros2 launch hand_teleop_bringup teleop.launch.py console:=terminal  # console in a new window

The keyboard console needs a terminal; by default run it separately:
    ros2 run teleop_console keyboard_console
"""

import os
import shlex
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from diff_ik_controller import node_parameters
from hand_teleop_bringup import SIM_TIME_REMAPS, config_dir, domain_warning, rviz_nodes

ENV_VARS = ("AMENT_PREFIX_PATH", "PYTHONPATH", "LD_LIBRARY_PATH", "PATH", "ROS_DOMAIN_ID",
            "ROS_VERSION", "ROS_DISTRO", "ROS_PYTHON_VERSION", "RMW_IMPLEMENTATION",
            "HAND_TELEOP_CONFIG", "DISPLAY", "HOME")


def terminal_prefix() -> str:
    """gnome-terminal starts commands with its server's environment: wrap with ours."""
    fd, path = tempfile.mkstemp(prefix="hand_teleop_console_", suffix=".sh", dir="/tmp")
    with os.fdopen(fd, "w") as f:
        f.write("#!/bin/bash\n")
        for k in ENV_VARS:
            if k in os.environ:
                f.write(f"export {k}={shlex.quote(os.environ[k])}\n")
        f.write('exec "$@"\n')
    os.chmod(path, 0o755)
    return f"gnome-terminal --wait -- {path}"


def _nodes(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    cfg = config_dir()
    video = arg("video")
    hand = "video" if video else arg("hand")
    sim_time = {"use_sim_time": True}
    nodes = [
        Node(package="mujoco_sim_ros", executable="mujoco_sim", name="mujoco_sim",
             output="screen",
             parameters=[{"config_dir": cfg, "viewer": arg("viewer") == "true",
                          "seed": int(arg("seed")), "demo_view": arg("demo_view") == "true"}]),
        Node(package="teleop_mapper", executable="teleop_mapper", output="screen",
             parameters=[{"config_dir": cfg}, sim_time], remappings=SIM_TIME_REMAPS),
        Node(package="diff_ik_controller", executable="diff_ik_node", output="screen",
             parameters=[node_parameters(cfg), sim_time], remappings=SIM_TIME_REMAPS),
    ]
    if hand == "fake":
        nodes.append(Node(package="hand_tracker", executable="fake_hand", output="screen",
                          parameters=[{"config_dir": cfg}, sim_time],
                          remappings=SIM_TIME_REMAPS))
    else:
        nodes.append(Node(package="hand_tracker", executable="hand_tracker", output="screen",
                          parameters=[{"config_dir": cfg, "video": video,
                                       "loop_video": arg("loop_video") == "true",
                                       "show_window": arg("window") == "true"}, sim_time],
                          remappings=SIM_TIME_REMAPS))
    console = arg("console")
    if console == "terminal":
        nodes.append(Node(package="teleop_console", executable="keyboard_console",
                          prefix=terminal_prefix(), parameters=[{"config_dir": cfg}]))
    else:
        nodes.append(LogInfo(msg="keyboard console: run `ros2 run teleop_console "
                                 "keyboard_console` in another terminal (or console:=terminal)"))
    if arg("rviz") == "true":
        nodes += rviz_nodes()
    return nodes


def generate_launch_description():
    return LaunchDescription(domain_warning() + [
        DeclareLaunchArgument("seed", default_value="0"),
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("video", default_value="", description="offline hand input"),
        DeclareLaunchArgument("loop_video", default_value="false"),
        DeclareLaunchArgument("hand", default_value="webcam", description="webcam | fake"),
        DeclareLaunchArgument("console", default_value="none", description="none | terminal"),
        DeclareLaunchArgument("demo_view", default_value="false"),
        DeclareLaunchArgument("window", default_value="true",
                              description="camera window with the hand overlay"),
        OpaqueFunction(function=_nodes),
    ])
