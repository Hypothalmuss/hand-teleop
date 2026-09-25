"""Shared launch helpers: config location and the topic remaps every node gets."""

import os

from ament_index_python.packages import get_package_share_directory


def config_dir() -> str:
    return os.environ.get(
        "HAND_TELEOP_CONFIG",
        os.path.join(get_package_share_directory("hand_teleop_bringup"), "config"))


# Every node runs on sim time; the simulator publishes it on /sim/clock.
SIM_TIME_REMAPS = [("/clock", "/sim/clock")]


def rviz_nodes():
    """RViz with the sim config, plus robot_state_publisher when ur_description is installed."""
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    from launch_ros.actions import Node

    share = get_package_share_directory("hand_teleop_bringup")
    out = [Node(package="rviz2", executable="rviz2",
                arguments=["-d", os.path.join(share, "rviz", "sim.rviz")],
                parameters=[{"use_sim_time": True}], remappings=SIM_TIME_REMAPS)]
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
