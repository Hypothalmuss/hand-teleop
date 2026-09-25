"""Shared launch helpers: config location and the topic remaps every node gets."""

import os

from ament_index_python.packages import get_package_share_directory


def config_dir() -> str:
    return os.environ.get(
        "HAND_TELEOP_CONFIG",
        os.path.join(get_package_share_directory("hand_teleop_bringup"), "config"))


# Every node runs on sim time; the simulator publishes it on /sim/clock.
SIM_TIME_REMAPS = [("/clock", "/sim/clock")]
