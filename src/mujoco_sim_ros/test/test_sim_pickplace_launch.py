"""Launch test: scripted pick-and-place through ROS succeeds in 10 of 10 seeded runs."""

import os

os.environ["ROS_DOMAIN_ID"] = "63"

import json  # noqa: E402
import unittest  # noqa: E402

import launch  # noqa: E402
import launch_ros.actions  # noqa: E402
import launch_testing.actions  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from sim_test_utils import spin_until  # noqa: E402
from std_msgs.msg import String  # noqa: E402

EPISODES = 10


@pytest.mark.launch_test
def generate_test_description():
    remap = [("/clock", "/sim/clock")]
    sim = launch_ros.actions.Node(package="mujoco_sim_ros", executable="mujoco_sim",
                                  parameters=[{"render": False, "rt_factor": 2.0}],
                                  output="screen")
    script = launch_ros.actions.Node(
        package="mujoco_sim_ros", executable="scripted_trajectory", output="screen",
        parameters=[{"mode": "pick_place", "episodes": EPISODES, "seed": 100,
                     "use_sim_time": True}], remappings=remap)
    return launch.LaunchDescription([sim, script, launch_testing.actions.ReadyToTest()])


class TestPickPlace(unittest.TestCase):
    def test_ten_episodes(self):
        rclpy.init()
        node = rclpy.create_node("test_pick_place")
        results = []
        node.create_subscription(String, "scripted/result",
                                 lambda m: results.append(json.loads(m.data)), 20)
        spin_until(node, lambda: len(results) >= EPISODES, 180.0)
        node.destroy_node()
        rclpy.shutdown()
        print(results)
        self.assertEqual(len(results), EPISODES)
        self.assertEqual(sum(r["success"] for r in results), EPISODES)
