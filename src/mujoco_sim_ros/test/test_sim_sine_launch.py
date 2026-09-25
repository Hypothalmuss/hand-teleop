"""Launch test: joint positions track a sine-sweep command (< 0.02 rad RMS after 1 s)."""

import os

os.environ["ROS_DOMAIN_ID"] = "62"

import bisect  # noqa: E402
import unittest  # noqa: E402

import launch  # noqa: E402
import launch_ros.actions  # noqa: E402
import launch_testing.actions  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from sim_test_utils import spin_for, spin_until, stamp_s  # noqa: E402

from hand_teleop_msgs.msg import JointCommand  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    remap = [("/clock", "/sim/clock")]
    sim = launch_ros.actions.Node(package="mujoco_sim_ros", executable="mujoco_sim",
                                  parameters=[{"render": False}], output="screen")
    sine = launch_ros.actions.Node(
        package="mujoco_sim_ros", executable="scripted_trajectory", output="screen",
        parameters=[{"mode": "sine", "use_sim_time": True}], remappings=remap)
    return launch.LaunchDescription([sim, sine, launch_testing.actions.ReadyToTest()])


class TestSine(unittest.TestCase):
    def test_tracking(self):
        rclpy.init()
        node = rclpy.create_node("test_sine")
        cmds, meas = [], []
        node.create_subscription(
            JointCommand, "arm/joint_command",
            lambda m: cmds.append((stamp_s(m.header.stamp), np.array(m.positions))), 100)
        node.create_subscription(
            JointState, "joint_states",
            lambda m: meas.append((stamp_s(m.header.stamp), np.array(m.position[:6]))), 100)
        self.assertTrue(spin_until(node, lambda: len(cmds) > 0, 30.0), "no commands")
        spin_for(node, 6.0)
        node.destroy_node()
        rclpy.shutdown()

        t_cmd = [c[0] for c in cmds]
        t_start = t_cmd[0] + 1.0
        errs = []
        for ts, q in meas:
            if ts < t_start:
                continue
            i = bisect.bisect_right(t_cmd, ts) - 1
            if i >= 0:
                errs.append(q - cmds[i][1])
        rms = float(np.sqrt(np.mean(np.square(errs))))
        print(f"sine tracking RMS {rms:.5f} rad over {len(errs)} samples")
        self.assertGreater(len(errs), 200)
        self.assertLess(rms, 0.02)
