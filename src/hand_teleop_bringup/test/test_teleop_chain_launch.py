"""Integration launch test: the full teleop chain with a synthetic hand (no camera).

teleop.launch.py hand:=fake viewer:=false -> fake_hand -> teleop_mapper -> diff_ik_controller
-> mujoco_sim. Checks startup time, rates, that the arm follows, and tracking error.
"""

import os

os.environ["ROS_DOMAIN_ID"] = "66"

import time  # noqa: E402
import unittest  # noqa: E402

import numpy as np  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from launch import LaunchDescription  # noqa: E402
from launch.actions import IncludeLaunchDescription  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402
from launch_testing.actions import ReadyToTest  # noqa: E402

from hand_teleop_msgs.msg import JointCommand, LatencySample, TeleopTarget  # noqa: E402

T_LAUNCH = time.monotonic()


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(get_package_share_directory("hand_teleop_bringup"), "launch",
                               "teleop.launch.py")
    teleop = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file),
                                      launch_arguments={"hand": "fake", "viewer": "false"}.items())
    return LaunchDescription([teleop, ReadyToTest()])


class TestTeleopChain(unittest.TestCase):
    def test_chain(self):
        rclpy.init()
        node = rclpy.create_node("test_teleop_chain")
        cmds, lat, ee, tgt = [], [], [], []
        node.create_subscription(JointCommand, "/arm/joint_command",
                                 lambda m: cmds.append(time.monotonic()), 500)
        node.create_subscription(LatencySample, "/metrics/latency",
                                 lambda m: lat.append((time.monotonic(), m)), 100)
        node.create_subscription(PoseStamped, "/sim/ee_pose", lambda m: ee.append(
            np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])), 100)
        node.create_subscription(TeleopTarget, "/teleop/target",
                                 lambda m: tgt.append(m.engaged), 100)
        end = time.monotonic() + 30.0
        while not cmds and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
        startup = cmds[0] - T_LAUNCH if cmds else float("inf")
        cmds.clear()
        lat.clear()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 12.0:
            rclpy.spin_once(node, timeout_sec=0.002)
        dt = time.monotonic() - t0
        node.destroy_node()
        rclpy.shutdown()

        cmd_rate = len(cmds) / dt
        lat_rate = len(lat) / dt
        travel = np.linalg.norm(np.ptp(np.array(ee[-600:]), axis=0))
        trk = np.array([m.tracking_error_m for _, m in lat[60:]])
        rms = float(np.sqrt(np.mean(trk ** 2)))
        e2e = np.array([(m.command_stamp.sec - m.capture_stamp.sec) * 1e3
                        + (m.command_stamp.nanosec - m.capture_stamp.nanosec) * 1e-6
                        for _, m in lat])
        print(f"startup {startup:.1f} s; commands {cmd_rate:.0f} Hz; latency samples "
              f"{lat_rate:.1f} Hz; engaged {np.mean(tgt):.0%}; TCP travel {travel * 100:.1f} cm;"
              f" tracking RMS {rms * 1000:.1f} mm; capture->command p50 "
              f"{np.percentile(e2e, 50):.0f} ms")
        self.assertLess(startup, 10.0)
        self.assertAlmostEqual(cmd_rate, 200.0, delta=20.0)
        self.assertAlmostEqual(lat_rate, 30.0, delta=3.0)
        self.assertGreater(np.mean(tgt), 0.9)
        self.assertGreater(travel, 0.05)
        self.assertLess(rms, 0.015)
