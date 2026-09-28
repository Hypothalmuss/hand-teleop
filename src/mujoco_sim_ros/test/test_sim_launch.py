"""Launch test: topic rates, real-time factor and reset determinism of mujoco_sim."""

import os

os.environ["ROS_DOMAIN_ID"] = "61"

import time  # noqa: E402
import unittest  # noqa: E402

import launch  # noqa: E402
import launch_ros.actions  # noqa: E402
import launch_testing.actions  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import Image, JointState  # noqa: E402
from sim_test_utils import spin_for, spin_until, stamp_s  # noqa: E402

from hand_teleop_msgs.srv import ResetScene  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    sim = launch_ros.actions.Node(
        package="mujoco_sim_ros", executable="mujoco_sim", output="screen",
        additional_env={"MUJOCO_GL": "egl"})
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


class TestSim(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_sim")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_rates_and_rtf(self):
        counts = {"js": 0, "front": 0, "wrist": 0}
        clock = []
        n = self.node
        n.create_subscription(JointState, "joint_states",
                              lambda m: counts.__setitem__("js", counts["js"] + 1), 50)
        n.create_subscription(Image, "sim/front/image",
                              lambda m: counts.__setitem__("front", counts["front"] + 1), 10)
        n.create_subscription(Image, "sim/wrist/image",
                              lambda m: counts.__setitem__("wrist", counts["wrist"] + 1), 10)
        n.create_subscription(
            Clock, "sim/clock", lambda m: clock.append((time.monotonic(), stamp_s(m.clock))),
            QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.assertTrue(spin_until(n, lambda: counts["front"] > 5, 30.0), "no images")
        spin_for(n, 1.0)
        for k in counts:
            counts[k] = 0
        clock.clear()
        t0 = time.monotonic()
        spin_for(n, 5.0)
        dt = time.monotonic() - t0
        js_rate, front, wrist = counts["js"] / dt, counts["front"] / dt, counts["wrist"] / dt
        rtf = (clock[-1][1] - clock[0][1]) / (clock[-1][0] - clock[0][0])
        print(f"joint_states {js_rate:.1f} Hz, front {front:.1f} Hz, wrist {wrist:.1f} Hz, "
              f"RTF {rtf:.3f}")
        self.assertAlmostEqual(js_rate, 100.0, delta=5.0)
        self.assertGreaterEqual(rtf, 0.95)
        if os.environ.get("HAND_TELEOP_SOFTWARE_GL") == "1":
            # CI runners have no GPU; software EGL cannot render two cameras at 30 Hz. The
            # physics/state rates and RTF above are still checked.
            print("camera-rate check skipped: HAND_TELEOP_SOFTWARE_GL=1")
            return
        self.assertAlmostEqual(front, 30.0, delta=3.0)
        self.assertAlmostEqual(wrist, 30.0, delta=3.0)

    def test_reset_determinism(self):
        cli = self.node.create_client(ResetScene, "sim/reset")
        self.assertTrue(cli.wait_for_service(timeout_sec=30.0))

        def reset(seed):
            fut = cli.call_async(ResetScene.Request(seed=seed, randomize_target=False))
            self.assertTrue(spin_until(self.node, fut.done, 10.0))
            p = fut.result().cube
            return (p.position.x, p.position.y, p.position.z, p.orientation.w, p.orientation.z)

        a, b, c = reset(5), reset(5), reset(6)
        self.assertEqual(a, b)
        self.assertNotEqual(a, c)
