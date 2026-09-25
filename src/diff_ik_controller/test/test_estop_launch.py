"""Launch test: e-stop freezes the command (targets ignored, drift < 1e-3 rad over 2 s) and
clearing resyncs the command to the measured joints.

Drift is measured from the rest pose after braking (joint speeds < 1e-3 rad/s): stopping from
teleop speed takes a few mrad of travel under the Menagerie servo gains; see tuning_notes.md."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # launch_test ignores conftest
os.environ["ROS_DOMAIN_ID"] = "65"

import time  # noqa: E402
import unittest  # noqa: E402

import numpy as np  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from ik_test_utils import make_node, sim_and_ik_description, spin_for, spin_until  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from hand_teleop_msgs.msg import JointCommand, TeleopTarget  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    return sim_and_ik_description()


class TestEstop(unittest.TestCase):
    def test_estop_latch_and_clear(self):
        rclpy.init()
        node = make_node("test_estop")
        ee, cmds, js = [], [], []
        node.create_subscription(PoseStamped, "sim/ee_pose", lambda m: ee.append(m.pose), 10)
        node.create_subscription(JointCommand, "arm/joint_command",
                                 lambda m: cmds.append((time.monotonic(),
                                                        np.array(m.positions))), 500)
        node.create_subscription(JointState, "joint_states", lambda m: js.append(
            (time.monotonic(), np.array(m.position[:6]), np.array(m.velocity[:6]))), 500)
        pub = node.create_publisher(TeleopTarget, "teleop/target", 10)
        estop = node.create_publisher(Bool, "estop", QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        clear = node.create_client(Trigger, "estop/clear")
        self.assertTrue(spin_until(node, lambda: len(ee) > 5 and len(cmds) > 5, 30.0))
        start = ee[-1]
        t0 = time.monotonic()

        def publish():  # target moving at 0.1 m/s along y
            t = time.monotonic() - t0
            msg = TeleopTarget()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.capture_stamp = msg.header.stamp
            msg.ee_target = PoseStamped().pose
            msg.ee_target.position.x = start.position.x
            msg.ee_target.position.y = start.position.y - 0.1 * t
            msg.ee_target.position.z = start.position.z - 0.05
            msg.ee_target.orientation = start.orientation
            msg.engaged, msg.gripper = True, 1.0
            pub.publish(msg)

        spin_for(node, 1.5, publish, 1 / 30)
        moving = np.abs(cmds[-1][1] - cmds[-30][1]).max()
        self.assertGreater(moving, 1e-3, "arm was not moving before the e-stop")

        t_stop = time.monotonic()
        estop.publish(Bool(data=True))
        spin_for(node, 2.2, publish, 1 / 30)  # targets keep coming and must be ignored
        after = [q for t, q in cmds if t > t_stop + 0.1]
        cmd_change = max(np.abs(q - after[0]).max() for q in after)
        window = [(t, q, qd) for t, q, qd in js if t_stop < t < t_stop + 2.0]
        q_stop = window[0][1]
        # braking: until every joint is slower than 1e-3 rad/s; then the arm must hold
        slow = [np.abs(qd).max() < 1e-3 for _, _, qd in window]
        i_rest = next(i for i in range(len(slow)) if all(slow[i:i + 10]))  # 100 ms at rest
        q_rest = window[i_rest][1]
        braking = np.abs(q_rest - q_stop).max()
        drift = max(np.abs(q - q_rest).max() for _, q, _ in window[i_rest:])
        rest_time = window[i_rest][0] - t_stop

        fut = clear.call_async(Trigger.Request())
        self.assertTrue(spin_until(node, fut.done, 5.0))
        t_clear = time.monotonic()
        estop.publish(Bool(data=False))
        spin_for(node, 0.3)
        q_meas = [q for t, q, _ in js if t <= t_clear][-1]
        first_after_clear = [q for t, q in cmds if t > t_clear][0]
        node.destroy_node()
        rclpy.shutdown()

        print(f"command change during e-stop {cmd_change:.2e} rad; braking {braking:.2e} rad "
              f"(at rest after {rest_time * 1000:.0f} ms); hold drift {drift:.2e} rad; "
              f"resync error {np.abs(first_after_clear - q_meas).max():.2e}")
        self.assertTrue(fut.result().success)
        self.assertEqual(cmd_change, 0.0)
        self.assertLess(drift, 1e-3)
        self.assertLess(np.abs(first_after_clear - q_meas).max(), 1e-3)
