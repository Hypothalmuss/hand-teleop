"""Launch test: the IK tracks a 10 cm square at 0.1 m/s (TCP RMS < 5 mm) and publishes a
LatencySample per target at 30 +- 3 Hz."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # launch_test ignores conftest
os.environ["ROS_DOMAIN_ID"] = "64"

import time  # noqa: E402
import unittest  # noqa: E402

import numpy as np  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from ik_test_utils import (  # noqa: E402,E501
    make_node,
    sim_and_ik_description,
    spin_for,
    spin_until,
    stamp_s,
)

from diff_ik_controller.square import SquarePath  # noqa: E402
from hand_teleop_msgs.msg import LatencySample, TeleopTarget  # noqa: E402

RATE = 30.0


@pytest.mark.launch_test
def generate_test_description():
    return sim_and_ik_description()


class TestSquare(unittest.TestCase):
    def test_square_tracking(self):
        rclpy.init()
        node = make_node("test_square")
        ee, lat, sent = [], [], []
        node.create_subscription(PoseStamped, "sim/ee_pose", lambda m: ee.append(
            (stamp_s(m.header.stamp), np.array([m.pose.position.x, m.pose.position.y,
                                                m.pose.position.z]),
             m.pose.orientation)), 100)
        node.create_subscription(LatencySample, "metrics/latency",
                                 lambda m: lat.append(time.monotonic()), 100)
        pub = node.create_publisher(TeleopTarget, "teleop/target", 10)
        self.assertTrue(spin_until(node, lambda: len(ee) > 10, 30.0), "no ee pose")
        spin_for(node, 1.0)
        start, quat = ee[-1][1], ee[-1][2]
        path = SquarePath(start, center=[0.45, 0.0, 0.20])
        t0 = time.monotonic()

        def publish():
            t = time.monotonic() - t0
            p = path(t)
            msg = TeleopTarget()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.capture_stamp = msg.header.stamp
            msg.ee_target.position.x, msg.ee_target.position.y, msg.ee_target.position.z = p
            msg.ee_target.orientation = quat
            msg.gripper, msg.engaged = 1.0, True
            pub.publish(msg)
            sent.append((stamp_s(msg.header.stamp), p))

        spin_for(node, path.duration + 1.0, publish, 1.0 / RATE)
        node.destroy_node()
        rclpy.shutdown()

        # Reference: the published targets linearly interpolated in sim time.
        ts = np.array([s[0] for s in sent])
        ps = np.array([s[1] for s in sent])
        t_lap0 = ts[0] + path.lead_in + 0.5
        t_end = ts[0] + path.duration
        errs = []
        for t, p, _ in ee:
            if t_lap0 <= t <= t_end:
                ref = np.array([np.interp(t, ts, ps[:, i]) for i in range(3)])
                errs.append(np.linalg.norm(p - ref))
        rms = float(np.sqrt(np.mean(np.square(errs))))
        lat_t = np.array(lat)
        lat_rate = (len(lat_t) - 1) / (lat_t[-1] - lat_t[0])
        print(f"square tracking RMS {rms * 1000:.2f} mm (max {max(errs) * 1000:.2f} mm, "
              f"{len(errs)} samples); LatencySample {lat_rate:.1f} Hz")
        self.assertGreater(len(errs), 300)
        self.assertLess(rms, 0.005)
        self.assertAlmostEqual(lat_rate, RATE, delta=3.0)
