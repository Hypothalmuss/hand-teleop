"""``fake_hand``: synthetic /hand/state (no camera) for integration tests and pipeline checks.

The right hand is an open palm whose middle MCP traces a slow Lissajous pattern inside the
calibrated hand range (depth via palm_scale); the left hand pinches open and closed.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node

from hand_teleop_msgs.msg import Hand, HandState
from ur5e_2f85_mujoco.config import find_config_dir, load_config


def synthetic_hand(u: float, v: float, palm: float, pinch: float) -> np.ndarray:
    """(21, 3) open-hand landmarks with landmark 9 at image (u, 1 - v)."""
    lm = np.zeros((21, 3))
    x9, y9 = u, 1.0 - v
    lm[0] = [x9, y9 + palm, 0]
    lm[9] = [x9, y9, 0]
    for k, dx in enumerate((-0.25, 0.0, 0.25, 0.5)):
        base = 5 + 4 * k
        mcp = np.array([x9 + dx * palm, y9, 0])
        for j in range(4):
            lm[base + j] = mcp - [0, 0.28 * palm * j, 0]
    lm[1:5] = [[x9 - 0.35 * palm, y9 + 0.7 * palm - 0.1 * palm * j, 0] for j in range(4)]
    lm[4] = lm[8] + [pinch * palm * 0.7, 0.0, 0.0]
    return lm


class FakeHand(Node):
    def __init__(self):
        super().__init__("fake_hand")
        self.declare_parameter("config_dir", "")
        self.declare_parameter("period_s", 12.0)
        self.declare_parameter("amplitude", 0.35)  # fraction of the calibrated range
        cfg = find_config_dir(self.get_parameter("config_dir").value or None)
        self.ws = load_config("workspace.yaml", cfg)
        self.rate = rate = load_config("rates.yaml", cfg)["hand_tracker"]
        self.pub = self.create_publisher(HandState, "hand/state", 10)
        self.t0 = None
        self.create_timer(1.0 / rate, self._tick)

    def _tick(self) -> None:
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if t == 0.0:
            return  # waiting for the sim clock
        if self.t0 is None:
            self.t0 = t
        t -= self.t0
        w = 2 * np.pi / float(self.get_parameter("period_s").value)
        a = float(self.get_parameter("amplitude").value)
        r = self.ws["hand_range"]
        mid = {k: 0.5 * (r[k][0] + r[k][1]) for k in r}
        half = {k: 0.5 * (r[k][1] - r[k][0]) for k in r}
        u = mid["u"] + a * half["u"] * np.sin(w * t)
        v = mid["v"] + a * half["v"] * np.sin(2 * w * t)
        d = mid["d"] + a * half["d"] * np.sin(0.5 * w * t)
        g = self.ws["gripper"]
        pinch = g["pinch_closed"] + (g["pinch_open"] - g["pinch_closed"]) * (
            0.5 + 0.5 * np.cos(0.5 * w * t))
        msg = HandState()
        msg.header.stamp = now.to_msg()
        msg.capture_stamp = msg.header.stamp
        palm = 1.0 / d
        msg.right = Hand(present=True, landmarks=synthetic_hand(u, v, palm, 1.0).reshape(-1)
                         .tolist(), palm_scale=palm, pinch=1.0, is_open=True, is_fist=False)
        msg.left = Hand(present=True, landmarks=synthetic_hand(0.3, 0.5, palm, pinch)
                        .reshape(-1).tolist(), palm_scale=palm, pinch=float(pinch),
                        is_open=False, is_fist=False)
        msg.fps = float(self.rate)
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeHand()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
