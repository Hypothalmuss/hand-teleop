"""``teleop_mapper`` node: /hand/state -> /teleop/target (+ RViz marker)."""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

from hand_teleop_msgs.msg import HandState, TeleopTarget
from teleop_mapper.mapping import HandInput, TeleopMapperCore
from ur5e_2f85_mujoco import load_model_names
from ur5e_2f85_mujoco.config import find_config_dir, load_config

ESTOP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)


def to_input(h) -> HandInput:
    return HandInput(bool(h.present), np.array(h.landmarks), float(h.palm_scale),
                     float(h.pinch), bool(h.is_open), bool(h.is_fist))


class TeleopMapperNode(Node):
    def __init__(self):
        super().__init__("teleop_mapper")
        self.declare_parameter("config_dir", "")
        cfg_dir = find_config_dir(self.get_parameter("config_dir").value or None)
        ws = load_config("workspace.yaml", cfg_dir)
        rates = load_config("rates.yaml", cfg_dir)
        self.core = TeleopMapperCore(ws, rates["teleop_mapper"])
        q = load_model_names()["tcp_home_quat_wxyz"]  # fixed orientation: gripper down
        self.quat = [float(v) for v in q]
        self.ee_pos = None
        self.estop = False
        self.pub = self.create_publisher(TeleopTarget, "teleop/target", 10)
        self.pub_marker = self.create_publisher(Marker, "teleop/target_marker", 10)
        self.create_subscription(HandState, "hand/state", self._on_hand, 10)
        self.create_subscription(PoseStamped, "sim/ee_pose", self._on_ee, 10)
        self.create_subscription(Bool, "estop", self._on_estop, ESTOP_QOS)
        self._was_engaged = False

    def _on_ee(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.ee_pos = np.array([p.x, p.y, p.z])

    def _on_estop(self, msg: Bool) -> None:
        self.estop = bool(msg.data)

    def _on_hand(self, msg: HandState) -> None:
        t = msg.capture_stamp.sec + msg.capture_stamp.nanosec * 1e-9
        out = self.core.update(to_input(msg.right), to_input(msg.left), self.ee_pos,
                               self.estop, t)
        if out.target is None:
            return  # no EE pose yet
        if out.engaged != self._was_engaged:
            self.get_logger().info("engaged" if out.engaged else "disengaged")
            self._was_engaged = out.engaged
        tgt = TeleopTarget()
        tgt.header.stamp = self.get_clock().now().to_msg()
        tgt.header.frame_id = "base_link"
        tgt.capture_stamp = msg.capture_stamp
        p = tgt.ee_target.position
        p.x, p.y, p.z = (float(v) for v in out.target)
        o = tgt.ee_target.orientation
        o.w, o.x, o.y, o.z = self.quat
        tgt.gripper = float(out.gripper)
        tgt.engaged = bool(out.engaged)
        self.pub.publish(tgt)

        mk = Marker()
        mk.header = tgt.header
        mk.ns, mk.id, mk.type, mk.action = "teleop_target", 0, Marker.SPHERE, Marker.ADD
        mk.pose = tgt.ee_target
        mk.scale.x = mk.scale.y = mk.scale.z = 0.03
        mk.color.a = 0.8
        mk.color.g = 1.0 if out.engaged else 0.3
        mk.color.r = 0.0 if out.engaged else 1.0
        self.pub_marker.publish(mk)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopMapperNode()
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
