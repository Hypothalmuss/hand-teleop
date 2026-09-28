"""``keyboard_console``: ESC e-stop (latched), c clear, r reset the scene (next seed), q quit.

Needs a terminal: run it in its own terminal, or launch with ``console:=terminal``.
"""

from __future__ import annotations

import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from hand_teleop_msgs.msg import TeleopTarget
from hand_teleop_msgs.srv import ResetScene
from teleop_console.console_state import ConsoleState
from ur5e_2f85_mujoco.config import find_config_dir
from ur5e_2f85_mujoco.task import load_seeds

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
HELP = "ESC e-stop | c clear e-stop | r reset scene | q quit"


class KeyboardConsole(Node):
    def __init__(self):
        super().__init__("keyboard_console")
        self.declare_parameter("config_dir", "")
        self.declare_parameter("randomize_target", False)
        cfg = find_config_dir(self.get_parameter("config_dir").value or None)
        self.state = ConsoleState(load_seeds(cfg / "seeds.txt"))
        self.pub_estop = self.create_publisher(Bool, "estop", LATCHED)
        self.clear_cli = self.create_client(Trigger, "estop/clear")
        self.reset_cli = self.create_client(ResetScene, "sim/reset")
        self.create_subscription(TeleopTarget, "teleop/target", self._on_target, 10)
        self.create_timer(0.1, self._print)

    def _on_target(self, msg: TeleopTarget) -> None:
        self.state.engaged = msg.engaged

    def _print(self) -> None:
        sys.stdout.write("\r\033[K" + self.state.status_line())
        sys.stdout.flush()

    def _call(self, cli, req, on_done) -> None:
        if not cli.service_is_ready():
            self.state.message = f"{cli.srv_name} not available"
            return
        cli.call_async(req).add_done_callback(lambda f: on_done(f.result()))

    def key(self, k: str) -> None:
        action = self.state.handle(k)
        if action is None:
            return
        name, args = action
        st = self.state
        if name == "estop":
            self.pub_estop.publish(Bool(data=True))
            st.estop, st.message = True, "E-STOP latched (c to clear)"
        elif name == "clear":
            def done(res):
                if res.success:
                    self.pub_estop.publish(Bool(data=False))
                    st.estop = False
                st.message = res.message
            self._call(self.clear_cli, Trigger.Request(), done)
        elif name == "reset":
            req = ResetScene.Request(
                seed=args["seed"], randomize_target=self.get_parameter("randomize_target").value)
            self._call(self.reset_cli, req, lambda res: setattr(
                st, "message", f"reset: cube ({res.cube.position.x:.3f}, "
                               f"{res.cube.position.y:.3f})"))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardConsole()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    if not sys.stdin.isatty():
        node.get_logger().error("keyboard_console needs a terminal (run it in its own terminal)")
        rclpy.shutdown()
        return
    print(HELP)
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                k = sys.stdin.read(1)
                if k == "q":
                    break
                if k == "\x1b":  # a lone ESC; drain arrow-key escape sequences
                    while select.select([sys.stdin], [], [], 0.01)[0]:
                        k = None
                        sys.stdin.read(1)
                    if k is None:
                        continue
                node.key(k)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        print()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
