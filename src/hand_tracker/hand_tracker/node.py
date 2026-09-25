"""``hand_tracker`` node: webcam (or video file) -> MediaPipe -> /hand/state + debug image.

    ros2 run hand_tracker hand_tracker --ros-args -p video:=clip.mp4

Capture and inference run sequentially in one thread with a 1-frame camera buffer, so every
published state belongs to the freshest frame. ``capture_stamp`` is the node clock (sim time
when ``use_sim_time`` is set) at frame read and is copied unchanged downstream.
"""

from __future__ import annotations

import array
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image

from hand_teleop_msgs.msg import Hand, HandState
from hand_tracker import overlay
from hand_tracker.tracker import HandTracker
from ur5e_2f85_mujoco.config import find_config_dir, load_config


def open_source(video: str, cam: dict, fps: float) -> cv2.VideoCapture:
    if video:
        cap = cv2.VideoCapture(video)
    else:
        cap = cv2.VideoCapture(int(cam["index"]), cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*cam["fourcc"]))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam["width"])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam["height"])
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, cam["buffer_size"])
    if not cap.isOpened():
        raise RuntimeError(f"cannot open {'video ' + video if video else 'camera'}")
    return cap


def to_hand_msg(hand, raw: bool = False) -> Hand:
    msg = Hand()
    msg.present = bool(hand.present)
    lm = hand.raw_landmarks if raw else hand.landmarks
    msg.landmarks = [float(v) for v in np.asarray(lm).reshape(-1)]
    msg.palm_scale = float(hand.raw_palm_scale if raw else hand.palm_scale)
    msg.pinch = float(hand.raw_pinch if raw else hand.pinch)
    msg.is_open, msg.is_fist = bool(hand.is_open), bool(hand.is_fist)
    return msg


class HandTrackerNode(Node):
    def __init__(self, **kwargs):
        super().__init__("hand_tracker", **kwargs)
        self.declare_parameter("config_dir", "")
        self.declare_parameter("video", "")
        self.declare_parameter("loop_video", False)
        self.declare_parameter("publish_debug_image", True)
        cfg_dir = find_config_dir(self.get_parameter("config_dir").value or None)
        self.cfg = load_config("filters.yaml", cfg_dir)
        self.rates = load_config("rates.yaml", cfg_dir)
        ws = load_config("workspace.yaml", cfg_dir)
        self.pinch_range = (ws["gripper"]["pinch_closed"], ws["gripper"]["pinch_open"])
        self.video = self.get_parameter("video").value
        self.cap = open_source(self.video, self.cfg["camera"], self.rates["webcam"])
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.tracker = HandTracker(self.cfg, w, h)
        self.publish_raw = bool(self.cfg.get("publish_raw", False))
        self.debug_image = bool(self.get_parameter("publish_debug_image").value)

        self.pub = self.create_publisher(HandState, "hand/state", 10)
        self.pub_raw = self.create_publisher(HandState, "hand/state_raw", 10) \
            if self.publish_raw else None
        self.pub_img = self.create_publisher(Image, "hand/debug_image", 2)
        self.sim_time = None
        # /clock (remapped to /sim/clock) is published best-effort.
        self.create_subscription(Clock, "/clock", self._on_clock, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.latency_ms = deque(maxlen=300)
        self.fps = 0.0
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        self.get_logger().info(
            f"hand_tracker: {'video ' + self.video if self.video else 'webcam'} {w}x{h}")

    def _on_clock(self, msg: Clock) -> None:
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _read(self):
        ok, frame = self.cap.read()
        if not ok and self.video and self.get_parameter("loop_video").value:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        return ok, frame

    def _loop(self) -> None:
        last_wall = None
        n_frames = 0
        file_fps = self.cap.get(cv2.CAP_PROP_FPS) or self.rates["webcam"]
        wall_t0 = time.monotonic()
        while self.running and rclpy.ok():
            ok, frame = self._read()
            if not ok:
                self.get_logger().info("video finished" if self.video else "camera read failed")
                break
            if self.video:
                # Continuous file time (frame count / fps) drives filters and MediaPipe, also
                # across loops; pace playback in real time.
                t = n_frames / file_fps
                n_frames += 1
                delay = t - (time.monotonic() - wall_t0)
                if delay > 0:
                    time.sleep(delay)  # the frame "arrives" now
            else:
                t = time.monotonic()
            t_read = time.perf_counter()
            capture_stamp = self.get_clock().now().to_msg()
            frame = cv2.flip(frame, 1)  # mirror: the image behaves like a mirror
            hands = self.tracker.process(frame, t)

            now = time.monotonic()
            if last_wall is not None:
                inst = 1.0 / max(now - last_wall, 1e-6)
                self.fps = inst if self.fps == 0.0 else 0.9 * self.fps + 0.1 * inst
            last_wall = now

            msg = HandState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"
            msg.capture_stamp = capture_stamp
            msg.left, msg.right = to_hand_msg(hands["left"]), to_hand_msg(hands["right"])
            msg.fps = float(self.fps)
            self.pub.publish(msg)
            self.latency_ms.append((time.perf_counter() - t_read) * 1e3)
            if self.pub_raw is not None:
                raw = HandState(header=msg.header, capture_stamp=capture_stamp, fps=msg.fps)
                raw.left = to_hand_msg(hands["left"], raw=True)
                raw.right = to_hand_msg(hands["right"], raw=True)
                self.pub_raw.publish(raw)
            if self.debug_image and self.pub_img.get_subscription_count() > 0:
                img = overlay.draw(frame, hands, self.fps, self.latency_ms[-1], self.sim_time,
                                   self.pinch_range)
                im = Image()
                im.header = msg.header
                im.height, im.width = img.shape[:2]
                im.encoding, im.step = "bgr8", img.shape[1] * 3
                im.data = array.array("B", img.tobytes())
                self.pub_img.publish(im)
            if len(self.latency_ms) == self.latency_ms.maxlen:
                lat = np.array(self.latency_ms)
                self.get_logger().info(
                    f"{self.fps:.1f} fps, frame->publish p50 {np.percentile(lat, 50):.1f} ms "
                    f"p95 {np.percentile(lat, 95):.1f} ms")
                self.latency_ms.clear()

    def destroy_node(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.cap.release()
        self.tracker.close()
        super().destroy_node()


def main(args=None) -> None:
    import argparse
    import sys

    from rclpy.parameter import Parameter

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--video", default=None, help="offline mode: video file instead of webcam")
    known, rest = parser.parse_known_args(sys.argv[1:] if args is None else args)
    rclpy.init(args=[sys.argv[0]] + rest)
    overrides = [Parameter("video", value=known.video)] if known.video else []
    node = HandTrackerNode(parameter_overrides=overrides)
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
