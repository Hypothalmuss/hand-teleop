"""``hand_tracker`` node: webcam (or video file) -> MediaPipe -> /hand/state + debug image.

    ros2 run hand_tracker hand_tracker --ros-args -p video:=clip.mp4

Capture and inference run sequentially in one thread with a 1-frame camera buffer, so every
published state belongs to the freshest frame. ``capture_stamp`` is the node clock (sim time
when ``use_sim_time`` is set) at frame read and is copied unchanged downstream.
"""

from __future__ import annotations

import array
import os
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image

from hand_teleop_msgs.msg import Hand, HandState
from hand_tracker import overlay
from hand_tracker.camera import open_camera
from hand_tracker.tracker import HandTracker
from ur5e_2f85_mujoco.config import find_config_dir, load_config

WINDOW = "hand_tracker"
MAX_FRAME_AGE_S = 0.5  # larger means the driver gives no usable buffer timestamp


def open_source(video: str, cam: dict, fps: float, log=print):
    """Returns (VideoCapture, width, height) for a video file or the webcam."""
    if not video:
        return open_camera(cam, fps, log)
    cap = cv2.VideoCapture(video)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open video {video}")
    return cap, int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))


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
        self.declare_parameter("show_window", True)  # local camera window with the overlay
        cfg_dir = find_config_dir(self.get_parameter("config_dir").value or None)
        self.cfg = load_config("filters.yaml", cfg_dir)
        self.rates = load_config("rates.yaml", cfg_dir)
        ws = load_config("workspace.yaml", cfg_dir)
        self.pinch_range = (ws["gripper"]["pinch_closed"], ws["gripper"]["pinch_open"])
        self.video = self.get_parameter("video").value
        self.cap, w, h = open_source(self.video, self.cfg["camera"], self.rates["webcam"],
                                     self.get_logger().info)
        self.tracker = HandTracker(self.cfg, w, h)
        self.publish_raw = bool(self.cfg.get("publish_raw", False))
        self.debug_image = bool(self.get_parameter("publish_debug_image").value)
        disp = self.cfg["display"]
        self.show_window = bool(self.get_parameter("show_window").value and disp["show_window"])
        if self.show_window and not os.environ.get("DISPLAY"):
            self.get_logger().warn("no DISPLAY: camera window disabled")
            self.show_window = False
        self.view_width = int(disp["view_width"])  # overlay / window / debug image width

        self.pub = self.create_publisher(HandState, "hand/state", 10)
        self.pub_raw = self.create_publisher(HandState, "hand/state_raw", 10) \
            if self.publish_raw else None
        self.pub_img = self.create_publisher(Image, "hand/debug_image", 2)
        self.sim_time = None
        # /clock (remapped to /sim/clock) is published best-effort.
        self.create_subscription(Clock, "/clock", self._on_clock, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        self._window_open = False
        self.latency_ms = deque(maxlen=300)
        self.frame_age_ms = deque(maxlen=300)
        self.fps = 0.0
        self.running = True
        self.thread = threading.Thread(target=self._safe_loop, daemon=True)
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

    def _safe_loop(self) -> None:
        try:
            self._loop()
        except Exception as exc:  # noqa: BLE001
            if rclpy.ok() and self.running:
                raise
            del exc  # publisher/context gone during shutdown

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
            now = self.get_clock().now()
            age = 0.0
            if not self.video:
                # V4L2 buffer timestamp (CLOCK_MONOTONIC): exposure/USB/decode/queue time
                # before the read (~40 ms) belongs in the end-to-end latency.
                buf_t = self.cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
                if 0.0 <= t - buf_t < MAX_FRAME_AGE_S:
                    age, t = t - buf_t, buf_t
            self.frame_age_ms.append(age * 1e3)
            capture_stamp = (now - Duration(nanoseconds=int(age * 1e9))).to_msg()
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
            publish_img = self.debug_image and self.pub_img.get_subscription_count() > 0
            if publish_img or self.show_window:
                # Draw on a downscaled copy: cheap at 1080p, same look at any resolution.
                scale = self.view_width / frame.shape[1]
                view = cv2.resize(frame, (self.view_width, int(round(frame.shape[0] * scale))),
                                  interpolation=cv2.INTER_AREA) if scale < 1.0 else frame
                img = overlay.draw(view, hands, self.fps, self.latency_ms[-1], self.sim_time,
                                   self.pinch_range)
                if publish_img:
                    im = Image()
                    im.header = msg.header
                    im.height, im.width = img.shape[:2]
                    im.encoding, im.step = "bgr8", img.shape[1] * 3
                    im.data = array.array("B", img.tobytes())
                    self.pub_img.publish(im)
                if self.show_window:
                    self._show(img)
            if len(self.latency_ms) == self.latency_ms.maxlen:
                lat = np.array(self.latency_ms)
                self.get_logger().info(
                    f"{self.fps:.1f} fps, frame->publish p50 {np.percentile(lat, 50):.1f} ms "
                    f"p95 {np.percentile(lat, 95):.1f} ms, frame age at read p50 "
                    f"{np.percentile(self.frame_age_ms, 50):.1f} ms")
                self.latency_ms.clear()
                self.frame_age_ms.clear()

    def _show(self, img) -> None:
        if not self._window_open:
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
            cv2.resizeWindow(WINDOW, img.shape[1], img.shape[0])
            self._window_open = True
        elif cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
            self.show_window = False  # closed by the user: stop drawing it
            return
        cv2.imshow(WINDOW, img)
        cv2.waitKey(1)

    def destroy_node(self):
        self.running = False
        self.thread.join(timeout=1.0)
        if self._window_open:
            cv2.destroyAllWindows()
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
