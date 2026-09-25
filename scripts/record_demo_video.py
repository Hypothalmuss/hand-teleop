#!/usr/bin/env python3
"""Record a split-screen demo video: webcam overlay (left) | simulator view (right).

    ros2 launch hand_teleop_bringup teleop.launch.py demo_view:=true &
    python3 scripts/record_demo_video.py --seconds 60 --out results/demo_v1.mp4

Left: /hand/debug_image (skeleton, fps, latency, wall and sim clocks). Right: /sim/demo/image
(640x480 free camera, enabled by demo_view:=true). Frames are composed at 30 fps from the
latest image of each stream and encoded with ffmpeg (H.264).
"""

import argparse
import subprocess
import time

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image

W, H, FPS = 640, 480, 30


def to_bgr(msg: Image) -> np.ndarray:
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
    img = img if msg.encoding == "bgr8" else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return cv2.resize(img, (W, H)) if img.shape[:2] != (H, W) else img


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--out", default="results/demo_v1.mp4")
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node("record_demo_video")
    latest = {"hand": None, "sim": None}
    node.create_subscription(Image, "/hand/debug_image",
                             lambda m: latest.__setitem__("hand", m), 2)
    node.create_subscription(Image, "/sim/demo/image", lambda m: latest.__setitem__("sim", m), 2)
    ff = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s",
         f"{2 * W}x{H}", "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p",
         "-crf", "20", args.out], stdin=subprocess.PIPE)
    blank = np.zeros((H, W, 3), np.uint8)
    print(f"recording {args.seconds:.0f} s to {args.out} (Ctrl-C to stop early)")
    t0 = time.monotonic()
    next_frame = t0
    n = 0
    try:
        while time.monotonic() - t0 < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.002)
            if time.monotonic() < next_frame:
                continue
            next_frame += 1.0 / FPS
            left = to_bgr(latest["hand"]) if latest["hand"] else blank
            right = to_bgr(latest["sim"]) if latest["sim"] else blank
            ff.stdin.write(np.hstack([left, right]).tobytes())
            n += 1
    except KeyboardInterrupt:
        pass
    ff.stdin.close()
    ff.wait()
    node.destroy_node()
    rclpy.shutdown()
    print(f"wrote {n} frames ({n / FPS:.1f} s) to {args.out}")


if __name__ == "__main__":
    main()
