"""Webcam opening shared by the tracker node and the operator scripts. No ROS.

``camera.width`` / ``camera.height`` may be ``max``: the widest mode in ``MAX_MODES`` that the
camera accepts and that measurably delivers ``min_fps`` is used.
"""

from __future__ import annotations

import time

import cv2

# Largest first; common MJPG webcam modes.
MAX_MODES = [(3840, 2160), (2560, 1440), (1920, 1080), (1600, 896), (1280, 720), (1024, 576),
             (800, 600), (640, 480)]
FPS_PROBE_FRAMES = 20


def _open(index: int, fourcc: str, w: int, h: int, fps: float, buffers: int):
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)
    # >= 2: with a single V4L2 buffer the driver drops every other frame while one is being
    # decoded (a 30 fps camera delivers 15 fps).
    cap.set(cv2.CAP_PROP_BUFFERSIZE, buffers)
    return cap


def measured_fps(cap, frames: int = FPS_PROBE_FRAMES) -> float:
    for _ in range(3):
        cap.read()
    t0 = time.perf_counter()
    for _ in range(frames):
        if not cap.read()[0]:
            return 0.0
    return frames / (time.perf_counter() - t0)


def open_camera(cam: dict, fps: float, log=print):
    """Returns (VideoCapture, width, height)."""
    index, fourcc = int(cam["index"]), str(cam["fourcc"])
    buffers = int(cam.get("buffer_size", 2))
    if str(cam["width"]) != "max":
        cap = _open(index, fourcc, int(cam["width"]), int(cam["height"]), fps, buffers)
        if not cap.isOpened():
            raise RuntimeError(f"cannot open camera {index}")
        return cap, int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    min_fps = float(cam.get("min_fps", 0.8 * fps))
    for w, h in MAX_MODES:
        cap = _open(index, fourcc, w, h, fps, buffers)
        if not cap.isOpened():
            raise RuntimeError(f"cannot open camera {index}")
        got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if got != (w, h):
            cap.release()
            continue
        rate = measured_fps(cap)
        if rate >= min_fps:
            log(f"camera {index}: {w}x{h} {fourcc} at {rate:.1f} fps")
            return cap, w, h
        log(f"camera {index}: {w}x{h} only {rate:.1f} fps, trying a smaller mode")
        cap.release()
    raise RuntimeError(f"camera {index}: no mode reaches {min_fps:.0f} fps")
