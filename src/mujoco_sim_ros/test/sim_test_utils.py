"""Helpers shared by the mujoco_sim_ros launch tests."""

import time

import rclpy
from rclpy.node import Node


def stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def spin_for(node: Node, seconds: float) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


def spin_until(node: Node, predicate, timeout: float) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        rclpy.spin_once(node, timeout_sec=0.01)
    return predicate()
