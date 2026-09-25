"""Shared launch description and helpers for the diff_ik_controller launch tests."""

import time

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy

from diff_ik_controller import node_parameters

REMAP = [("/clock", "/sim/clock")]


def sim_and_ik_description():
    sim = launch_ros.actions.Node(package="mujoco_sim_ros", executable="mujoco_sim",
                                  parameters=[{"render": False}], output="screen")
    ik = launch_ros.actions.Node(
        package="diff_ik_controller", executable="diff_ik_node", output="screen",
        parameters=[node_parameters(), {"use_sim_time": True}], remappings=REMAP)
    return launch.LaunchDescription([sim, ik, launch_testing.actions.ReadyToTest()])


def make_node(name):
    return rclpy.create_node(name, cli_args=["--ros-args", "-p", "use_sim_time:=true", "-r",
                                             "/clock:=/sim/clock"])


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def spin_for(node, seconds, on_tick=None, period=None):
    end = time.monotonic() + seconds
    next_tick = time.monotonic()
    while time.monotonic() < end:
        if on_tick is not None and time.monotonic() >= next_tick:
            on_tick()
            next_tick += period
        rclpy.spin_once(node, timeout_sec=0.002)


def spin_until(node, predicate, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end and not predicate():
        rclpy.spin_once(node, timeout_sec=0.01)
    return predicate()
