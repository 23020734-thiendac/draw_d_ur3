#!/usr/bin/env python3
"""Fail clearly on stale simulations; wait for measured state before RViz starts."""

import argparse
import math
import os
import time
import xml.etree.ElementTree as ET

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def preflight(node):
    # Discovery takes wall time even if an old simulation has stopped /clock.
    deadline = time.monotonic() + 3.0
    conflicts = set()
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        conflicts.update(
            name for name, namespace in node.get_node_names_and_namespaces()
            if namespace == "/" and name in {
                "controller_manager", "robot_state_publisher", "move_group", "ros_gz_bridge"
            }
        )
    if conflicts:
        node.get_logger().error(
            "Existing simulation nodes in ROS_DOMAIN_ID="
            + os.environ.get("ROS_DOMAIN_ID", "0") + ": " + ", ".join(sorted(conflicts))
            + ". Stop the previous launch and its leftover Gazebo server first, "
            "or use a different ROS_DOMAIN_ID in ALL demo terminals. "
            "Refusing to create duplicate controllers, /joint_states and /clock."
        )
        return 1
    return 0


class SimulationReady(Node):
    def __init__(self):
        super().__init__("ur3_d_simulation_ready")
        self.clock = None
        self.clock_advanced_at = -math.inf
        self.joints = None
        self.joints_received_at = -math.inf
        self.links = set()
        self.required_joints = set()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        self.create_subscription(
            JointState, "/joint_states", self.on_joints, qos_profile_sensor_data)
        self.create_subscription(
            String, "/robot_description", self.on_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.controllers = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.pending = None
        self.active_controllers = set()
        self.next_controller_query = 0.0

    def on_clock(self, msg):
        value = seconds(msg.clock)
        if self.clock is not None and value > self.clock:
            self.clock_advanced_at = time.monotonic()
        self.clock = value

    def on_joints(self, msg):
        self.joints = msg
        self.joints_received_at = time.monotonic()

    def on_description(self, msg):
        root = ET.fromstring(msg.data)
        self.links = {link.attrib["name"] for link in root.findall("link")}
        self.required_joints = {
            joint.attrib["name"] for joint in root.findall("joint")
            if joint.attrib["type"] in {"revolute", "continuous", "prismatic"}
            and joint.find("mimic") is None
        }

    def missing(self):
        now = time.monotonic()
        if self.pending is not None and self.pending.done():
            response = self.pending.result()
            self.active_controllers = {
                controller.name for controller in response.controller if controller.state == "active"
            }
            self.pending = None
        if self.pending is None and now >= self.next_controller_query:
            if self.controllers.service_is_ready():
                self.pending = self.controllers.call_async(ListControllers.Request())
            self.next_controller_query = now + 1.0

        missing = []
        if now - self.clock_advanced_at > 2.0:
            missing.append("advancing /clock (Gazebo must be running, not paused)")
        if not self.links:
            missing.append("/robot_description")
        positions = {} if self.joints is None else dict(zip(self.joints.name, self.joints.position))
        if (now - self.joints_received_at > 2.0
                or not self.required_joints
                or any(not math.isfinite(positions.get(joint, math.nan))
                       for joint in self.required_joints)):
            missing.append("fresh /joint_states for every robot joint")
        elif self.clock is None or abs(seconds(self.joints.header.stamp) - self.clock) > 1.0:
            missing.append("/joint_states timestamps matching /clock")
        for controller in ("joint_state_broadcaster", "joint_trajectory_controller"):
            if controller not in self.active_controllers:
                missing.append(controller + " active")
        bad_links = sorted(
            link for link in self.links if not self.buffer.can_transform("world", link, Time()))
        if bad_links:
            missing.append("TF world -> " + ", ".join(bad_links))
        return missing


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preflight", action="store_true")
    parser.add_argument("--timeout", type=float, default=90.0)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Node("ur3_d_preflight") if args.preflight else SimulationReady()
    try:
        if args.preflight:
            return preflight(node)
        deadline = time.monotonic() + args.timeout
        next_log = 0.0
        missing = ["simulation data"]
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            missing = node.missing()
            if not missing:
                node.get_logger().info(
                    "Simulation ready: clock advancing, both controllers active, "
                    "joint states current and all robot links connected to world.")
                return 0
            if time.monotonic() >= next_log:
                node.get_logger().info("Waiting for: " + "; ".join(missing))
                next_log = time.monotonic() + 5.0
        node.get_logger().error("Simulation startup timed out: " + "; ".join(missing))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
