#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINTS = ("left_wheel_joint", "right_wheel_joint", "mower_joint")


class VescStatus(Node):
    def __init__(self, args):
        super().__init__("hw_vesc_status")
        self.args = args
        self.last_joint_state = None
        self.last_odom = None
        self.joint_state_count = 0
        self.odom_count = 0
        self.create_subscription(JointState, args.joint_states_topic, self.on_joint_state, 10)
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, 10)

    def on_joint_state(self, msg):
        self.last_joint_state = msg
        self.joint_state_count += 1

    def on_odom(self, msg):
        self.last_odom = msg
        self.odom_count += 1

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def print_status(self):
        print("=== VESC hardware status ===")
        print(f"joint_states topic: {self.args.joint_states_topic} ({self.joint_state_count} msg)")
        print(f"odom topic:         {self.args.odom_topic} ({self.odom_count} msg)")

        if self.last_joint_state is None:
            print("joint_states: no messages received")
        else:
            by_name = {
                name: index for index, name in enumerate(self.last_joint_state.name)
            }
            print("joint_states:")
            for joint in JOINTS:
                index = by_name.get(joint)
                if index is None:
                    print(f"  {joint}: missing")
                    continue
                position = value_at(self.last_joint_state.position, index)
                velocity = value_at(self.last_joint_state.velocity, index)
                effort = value_at(self.last_joint_state.effort, index)
                print(
                    f"  {joint}: position={fmt(position)} rad "
                    f"velocity={fmt(velocity)} rad/s effort={fmt(effort)}"
                )

        if self.last_odom is None:
            print("odom: no messages received")
        else:
            twist = self.last_odom.twist.twist
            pose = self.last_odom.pose.pose
            print(
                "odom: "
                f"x={pose.position.x:.4f} y={pose.position.y:.4f} "
                f"linear.x={twist.linear.x:.4f} m/s angular.z={twist.angular.z:.4f} rad/s"
            )

        missing = self.last_joint_state is None or self.last_odom is None
        return 1 if missing else 0


def value_at(values, index):
    if index >= len(values):
        return math.nan
    return values[index]


def fmt(value):
    if math.isnan(value):
        return "nan"
    return f"{value:.4f}"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read VESC-backed joint states and diff-drive odometry."
    )
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--joint-states-topic", default="/joint_states")
    parser.add_argument("--odom-topic", default="/diff_drive_base_controller/odom")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = VescStatus(args)
    try:
        node.spin_for(args.duration)
        exit_code = node.print_status()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
