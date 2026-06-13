#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINTS = ("left_wheel_joint", "right_wheel_joint", "mower_joint")


class VescMotorTest(Node):
    def __init__(self, args):
        super().__init__("hw_vesc_motor_test")
        self.args = args
        self.last_joint_state = None
        self.last_odom = None
        self.cmd_pub = self.create_publisher(TwistStamped, args.cmd_vel_topic, 10)
        self.mower_pub = self.create_publisher(Float64MultiArray, args.mower_command_topic, 10)
        self.create_subscription(JointState, args.joint_states_topic, self.on_joint_state, 10)
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, 10)

    def on_joint_state(self, msg):
        self.last_joint_state = msg

    def on_odom(self, msg):
        self.last_odom = msg

    def wait_for_feedback(self):
        deadline = time.monotonic() + self.args.wait_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_joint_state is not None and self.last_odom is not None:
                return True
        return False

    def publish_drive(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.cmd_pub.publish(msg)

    def publish_mower(self, command):
        msg = Float64MultiArray()
        msg.data = [command]
        self.mower_pub.publish(msg)

    def stop_all(self):
        for _ in range(10):
            self.publish_drive(0.0, 0.0)
            self.publish_mower(0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    def run(self):
        if not self.args.armed:
            print("Refusing to move hardware without --armed.")
            print("Lift wheels/blade safely, clear the area, then rerun with --armed.")
            return 2

        print("Waiting for joint state and odometry feedback...")
        if not self.wait_for_feedback():
            print("Timed out waiting for /joint_states and odometry feedback.")
            return 1

        self.stop_all()
        sequence = self.build_sequence()
        try:
            for step in sequence:
                self.run_step(step)
            print("Hardware motor test completed.")
            return 0
        finally:
            self.stop_all()

    def build_sequence(self):
        requested = self.args.motor
        steps = []
        if requested in ("left", "all"):
            steps.append(("left wheel", "left", self.args.wheel_speed))
        if requested in ("right", "all"):
            steps.append(("right wheel", "right", self.args.wheel_speed))
        if requested in ("drive", "all"):
            steps.append(("both drive wheels", "drive", self.args.wheel_speed))
        if requested in ("mower", "all"):
            steps.append(("mower motor", "mower", self.args.mower_command))
        return steps

    def run_step(self, step):
        label, mode, command = step
        before = self.snapshot()
        print(f"\n=== Testing {label}: command={command:.4f} for {self.args.duration:.2f}s ===")
        deadline = time.monotonic() + self.args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            if mode == "left":
                linear, angular = left_only_twist(command, self.args.wheel_separation)
                self.publish_drive(linear, angular)
                self.publish_mower(0.0)
            elif mode == "right":
                linear, angular = right_only_twist(command, self.args.wheel_separation)
                self.publish_drive(linear, angular)
                self.publish_mower(0.0)
            elif mode == "drive":
                self.publish_drive(command, 0.0)
                self.publish_mower(0.0)
            elif mode == "mower":
                self.publish_drive(0.0, 0.0)
                self.publish_mower(command)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop_all()
        time.sleep(self.args.settle)
        self.spin_some(self.args.settle)
        after = self.snapshot()
        self.print_delta(before, after)

    def spin_some(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def snapshot(self):
        joints = {}
        if self.last_joint_state is not None:
            by_name = {
                name: index for index, name in enumerate(self.last_joint_state.name)
            }
            for joint in JOINTS:
                index = by_name.get(joint)
                if index is None:
                    joints[joint] = (math.nan, math.nan, math.nan)
                    continue
                joints[joint] = (
                    value_at(self.last_joint_state.position, index),
                    value_at(self.last_joint_state.velocity, index),
                    value_at(self.last_joint_state.effort, index),
                )

        odom = None
        if self.last_odom is not None:
            pose = self.last_odom.pose.pose
            twist = self.last_odom.twist.twist
            odom = (
                pose.position.x,
                pose.position.y,
                twist.linear.x,
                twist.angular.z,
            )
        return {"joints": joints, "odom": odom}

    def print_delta(self, before, after):
        print("joint deltas:")
        for joint in JOINTS:
            before_pos = before["joints"].get(joint, (math.nan,))[0]
            after_pos = after["joints"].get(joint, (math.nan,))[0]
            velocity = after["joints"].get(joint, (math.nan, math.nan))[1]
            if math.isnan(before_pos) or math.isnan(after_pos):
                print(f"  {joint}: missing")
            else:
                print(
                    f"  {joint}: dpos={after_pos - before_pos:+.4f} rad "
                    f"last_velocity={fmt(velocity)} rad/s"
                )

        if before["odom"] is not None and after["odom"] is not None:
            bx, by, _, _ = before["odom"]
            ax, ay, vx, wz = after["odom"]
            print(
                "odom delta: "
                f"dx={ax - bx:+.4f} m dy={ay - by:+.4f} m "
                f"last_linear.x={vx:+.4f} m/s last_angular.z={wz:+.4f} rad/s"
            )
        else:
            print("odom delta: missing")


def left_only_twist(left_speed, wheel_separation):
    linear = left_speed / 2.0
    angular = -left_speed / wheel_separation
    return linear, angular


def right_only_twist(right_speed, wheel_separation):
    linear = right_speed / 2.0
    angular = right_speed / wheel_separation
    return linear, angular


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
        description="Move VESC-backed motors briefly and report joint/odom feedback."
    )
    parser.add_argument(
        "--armed",
        action="store_true",
        help="Required safety acknowledgement before any motor command is sent.",
    )
    parser.add_argument(
        "--motor",
        choices=("left", "right", "drive", "mower", "all"),
        default="all",
        help="Which motor group to test.",
    )
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--settle", type=float, default=0.25)
    parser.add_argument("--wait-timeout", type=float, default=5.0)
    parser.add_argument("--wheel-speed", type=float, default=0.08, help="Wheel surface speed in m/s.")
    parser.add_argument("--wheel-separation", type=float, default=0.32)
    parser.add_argument(
        "--mower-command",
        type=float,
        default=0.20,
        help="Command sent to mower_controller; current hardware maps it as velocity.",
    )
    parser.add_argument("--cmd-vel-topic", default="/diff_drive_base_controller/cmd_vel")
    parser.add_argument("--mower-command-topic", default="/mower_controller/commands")
    parser.add_argument("--joint-states-topic", default="/joint_states")
    parser.add_argument("--odom-topic", default="/diff_drive_base_controller/odom")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = VescMotorTest(args)
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
