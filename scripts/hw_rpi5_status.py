#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Temperature


class Rpi5Status(Node):
    def __init__(self, args):
        super().__init__("hw_rpi5_status")
        self.args = args
        self.last_temperature = None
        self.temperature_count = 0
        self.create_subscription(
            Temperature,
            args.temperature_topic,
            self.on_temperature,
            10,
        )

    def on_temperature(self, msg):
        self.last_temperature = msg
        self.temperature_count += 1

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def print_status(self):
        print("=== Raspberry Pi 5 hardware status ===")
        print(
            f"temperature topic: {self.args.temperature_topic} "
            f"({self.temperature_count} msg)"
        )

        if self.last_temperature is None:
            print("cpu temperature: no messages received")
            return 1

        msg = self.last_temperature
        age = self.now() - Time.from_msg(msg.header.stamp)
        print(
            f"cpu temperature: {fmt(msg.temperature)} C "
            f"frame={msg.header.frame_id or 'unset'} "
            f"age={age.nanoseconds / 1e9:.3f} s"
        )
        if not math.isnan(msg.variance):
            print(f"variance: {fmt(msg.variance)}")
        return 0


def fmt(value):
    if math.isnan(value):
        return "nan"
    return f"{value:.3f}"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read Raspberry Pi 5 hardware diagnostic telemetry."
    )
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument(
        "--temperature-topic",
        default="/hardware/raspi/cpu_temperature",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = Rpi5Status(args)
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
