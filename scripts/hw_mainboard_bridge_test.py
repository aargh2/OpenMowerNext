#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool, Trigger


class MainboardBridgeTest(Node):
    def __init__(self, args):
        super().__init__("hw_mainboard_bridge_test")
        self.args = args
        self.battery_count = 0
        self.imu_count = 0
        self.emergency_count = 0
        self.rain_count = 0
        self.charger_present_count = 0
        self.charge_voltage_count = 0
        self.last_battery = None
        self.last_imu = None
        self.last_emergency = None
        self.last_rain = None
        self.last_charger_present = None
        self.last_charge_voltage = None

        self.create_subscription(BatteryState, args.battery_topic, self.on_battery, 10)
        self.create_subscription(Imu, args.imu_topic, self.on_imu, 50)
        self.create_subscription(Bool, args.emergency_topic, self.on_emergency, 10)
        self.create_subscription(Bool, args.rain_topic, self.on_rain, 10)
        self.create_subscription(Bool, args.charger_present_topic, self.on_charger_present, 10)
        self.create_subscription(Float32, args.charge_voltage_topic, self.on_charge_voltage, 10)

    def on_battery(self, msg):
        self.battery_count += 1
        self.last_battery = msg

    def on_imu(self, msg):
        self.imu_count += 1
        self.last_imu = msg

    def on_emergency(self, msg):
        self.emergency_count += 1
        self.last_emergency = msg

    def on_rain(self, msg):
        self.rain_count += 1
        self.last_rain = msg

    def on_charger_present(self, msg):
        self.charger_present_count += 1
        self.last_charger_present = msg

    def on_charge_voltage(self, msg):
        self.charge_voltage_count += 1
        self.last_charge_voltage = msg

    def run(self):
        start = time.monotonic()
        deadline = start + self.args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        service_results = {}
        if self.args.exercise_clear_emergency:
            service_results["clear_emergency"] = self.call_clear_emergency()
        if self.args.exercise_set_emergency:
            service_results["set_emergency_false"] = self.call_set_emergency(False)

        elapsed = max(time.monotonic() - start, 0.001)
        self.print_report(elapsed, service_results)
        return self.exit_code(service_results)

    def call_clear_emergency(self):
        client = self.create_client(Trigger, self.args.clear_emergency_service)
        if not client.wait_for_service(timeout_sec=self.args.service_timeout):
            self.destroy_client(client)
            return (False, "service unavailable")

        future = client.call_async(Trigger.Request())
        result = self.wait_for_future(future)
        self.destroy_client(client)
        if result is None:
            return (False, "service call timed out")
        return (bool(result.success), result.message)

    def call_set_emergency(self, value):
        client = self.create_client(SetBool, self.args.set_emergency_service)
        if not client.wait_for_service(timeout_sec=self.args.service_timeout):
            self.destroy_client(client)
            return (False, "service unavailable")

        request = SetBool.Request()
        request.data = value
        future = client.call_async(request)
        result = self.wait_for_future(future)
        self.destroy_client(client)
        if result is None:
            return (False, "service call timed out")
        return (bool(result.success), result.message)

    def wait_for_future(self, future):
        deadline = time.monotonic() + self.args.service_timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return None
        return future.result()

    def print_report(self, elapsed, service_results):
        nodes = sorted(name for name, _ in self.get_node_names_and_namespaces())
        params = self.get_params(
            self.args.bridge_node,
            (
                "serial_port",
                "baudrate",
                "reconnect_period_s",
                "heartbeat_rate_hz",
                "charger_present_voltage_threshold",
                "imu_frame_id",
                "battery_frame_id",
            ),
        )

        print("=== Mainboard hardware bridge test ===")
        print("transport: mainboard_serial_bridge")
        print(f"duration: {elapsed:.2f}s")
        print()
        print("nodes:")
        print(f"  {self.args.bridge_node}: {present_node(self.args.bridge_node, nodes)}")
        print()
        print("bridge parameters:")
        if params:
            for key in (
                "serial_port",
                "baudrate",
                "reconnect_period_s",
                "heartbeat_rate_hz",
                "charger_present_voltage_threshold",
                "imu_frame_id",
                "battery_frame_id",
            ):
                print(f"  {key}: {params.get(key, '<unset>')}")
        else:
            print("  unavailable")

        print()
        print("topics:")
        self.print_topic(self.args.battery_topic, self.battery_count, elapsed)
        self.print_topic(self.args.imu_topic, self.imu_count, elapsed)
        self.print_topic(self.args.emergency_topic, self.emergency_count, elapsed)
        self.print_topic(self.args.rain_topic, self.rain_count, elapsed)
        self.print_topic(self.args.charger_present_topic, self.charger_present_count, elapsed)
        self.print_topic(self.args.charge_voltage_topic, self.charge_voltage_count, elapsed)

        print()
        self.print_battery()
        self.print_imu()
        self.print_bool("emergency", self.last_emergency)
        self.print_bool("rain", self.last_rain)
        self.print_bool("charger_present", self.last_charger_present)
        self.print_float("charge_voltage", self.last_charge_voltage)

        if service_results:
            print()
            print("service checks:")
            for name, (success, message) in service_results.items():
                status = "ok" if success else "failed"
                print(f"  {name}: {status} ({message})")

    def get_params(self, node_name, names):
        client = self.create_client(GetParameters, f"{node_name}/get_parameters")
        if not client.wait_for_service(timeout_sec=self.args.service_timeout):
            self.destroy_client(client)
            return {}

        request = GetParameters.Request()
        request.names = list(names)
        future = client.call_async(request)
        result = self.wait_for_future(future)
        self.destroy_client(client)
        if result is None:
            return {}

        return {
            name: parameter_value_to_python(value)
            for name, value in zip(names, result.values)
        }

    @staticmethod
    def print_topic(name, count, elapsed):
        print(f"  {name}: {count} msg ({count / elapsed:.2f} Hz)")

    def print_battery(self):
        if self.last_battery is None:
            print("battery: missing")
            return
        msg = self.last_battery
        print(
            "battery: "
            f"voltage={fmt(msg.voltage)} V current={fmt(msg.current)} A "
            f"percentage={fmt(msg.percentage * 100.0)}% "
            f"status={msg.power_supply_status}"
        )

    def print_imu(self):
        if self.last_imu is None:
            print("imu: missing")
            return
        msg = self.last_imu
        print(
            "imu: "
            f"frame={msg.header.frame_id or '<empty>'} "
            f"accel=({fmt(msg.linear_acceleration.x)}, "
            f"{fmt(msg.linear_acceleration.y)}, {fmt(msg.linear_acceleration.z)}) m/s^2 "
            f"gyro=({fmt(msg.angular_velocity.x)}, "
            f"{fmt(msg.angular_velocity.y)}, {fmt(msg.angular_velocity.z)}) rad/s"
        )

    @staticmethod
    def print_bool(label, msg):
        if msg is None:
            print(f"{label}: missing")
        else:
            print(f"{label}: {msg.data}")

    @staticmethod
    def print_float(label, msg):
        if msg is None:
            print(f"{label}: missing")
        else:
            print(f"{label}: {fmt(msg.data)}")

    def exit_code(self, service_results):
        if self.args.require_battery and self.battery_count == 0:
            return 1
        if self.args.require_imu and self.imu_count == 0:
            return 2
        if self.args.require_status and (
            self.emergency_count == 0
            or self.rain_count == 0
            or self.charger_present_count == 0
            or self.charge_voltage_count == 0
        ):
            return 3
        if any(not success for success, _ in service_results.values()):
            return 4
        return 0


def parameter_value_to_python(value):
    if value.type == 1:
        return value.bool_value
    if value.type == 2:
        return value.integer_value
    if value.type == 3:
        return value.double_value
    if value.type == 4:
        return value.string_value
    return "<unset>"


def present_node(name, nodes):
    normalized = name[1:] if name.startswith("/") else name
    return "yes" if normalized in nodes else "no"


def fmt(value):
    if math.isnan(value):
        return "nan"
    return f"{value:.4f}"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Check communication through mainboard_serial_bridge."
    )
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--service-timeout", type=float, default=1.0)
    parser.add_argument("--bridge-node", default="/mainboard_serial_bridge")
    parser.add_argument("--battery-topic", default="/power")
    parser.add_argument("--imu-topic", default="/imu/data_raw")
    parser.add_argument("--emergency-topic", default="/hardware/emergency")
    parser.add_argument("--rain-topic", default="/hardware/rain")
    parser.add_argument("--charger-present-topic", default="/power/charger_present")
    parser.add_argument("--charge-voltage-topic", default="/power/charge_voltage")
    parser.add_argument("--set-emergency-service", default="/hardware/set_emergency")
    parser.add_argument("--clear-emergency-service", default="/hardware/clear_emergency")
    parser.add_argument("--require-battery", action="store_true", default=True)
    parser.add_argument("--no-require-battery", dest="require_battery", action="store_false")
    parser.add_argument("--require-imu", action="store_true", default=True)
    parser.add_argument("--no-require-imu", dest="require_imu", action="store_false")
    parser.add_argument("--require-status", action="store_true", default=True)
    parser.add_argument("--no-require-status", dest="require_status", action="store_false")
    parser.add_argument(
        "--exercise-clear-emergency",
        action="store_true",
        help="Call /hardware/clear_emergency to verify service communication.",
    )
    parser.add_argument(
        "--exercise-set-emergency",
        action="store_true",
        help="Call /hardware/set_emergency with data=false only. It does not latch emergency.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = MainboardBridgeTest(args)
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
